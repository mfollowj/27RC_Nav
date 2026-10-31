#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""真实 Mid-360 雷达数据预处理：控频率、降点数、去脏点，同时**不破坏 LIO 精度**。

为什么不能简单地"降采样"
----------------------
真实 Mid-360 点率固定 20 万点/秒（15 Hz 分组时每帧约 1.3 万点）。直接把原始 CustomMsg 丢给
FAST-LIVO2 时，CPU 很容易跟不上（表现为点云/位姿发布掉帧、建图出现拖影）。
但如果把雷达换成 PointCloud2 格式（xfer_format:=0）再抽稀，就会**丢掉 offset_time**，
LIO 无法做运动补偿（去畸变），转弯时精度反而更差。

本节点的做法（兼顾效率与精度）
----------------------------
* 订阅的还是 **CustomMsg**（带 offset_time），发给 LIO 的也是 CustomMsg ——
  抽稀后每个点**仍保留原来的 offset_time**，所以 LIO 的逐点去畸变照常工作；
* 抽稀用"**等步长抽样**"（在整帧的扫描序列上均匀取样），而不是"只取前 N 个"或
  "空间体素合并"：Livox 非重复扫描在时间上就把整个视场铺开，等步长抽样依然覆盖
  全视场，只是每个方向上的点变稀 —— 对 LIO 的平面/边缘约束几乎没有损失；
* Python 侧只访问被保留下来的那些点（`for i in range(0, n, step)`），
  24 万点/秒的属性访问量降到约 4 万/秒，单核 CPU 占用很低；
* 过滤规则：无回波假点(|p|=0)、量程、高度带、reflectivity 下限、NaN/inf；
* 可选限频（`--rate`）：默认 15 Hz（= 驱动 publish_freq = LIO scan_rate）；
  CPU 很弱时可以降到 5~10 Hz，LIO 侧靠 IMU 兜住。

    ros2 run 不需要本包：直接 python3 跑即可（launch 里就是这么调用的）
    python3 livox_preprocess.py --points-per-frame 4000 --rate 15

订阅：/livox/lidar            (livox_ros_driver2/msg/CustomMsg)
发布：/livox/lidar_filtered   (CustomMsg，给 FAST-LIVO2)
      /livox/points_filtered  (PointCloud2，可选，给其它消费者/调试)
      /livox_preprocess_stats (std_msgs/String，统计信息，便于 RViz/终端观察)
"""
import argparse
import math
import sys
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from livox_ros_driver2.msg import CustomMsg, CustomPoint
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String, Header


class LivoxPreprocess(Node):
    def __init__(self, a):
        super().__init__('livox_preprocess')
        self.a = a
        # 驱动是 RELIABLE 发布，这里必须用 RELIABLE，否则一帧都收不到
        qos = QoSProfile(depth=a.queue, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(CustomMsg, a.out_topic, qos)
        self.pub_pc2 = None
        if a.pc2_topic:
            self.pub_pc2 = self.create_publisher(PointCloud2, a.pc2_topic, 5)
        self.pub_stat = self.create_publisher(String, '/livox_preprocess_stats', 5)
        self.create_subscription(CustomMsg, a.in_topic, self.cb, qos)

        self.n_in = 0
        self.n_out = 0
        self.n_frames_in = 0
        self.n_frames_out = 0
        self.next_pub_t = 0.0          # 下一次允许发布的时间（绝对时刻，不是"距上次多久"）
        self.last_report = time.time()
        self.kept = 0
        self.dropped = 0
        self.d_range = 0               # 分原因计数，方便按实际数据调门限
        self.d_z = 0
        self.d_refl = 0
        self.d_nan = 0
        self.create_timer(5.0, self.report)
        self.get_logger().info(
            '雷达预处理：%s -> %s | 每帧保留<=%d 点(等步长) 限频<=%.1fHz '
            '量程[%.2f, %.1f]m 高度[%.2f, %.2f]m reflectivity>=%d'
            % (a.in_topic, a.out_topic, a.points_per_frame, a.rate,
               a.min_range, a.max_range, a.min_z, a.max_z, a.min_reflectivity))

    # ------------------------------------------------------------------
    def cb(self, msg):
        self.n_frames_in += 1
        n = msg.point_num
        if n <= 0:
            return

        # ---- 限频 ----
        # ★ 用"下一次允许发布的绝对时刻"而不是"距上次发布是否 < 1/rate"：
        #   后者在输入帧率已经≈目标帧率时会因为抖动**成片丢帧**
        #   （实测：输入 9.9 Hz、目标 10 Hz，输出只有 6.9 Hz）。
        now = time.time()
        period = 1.0 / self.a.rate if self.a.rate > 0 else 0.0
        # 留 25% 的容差：输入帧率≈目标帧率时，抖动会让某一帧早到几微秒，
        # 没有容差就会"丢一帧、等一帧"，输出直接掉到一半（实测踩过）
        if period > 0.0 and now < self.next_pub_t - 0.25 * period:
            return

        # ---- 等步长抽样：只访问保留下来的点，Python 开销与 --points-per-frame 成正比 ----
        step = max(1, int(math.ceil(n / float(self.a.points_per_frame))))
        keep = []
        r2max = self.a.max_range * self.a.max_range
        r2min = self.a.min_range * self.a.min_range
        pts = msg.points
        for i in range(0, n, step):
            p = pts[i]
            x = p.x
            y = p.y
            z = p.z
            r2 = x * x + y * y + z * z
            # 1) 无回波假点（x=y=z=0）与盲区
            if r2 < r2min or r2 > r2max:
                self.dropped += 1
                self.d_range += 1
                continue
            # 2) NaN / inf 保护（个别固件会吐脏值）
            if not (x == x and y == y and z == z):
                self.dropped += 1
                self.d_nan += 1
                continue
            # 3) 高度带：雷达坐标系下 z（可选，默认放得很宽，不砍结构）
            if z < self.a.min_z or z > self.a.max_z:
                self.dropped += 1
                self.d_z += 1
                continue
            # 4) reflectivity 下限：滤掉极低反射率的浮尘/噪声
            if p.reflectivity < self.a.min_reflectivity:
                self.dropped += 1
                self.d_refl += 1
                continue
            keep.append(p)
            self.kept += 1

        if len(keep) < self.a.min_points:
            if self.a.verbose:
                self.get_logger().warn('有效点只有 %d，本帧丢弃' % len(keep))
            return

        out = CustomMsg()
        out.header = msg.header
        out.timebase = msg.timebase
        out.lidar_id = msg.lidar_id
        out.rsvd = msg.rsvd
        out.points = keep
        out.point_num = len(keep)
        self.pub.publish(out)
        # 目标帧率用一个绝对时刻递推：输入帧率≈目标帧率时不会因为抖动丢帧，
        # 输入远高于目标时又能稳定压到目标帧率
        # 用 max(now, next_pub_t) 递推：早到/晚到都不会让节拍累积漂移
        self.next_pub_t = (max(now, self.next_pub_t) + period) if period > 0.0 else 0.0
        self.n_frames_out += 1
        self.n_out += len(keep)

        if self.pub_pc2 is not None:
            self.publish_pc2(keep, msg.header)

    # ------------------------------------------------------------------
    def publish_pc2(self, keep, hdr):
        arr = np.empty((len(keep), 4), dtype=np.float32)
        for i, p in enumerate(keep):
            arr[i, 0] = p.x
            arr[i, 1] = p.y
            arr[i, 2] = p.z
            arr[i, 3] = float(p.reflectivity)
        m = PointCloud2()
        m.header = Header()
        m.header.stamp = hdr.stamp
        m.header.frame_id = self.a.frame_id or hdr.frame_id
        m.height = 1
        m.width = arr.shape[0]
        m.fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)]
        m.is_bigendian = False
        m.point_step = 16
        m.row_step = 16 * arr.shape[0]
        m.is_dense = True
        m.data = arr.tobytes()
        self.pub_pc2.publish(m)

    # ------------------------------------------------------------------
    def report(self):
        dt = time.time() - self.last_report
        self.last_report = time.time()
        fin = self.n_frames_in
        fout = self.n_frames_out
        self.n_frames_in = 0
        self.n_frames_out = 0
        tot = self.kept + self.dropped
        rate_in = fin / dt if dt > 0 else 0.0
        rate_out = fout / dt if dt > 0 else 0.0
        pps = self.n_out / dt if dt > 0 else 0.0
        pct = (100.0 * self.dropped / tot) if tot else 0.0
        s = ('输入 %.1f Hz / 输出 %.1f Hz | 每帧保留 %.0f 点 | 输出 %.0f 点/秒 | '
             '过滤掉 %.0f%%（量程 %d / 高度 %d / 反射率 %d / 坏值 %d）'
             % (rate_in, rate_out, (self.n_out / fout) if fout else 0.0, pps, pct,
                self.d_range, self.d_z, self.d_refl, self.d_nan))
        self.get_logger().info(s)
        self.pub_stat.publish(String(data=s))
        self.n_out = 0
        self.kept = 0
        self.dropped = 0
        self.d_range = self.d_z = self.d_refl = self.d_nan = 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--in-topic', default='/livox/lidar')
    ap.add_argument('--out-topic', default='/livox/lidar_filtered')
    ap.add_argument('--pc2-topic', default='',
                    help='可选：再发一份 PointCloud2（默认不发，LIO 用 /cloud_registered 就够）')
    ap.add_argument('--points-per-frame', type=int, default=4000,
                    help='每帧最多保留多少点（等步长抽样）。真实 Mid-360 一帧约 2.4 万点，'
                         '取 3000~6000 是"适中"区间：LIO 精度基本无损，CPU 降一个量级；'
                         '低于 1500 点会明显影响建图质量')
    ap.add_argument('--rate', type=float, default=15.0,
                    help='输出频率上限 Hz（0=不限）。与驱动 publish_freq / '
                         'LIO 的 scan_rate 保持一致（默认 15）')
    ap.add_argument('--min-range', type=float, default=0.5,
                    help='最小量程（车体自身/支架），Mid-360 盲区约 0.1m，装车后 0.5 比较稳')
    ap.add_argument('--max-range', type=float, default=60.0,
                    help='最大量程。室内 30~60 足够；调小省 CPU 也更抗远距离噪声')
    ap.add_argument('--min-z', type=float, default=-3.0)
    ap.add_argument('--max-z', type=float, default=10.0)
    ap.add_argument('--min-reflectivity', type=int, default=0)
    ap.add_argument('--min-points', type=int, default=50,
                    help='一帧有效点少于该值就整帧丢弃（避免把空帧喂给 LIO）')
    ap.add_argument('--frame-id', default='', help='留空则沿用输入 frame_id')
    ap.add_argument('--queue', type=int, default=10)
    ap.add_argument('--verbose', action='store_true')
    args, _ = ap.parse_known_args()

    rclpy.init()
    n = LivoxPreprocess(args)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
