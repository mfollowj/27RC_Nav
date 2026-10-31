#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""记录雷达轨迹（每行 "x y z"，odom 系），供 3D->2D 代价地图做射线投射。

    python3 traj_recorder.py -o maps/prior_traj.txt -t /aft_mapped_to_init
"""
import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry


class Rec(Node):
    def __init__(self, topic, path, min_step, min_yaw, rate):
        super().__init__('traj_recorder')
        self.path, self.topic = path, topic
        self.min_step, self.min_yaw, self.last, self.n = min_step, min_yaw, None, 0
        self.last_yaw = None
        # ★ 等到第一个点再打开/清空文件：没数据的失败运行不会把上一次的好轨迹清掉
        self.f = None
        self.create_subscription(Odometry, topic, self.cb,
                                 QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE))
        self.create_timer(1.0 / rate, self._flush)
        self.get_logger().info('记录轨迹 %s -> %s（收到第一个点前不覆盖旧文件）' % (topic, path))

    def _flush(self):
        if self.f is not None:
            self.f.flush()

    def cb(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        # yaw：原地旋转时 x/y 不变，只看位移会一个点都记不下来（单视点建图的坑）
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if self.last is not None:
            dxy = ((p.x - self.last[0]) ** 2 + (p.y - self.last[1]) ** 2) ** 0.5
            dyaw = abs(math.atan2(math.sin(yaw - self.last_yaw),
                                  math.cos(yaw - self.last_yaw)))
            if dxy < self.min_step and dyaw < self.min_yaw:
                return
        self.last = (p.x, p.y, p.z)
        self.last_yaw = yaw
        if self.f is None:                      # 第一个点：这时才覆盖旧轨迹
            self.f = open(self.path, 'w')
            self.f.write('# x y z  (from %s)\n' % self.topic)
        self.f.write('%.4f %.4f %.4f\n' % (p.x, p.y, p.z))
        self.n += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-t', '--topic', default='/aft_mapped_to_init')
    ap.add_argument('-o', '--out', required=True)
    ap.add_argument('--min-step', type=float, default=0.02, help='平面位移阈值 (m)')
    ap.add_argument('--min-yaw', type=float, default=math.radians(2.0),
                    help='旋转阈值 (rad)；原地旋转时靠它记点')
    ap.add_argument('--rate', type=float, default=2.0)
    args, _ = ap.parse_known_args()
    rclpy.init()
    n = Rec(args.topic, args.out, args.min_step, args.min_yaw, args.rate)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, Exception):   # SIGTERM 时 rclpy 会抛 ExternalShutdownException
        pass
    finally:
        if n.f is not None:
            n.f.close()
        if n.n > 0:
            print('[traj_recorder] %d 点 -> %s' % (n.n, args.out))
        else:
            print('[traj_recorder] 没记录到点，保留原轨迹文件：%s' % args.out)
        # 被 timeout/SIGTERM 结束时 rclpy 可能已经自己 shutdown 过了，
        # 再调一次会抛 "rcl_shutdown already called"（看着像报错，其实没事）
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
