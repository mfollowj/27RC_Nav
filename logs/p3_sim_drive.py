#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""[一次性调试脚本] 仿真建图时自动把场地走一遍（不走键盘）。

用法：在 sim_map.sh 已经跑起来之后，另开终端
    python3 logs/p3_sim_drive.py            # 默认动作序列
    python3 logs/p3_sim_drive.py --speed 0.35
"""
import argparse
import time

import rclpy
from geometry_msgs.msg import Twist


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--speed', type=float, default=0.45, help='线速度 m/s')
    ap.add_argument('--turn', type=float, default=0.5, help='角速度 rad/s')
    ap.add_argument('--settle', type=float, default=5.0,
                    help='起步前静止等待（FAST-LIVO2 重力对齐要 1~2 s）')
    a = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('p3_sim_drive')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    def cmd(vx=0.0, vy=0.0, wz=0.0, dur=1.0):
        t0 = time.time()
        n = 0
        while time.time() - t0 < dur:
            m = Twist()
            m.linear.x = vx
            m.linear.y = vy
            m.angular.z = wz
            pub.publish(m)
            n += 1
            time.sleep(0.05)
        node.get_logger().info('cmd vx=%.2f vy=%.2f wz=%.2f %.1fs (%d 帧)'
                               % (vx, vy, wz, dur, n))

    def stop(dur=0.6):
        cmd(0.0, 0.0, 0.0, dur)

    node.get_logger().info('静止 %.1f s 等 LIO 初始化…' % a.settle)
    stop(a.settle)

    # 1) 原地转一圈：先把 360° 视野铺进地图
    for _ in range(4):
        cmd(wz=a.turn, dur=3.2)
        stop()
    # 2) 往前走 + 横移，把 -4.3,1.5 附近铺开
    cmd(vx=a.speed, dur=3.0); stop()
    cmd(vy=a.speed, dur=3.0); stop()
    cmd(vx=a.speed, dur=2.5); stop()
    # 3) 再转一圈（换视角再看一遍同一片区域，建图更实）
    for _ in range(4):
        cmd(wz=a.turn, dur=3.2)
        stop()
    # 4) 往回走，覆盖另一侧
    cmd(vx=-a.speed, dur=2.0); stop()
    cmd(vy=-a.speed, dur=2.5); stop()
    cmd(vx=a.speed, dur=2.0); stop()
    # 5) 最后再来一圈
    for _ in range(2):
        cmd(wz=-a.turn, dur=3.2)
        stop()

    stop(1.0)
    node.get_logger().info('动作序列结束')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
