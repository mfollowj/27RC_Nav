#!/usr/bin/env python3
"""临时验证：把机器人当前位姿按 mcu_bridge 的规则编码成真实字节打印出来。

    odom(ROS: x前 y左 yaw逆时针正) --y取反--> 线上(x前 y右 yaw逆时针正) -> 15 字节帧

同时订阅 /goal_pose（odom 系），把它也编成一帧，方便核对目标方向。
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

sys.path.insert(0, '/home/rc/SCU27RC_Nav/Nav/ros2_ws/src/mcu_bridge')
from mcu_bridge.frame import build_position_frame   # noqa: E402

Y_SIGN = -1.0        # 与 mcu_bridge 配置一致
YAW_SIGN = 1.0
SOF, EOF = 0x7E, 0x21


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wire(pose):
    x, y, yaw = pose
    return (x, Y_SIGN * y, YAW_SIGN * yaw)


def hexs(frame):
    return ' '.join('%02X' % b for b in frame)


class Mon(Node):
    def __init__(self):
        super().__init__('wire_monitor')
        self.cur = None
        self.tgt = None
        self.create_subscription(Odometry, '/aft_mapped_to_init', self.on_odom, 20)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)

    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.cur = (p.x, p.y, yaw_of(q))

    def on_goal(self, m):
        self.tgt = (m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation))

    def dump(self):
        if self.cur is None:
            return
        cw = wire(self.cur)
        fc = build_position_frame(0x01, cw[0], cw[1], cw[2])
        line = ('odom(%6.2f,%6.2f,%7.1f°)  ->  wire(%6.2f,%6.2f,%7.1f°)  cur帧 %s'
                % (self.cur[0], self.cur[1], math.degrees(self.cur[2]),
                   cw[0], cw[1], math.degrees(cw[2]), hexs(fc)))
        print(line, flush=True)
        if self.tgt is not None:
            tw = wire(self.tgt)
            ft = build_position_frame(0x02, tw[0], tw[1], tw[2])
            print('   目标 odom(%6.2f,%6.2f,%7.1f°) -> wire(%6.2f,%6.2f,%7.1f°)  tgt帧 %s'
                  % (self.tgt[0], self.tgt[1], math.degrees(self.tgt[2]),
                     tw[0], tw[1], math.degrees(tw[2]), hexs(ft)), flush=True)


def main():
    rclpy.init()
    node = Mon()
    end = time.time() + float(sys.argv[1] if len(sys.argv) > 1 else 20)
    next_dump = 0.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() >= next_dump:
            node.dump()
            next_dump = time.time() + 2.0
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
