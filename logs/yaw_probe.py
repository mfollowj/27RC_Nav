#!/usr/bin/env python3
"""临时诊断：打印 /aft_mapped_to_init 的 yaw 随时间变化，以及 /imu/data 首帧姿态。"""
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu

T0 = time.time()


def yaw_of(q):
    return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


class Probe(Node):
    def __init__(self):
        super().__init__('yaw_probe')
        self.create_subscription(Odometry, '/aft_mapped_to_init', self.on_odom, 20)
        self.create_subscription(Imu, '/imu/data', self.on_imu, 20)
        self.n = 0
        self.imu_n = 0

    def on_odom(self, m):
        self.n += 1
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        if self.n <= 3 or self.n % 25 == 0:
            print('%6.2fs  #%-5d yaw=%8.2f deg  xyz=(%.3f, %.3f, %.3f)  quat=(%.3f,%.3f,%.3f,%.3f)'
                  % (time.time() - T0, self.n, yaw_of(q), p.x, p.y, p.z,
                     q.x, q.y, q.z, q.w), flush=True)

    def on_imu(self, m):
        self.imu_n += 1
        if self.imu_n <= 3:
            q = m.orientation
            a = m.linear_acceleration
            print('%6.2fs  IMU #%d orientation=(%.3f,%.3f,%.3f,%.3f) acc=(%.2f, %.2f, %.2f)'
                  % (time.time() - T0, self.imu_n, q.x, q.y, q.z, q.w,
                     a.x, a.y, a.z), flush=True)


def main():
    rclpy.init()
    node = Probe()
    end = time.time() + float(sys.argv[1] if len(sys.argv) > 1 else 60.0)
    try:
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
