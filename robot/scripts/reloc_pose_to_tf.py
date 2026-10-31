#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 /relocalization_result（雷达在 map 系的位姿）变成 TF  map -> <initial_frame>。

用在哪：FAST-LIVO2 的 **locate_in_prior_map（先验地图跟踪）模式**下，
它自己的 odom 系叫 `camera_init`（由 publish.initial_frame_id 指定），
先验地图被搬到这个系里做连续跟踪；但 Nav2 / RViz 要的是 `map` 系。
本节点把"重定位给出的雷达在 map 系位姿"换算成 map->camera_init 广播出去。

    T_map_init = T_map_sensor * inv(T_init_sensor)
其中 T_init_sensor 就是同一时刻 LIO 报的位姿（刚开始时约等于单位阵）。

    python3 reloc_pose_to_tf.py --odom-topic /aft_mapped_to_init --child-frame camera_init
"""
import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Bridge(Node):
    def __init__(self, a):
        super().__init__('reloc_pose_to_tf')
        self.a = a
        self.reloc = None
        self.odom = None
        self.base = None          # map->init（2D 近似）
        self._last_ns = None
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PoseWithCovarianceStamped, a.reloc_topic, self.on_reloc, qos)
        self.create_subscription(Odometry, a.odom_topic, self.on_odom, 10)
        self.br = TransformBroadcaster(self)
        self.create_timer(1.0 / a.rate, self.tick)
        self.get_logger().info('%s + %s -> TF %s -> %s'
                               % (a.reloc_topic, a.odom_topic, a.map_frame, a.child_frame))

    def on_reloc(self, m):
        p = m.pose.pose
        self.reloc = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_odom(self, m):
        p = m.pose.pose.position
        self.odom = (p.x, p.y, yaw_of(m.pose.pose.orientation))

    def tick(self):
        if self.reloc is None:
            return
        if self.base is None:
            # 第一次收到重定位结果：用当时的 LIO 位姿把 map->init 固定下来
            o = self.odom or (0.0, 0.0, 0.0)
            self.base = (self.reloc[0] - o[0], self.reloc[1] - o[1],
                         wrap(self.reloc[2] - o[2]))
            self.get_logger().info('★ map->%s 固定为 x=%.3f y=%.3f yaw=%.2f deg'
                                   % (self.a.child_frame, self.base[0], self.base[1],
                                      math.degrees(self.base[2])))
        now = self.get_clock().now()
        if self._last_ns is not None and now.nanoseconds <= self._last_ns:
            return
        self._last_ns = now.nanoseconds
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.a.map_frame
        t.child_frame_id = self.a.child_frame
        t.transform.translation.x = float(self.base[0])
        t.transform.translation.y = float(self.base[1])
        t.transform.rotation.z = math.sin(self.base[2] / 2.0)
        t.transform.rotation.w = math.cos(self.base[2] / 2.0)
        self.br.sendTransform(t)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--reloc-topic', default='/relocalization_result')
    ap.add_argument('--odom-topic', default='/aft_mapped_to_init')
    ap.add_argument('--map-frame', default='map')
    ap.add_argument('--child-frame', default='camera_init')
    ap.add_argument('--rate', type=float, default=30.0)
    args, _ = ap.parse_known_args()
    rclpy.init()
    n = Bridge(args)
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
