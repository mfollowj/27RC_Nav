#!/usr/bin/env python3
"""把 RViz 里点出来的目标点 + 机器人当前位姿，按 0x7E/0x21 协议发给单片机。

链路（全部在机器人小电脑上跑）：

    RViz(本机) --/goal_pose--> mcu_bridge ──串口 0x7E…0x21──> 单片机
    LIO/重定位 --TF--> ────────┘  (当前位姿)

帧（详见 frame.py / README.md）：

    当前位置  7E 01 | x | y | yaw | 21
    目标位置  7E 02 | x | y | yaw | 21

    坐标：output_frame（默认 odom）下的米/弧度，×10000，大端 int32。
    目标点从 RViz 的 fixed frame（通常是 map）用 TF 换算到 output_frame 再发。

安全行为
--------
* 串口写只发生在定时器里，订阅回调里不碰串口。
* 拿不到当前位姿（TF 没起来 / LIO 没出 odom）时**不发当前帧**，并在日志里限流告警；
  MCU 侧按"当前帧超时"处理即可（不会收到假的 (0,0,0) 把车往原点拽）。
* 没收到过目标点之前不发目标帧（MCU 不会因为收到 0 目标就乱走）。
* 串口打不开/写失败按 reconnect_interval 重连，不崩。
* dry_run:=true 时不开串口，只把第一帧的十六进制打到日志里 —— 上车联调第一步用它。
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Empty, String
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)

from mcu_bridge.frame import (MSG_ID_CURRENT, MSG_ID_TARGET, EOF_POS, SOF_POS,
                              build_position_frame, build_velocity_frame)
from mcu_bridge.serial_link import SerialLink


def yaw_from_quat(q):
    """四元数 -> yaw（弧度，绕 z）。"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_rotate(q, vx, vy, vz):
    """用四元数旋转一个向量（不引入 tf_transformations 依赖）。"""
    tx = 2.0 * (q.y * vz - q.z * vy)
    ty = 2.0 * (q.z * vx - q.x * vz)
    tz = 2.0 * (q.x * vy - q.y * vx)
    return (vx + q.w * tx + (q.y * tz - q.z * ty),
            vy + q.w * ty + (q.z * tx - q.x * tz),
            vz + q.w * tz + (q.x * ty - q.y * tx))


class McuBridge(Node):

    def __init__(self):
        super().__init__('mcu_bridge')

        # ---------------- 参数 ----------------
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('output_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        # 当前位姿首选来源：LIO 的里程计话题（比 TF 稳，不受 LIO 帧名影响）
        # 置空则只用 TF
        self.declare_parameter('odom_topic', '/aft_mapped_to_init')
        self.declare_parameter('odom_timeout', 0.5)
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('target_rate_hz', 10.0)
        self.declare_parameter('current_id', MSG_ID_CURRENT)
        self.declare_parameter('target_id', MSG_ID_TARGET)
        self.declare_parameter('sof', SOF_POS)
        self.declare_parameter('eof', EOF_POS)
        self.declare_parameter('with_id', True)
        self.declare_parameter('pos_scale', 10000.0)
        self.declare_parameter('ang_scale', 10000.0)
        self.declare_parameter('byte_order', 'big')
        # 坐标约定（默认 = ROS 标准：x 前、y 左、yaw 逆时针正）
        self.declare_parameter('mount_yaw_deg', 0.0)   # 雷达相对车头的安装偏角
        self.declare_parameter('y_sign', 1)            # 1 / -1：旧约定 y 向右时填 -1
        self.declare_parameter('yaw_sign', 1)          # 1 / -1：旧约定 yaw 顺时针正时填 -1
        self.declare_parameter('reconnect_interval', 1.0)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('publish_poses', True)
        # 目标帧保持时间（秒）：收到 /goal_pose 后最多再发这么久；0 = 永远重复发
        self.declare_parameter('target_hold_sec', 5.0)
        # 旧速度帧（可选）：一个进程同时发位置和速度时用
        self.declare_parameter('enable_velocity', False)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('velocity_rate_hz', 50.0)
        self.declare_parameter('velocity_timeout', 0.2)
        self.declare_parameter('linear_scale', 1000.0)
        self.declare_parameter('angular_scale', 1000.0)
        self.declare_parameter('velocity_byte_order', 'little')
        self.declare_parameter('send_zero_on_shutdown', True)

        g = self.get_parameter
        self.port = str(g('port').value)
        self.baudrate = int(g('baudrate').value)
        self.output_frame = str(g('output_frame').value)
        self.base_frame = str(g('base_frame').value)
        self.goal_topic = str(g('goal_topic').value)
        self.odom_topic = str(g('odom_topic').value)
        self.odom_timeout = float(g('odom_timeout').value)
        self.rate = float(g('publish_rate_hz').value)
        self.target_rate = float(g('target_rate_hz').value)
        self.current_id = int(g('current_id').value) & 0xFF
        self.target_id = int(g('target_id').value) & 0xFF
        self.sof = int(g('sof').value) & 0xFF
        self.eof = int(g('eof').value) & 0xFF
        self.with_id = bool(g('with_id').value)
        self.pos_scale = float(g('pos_scale').value)
        self.ang_scale = float(g('ang_scale').value)
        self.byte_order = str(g('byte_order').value)
        self.dry_run = bool(g('dry_run').value)
        self.mount_yaw = math.radians(float(g('mount_yaw_deg').value))
        self.y_sign = 1.0 if int(g('y_sign').value) >= 0 else -1.0
        self.yaw_sign = 1.0 if int(g('yaw_sign').value) >= 0 else -1.0
        self.publish_poses = bool(g('publish_poses').value)
        self.target_hold_sec = float(g('target_hold_sec').value)
        self.enable_velocity = bool(g('enable_velocity').value)
        self.velocity_rate = float(g('velocity_rate_hz').value)
        self.velocity_timeout = float(g('velocity_timeout').value)
        self.linear_scale = float(g('linear_scale').value)
        self.angular_scale = float(g('angular_scale').value)
        self.velocity_byte_order = str(g('velocity_byte_order').value)
        self.send_zero_on_shutdown = bool(g('send_zero_on_shutdown').value)

        if self.rate <= 0.0:
            raise ValueError('publish_rate_hz 必须 > 0')
        if self.target_rate <= 0.0:
            raise ValueError('target_rate_hz 必须 > 0')
        if self.pos_scale == 0.0 or self.ang_scale == 0.0:
            raise ValueError('pos_scale / ang_scale 不能为 0')

        # 提前编一帧：参数写错（比如 byte_order 拼错）立刻报错，不要等上车
        build_position_frame(self.current_id, 0.0, 0.0, 0.0,
                             self.pos_scale, self.ang_scale,
                             self.byte_order, self.with_id,
                             self.sof, self.eof)

        # ---------------- 状态 ----------------
        self.target = None            # (x, y, yaw) in output_frame
        self.target_frame = None      # 目标点原本的坐标系（日志用）
        self.target_stamp = 0.0
        self.odom_pose = None         # (x, y, yaw) 来自 odom_topic
        self.odom_stamp = 0.0
        self.velocity = None          # (vx, vy, wz)
        self.velocity_stamp = 0.0
        self._last_no_tf_warn = 0.0
        self._last_goal_ok = 0.0

        # ---------------- 串口 ----------------
        self.link = SerialLink(self.port, self.baudrate,
                               reconnect_interval=float(g('reconnect_interval').value),
                               dry_run=self.dry_run,
                               logger=self.get_logger())

        # ---------------- TF ----------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------- 话题 ----------------
        self.create_subscription(PoseStamped, self.goal_topic,
                                 self._on_goal, 10)
        # 手动清掉目标：ros2 topic pub --once /mcu_bridge/clear std_msgs/msg/Empty {}
        self.create_subscription(Empty, '~/clear', self._on_clear, 10)
        self._target_dropped = False
        if self.odom_topic:
            self.create_subscription(Odometry, self.odom_topic,
                                     self._on_odom, 20)
        if self.enable_velocity:
            self.create_subscription(Twist, str(g('cmd_vel_topic').value),
                                     self._on_cmd_vel, 10)

        self.pub_status = self.create_publisher(String, '~/status', 10)
        self.pub_current = self.create_publisher(PoseStamped, '~/current', 10)
        self.pub_target = self.create_publisher(PoseStamped, '~/target', 10)

        # ---------------- 定时器 ----------------
        self.create_timer(1.0 / self.rate, self._send_current)
        self.create_timer(1.0 / self.target_rate, self._send_target)
        if self.enable_velocity:
            self.create_timer(1.0 / self.velocity_rate, self._send_velocity)
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            'mcu_bridge: %s | %s -> %s | %d/%d Hz | 帧 %s0x%02X {%.0f}0x%02X '
            '| x/y/yaw ×%.0f/%.0f %s | dry_run=%s'
            % (self.port, self.base_frame, self.output_frame,
               self.rate, self.target_rate,
               '[ID] ' if self.with_id else '',
               self.sof, self.pos_scale, self.eof,
               self.pos_scale, self.ang_scale, self.byte_order,
               self.dry_run))

    # ------------------------------------------------------------------ TF
    def _lookup(self, target_frame, source_frame):
        try:
            return self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self.odom_pose = (float(p.x), float(p.y),
                          yaw_from_quat(msg.pose.pose.orientation))
        self.odom_stamp = time.monotonic()

    def _to_mcu_frame(self, pose):
        """odom 系 -> 发给 MCU 的系。

        1) 减去雷达相对车头的安装偏角（mount_yaw_deg）；
        2) 按需翻转 y / yaw（兼容"y 向右、yaw 顺时针为正"的旧约定）。
        当前位姿和目标点都走这一个函数，保证闭环一致。
        """
        x, y, yaw = pose
        if self.mount_yaw != 0.0:
            c = math.cos(-self.mount_yaw)
            s = math.sin(-self.mount_yaw)
            x, y = x * c - y * s, x * s + y * c
            yaw = yaw - self.mount_yaw
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        return (x, y * self.y_sign, yaw * self.yaw_sign)

    def _current_pose(self):
        # 首选 LIO 里程计话题：机器人上跑 SCURM/New_Nav 的 fast_livo 时，
        # TF 帧名是 camera_init/aft_mapped，对不上 odom/base_link，
        # 用话题就不用关心帧名（帧名只在 goal 的坐标系换算时才需要）
        if self.odom_topic and self.odom_pose is not None and \
                time.monotonic() - self.odom_stamp < self.odom_timeout:
            return self.odom_pose
        tf = self._lookup(self.output_frame, self.base_frame)
        if tf is None:
            now = time.monotonic()
            if now - self._last_no_tf_warn > 5.0:
                self._last_no_tf_warn = now
                self.get_logger().warn(
                    '拿不到 TF %s -> %s，暂不发当前位置帧'
                    '（先起 LIO/重定位，确认 tf 树连通）'
                    % (self.output_frame, self.base_frame))
            return None
        t = tf.transform.translation
        yaw = yaw_from_quat(tf.transform.rotation)
        return (t.x, t.y, yaw)

    # --------------------------------------------------------------- 目标点
    def _on_clear(self, _msg):
        if self.target is not None:
            self.get_logger().info('收到 /mcu_bridge/clear，停止发送目标帧')
        self.target = None
        self._target_dropped = True

    def _on_goal(self, msg):
        frame = (msg.header.frame_id or self.output_frame).lstrip('/')
        px = float(msg.pose.position.x)
        py = float(msg.pose.position.y)
        pyaw = yaw_from_quat(msg.pose.orientation)

        if frame != self.output_frame:
            tf = self._lookup(self.output_frame, frame)
            if tf is None:
                self.get_logger().warn(
                    '收到目标点（%s 系），但拿不到 TF %s -> %s，先忽略。'
                    'RViz 的 Fixed Frame 是 map 时，需要重定位链路'
                    '（reloc_bridge 发 map->odom）' % (frame, self.output_frame, frame))
                return
            t = tf.transform.translation
            r = tf.transform.rotation
            dx, dy, dz = quat_rotate(r, px, py, 0.0)
            px, py = t.x + dx, t.y + dy
            pyaw = yaw_from_quat(r) + pyaw

        # 归一化到 [-pi, pi]
        pyaw = math.atan2(math.sin(pyaw), math.cos(pyaw))
        old = self.target
        self.target = (px, py, pyaw)
        self.target_frame = frame
        self.target_stamp = time.monotonic()
        self._target_dropped = False
        self._last_goal_ok = self.target_stamp
        # 有的工具会以高频重复发布同一个目标，只在真的变了的时候打日志
        changed = (old is None
                   or abs(old[0] - px) > 1e-3
                   or abs(old[1] - py) > 1e-3
                   or abs(old[2] - pyaw) > 1e-3)
        if changed:
            self.get_logger().info(
                '新目标：%s(%.2f, %.2f, %.1f°) -> %s(%.2f, %.2f, %.1f°)'
                % (frame,
                   float(msg.pose.position.x), float(msg.pose.position.y),
                   math.degrees(yaw_from_quat(msg.pose.orientation)),
                   self.output_frame, px, py, math.degrees(pyaw)))

    # --------------------------------------------------------------- 发送
    def _write(self, frame, kind):
        if not self.link.write(frame):
            return False
        if self.link.frames_sent % 100 == 1:
            self.get_logger().debug(
                '%s 帧 %s' % (kind, frame.hex(' ').upper()))
        return True

    def _send_current(self):
        pose = self._current_pose()
        if pose is None:
            return
        mcu = self._to_mcu_frame(pose)          # 串口用 MCU 约定的系
        frame = build_position_frame(
            self.current_id, mcu[0], mcu[1], mcu[2],
            self.pos_scale, self.ang_scale, self.byte_order, self.with_id,
            self.sof, self.eof)
        self._write(frame, 'current')
        if self.publish_poses:
            self._publish_pose(self.pub_current, pose)   # 调试话题仍用 odom 系

    def _send_target(self):
        if self.target is None:
            return
        if (self.target_hold_sec > 0.0 and
                time.monotonic() - self.target_stamp > self.target_hold_sec):
            # 目标已经发够时间了：停发，MCU 侧保持最后一次收到的目标
            if not self._target_dropped:
                self._target_dropped = True
                self.get_logger().info(
                    '目标帧已发满 %.1f s，停止发送（收到新目标或 /mcu_bridge/clear 前不再发）'
                    % self.target_hold_sec)
            return
        mcu = self._to_mcu_frame(self.target)
        frame = build_position_frame(
            self.target_id, mcu[0], mcu[1], mcu[2],
            self.pos_scale, self.ang_scale, self.byte_order, self.with_id,
            self.sof, self.eof)
        self._write(frame, 'target')
        if self.publish_poses:
            self._publish_pose(self.pub_target, self.target)   # 调试话题仍用 odom 系

    # ----------------------------------------------------------- 旧速度帧
    def _on_cmd_vel(self, msg):
        self.velocity = (float(msg.linear.x), float(msg.linear.y),
                         float(msg.angular.z))
        self.velocity_stamp = time.monotonic()

    def _send_velocity(self):
        if self.velocity is None:
            command = (0.0, 0.0, 0.0)
        elif (self.velocity_timeout > 0.0 and
              time.monotonic() - self.velocity_stamp > self.velocity_timeout):
            command = (0.0, 0.0, 0.0)
        else:
            command = self.velocity
        frame = build_velocity_frame(
            command[0], command[1], command[2],
            self.linear_scale, self.angular_scale, self.velocity_byte_order)
        self.link.write(frame)

    # --------------------------------------------------------------- 状态
    def _publish_pose(self, pub, pose):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.output_frame
        msg.pose.position.x = float(pose[0])
        msg.pose.position.y = float(pose[1])
        msg.pose.orientation.z = math.sin(pose[2] * 0.5)
        msg.pose.orientation.w = math.cos(pose[2] * 0.5)
        pub.publish(msg)

    def _publish_status(self):
        cur = self._current_pose()
        cur_s = ('(%.2f, %.2f, %.0f°)'
                 % (cur[0], cur[1], math.degrees(cur[2]))) if cur else 'NONE'
        tgt_s = ('(%.2f, %.2f, %.0f°)'
                 % (self.target[0], self.target[1],
                    math.degrees(self.target[2]))) if self.target else 'NONE'
        # 线上真实值（odom 经过 y_sign/yaw_sign/mount 之后），方便和 MCU 对账
        wire_c = self._to_mcu_frame(cur) if cur else None
        wire_t = self._to_mcu_frame(self.target) if self.target else None
        wire_cs = ('(%.2f, %.2f, %.1f°)'
                   % (wire_c[0], wire_c[1], math.degrees(wire_c[2]))) if wire_c else 'NONE'
        wire_ts = ('(%.2f, %.2f, %.1f°)'
                   % (wire_t[0], wire_t[1], math.degrees(wire_t[2]))) if wire_t else 'NONE'
        msg = String()
        src = 'odom_topic' if self.odom_topic else 'tf'
        msg.data = ('cur=%s tgt=%s | wire cur=%s tgt=%s | src=%s '
                    'frame[y_sign=%+d yaw_sign=%+d mount=%.1f°] %s %s'
                    % (cur_s, tgt_s, wire_cs, wire_ts, src,
                       int(self.y_sign), int(self.yaw_sign),
                       math.degrees(self.mount_yaw),
                       'port_open=%s' % self.link.is_open,
                       self.link.stats()))
        self.pub_status.publish(msg)

    # -------------------------------------------------------------- 退出
    def shutdown(self):
        if self.enable_velocity and self.send_zero_on_shutdown:
            try:
                self.link.write(build_velocity_frame(0.0, 0.0, 0.0,
                                                     self.linear_scale,
                                                     self.angular_scale,
                                                     self.velocity_byte_order))
            except Exception:               # noqa: BLE001
                pass
        self.link.close()
        # 退出阶段 rclpy context 可能已经失效，用 print 保证这行一定打得出来
        print('[mcu_bridge] 停止 | %s' % self.link.stats(), flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = McuBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGINT / SIGTERM（Ctrl-C、systemd stop、timeout）都走这里，不要打 traceback
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
