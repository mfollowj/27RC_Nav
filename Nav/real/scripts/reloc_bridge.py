#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""reloc_bridge —— 把 6-DOF 全局重定位的结果喂给 FAST-LIVO2（支持大范围漂移后恢复）。

为什么需要这一层
----------------
`teaser_gicp_node`（relocalization 包）只订阅**原始雷达**，输出的是
    S = map ← livox_frame
也就是"雷达在 map 里的位姿"。而 FAST-LIVO2 的 `initial_pose_cbk` 需要的是
    P = map ← base_link（机器人位姿），并且在 handleLIO 里会把 odom 归零、
    把 P 直接当作 map→odom 使用。

于是这里做三件事：
  1. 坐标换算：P = S · E⁻¹，E = base_link←livox_frame（静态外参）；
  2. 稳定性过滤：最近 N 帧 P 彼此一致才认（甩雷达时重定位输出会乱跳）；
  3. 只在"可信 + 与已生效位姿差得够多"时发一条 /relocalization_result，
     同时把 map→odom 发成 P（odom 在 LIO 那侧已归零，所以就是 P）。

话题
----
订阅  /icp_result            PoseWithCovarianceStamped   （map←livox）
      /aft_mapped_to_init    Odometry                    （仅用于状态显示/漂移提示）
发布  /relocalization_result PoseWithCovarianceStamped   （map←base，给 FAST-LIVO2）
      /reloc_status          String
      TF map→odom

用法
----
    python3 reloc_bridge.py --map-frame map --base-frame base_link \
        --lidar-frame livox_frame --ext-x -0.011 --ext-y -0.02329 --ext-z 0.04412
"""
import argparse
import math
import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


# ----------------------------------------------------------------------------
# 小工具：四元数 / 矩阵
# ----------------------------------------------------------------------------
def quat_to_rot(q):
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-9:
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    w, x, y, z = w / n, x / n, y / n, z / n
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ]


def rot_to_quat(R):
    tr = R[0][0] + R[1][1] + R[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2][1] - R[1][2]) / s
        y = (R[0][2] - R[2][0]) / s
        z = (R[1][0] - R[0][1]) / s
    elif R[0][0] > R[1][1] and R[0][0] > R[2][2]:
        s = math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2
        w = (R[2][1] - R[1][2]) / s
        x = 0.25 * s
        y = (R[0][1] + R[1][0]) / s
        z = (R[0][2] + R[2][0]) / s
    elif R[1][1] > R[2][2]:
        s = math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2
        w = (R[0][2] - R[2][0]) / s
        x = (R[0][1] + R[1][0]) / s
        y = 0.25 * s
        z = (R[1][2] + R[2][1]) / s
    else:
        s = math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2
        w = (R[1][0] - R[0][1]) / s
        x = (R[0][2] + R[2][0]) / s
        y = (R[1][2] + R[2][1]) / s
        z = 0.25 * s
    return w, x, y, z


def rot_diff_deg(R1, R2):
    s = 0.0
    for i in range(3):
        for j in range(3):
            s += R1[i][j] * R2[i][j]
    c = max(-1.0, min(1.0, (s - 1.0) * 0.5))
    return math.degrees(math.acos(c))


def pose_to_T(msg):
    p = msg.pose.pose.position
    o = msg.pose.pose.orientation
    R = quat_to_rot((o.w, o.x, o.y, o.z))
    T = [
        [R[0][0], R[0][1], R[0][2], p.x],
        [R[1][0], R[1][1], R[1][2], p.y],
        [R[2][0], R[2][1], R[2][2], p.z],
        [0.0, 0.0, 0.0, 1.0],
    ]
    return T


def T_mul(A, B):
    C = [[0.0] * 4 for _ in range(4)]
    for i in range(4):
        for j in range(4):
            C[i][j] = sum(A[i][k] * B[k][j] for k in range(4))
    return C


def T_inv(T):
    R = [[T[0][0], T[0][1], T[0][2]],
         [T[1][0], T[1][1], T[1][2]],
         [T[2][0], T[2][1], T[2][2]]]
    t = [T[0][3], T[1][3], T[2][3]]
    Rt = [[R[j][i] for j in range(3)] for i in range(3)]
    ti = [-sum(Rt[i][k] * t[k] for k in range(3)) for i in range(3)]
    return [
        [Rt[0][0], Rt[0][1], Rt[0][2], ti[0]],
        [Rt[1][0], Rt[1][1], Rt[1][2], ti[1]],
        [Rt[2][0], Rt[2][1], Rt[2][2], ti[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def T_trans(T):
    return math.sqrt(T[0][3] ** 2 + T[1][3] ** 2 + T[2][3] ** 2)


def T_rot(T):
    return [[T[0][0], T[0][1], T[0][2]],
            [T[1][0], T[1][1], T[1][2]],
            [T[2][0], T[2][1], T[2][2]]]


IDENTITY = [[1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]]


def euler_to_R(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


class RelocBridge(Node):
    def __init__(self, a):
        super().__init__('reloc_bridge')
        self.a = a

        # E = base ← livox（静态外参）
        R = euler_to_R(a.ext_roll, a.ext_pitch, a.ext_yaw)
        self.T_base_lidar = [
            [R[0][0], R[0][1], R[0][2], a.ext_x],
            [R[1][0], R[1][1], R[1][2], a.ext_y],
            [R[2][0], R[2][1], R[2][2], a.ext_z],
            [0.0, 0.0, 0.0, 1.0],
        ]
        self.T_lidar_base = T_inv(self.T_base_lidar)

        self.hist = deque(maxlen=max(1, a.stable_count))
        self.res_hist = deque(maxlen=max(1, a.stable_count))
        self.applied = None          # 已生效的 map←base
        self.candidate = None
        self.spread_t = 0.0
        self.spread_r = 0.0
        self.n_icp = 0
        self.n_applied = 0
        self.n_first = 0
        self.n_recover = 0
        self.res_max = 0.0
        self.resr_max = 0.0
        self.t_start = time.time()
        self.t_last_log = 0.0
        self.lio_pos = None
        self.odom_T = None          # odom←base（LIO 当前输出）
        self.t_last_apply = 0.0
        self.last_diff = (0.0, 0.0)
        self.publish_tf_en = not getattr(a, 'no_tf', False)
        self.bootstrap_tf = not getattr(a, 'no_bootstrap_tf', False)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PoseWithCovarianceStamped, a.icp_topic, self.on_icp, qos)
        if a.odom_topic:
            self.create_subscription(Odometry, a.odom_topic, self.on_odom, 10)

        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, a.out_topic, 10)
        self.pub_stat = self.create_publisher(String, a.status_topic, 10)
        self.tf_br = TransformBroadcaster(self)

        self.create_timer(1.0 / max(a.tf_rate, 1.0), self.publish_tf)
        self.create_timer(0.5, self.publish_status)
        self.get_logger().info(
            'reloc_bridge: %s(map<-livox) -> %s(map<-base)；外参 E=%.3f,%.3f,%.3f rpy=%.1f,%.1f,%.1f；'
            '稳定 %d 帧 / %.2f m / %.1f deg；与已生效位姿差 > %.2f m 或 %.1f deg 才重建'
            % (a.icp_topic, a.out_topic, a.ext_x, a.ext_y, a.ext_z,
               a.ext_roll, a.ext_pitch, a.ext_yaw, a.stable_count,
               a.stable_trans, a.stable_rot, a.min_change_trans, a.min_change_rot))
        if self.bootstrap_tf:
            self.get_logger().info(
                '未定位前先发 identity map->odom（此时 map=odom，仅用于让 RViz 有 map 帧）；'
                '第一次定位成功后会切换成真实值。')

    # ---------------- 回调 ----------------
    def on_odom(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        R = quat_to_rot((o.w, o.x, o.y, o.z))
        self.odom_T = [
            [R[0][0], R[0][1], R[0][2], p.x],
            [R[1][0], R[1][1], R[1][2], p.y],
            [R[2][0], R[2][1], R[2][2], p.z],
            [0.0, 0.0, 0.0, 1.0],
        ]
        self.lio_pos = math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z)

    def on_icp(self, msg):
        self.n_icp += 1
        T_map_lidar = pose_to_T(msg)
        P = T_mul(T_map_lidar, self.T_lidar_base)      # map←base
        self.hist.append(P)

        if len(self.hist) < self.a.stable_count:
            return

        worst_t = 0.0
        worst_r = 0.0
        for T in self.hist:
            dt = math.sqrt(sum((T[i][3] - P[i][3]) ** 2 for i in range(3)))
            dr = rot_diff_deg(T_rot(T), T_rot(P))
            worst_t = max(worst_t, dt)
            worst_r = max(worst_r, dr)
        self.spread_t, self.spread_r = worst_t, worst_r
        self.candidate = P

        if self.applied is None:
            # ---- 首次定位：车应静止，要求绝对位姿连续一致 ----
            if worst_t > self.a.stable_trans or worst_r > self.a.stable_rot:
                return
            self.apply(P, 'FIRST')
            return

        # ---- 发散保护：LIO 已经飞了（odom 模长或残差远超常理）----
        # 此时残差每帧都在变，正常的"残差稳定"判据永远过不了，必须走这条旁路：
        # 只看**重定位自身**是否稳定，稳定就立刻采纳，让 LIO 收到远处位姿 ->
        # diff 超过 map_reinit_* -> odom 归零重建。
        expected0 = T_mul(self.applied, self.odom_T) if self.odom_T is not None else self.applied
        R0 = T_mul(P, T_inv(expected0))
        rt0 = math.sqrt(R0[0][3] ** 2 + R0[1][3] ** 2 + R0[2][3] ** 2)
        rr0 = rot_diff_deg(T_rot(R0), [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
        lio_drift = (self.lio_pos is not None) and (self.lio_pos > self.a.lost_trans)
        if lio_drift or rt0 > self.a.lost_residual or rr0 > self.a.lost_rot:
            self.last_diff = (rt0, rr0)
            self.res_max = max(self.res_max, rt0)
            self.resr_max = max(self.resr_max, rr0)
            # 1 s 保持：应用一次后先等 LIO 归零/重建，避免每帧重复发
            if time.time() - self.t_last_apply < 1.0:
                return
            if worst_t <= self.a.stable_trans and worst_r <= self.a.stable_rot:
                self.apply(P, 'LOST_RECOVER(%.0f m / %.0f deg)' % (rt0, rr0))
            return

        if time.time() - self.t_last_apply < self.a.cooldown:
            return                                # 刚应用过，等 LIO 把 odom 归零/稳定

        # ---- 已定位：用"重定位结果 vs 当前估计(applied·odom)"的**残差**判稳 ----
        # 车在动时 P 和 expected 一起动，残差仍然稳定；只有 LIO 真漂了残差才会跳。
        expected = T_mul(self.applied, self.odom_T) if self.odom_T is not None else self.applied
        R = T_mul(P, T_inv(expected))
        rt = math.sqrt(R[0][3] ** 2 + R[1][3] ** 2 + R[2][3] ** 2)
        rr = rot_diff_deg(T_rot(R), [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
        self.last_diff = (rt, rr)
        self.res_max = max(self.res_max, rt)
        self.resr_max = max(self.resr_max, rr)
        self.res_hist.append(R)
        if len(self.res_hist) < self.a.stable_count:
            return
        worst_rt = worst_rr = 0.0
        for T in self.res_hist:
            d = T_mul(T_inv(T), R)
            worst_rt = max(worst_rt, math.sqrt(d[0][3] ** 2 + d[1][3] ** 2 + d[2][3] ** 2))
            worst_rr = max(worst_rr, rot_diff_deg(T_rot(T), T_rot(R)))
        if worst_rt > self.a.stable_trans or worst_rr > self.a.stable_rot:
            return
        if rt > self.a.min_change_trans or rr > self.a.min_change_rot:
            self.apply(P, 'RECOVER(%.2f m / %.1f deg)' % (rt, rr))

    def apply(self, P, why):
        self.applied = P
        self.res_hist.clear()
        self.n_applied += 1
        if why.startswith('FIRST'):
            self.n_first += 1
        else:
            self.n_recover += 1
        self.t_last_apply = time.time()
        self.get_logger().warn(
            '★ 接受重定位 [%s]：map<-base x=%.3f y=%.3f z=%.3f（第 %d 次）'
            % (why, P[0][3], P[1][3], P[2][3], self.n_applied))
        self.publish_pose()

    # ---------------- 输出 ----------------
    def publish_pose(self):
        if self.applied is None:
            return
        T = self.applied
        m = PoseWithCovarianceStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.a.map_frame
        m.pose.pose.position.x = float(T[0][3])
        m.pose.pose.position.y = float(T[1][3])
        m.pose.pose.position.z = float(T[2][3])
        w, x, y, z = rot_to_quat(T_rot(T))
        m.pose.pose.orientation.w = w
        m.pose.pose.orientation.x = x
        m.pose.pose.orientation.y = y
        m.pose.pose.orientation.z = z
        # 给一个中等置信度，便于下游判断
        m.pose.covariance[0] = 0.05 ** 2
        m.pose.covariance[7] = 0.05 ** 2
        m.pose.covariance[35] = math.radians(2.0) ** 2
        self.pub_pose.publish(m)

    def publish_tf(self):
        if not self.publish_tf_en:
            return
        if self.applied is not None:
            # odom 在 LIO 那侧已归零，所以 map→odom 就是已生效的 map←base
            T = self.applied
        elif self.bootstrap_tf:
            # 还没定位：先发 identity，让 RViz / Nav2 有个 map 帧可用（仅可视化，
            # 此时 map 与 odom 重合，先验图和实时点云可能是错位的）。
            T = IDENTITY
        else:
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.a.map_frame
        t.child_frame_id = self.a.odom_frame
        t.transform.translation.x = float(T[0][3])
        t.transform.translation.y = float(T[1][3])
        t.transform.translation.z = float(T[2][3])
        w, x, y, z = rot_to_quat(T_rot(T))
        t.transform.rotation.w = w
        t.transform.rotation.x = x
        t.transform.rotation.y = y
        t.transform.rotation.z = z
        self.tf_br.sendTransform(t)

    def publish_status(self):
        # 周期重发一次已生效位姿：晚加入的订阅者也能拿到，LIO 侧会判定"与已生效一致"
        if self.applied is not None:
            self.publish_pose()
        if self.applied is None:
            mode = 'BOOTSTRAP' if self.bootstrap_tf else 'SEARCHING'
        else:
            dt = time.time() - self.t_last_apply
            mode = 'APPLIED' if dt < 2.0 else 'TRACKING'
        lio = 'lio_norm=%.2f' % self.lio_pos if self.lio_pos is not None else 'lio_norm=NA'
        self.pub_stat.publish(String(data=(
            'mode=%s icp=%d applied=%d first=%d recover=%d spread=%.2f m / %.1f deg '
            'diff=%.2f m / %.1f deg res_max=%.2f m / %.1f deg %s'
            % (mode, self.n_icp, self.n_applied, self.n_first, self.n_recover,
               self.spread_t, self.spread_r, self.last_diff[0], self.last_diff[1],
               self.res_max, self.resr_max, lio))))

        # ---- 周期打印残差：判断"真漂"还是"重定位噪声" ----
        now = time.time()
        if now - self.t_last_log >= 2.0:
            self.t_last_log = now
            self.get_logger().info(
                'icp=%d applied=%d(first %d/recover %d) residual=%.2f m / %.1f deg '
                '(max %.2f m / %.1f deg) %s'
                % (self.n_icp, self.n_applied, self.n_first, self.n_recover,
                   self.last_diff[0], self.last_diff[1], self.res_max, self.resr_max, lio))

    def print_summary(self):
        dur = time.time() - self.t_start
        # 退出时 rclpy 上下文可能已经关闭，用 print 而不是 rosout，保证一定能看到
        print('[reloc_bridge] ===== 导航结束：重定位 %d 次（首次定位 %d / 漂移修正 %d），'
              '收到 %d 帧重定位结果，最大残差 %.2f m / %.1f deg，时长 %.0f s ====='
              % (self.n_applied, self.n_first, self.n_recover, self.n_icp,
                 self.res_max, self.resr_max, dur), flush=True)


def main():
    ap = argparse.ArgumentParser(description='6-DOF 重定位结果 -> FAST-LIVO2（支持漂移恢复）')
    ap.add_argument('--icp-topic', default='/icp_result')
    ap.add_argument('--odom-topic', default='/aft_mapped_to_init')
    ap.add_argument('--out-topic', default='/relocalization_result')
    ap.add_argument('--status-topic', default='/reloc_status')
    ap.add_argument('--map-frame', default='map')
    ap.add_argument('--odom-frame', default='odom')
    ap.add_argument('--base-frame', default='base_link')
    ap.add_argument('--lidar-frame', default='livox_frame')
    # base ← livox 外参（与 real_localization.launch.py 的静态 TF 一致）
    ap.add_argument('--ext-x', type=float, default=-0.011)
    ap.add_argument('--ext-y', type=float, default=-0.02329)
    ap.add_argument('--ext-z', type=float, default=0.04412)
    ap.add_argument('--ext-roll', type=float, default=0.0, help='度')
    ap.add_argument('--ext-pitch', type=float, default=0.0, help='度')
    ap.add_argument('--ext-yaw', type=float, default=0.0, help='度')
    # 稳定性 / 触发阈值
    ap.add_argument('--stable-count', type=int, default=3)
    ap.add_argument('--stable-trans', type=float, default=0.25)
    ap.add_argument('--stable-rot', type=float, default=8.0)
    ap.add_argument('--min-change-trans', type=float, default=0.5)
    ap.add_argument('--min-change-rot', type=float, default=10.0)
    ap.add_argument('--lost-trans', type=float, default=50.0,
                    help='LIO 位姿模长超过它视为发散，跳过残差稳定判据直接采纳重定位')
    ap.add_argument('--lost-residual', type=float, default=5.0,
                    help='残差平移超过它也视为发散 (m)')
    ap.add_argument('--lost-rot', type=float, default=30.0,
                    help='残差转角超过它也视为发散 (deg)')
    ap.add_argument('--cooldown', type=float, default=3.0,
                    help='应用一次后多少秒内不再接受新结果（等 LIO 归零/稳定）')
    ap.add_argument('--tf-rate', type=float, default=30.0)
    ap.add_argument('--no-tf', action='store_true', help='不发 map->odom（调试）')
    ap.add_argument('--no-bootstrap-tf', action='store_true',
                    help='未定位前不发 identity map->odom（默认发，方便 RViz 有 map 帧）')
    a, _ = ap.parse_known_args()

    rclpy.init()
    node = RelocBridge(a)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):   # SIGTERM 时 rclpy 会抛 ExternalShutdownException
        pass
    finally:
        node.print_summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
