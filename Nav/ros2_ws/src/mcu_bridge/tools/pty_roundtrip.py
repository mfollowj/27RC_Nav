#!/usr/bin/env python3
"""PTY 假串口端到端测试：不需要单片机、不碰 /dev/ttyUSB0。

做完三件事：
  1. 开一个 pty，把它的 slave 端当串口交给 mcu_bridge_node；
  2. 发一个静态 TF map->base_link，再往 /goal_pose 发一个目标点；
  3. 从 pty 的 master 端把字节读回来，拆帧、解码、断言。

用法（机器人上，或任何有 ROS 2 的机器）：

    source /opt/ros/humble/setup.bash && source install/setup.bash
    python3 Nav/ros2_ws/src/mcu_bridge/tools/pty_roundtrip.py
"""
import os
import pty
import select
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from mcu_bridge.frame import (MSG_ID_CURRENT, MSG_ID_TARGET,  # noqa: E402
                              decode_position_frame)
from mcu_bridge.sniff import frames_from_buffer  # noqa: E402

GOAL = (1.5, -0.8, 1.5707963)          # x, y, yaw(rad)
TOL = 2e-4                              # 1/10000 的量化误差


def make_tf(node, parent, child, x, y, yaw=0.0):
    import math
    t = TransformStamped()
    t.header.stamp = node.get_clock().now().to_msg()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.rotation.z = math.sin(yaw / 2.0)
    t.transform.rotation.w = math.cos(yaw / 2.0)
    return t


def main():
    master, slave = pty.openpty()
    slave_name = os.ttyname(slave)
    print('pty slave: %s' % slave_name, flush=True)

    env = dict(os.environ)
    env.setdefault('ROS_DOMAIN_ID', '42')
    cmd = ['ros2', 'run', 'mcu_bridge', 'mcu_bridge_node', '--ros-args',
           '-p', 'port:=%s' % slave_name,
           '-p', 'baudrate:=115200',
           '-p', 'output_frame:=map',
           '-p', 'base_frame:=base_link',
           '-p', 'publish_rate_hz:=50.0',
           '-p', 'target_rate_hz:=30.0',
           '-p', 'publish_poses:=false']
    # 子进程的 stdout 必须重定向到文件：如果它继承我们的 stdout，
    # 被 kill 剩下来的进程会一直占着管道，调用方（ssh | tail）永远等不到 EOF。
    node_log = tempfile.NamedTemporaryFile(prefix='mcu_bridge_pty_',
                                           suffix='.log', delete=False)
    proc = subprocess.Popen(cmd, env=env, stdout=node_log,
                            stderr=subprocess.STDOUT, start_new_session=True)
    print('node pid: %d | log: %s' % (proc.pid, node_log.name), flush=True)
    rclpy.init()
    node = Node('pty_roundtrip_driver')
    tf_pub = StaticTransformBroadcaster(node)
    goal_pub = node.create_publisher(PoseStamped, '/goal_pose', 10)

    buf = b''
    frames = []
    try:
        time.sleep(3.0)
        tf_pub.sendTransform(make_tf(node, 'map', 'base_link', 0.2, 0.3, 0.1))

        goal = PoseStamped()
        goal.header.stamp = node.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x = GOAL[0]
        goal.pose.position.y = GOAL[1]
        import math
        goal.pose.orientation.z = math.sin(GOAL[2] / 2.0)
        goal.pose.orientation.w = math.cos(GOAL[2] / 2.0)

        t_end = time.time() + 6.0
        while time.time() < t_end:
            goal_pub.publish(goal)
            rclpy.spin_once(node, timeout_sec=0.02)
            r, _, _ = select.select([master], [], [], 0.02)
            if r:
                buf += os.read(master, 4096)
            new, buf = frames_from_buffer(buf, with_id=True)
            frames.extend(new)
    finally:
        # start_new_session=True -> 杀整个进程组，避免 ros2 run 的孙子进程残留
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except (ProcessLookupError, PermissionError):
            pass
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass
            proc.wait(timeout=5)
        node_log.close()
        node.destroy_node()
        rclpy.shutdown()
        os.close(master)
        os.close(slave)

    cur = [f for f in frames if f[0] == MSG_ID_CURRENT]
    tgt = [f for f in frames if f[0] == MSG_ID_TARGET]
    print('收到 %d 帧：current=%d target=%d' % (len(frames), len(cur), len(tgt)))
    with open(node_log.name, encoding='utf-8', errors='replace') as f:
        tail = f.read().strip().splitlines()[-8:]
    if tail:
        print('--- 节点日志 ---')
        for line in tail:
            print('   ', line)
    if frames:
        print('样例 current:', cur[0] if cur else None)
        print('样例 target :', tgt[0] if tgt else None)

    ok = True
    scale = 10000.0
    if not cur:
        print('FAIL 没有当前位置帧')
        ok = False
    else:
        x, y, yaw = [v / scale for v in cur[-1][1:]]
        if abs(x - 0.2) > TOL or abs(y - 0.3) > TOL or abs(yaw - 0.1) > TOL:
            print('FAIL 当前位姿不对：%.4f %.4f %.4f（应为 0.2 0.3 0.1）'
                  % (x, y, yaw))
            ok = False
    if not tgt:
        print('FAIL 没有目标位置帧')
        ok = False
    else:
        x, y, yaw = [v / scale for v in tgt[-1][1:]]
        if abs(x - GOAL[0]) > TOL or abs(y - GOAL[1]) > TOL or abs(yaw - GOAL[2]) > TOL:
            print('FAIL 目标位姿不对：%.4f %.4f %.4f（应为 %.4f %.4f %.4f）'
                  % (x, y, yaw, GOAL[0], GOAL[1], GOAL[2]))
            ok = False
    print('---')
    print('PTY ROUNDTRIP: %s' % ('PASS' if ok else 'FAIL'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
