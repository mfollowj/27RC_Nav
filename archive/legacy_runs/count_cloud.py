import sys, time, rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy
topic = sys.argv[1] if len(sys.argv) > 1 else '/Laser_map'
rclpy.init(); n = Node('count_cloud'); got = []
n.create_subscription(PointCloud2, topic, lambda m: got.append((m.width*m.height, m.header.frame_id)),
                      QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
t = time.time()
while time.time() - t < 12 and not got:
    rclpy.spin_once(n, timeout_sec=0.2)
if got:
    # 取最多的那一帧
    best = max(got)
    print(f'{topic}: {best[0]} 点, frame={best[1]}')
else:
    print(f'{topic}: 没收到')
