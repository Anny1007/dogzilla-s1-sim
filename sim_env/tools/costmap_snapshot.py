"""把 /map、/global_costmap/costmap 與機器狗 SLAM 位姿存成 PNG, 用來檢查地圖與代價地圖。 用法: python3 costmap_snapshot.py <輸出.png>"""
import sys, math, rclpy, numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
rclpy.init(); n = Node('snap'); d = {}
n.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(OccupancyGrid, '/map', lambda m: d.__setitem__('map', m), tl)
n.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: d.__setitem__('cost', m), 10)
b = Buffer(); TransformListener(b, n)
while len(d) < 2 or not b.can_transform('map', 'base_link', rclpy.time.Time()):
    rclpy.spin_once(n, timeout_sec=0.1)
t = b.lookup_transform('map', 'base_link', rclpy.time.Time()).transform
fig, ax = plt.subplots(1, 2, figsize=(12, 6))
for a, k in zip(ax, ['map', 'cost']):
    m = d[k]; g = np.array(m.data, dtype=float).reshape(m.info.height, m.info.width)
    o = m.info.origin.position; r = m.info.resolution
    a.imshow(g, origin='lower', cmap='gray_r' if k == 'map' else 'viridis',
             extent=[o.x, o.x + m.info.width * r, o.y, o.y + m.info.height * r])
    a.plot(t.translation.x, t.translation.y, 'r*', ms=15); a.set_title(k)
plt.savefig(sys.argv[1] if len(sys.argv) > 1 else '/logs/costmap.png', dpi=80)
print('robot(map)', round(t.translation.x, 2), round(t.translation.y, 2))
