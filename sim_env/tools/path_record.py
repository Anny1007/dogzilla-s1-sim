"""把 Gazebo 真值位置錄成 CSV (t, x, y, 傾斜), 給 tools/plot_path.py 畫路徑圖用。
用法 (容器內, 與 benchmark 同時跑): python3 /tools/path_record.py /logs/path_xxx.csv [最長秒數]"""
import math, sys, time
import rclpy
from nav_msgs.msg import Odometry
out = open(sys.argv[1], 'w'); out.write('t,x,y,tilt_deg\n')
T = float(sys.argv[2]) if len(sys.argv) > 2 else 1300
rclpy.init(); n = rclpy.create_node('path_record'); t0 = time.time(); last = [0.0]
def cb(m):
    t = time.time() - t0
    if t - last[0] < 0.2: return
    last[0] = t; p = m.pose.pose; q = p.orientation
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (q.x * q.x + q.y * q.y)))))
    out.write(f'{t:.2f},{p.position.x:.4f},{p.position.y:.4f},{tilt:.1f}\n'); out.flush()
n.create_subscription(Odometry, '/demo/odom/ground_truth', cb, 10)
while time.time() - t0 < T: rclpy.spin_once(n, timeout_sec=0.1)
