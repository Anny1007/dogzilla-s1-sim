"""檢查 /scan_filtered 裡漏網的地面點: 用 Gazebo 真值位姿把每個保留的點轉到世界座標,
離地 < 5 cm 的就是地面點 (場地障礙物都 1 m 高, 真正打到障礙物的點應在掃描平面高度附近)。
同時開車 (前進+轉彎) 製造晃動。用法: python3 leak_check.py [秒數]"""
import sys, time, math, rclpy, numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
LASER_OFF = np.array([-0.030, 0.0, 0.0275 + 0.035])      # base_link -> 射線原點 (dogzilla_s1.urdf.xacro)
def R(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)], [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)], [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])
rclpy.init(); n = rclpy.create_node('leak'); d = {}; stats = dict(scans=0, kept=0, leak=0); leaks = []
n.create_subscription(Odometry, '/demo/odom/ground_truth', lambda m: d.__setitem__('gt', m.pose.pose), 10)
def on_scan(m):
    if 'gt' not in d: return
    p = d['gt']; Rm = R(p.orientation); o = np.array([p.position.x, p.position.y, p.position.z]) + Rm @ LASER_OFF
    r = np.array(m.ranges); a = m.angle_min + m.angle_increment * np.arange(len(r)); ok = np.isfinite(r) & (r < m.range_max)
    pts = o + (Rm @ np.vstack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]), np.zeros(ok.sum())])).T
    lk = pts[:, 2] < 0.05
    stats['scans'] += 1; stats['kept'] += int(ok.sum()); stats['leak'] += int(lk.sum())
    for rr in r[ok][lk]: leaks.append(rr)
n.create_subscription(LaserScan, '/scan_filtered', on_scan, qos_profile_sensor_data)
pub = n.create_publisher(Twist, '/cmd_vel', 10); T = float(sys.argv[1]) if len(sys.argv) > 1 else 20; t0 = time.time()
while time.time() - t0 < T:
    k = int((time.time() - t0) / 4) % 3
    tw = Twist(); tw.linear.x, tw.angular.z = [(0.15, 0.0), (0.0, 0.8), (0.12, -0.5)][k]; pub.publish(tw); rclpy.spin_once(n, timeout_sec=0.02)
pub.publish(Twist())
print(f"{stats['scans']} 次掃描, 保留 {stats['kept']} 點, 其中離地 < 5 cm (漏網地面點) {stats['leak']} 點 "
      f"({100*stats['leak']/max(stats['kept'],1):.2f}%)" + (f", 距離中位數 {np.median(leaks):.2f} m" if leaks else ""))
