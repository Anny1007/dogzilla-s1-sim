"""靜止站立時的漂移: 不送任何速度指令, 每 5 秒印一次真值位置與傾斜。用法: python3 stand_drift.py [秒數]"""
import math, sys, time
import rclpy
from nav_msgs.msg import Odometry
T = float(sys.argv[1]) if len(sys.argv) > 1 else 30
rclpy.init(); n = rclpy.create_node('stand_drift'); d = {}
n.create_subscription(Odometry, '/demo/odom/ground_truth', lambda m: d.__setitem__('m', m), 10)
while 'm' not in d: rclpy.spin_once(n, timeout_sec=0.1)
t0 = time.time(); p0 = d['m'].pose.pose.position; x0, y0 = p0.x, p0.y; nxt = 0
while time.time() - t0 < T + 0.1:
    rclpy.spin_once(n, timeout_sec=0.1)
    if time.time() - t0 >= nxt:
        p = d['m'].pose.pose; q = p.orientation
        pitch = math.degrees(math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x)))))
        print(f't={nxt:4.0f}s  x={p.position.x:+.3f} y={p.position.y:+.3f} z={p.position.z:.3f}  pitch={pitch:+.1f}°', flush=True); nxt += 5
p = d['m'].pose.pose.position
print(f'DRIFT {math.hypot(p.x - x0, p.y - y0) / T * 1000:.1f} mm/s')
