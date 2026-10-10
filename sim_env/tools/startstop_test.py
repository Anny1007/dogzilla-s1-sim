"""起停壓力測試: 前進速度指令在 0 與 V 之間來回 (方波, 經 0.5 m/s² 加速度限制, 與 RL 節點輸出相同), 同時小幅左右轉向,
看機身最大傾斜與是否翻倒。RL 在障礙物旁猶豫時就是這種指令。
用法 (空場地啟動後): python3 startstop_test.py <V> [週期秒=1.2] [總秒數=60]"""
import math, sys, time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
V = float(sys.argv[1]); PERIOD = float(sys.argv[2]) if len(sys.argv) > 2 else 1.2; T = float(sys.argv[3]) if len(sys.argv) > 3 else 60
rclpy.init(); n = rclpy.create_node('startstop'); d = {'tilt': 0.0}
def cb(m):
    q = m.pose.pose.orientation
    d['tilt'] = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (q.x * q.x + q.y * q.y)))))
n.create_subscription(Odometry, '/demo/odom/ground_truth', cb, 50)
pub = n.create_publisher(Twist, '/cmd_vel', 10)
t0 = time.time(); v = 0.0; nxt = 0.0; mx = 0.0; over15 = 0; flipped = None
while time.time() - t0 < T:
    rclpy.spin_once(n, timeout_sec=0.01)
    t = time.time() - t0
    mx = max(mx, d['tilt'])
    if d['tilt'] > 60: flipped = t; break
    if t >= nxt:
        nxt += 0.1
        over15 += d['tilt'] > 15
        target = V if int(t / (PERIOD / 2)) % 2 == 0 else 0.0
        v += max(-0.05, min(0.05, target - v))
        w = 0.3 * math.sin(2 * math.pi * t / 3.1)
        m = Twist(); m.linear.x = v * max(0.15, 1 - abs(w)); m.angular.z = w; pub.publish(m)
pub.publish(Twist())
print(f'V={V} 週期={PERIOD}s: 最大傾斜 {mx:.1f}°, 傾斜超過 15° 的時間 {over15 * 0.1:.1f} s, ' + (f'{flipped:.1f} s 時翻倒' if flipped else f'{T:.0f} s 內沒有翻倒'), flush=True)
