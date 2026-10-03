"""監測翻倒: 每 0.2 s 記錄真值位置、傾斜與 /cmd_vel, 翻倒 (機身傾斜 > 60°) 時印出前 6 秒的紀錄後結束。
用法: python3 flip_monitor.py [最長秒數]"""
import rclpy, time, math, sys, collections
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
rclpy.init(); n = rclpy.create_node('flipmon'); d = {'cmd': (0.0, 0.0)}; hist = collections.deque(maxlen=30)
n.create_subscription(Odometry, '/demo/odom/ground_truth', lambda m: d.__setitem__('p', m.pose.pose), 10)
n.create_subscription(Twist, '/cmd_vel', lambda m: d.__setitem__('cmd', (m.linear.x, m.angular.z)), 10)
T = float(sys.argv[1]) if len(sys.argv) > 1 else 300; t0 = time.time(); nxt = 0
while time.time() - t0 < T:
    rclpy.spin_once(n, timeout_sec=0.02)
    if 'p' not in d or time.time() - t0 < nxt: continue
    nxt += 0.2; p = d['p']; q = p.orientation
    tilt = math.degrees(math.acos(max(-1, min(1, 1 - 2*(q.x*q.x+q.y*q.y)))))
    hist.append(f"t={time.time()-t0:6.1f} pos=({p.position.x:+.2f},{p.position.y:+.2f}) z={p.position.z:.3f} 傾斜 {tilt:5.1f}°  cmd v={d['cmd'][0]:+.2f} w={d['cmd'][1]:+.2f}")
    if tilt > 60:
        print("== 翻倒! 前 6 秒:"); print("\n".join(hist)); break
else:
    print("== 沒有翻倒")
