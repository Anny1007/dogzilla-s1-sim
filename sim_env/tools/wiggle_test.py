"""模擬 RL 左右來回切換轉向 (經過與 rl_controller 相同的加速度限制), 看是否會翻倒。
用法: python3 wiggle_test.py <v> <切換週期 s> [秒數]"""
import rclpy, time, math, sys
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
rclpy.init(); n = rclpy.create_node('wiggle'); d = {}
n.create_subscription(Odometry, '/demo/odom/ground_truth', lambda m: d.__setitem__('q', m.pose.pose.orientation), 10)
pub = n.create_publisher(Twist, '/cmd_vel', 10)
v, per = float(sys.argv[1]), float(sys.argv[2]); T = float(sys.argv[3]) if len(sys.argv) > 3 else 15
w_out = 0.0; t0 = time.time(); mt = 0.0; flip = False
while time.time() - t0 < T:
    w_cmd = 1.0 if int((time.time() - t0) / per) % 2 == 0 else -1.0
    w_out += max(-0.2, min(0.2, w_cmd - w_out))            # 2.0 rad/s² * 0.1 s
    tw = Twist(); tw.linear.x = v; tw.angular.z = w_out; pub.publish(tw)
    t1 = time.time()
    while time.time() - t1 < 0.1: rclpy.spin_once(n, timeout_sec=0.02)
    if 'q' in d:
        q = d['q']; up = 1 - 2*(q.x*q.x+q.y*q.y); mt = max(mt, math.degrees(math.acos(max(-1, min(1, up)))))
        if up < 0.5: flip = True; break
pub.publish(Twist())
print(f"v={v} 切換週期 {per}s: 最大傾斜 {mt:.1f}°  {'翻倒!' if flip else 'ok'}", flush=True)
