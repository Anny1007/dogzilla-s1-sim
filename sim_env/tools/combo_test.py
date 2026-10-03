"""測試前進+轉向組合是否會讓機器狗翻倒: 每組 8 秒, 記錄最大 roll/pitch 與是否翻倒。"""
import rclpy, time, math, sys
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
rclpy.init(); n = rclpy.create_node('combo'); d = {}
n.create_subscription(Odometry, '/demo/odom/ground_truth', lambda m: d.__setitem__('q', m.pose.pose.orientation), 10)
pub = n.create_publisher(Twist, '/cmd_vel', 10)
combos = [(float(a), float(b)) for a, b in (c.split(",") for c in sys.argv[1:])] if len(sys.argv) > 1 else [(0.25, 0.0), (0.3, 0.0), (0.15, 0.6), (0.25, 0.6), (0.3, 1.0), (0.3, -1.0)]
for vx, wz in combos:
    t0 = time.time(); mr = mp = 0.0
    while time.time() - t0 < 8:
        tw = Twist(); tw.linear.x = vx; tw.angular.z = wz; pub.publish(tw); rclpy.spin_once(n, timeout_sec=0.02)
        if 'q' in d:
            q = d['q']
            mr = max(mr, abs(math.degrees(math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y)))))
            mp = max(mp, abs(math.degrees(math.asin(max(-1, min(1, 2*(q.w*q.y-q.z*q.x)))))))
    q = d['q']; up = 1 - 2*(q.x*q.x+q.y*q.y)
    print(f"vx={vx} wz={wz:+}: max roll {mr:5.1f}° pitch {mp:5.1f}°  {'翻倒!' if up < 0.5 else 'ok'}", flush=True)
    if up < 0.5: break
    pub.publish(Twist()); time.sleep(2)
