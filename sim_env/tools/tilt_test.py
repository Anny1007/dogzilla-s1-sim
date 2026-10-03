"""原地轉與前進時量測機身 roll/pitch (Gazebo 真值), 以及 LiDAR 掃描平面最多會往下偏多少。"""
import rclpy, time, math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
rclpy.init(); n = Node("tilt"); rp = []
def cb(m):
    q = m.pose.pose.orientation
    roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y)); pitch = math.asin(max(-1, min(1, 2*(q.w*q.y-q.z*q.x))))
    rp.append((math.degrees(roll), math.degrees(pitch), m.pose.pose.position.z))
n.create_subscription(Odometry, "/demo/odom/ground_truth", cb, 10)
pub = n.create_publisher(Twist, "/cmd_vel", 10)
for vx, wz in [(0.0, 0.0), (0.0, 0.8), (0.15, 0.0)]:
    rp.clear(); t0 = time.time()
    while time.time() - t0 < 5:
        tw = Twist(); tw.linear.x = vx; tw.angular.z = wz; pub.publish(tw); rclpy.spin_once(n, timeout_sec=0.05)
    r = [abs(a) for a, _, _ in rp]; p = [abs(b) for _, b, _ in rp]; z = [c for _, _, c in rp]
    tilt = max(max(r), max(p))
    print(f"vx={vx} wz={wz}: max|roll|={max(r):.1f}° max|pitch|={max(p):.1f}° base z {min(z):.3f}~{max(z):.3f}  "
          f"掃描平面在 3.5 m 處最多下偏 {3.5*math.tan(math.radians(tilt)):.2f} m", flush=True)
pub.publish(Twist())
