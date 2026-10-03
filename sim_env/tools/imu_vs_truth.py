"""行走時比對 IMU 與 Gazebo 真值的 roll/pitch, 確認地面點過濾用的傾角是否可信。"""
import rclpy, time, math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
def rp(q):
    return (math.degrees(math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))),
            math.degrees(math.asin(max(-1, min(1, 2*(q.w*q.y-q.z*q.x))))))
rclpy.init(); n = rclpy.create_node('ivt'); d = {}; rows = []
n.create_subscription(Imu, '/imu/data', lambda m: d.__setitem__('imu', rp(m.orientation)), qos_profile_sensor_data)
def gt(m):
    if 'imu' in d: rows.append((rp(m.pose.pose.orientation), d['imu']))
n.create_subscription(Odometry, '/demo/odom/ground_truth', gt, 10)
pub = n.create_publisher(Twist, '/cmd_vel', 10); t0 = time.time()
while time.time() - t0 < 8:
    tw = Twist(); tw.linear.x = 0.15; tw.angular.z = 0.4; pub.publish(tw); rclpy.spin_once(n, timeout_sec=0.02)
pub.publish(Twist())
for (gr, gp), (ir, ip) in rows[::4]:
    print(f"真值 roll {gr:+5.1f} pitch {gp:+5.1f} | IMU roll {ir:+5.1f} pitch {ip:+5.1f}")
er = [abs(g[0]-i[0]) for g, i in rows]; ep = [abs(g[1]-i[1]) for g, i in rows]
print(f"平均誤差 roll {sum(er)/len(er):.2f}° pitch {sum(ep)/len(ep):.2f}°, 最大 roll {max(er):.1f}° pitch {max(ep):.1f}°")
