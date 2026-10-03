"""步態速度量測: 依序送 vx=0.2 / 原地轉 wz=0.8, 各 6 秒, 用 Gazebo 真值算實際速度。"""
import rclpy, time, math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
rclpy.init(); n = Node("walk"); d = {}
n.create_subscription(Odometry, "/demo/odom/ground_truth", lambda m: d.__setitem__("gt", m), 10)
n.create_subscription(Odometry, "/odom/raw", lambda m: d.__setitem__("raw", m), 10)
pub = n.create_publisher(Twist, "/cmd_vel", 10)
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
def run(vx, wz, T):
    t0 = time.time(); p0 = None; raw_v = []; dyaw = 0.0; last = None
    while time.time() - t0 < T:
        tw = Twist(); tw.linear.x = vx; tw.angular.z = wz; pub.publish(tw)
        rclpy.spin_once(n, timeout_sec=0.05)
        if "gt" in d and p0 is None and time.time() - t0 > 1.0:
            p = d["gt"].pose.pose; p0 = (p.position.x, p.position.y, yaw(p.orientation), time.time()); last = p0[2]
        if p0 is not None:   # 累加角度差, 轉超過半圈也量得對
            y = yaw(d["gt"].pose.pose.orientation); dyaw += math.atan2(math.sin(y - last), math.cos(y - last)); last = y
        if "raw" in d: raw_v.append((d["raw"].twist.twist.linear.x, d["raw"].twist.twist.angular.z))
    p = d["gt"].pose.pose; dt = time.time() - p0[3]
    print(f"cmd vx={vx} wz={wz}: 實際 v={math.hypot(p.position.x-p0[0], p.position.y-p0[1])/dt:.3f} m/s, "
          f"w={dyaw/dt:.3f} rad/s, z={p.position.z:.3f}, odom/raw 平均 vx={sum(a for a,_ in raw_v)/max(len(raw_v),1):.3f}", flush=True)
while "gt" not in d: rclpy.spin_once(n, timeout_sec=0.1)
run(0.2, 0.0, 6); run(0.0, 0.0, 2); run(0.0, 0.8, 6); run(0.0, 0.0, 1)
