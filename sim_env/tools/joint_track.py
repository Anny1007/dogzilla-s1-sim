"""送一段速度指令, 記錄 12 個關節的追蹤誤差 (期望角 - 實際角) 與實際施力, 找出是哪個關節跟不上。
用法: python3 joint_track.py <vx> <wz> [秒數]"""
import sys, time, rclpy, numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from control_msgs.msg import JointTrajectoryControllerState
rclpy.init(); n = Node('jt'); rec = []
vx, wz = float(sys.argv[1]), float(sys.argv[2]); T = float(sys.argv[3]) if len(sys.argv) > 3 else 5
n.create_subscription(JointTrajectoryControllerState, '/joint_group_effort_controller/controller_state',
                      lambda m: rec.append((m.joint_names, m.error.positions, m.output.effort)), 50)
pub = n.create_publisher(Twist, '/cmd_vel', 10)
t0 = time.time()
while time.time() - t0 < T:
    tw = Twist(); tw.linear.x = vx; tw.angular.z = wz; pub.publish(tw); rclpy.spin_once(n, timeout_sec=0.02)
pub.publish(Twist())
names = rec[-1][0]; err = np.abs(np.array([r[1] for r in rec[len(rec)//5:]])); eff = np.abs(np.array([r[2] for r in rec[len(rec)//5:]]))
for i, j in enumerate(names):
    print(f"{j:20s} 誤差 平均 {np.degrees(err[:, i].mean()):5.1f}° 最大 {np.degrees(err[:, i].max()):5.1f}°   "
          f"施力飽和(>=0.43) {100*(eff[:, i] >= 0.43).mean():4.0f}% 的時間" if eff.size else j)
