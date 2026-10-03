"""步態綜合評估: 靜止 / 前進 / 原地轉 / 前進+轉 各一段, 用 Gazebo 真值量
實際速度、機身晃動 (roll/pitch 的 RMS 與最大值)、高度起伏、靜止時的漂移。
用法: python3 gait_eval.py [每段秒數]      最後一行是給參數掃描用的摘要。"""
import rclpy, time, math, sys
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
T = float(sys.argv[1]) if len(sys.argv) > 1 else 6
rclpy.init(); n = rclpy.create_node('gait_eval'); buf = []
def cb(m):
    p = m.pose.pose; q = p.orientation
    buf.append((time.time(), p.position.x, p.position.y, p.position.z,
                math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y)),
                math.asin(max(-1, min(1, 2*(q.w*q.y-q.z*q.x)))),
                math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))))
n.create_subscription(Odometry, '/demo/odom/ground_truth', cb, 50)
pub = n.create_publisher(Twist, '/cmd_vel', 10)
def run(vx, wz):
    vx, wz = float(vx), float(wz)
    t0 = time.time()
    while time.time() - t0 < T:
        tw = Twist(); tw.linear.x = vx; tw.angular.z = wz; pub.publish(tw); rclpy.spin_once(n, timeout_sec=0.02)
    a = np.array([b for b in buf if b[0] > t0 + 1.5])          # 去掉前 1.5 s 的加速段
    dt = a[-1, 0] - a[0, 0]
    v = math.hypot(a[-1, 1] - a[0, 1], a[-1, 2] - a[0, 2]) / dt
    w = np.sum(np.angle(np.exp(1j * np.diff(a[:, 6])))) / dt
    r, p = np.degrees(a[:, 4]), np.degrees(a[:, 5])
    return dict(v=v, w=w, rms=math.sqrt(np.mean(r**2 + p**2)), mx=max(np.abs(r).max(), np.abs(p).max()),
                dz=(a[:, 3].max() - a[:, 3].min()) * 1000)
while not buf: rclpy.spin_once(n, timeout_sec=0.1)
res = {}
for name, vx, wz in [('靜止', 0, 0), ('前進0.2', 0.2, 0), ('原地轉0.8', 0, 0.8), ('前進+轉', 0.15, 0.5)]:
    res[name] = r = run(vx, wz)
    print(f"{name:8s} 速度 v={r['v']:.3f} m/s w={r['w']:+.2f} rad/s | 晃動 RMS {r['rms']:4.1f}° 最大 {r['mx']:4.1f}° | 高度起伏 {r['dz']:4.1f} mm", flush=True)
pub.publish(Twist())
print("SUMMARY drift=%.3f v=%.3f w=%.2f rms_walk=%.1f rms_turn=%.1f max=%.1f" % (
    res['靜止']['v'], res['前進0.2']['v'], res['原地轉0.8']['w'],
    res['前進0.2']['rms'], res['原地轉0.8']['rms'], max(r['mx'] for r in res.values())))
