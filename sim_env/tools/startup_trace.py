"""記錄開始物理模擬後前幾秒的機身高度與左前腿關節角 (實際/期望), 用來分析起身失敗。"""
import rclpy, time
from nav_msgs.msg import Odometry
from control_msgs.msg import JointTrajectoryControllerState
rclpy.init(); n = rclpy.create_node('trace'); d = {}; rows = []
n.create_subscription(Odometry, '/demo/odom/ground_truth', lambda m: d.__setitem__('z', m.pose.pose.position.z), 10)
n.create_subscription(JointTrajectoryControllerState, '/joint_group_effort_controller/controller_state',
                      lambda m: d.__setitem__('js', m), 10)
while 'z' not in d: rclpy.spin_once(n, timeout_sec=0.01)
t0 = time.time(); nxt = 0
while time.time() - t0 < 4:
    rclpy.spin_once(n, timeout_sec=0.01)
    if time.time() - t0 >= nxt and 'js' in d:
        m = d['js']; i = list(m.joint_names).index('lf_upper_leg_joint'); k = list(m.joint_names).index('lf_lower_leg_joint')
        print(f"t={time.time()-t0:4.2f} z={d['z']:.3f}  upper 實際 {m.feedback.positions[i]:+.2f} 期望 {m.reference.positions[i]:+.2f} 力 {m.output.effort[i]:+.2f}"
              f" | lower 實際 {m.feedback.positions[k]:+.2f} 期望 {m.reference.positions[k]:+.2f} 力 {m.output.effort[k]:+.2f}", flush=True)
        nxt += 0.25
