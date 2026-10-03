"""比對 SLAM (map->base_link)、里程計 (odom->base_link) 與 Gazebo 真值的位姿。"""
import rclpy, math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
rclpy.init(); n = Node("pose_check"); d = {}
n.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
n.create_subscription(Odometry, "/demo/odom/ground_truth", lambda m: d.__setitem__("gt", m.pose.pose), 10)
b = Buffer(); TransformListener(b, n)
yaw = lambda q: math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
while "gt" not in d or not b.can_transform("map", "base_link", rclpy.time.Time()): rclpy.spin_once(n, timeout_sec=0.1)
p = d["gt"]; print(f"truth      ({p.position.x:6.2f},{p.position.y:6.2f}) yaw {yaw(p.orientation):7.1f}")
for f in ["odom", "map"]:
    t = b.lookup_transform(f, "base_link", rclpy.time.Time()).transform
    print(f"{f:5s}->base ({t.translation.x:6.2f},{t.translation.y:6.2f}) yaw {yaw(t.rotation):7.1f}")
t = b.lookup_transform("map", "odom", rclpy.time.Time()).transform
print(f"map->odom  ({t.translation.x:6.2f},{t.translation.y:6.2f}) yaw {yaw(t.rotation):7.1f}")
