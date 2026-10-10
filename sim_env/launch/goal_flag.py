#!/usr/bin/env python3
"""目標旗子: 在 RViz 用「2D Goal Pose」點到哪裡, Gazebo 裡就在那裡插一支藍色旗子, 代表機器狗要走到的終點。

  /goal_pose (RViz 點的目標, map 座標)  -> 換成世界座標 -> 旗子
  /rl_goal   (benchmark 或手動發的目標, 世界座標) -> 旗子

旗子只有外觀、沒有碰撞, LiDAR 看不到, 不影響導航。每次新目標會放一支新旗子並刪掉上一支 (Gazebo 的 /spawn_entity、/delete_entity);
每支旗子的名稱不同 (goal_flag_1, _2, ...): Gazebo 刪除模型不是立刻生效, 用同一個名稱馬上再放會失敗。
map 與世界座標的換算與 rl/rl_controller.py 相同: 用機器狗的 SLAM 位姿 (tf map -> base_link) 與 Gazebo 真值位姿對齊,
所以旗子插在「機器狗以為的目標」實際對應的位置; SLAM 有誤差時, 旗子會和 RViz 地圖上點的位置差一點。
"""
import math

import rclpy
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

NAME = 'goal_flag'
BLUE = '<material><ambient>0.05 0.25 0.85 1</ambient><diffuse>0.05 0.25 0.85 1</diffuse><emissive>0.02 0.08 0.3 1</emissive></material>'
SDF = f"""<sdf version='1.6'><model name='goal_flag'><static>1</static><link name='l'>
<visual name='base'><pose>0 0 0.005 0 0 0</pose><geometry><cylinder><radius>0.12</radius><length>0.01</length></cylinder></geometry>{BLUE}</visual>
<visual name='pole'><pose>0 0 0.4 0 0 0</pose><geometry><cylinder><radius>0.012</radius><length>0.8</length></cylinder></geometry>
<material><ambient>0.85 0.85 0.85 1</ambient><diffuse>0.85 0.85 0.85 1</diffuse></material></visual>
<visual name='flag'><pose>0.13 0 0.7 0 0 0</pose><geometry><box><size>0.26 0.008 0.18</size></box></geometry>{BLUE}</visual>
</link></model></sdf>"""


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class GoalFlag(Node):
    def __init__(self):
        super().__init__('goal_flag')
        self.truth = None
        self.count, self.last = 0, None
        self.tfbuf = Buffer(); self.tfl = TransformListener(self.tfbuf, self)
        self.spawn = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete = self.create_client(DeleteEntity, '/delete_entity')
        self.create_subscription(Odometry, '/demo/odom/ground_truth', self.on_truth, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_rviz_goal, 10)
        self.create_subscription(PoseStamped, '/rl_goal', self.on_world_goal, 10)

    def on_truth(self, m):
        p = m.pose.pose
        self.truth = (p.position.x, p.position.y, yaw_of(p.orientation))

    def map_to_world(self, x, y):
        t = self.tfbuf.lookup_transform('map', 'base_link', rclpy.time.Time()).transform
        mx, my, myaw = t.translation.x, t.translation.y, yaw_of(t.rotation)
        wx, wy, wyaw = self.truth
        # 目標在機身座標的位置 (用 SLAM 位姿算), 再用真值位姿放回世界座標
        dx, dy = x - mx, y - my
        bx, by = dx * math.cos(-myaw) - dy * math.sin(-myaw), dx * math.sin(-myaw) + dy * math.cos(-myaw)
        return wx + bx * math.cos(wyaw) - by * math.sin(wyaw), wy + bx * math.sin(wyaw) + by * math.cos(wyaw)

    def on_rviz_goal(self, m):
        x, y = m.pose.position.x, m.pose.position.y
        if m.header.frame_id in ('map', '') and self.truth is not None:
            try:
                x, y = self.map_to_world(x, y)
            except Exception as e:       # 還沒有 map -> base_link (SLAM 剛啟動): 直接當世界座標用
                self.get_logger().warn(f'map 座標換算失敗, 旗子直接放在點選的座標: {e}')
        self.place(x, y)

    def on_world_goal(self, m):
        if not (math.isnan(m.pose.position.x) or math.isnan(m.pose.position.y)):
            self.place(m.pose.position.x, m.pose.position.y)

    def place(self, x, y):
        if not self.spawn.service_is_ready():
            self.get_logger().warn('Gazebo 的 /spawn_entity 還沒準備好, 這次不放旗子')
            return
        self.count += 1
        req = SpawnEntity.Request(); req.name = f'{NAME}_{self.count}'; req.xml = SDF
        req.initial_pose.position.x, req.initial_pose.position.y = float(x), float(y)
        self.spawn.call_async(req)
        if self.last is not None and self.delete.service_is_ready():
            self.delete.call_async(DeleteEntity.Request(name=self.last))
        self.last = req.name
        self.get_logger().info(f'目標旗子放在世界座標 ({x:.2f}, {y:.2f})')


def main():
    rclpy.init()
    rclpy.spin(GoalFlag())


if __name__ == '__main__':
    main()
