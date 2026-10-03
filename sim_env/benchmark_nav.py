#!/usr/bin/env python3
"""Nav2 控制器对比实验：在场地里依次走一串世界坐标目标点。

用法(容器内):  python3 /benchmark_nav.py <label> [nav2|rl] [project3|obstacle_test]   # 结果存到 /logs/bench_<label>.csv
到达/碰撞/最近距离全部用 Gazebo 真值 (/demo/odom/ground_truth) 判定，不受 SLAM 漂移影响。
世界坐标目标会按当前 map 与真值的偏差换算成 map 坐标后发给 Nav2。
"""
import csv, math, sys, time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

LEG_TIMEOUT = 150.0     # 每段最长时间(s)
SUCCESS_DIST = 0.35     # 真值距目标小于此值算到达 (Nav2 容差 0.25 + 坐标换算误差)
COLLIDE_DIST = 0.12     # 机器人中心距障碍物表面小于此值算碰撞/擦碰

# 各场地的世界坐标目标点 (x, y) 与障碍物 ('box', cx, cy, sx, sy) / ('cyl', cx, cy, r), 须与 worlds/*.world 一致
WORLDS = {
    # project.pptx 第 1 页场景 (默认): 第 1、7 段就是投影片的任务 —— 从起点绕过障碍物走到右上角目标
    'project3': dict(
        goals=[(5.9, 1.1), (0.0, 0.0), (5.6, -1.5), (0.3, 1.5), (3.1, 0.6), (1.5, -1.5), (5.9, 1.1)],
        obstacles=[
            ('box', 2.9, 2.0, 7.1, 0.1), ('box', 2.9, -2.0, 7.1, 0.1), ('box', -0.6, 0, 0.1, 4.1), ('box', 6.4, 0, 0.1, 4.1),
            ('box', 1.27, 0.0, 0.5, 1.5), ('box', 2.6, 1.4, 0.45, 0.45), ('box', 3.77, 1.1, 0.5, 1.7),
            ('cyl', 4.05, 0.05, 0.2), ('cyl', 2.65, -0.87, 0.28), ('cyl', 5.04, -0.72, 0.27),
        ]),
    # 旧的 6 m x 6 m 测试场地, 路线穿过场地中部
    'obstacle_test': dict(
        goals=[(2.2, 0.0), (-2.0, 0.3), (2.2, -1.8), (-2.0, -2.0), (2.2, 1.8), (-1.8, 1.8), (0.0, 0.0)],
        obstacles=[
            ('box', 0, 3, 6.1, 0.1), ('box', 0, -3, 6.1, 0.1), ('box', 3, 0, 0.1, 6.1), ('box', -3, 0, 0.1, 6.1),
            ('box', 1.0, 0.3, 0.4, 0.4), ('box', 1.6, -0.9, 0.5, 0.3), ('cyl', 1.4, 0.9, 0.2),
            ('cyl', 0.5, -1.0, 0.15), ('box', -1.0, 1.2, 0.6, 0.3), ('cyl', -1.2, -1.0, 0.25),
        ]),
}
WORLD = sys.argv[3] if len(sys.argv) > 3 else 'project3'
GOALS, OBSTACLES = WORLDS[WORLD]['goals'], WORLDS[WORLD]['obstacles']


def clearance(x, y):
    """机器人中心到最近障碍物表面的距离(m)，在内部为负。"""
    best = 1e9
    for o in OBSTACLES:
        if o[0] == 'box':
            _, cx, cy, sx, sy = o
            dx, dy = abs(x - cx) - sx / 2, abs(y - cy) - sy / 2
            d = math.hypot(max(dx, 0), max(dy, 0)) if (dx > 0 or dy > 0) else max(dx, dy)
        else:
            _, cx, cy, r = o
            d = math.hypot(x - cx, y - cy) - r
        best = min(best, d)
    return best


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Bench(Node):
    def __init__(self):
        super().__init__('nav_benchmark', parameter_overrides=[])
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.gt = None
        self.flipped = False
        self.trace = []
        self.create_subscription(Odometry, '/demo/odom/ground_truth', self.on_gt, 10)
        self.tfbuf = Buffer()
        self.tfl = TransformListener(self.tfbuf, self)
        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.rl_goal = self.create_publisher(PoseStamped, '/rl_goal', 10)
        self.rl_status = 'IDLE'
        self.create_subscription(String, '/rl_status', lambda m: setattr(self, 'rl_status', m.data), 10)

    def on_gt(self, m):
        p = m.pose.pose; q = p.orientation
        self.gt = (p.position.x, p.position.y, yaw_of(q))
        # 翻倒偵測: 機身 z 軸與世界 z 軸夾角 > 60°
        if 1 - 2 * (q.x * q.x + q.y * q.y) < 0.5:
            self.flipped = True
        self.trace.append((self.gt[0], self.gt[1]))

    def spin_for(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def map_pose(self):
        t = self.tfbuf.lookup_transform('map', 'base_link', rclpy.time.Time())
        p = t.transform.translation
        return p.x, p.y, yaw_of(t.transform.rotation)

    def world_to_map(self, gx, gy):
        """用 (map位姿, 世界位姿) 这对已知点求刚体变换，把世界坐标点变成 map 坐标。"""
        mx, my, myaw = self.map_pose()
        wx, wy, wyaw = self.gt
        dyaw = myaw - wyaw
        c, s = math.cos(dyaw), math.sin(dyaw)
        rx, ry = gx - wx, gy - wy
        return mx + c * rx - s * ry, my + s * rx + c * ry

    def run_leg_rl(self, gx, gy):
        """RL 模式: 世界坐标目标直接发给 rl_controller, 等它报告 REACHED 或超时。"""
        def send(x, y):
            g = PoseStamped(); g.header.frame_id = 'world'
            g.pose.position.x, g.pose.position.y = x, y
            self.rl_goal.publish(g)
        self.trace = []
        self.rl_status = 'IDLE'
        t0 = time.time()
        send(gx, gy)
        self.spin_for(0.5)
        while self.rl_status != 'REACHED' and time.time() - t0 < LEG_TIMEOUT:
            rclpy.spin_once(self, timeout_sec=0.05)
        status = 'SUCCEEDED' if self.rl_status == 'REACHED' else 'TIMEOUT'
        dt = time.time() - t0
        send(float('nan'), float('nan'))      # 取消目标, 停止
        self.spin_for(1.0)
        return status, dt

    def run_leg(self, gx, gy):
        mx, my = self.world_to_map(gx, gy)
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = mx, my
        goal.pose.pose.orientation.w = 1.0
        self.trace = []
        t0 = time.time()
        fut = self.ac.send_goal_async(goal)
        while not fut.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        handle = fut.result()
        if not handle.accepted:
            return 'REJECTED', 0.0
        res = handle.get_result_async()
        while not res.done() and time.time() - t0 < LEG_TIMEOUT:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not res.done():
            handle.cancel_goal_async()
            self.spin_for(2.0)
            return 'TIMEOUT', time.time() - t0
        return {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(res.result().status, str(res.result().status)), time.time() - t0


def main():
    label = sys.argv[1] if len(sys.argv) > 1 else 'run'
    rclpy.init()
    b = Bench()
    mode = sys.argv[2] if len(sys.argv) > 2 else 'nav2'
    if mode == 'nav2':
        b.ac.wait_for_server()
    while b.gt is None:
        rclpy.spin_once(b, timeout_sec=0.1)
    while not b.tfbuf.can_transform('map', 'base_link', rclpy.time.Time()):   # 等 SLAM 發出第一張地图
        rclpy.spin_once(b, timeout_sec=0.1)
    b.spin_for(1.0)
    rows = []
    for i, (gx, gy) in enumerate(GOALS, 1):
        status, dt = b.run_leg_rl(gx, gy) if mode == 'rl' else b.run_leg(gx, gy)
        fx, fy, _ = b.gt
        err = math.hypot(fx - gx, fy - gy)
        path = sum(math.hypot(b.trace[k][0] - b.trace[k - 1][0], b.trace[k][1] - b.trace[k - 1][1]) for k in range(1, len(b.trace)))
        cl = [clearance(x, y) for x, y in b.trace] or [float('nan')]
        episodes, inside = 0, False
        for c in cl:
            if c < COLLIDE_DIST and not inside:
                episodes += 1
            inside = c < COLLIDE_DIST
        ok = status == 'SUCCEEDED' and err < SUCCESS_DIST
        # 机器狗在世界原点、朝向 0 生成, SLAM 的 map 系起始与世界系重合, 所以 map 位姿与真值之差就是定位漂移
        mx, my, _ = b.map_pose()
        slam_err = math.hypot(mx - fx, my - fy)
        rows.append(dict(label=label, leg=i, goal=f'({gx},{gy})', status=status, success=int(ok), time_s=round(dt, 1),
                         path_m=round(path, 2), final_err_m=round(err, 2), min_clearance_m=round(min(cl), 2), collisions=episodes,
                         slam_err_m=round(slam_err, 2), flipped=int(b.flipped)))
        print(rows[-1], flush=True)
    with open(f'/logs/bench_{label}.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    n = len(rows); ok = [r for r in rows if r['success']]
    done = sum(r['status'] == 'SUCCEEDED' for r in rows)
    print(f"\n== SUMMARY {label}: 控制器回报完成 {done}/{n}, 真值到达 {len(ok)}/{n}, 碰撞次数 {sum(r['collisions'] for r in rows)}, "
          f"成功段平均用时 {sum(r['time_s'] for r in ok) / max(len(ok), 1):.1f}s, "
          f"最小离障距离 {min(r['min_clearance_m'] for r in rows):.2f}m, 翻倒 {'是' if b.flipped else '否'}")
    print('== END')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
