#!/usr/bin/env python3
"""强化学习局部控制器 ROS 2 节点，按架构图的逻辑运作:

    LiDAR (/scan) ----------------\
                                    >-- RL policy --> /cmd_vel --> Motion Control
    Nav2 Path Planning (/compute_path_to_pose, 全局路径) -- 取前方路径点 --/

目标: 向 /rl_goal (PoseStamped, 世界坐标) 发布即可; 也可以在 RViz 用「2D Goal Pose」点选 (/goal_pose, map 坐标)。
状态发布在 /rl_status ('IDLE'/'ACTIVE'/'REACHED')。
收到目标后先向 Nav2 planner_server 要一条全局路径(在 map 系下)，之后每步瞄准路径上前方
LOOKAHEAD 米处的路径点(而不是直接瞄准终点) —— 这样遇到挡在直线上的障碍物时，RL 不需要
自己想办法绕开局部最小值，Nav2 的全局规划已经绕好路了，RL 只负责沿路径做光达避障和走位。

仿真里位姿取 Gazebo 真值 (/demo/odom/ground_truth), 速度取 /odom 的 twist。实机需换成里程计/定位。
"""
import collections, math, sys
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, '/rl')
from rl_policy import Policy, SPEED_MODELS, V_MAX, W_MAX, V_TAU, W_TAU, local_target
from safety_shield import SafetyShield

N_BINS, R_MIN, R_MAX, GOAL_RADIUS = 36, 0.12, 3.5, 0.25
V_LIMIT = 0.20           # S1 安全速度上限 (config/gait_s1.yaml)
ACC_V, ACC_W = 0.5, 2.0  # 输出加速度限制 (m/s², rad/s²), 与 Nav2 velocity_smoother 相同。
                         # 策略每 0.1 s 直接跳变的指令会把又轻又窄的 S1 甩翻 (实测第 1 段就翻倒)
REPLAN_PERIOD = 2.0      # 每隔几秒向 Nav2 重新要一次全局路径 (SLAM 地图会边走边长, 第一次的路径可能穿过还没看到的障碍物)
# 瞄准点算法 (LOOKAHEAD 0.8 m, WAYPOINT_ADVANCE 0.4 m) 在 rl_policy.local_target, 与训练环境共用


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class RLController(Node):
    def __init__(self):
        super().__init__('rl_controller')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        # 权重档可用 ROS 参数指定 (launch: rl_policy:=/rl/models/s1_v2/policy.npz), 默认是部署用的 models/policy.npz
        path = self.declare_parameter('policy', '/rl/models/policy.npz').value
        self.policy = Policy(path)
        # 安全保护 (safety_shield.py): auto = 依权重档 (训练时有开就开), on / off = 强制
        mode = self.declare_parameter('shield', 'auto').value
        use = self.policy.shield if mode == 'auto' else mode == 'on'
        self.v_limit = min(V_LIMIT, self.policy.v_max)
        self.speed = SPEED_MODELS[self.policy.speed_model]
        self.shield = SafetyShield(self.policy.v_min, self.v_limit, W_MAX, ACC_V, ACC_W, 0.1, tau_v=V_TAU, tau_w=W_TAU) if use else None
        self.interventions = self.ticks = 0
        self.get_logger().info(f'载入策略 {path} (速度范围 {self.policy.v_min:.2f} ~ {self.policy.v_max:.2f} m/s, '
                               f'速度模型 {self.policy.speed_model}, '
                               f'安全保护 {"开" if use else "关"})')
        self.scan = self.pose = None
        self.v = self.w = 0.0
        self.v_out = self.w_out = 0.0   # 限制加速度后实际送出的速度
        # 观测里的「目前速度」: model = 用 rl_policy.speed_model 由送出的指令推算 (与训练环境同分布, 默认);
        # odom = 直接读腿部里程计 (机身随步伐摆头, 转向在 ±1.4 rad/s 间来回, 会让 RL 跟着抖)
        self.vel_obs = self.declare_parameter('vel_obs', 'model').value
        self.v_m = self.w_m = 0.0
        # 光达每个方向取最近 scan_hold 帧的最小值: 地面点过滤会让读数一帧有一帧没有 (实测每帧约 6/36 组跳动 > 0.5 m)
        # 机身朝向平滑 (秒, 0 = 关闭): 四足走路时机身随步伐左右摆头约 ±6°, 目标方位与光达读数跟着摆, RL 会跟着修正而抖。
        # 观测改用平滑后的朝向: 目标方位以它计算, 光达读数也转到它的方向 (安全保护仍用真实几何)
        self.yaw_tau = float(self.declare_parameter('yaw_filter', 0.2).value)
        self.yaw_f = None
        self.scans = collections.deque(maxlen=max(1, int(self.declare_parameter('scan_hold', 3).value)))
        self.goal = None            # 世界坐标终点
        self.path_world = []        # Nav2 规划的全局路径, 已转换到世界坐标 [(x,y), ...]
        self.path_idx = 0
        self.prev_act = np.zeros(2)
        self.tfbuf = Buffer(); TransformListener(self.tfbuf, self)
        self.ac = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/demo/odom/ground_truth', self.on_gt, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(PoseStamped, '/rl_goal', self.on_goal, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_rviz_goal, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status = self.create_publisher(String, '/rl_status', 10)
        self.path_pub = self.create_publisher(Path, '/rl_global_path', 10)   # 方便在 rviz 里查看
        self.intent_pub = self.create_publisher(Twist, '/rl_intent', 10)      # 诊断用: 安全保护之前 RL 想要的速度
        self.create_timer(0.1, self.tick)
        self.path_pending = False   # 已送出、还没回来的路径请求
        self.create_timer(REPLAN_PERIOD, self.replan)

    def on_gt(self, m):
        p = m.pose.pose; q = p.orientation
        self.pose = (p.position.x, p.position.y, yaw_of(q))
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.yaw_f is None or self.yaw_tau <= 0:
            self.yaw_f, self.yaw_t = self.pose[2], t
        else:
            k = min(1.0, max(0.0, t - self.yaw_t) / self.yaw_tau); self.yaw_t = t
            d = math.atan2(math.sin(self.pose[2] - self.yaw_f), math.cos(self.pose[2] - self.yaw_f))
            self.yaw_f = math.atan2(math.sin(self.yaw_f + k * d), math.cos(self.yaw_f + k * d))

    def on_scan(self, m):
        self.scan = m
        self.scans.append(np.array(m.ranges, dtype=np.float64))

    def on_odom(self, m):
        self.v, self.w = m.twist.twist.linear.x, m.twist.twist.angular.z

    # ---- 世界坐标 <-> map 坐标 (与 benchmark_nav.py 的 world_to_map 同一套做法) ----
    def map_pose(self):
        t = self.tfbuf.lookup_transform('map', 'base_link', rclpy.time.Time(), rclpy.duration.Duration(seconds=0.3))
        p = t.transform.translation
        return p.x, p.y, yaw_of(t.transform.rotation)

    def world_to_map(self, x, y):
        mx, my, myaw = self.map_pose(); wx, wy, wyaw = self.pose
        dyaw = myaw - wyaw; c, s = math.cos(dyaw), math.sin(dyaw)
        rx, ry = x - wx, y - wy
        return mx + c * rx - s * ry, my + s * rx + c * ry

    def map_to_world(self, x, y):
        mx, my, myaw = self.map_pose(); wx, wy, wyaw = self.pose
        dyaw = wyaw - myaw; c, s = math.cos(dyaw), math.sin(dyaw)
        rx, ry = x - mx, y - my
        return wx + c * rx - s * ry, wy + s * rx + c * ry

    def on_rviz_goal(self, m):
        """RViz 2D Goal Pose 给的是 map 坐标, 换成世界坐标后当作一般目标处理。"""
        if self.pose is None:
            self.get_logger().warn('还没收到位姿, 忽略 RViz 目标')
            return
        try:
            x, y = self.map_to_world(m.pose.position.x, m.pose.position.y)
        except Exception as e:
            self.get_logger().warn(f'map_to_world 失败, 忽略 RViz 目标: {e}')
            return
        self.get_logger().info(f'RViz 目标 map({m.pose.position.x:.2f}, {m.pose.position.y:.2f}) -> 世界({x:.2f}, {y:.2f})')
        self.set_goal(x, y)

    def on_goal(self, m):
        self.set_goal(m.pose.position.x, m.pose.position.y)

    def set_goal(self, x, y):
        if math.isnan(x) or math.isnan(y):
            self.goal, self.path_world = None, []
            self.v_out = self.w_out = self.v_m = self.w_m = 0.0
            self.pub.publish(Twist())       # 取消目标时立即停下, 否则步态控制器会沿用最后一个速度
            return
        self.goal = (x, y)
        self.path_world, self.path_idx = [], 0
        self.prev_act = np.zeros(2)
        self.path_pending = False
        self.request_path(x, y)

    def replan(self):
        if self.goal is None:
            return
        # 请求超过 5 秒没回应就放弃, 重送 (例如模拟刚启动时规划器还没上线, 那次请求永远不会回来)
        if self.path_pending and self.get_clock().now().nanoseconds * 1e-9 - self.path_sent > 5.0:
            self.get_logger().warn('路径请求 5 秒没有回应, 重送')
            self.path_pending = False
        if not self.path_pending:
            self.request_path(*self.goal)

    def request_path(self, gx, gy):
        if self.pose is None:
            return
        try:
            mgx, mgy = self.world_to_map(gx, gy)
        except Exception as e:
            self.get_logger().warn(f'world_to_map 失败, 直接用终点当路径: {e}')
            self.path_world = [(gx, gy)]
            return
        goal = ComputePathToPose.Goal()
        goal.goal = PoseStamped()
        goal.goal.header.frame_id = 'map'
        goal.goal.pose.position.x, goal.goal.pose.position.y = mgx, mgy
        goal.goal.pose.orientation.w = 1.0
        goal.use_start = False
        if not self.ac.server_is_ready():     # 规划器还没上线就先不送, 下一轮 replan 再试
            if not self.path_world:
                self.path_world = [self.goal]
            return
        self.path_pending = True
        self.path_sent = self.get_clock().now().nanoseconds * 1e-9
        fut = self.ac.send_goal_async(goal)
        fut.add_done_callback(lambda f, g=self.goal: self._on_path_goal_response(f, g))

    def _path_failed(self, msg):
        self.path_pending = False
        if not self.path_world:     # 还没有任何路径才退回直奔终点; 重新规划失败就沿用上一条
            self.get_logger().warn(msg + ', 退回直奔终点')
            self.path_world = [self.goal] if self.goal else []

    def _on_path_goal_response(self, fut, goal):
        handle = fut.result()
        if not handle.accepted:
            self._path_failed('Nav2 规划器拒绝了路径请求')
            return
        handle.get_result_async().add_done_callback(lambda f: self._on_path_result(f, goal))

    def _on_path_result(self, fut, goal):
        if goal != self.goal:       # 请求送出后目标换了, 丢掉旧结果
            self.path_pending = False
            return
        res = fut.result()
        if res.status != GoalStatus.STATUS_SUCCEEDED or len(res.result.path.poses) == 0:
            self._path_failed('Nav2 规划失败(可能被障碍物完全挡住)')
            return
        pts = []
        for ps in res.result.path.poses:
            pts.append(self.map_to_world(ps.pose.position.x, ps.pose.position.y))
        first = not self.path_world or len(self.path_world) == 1
        self.path_world, self.path_idx = pts, 0     # 新路径从目前位置出发, 瞄准点从头找
        self.path_pending = False
        if first:
            self.get_logger().info(f'拿到全局路径, {len(pts)} 个点 (之后每 {REPLAN_PERIOD:.0f} 秒重新规划)')
        path_msg = Path(); path_msg.header.frame_id = 'map'
        path_msg.poses = res.result.path.poses
        self.path_pub.publish(path_msg)

    def local_target(self):
        """沿全局路径向前找 LOOKAHEAD 处的点作为当前瞄准目标；没有路径时退回终点。"""
        if not self.path_world:
            return self.goal
        target, self.path_idx = local_target(self.path_world, self.path_idx, self.pose[0], self.pose[1])
        return target

    def tick(self):
        st = String()
        if self.goal is None or self.pose is None or self.scan is None:
            st.data = 'IDLE'; self.status.publish(st); return
        x, y, yaw = self.pose
        dist_final = math.hypot(self.goal[0] - x, self.goal[1] - y)
        if dist_final < GOAL_RADIUS:
            self.v_out = self.w_out = self.v_m = self.w_m = 0.0
            self.pub.publish(Twist()); st.data = 'REACHED'; self.status.publish(st); return
        target = self.local_target() or self.goal
        v_now, w_now = (self.v_m, self.w_m) if self.vel_obs == 'model' else (self.v, self.w)
        dist = math.hypot(target[0] - x, target[1] - y)
        held = np.min(np.stack(self.scans), axis=0) if len({len(a) for a in self.scans}) == 1 else self.scans[-1]
        # 转到平滑朝向: 机身比平滑朝向多转了 sway, 光达第 i 条射线在平滑坐标里是第 i + sway/角度间隔 条
        sway = math.atan2(math.sin(yaw - self.yaw_f), math.cos(yaw - self.yaw_f))
        r = np.roll(held, int(round(sway / self.scan.angle_increment)))
        r = np.where(np.isfinite(r), r, R_MAX)
        r = np.clip(r, R_MIN, R_MAX)
        if len(r) != 360:
            r = np.interp(np.linspace(0, len(r), 360, endpoint=False), np.arange(len(r)), r)
        bins = r.reshape(N_BINS, -1).min(axis=1) / R_MAX
        rel = math.atan2(target[1] - y, target[0] - x) - self.yaw_f
        rel = math.atan2(math.sin(rel), math.cos(rel))
        obs = np.concatenate([bins, [min(dist / 4.0, 1.0)], [math.sin(rel), math.cos(rel)], self.prev_act,
                              [v_now / V_MAX, w_now / W_MAX]]).astype(np.float32)
        v_cmd, w_cmd, act = self.policy.act(obs)
        self.prev_act = act
        it = Twist(); it.linear.x, it.angular.z = float(v_cmd), float(w_cmd); self.intent_pub.publish(it)
        dt = 0.1
        if self.shield is not None:
            # 光达原始距离 (已滤掉地面点与机身, 无回波为 inf), 以光达为原点; 光达与机身中心差几公分, 忽略
            raw = held.copy()
            ang = self.scan.angle_min + self.scan.angle_increment * np.arange(len(raw))
            raw = np.where(np.isfinite(raw) & (raw > self.scan.range_min), raw, np.inf)
            self.v_out, self.w_out, hit = self.shield.filter(min(v_cmd, self.v_limit), w_cmd, self.v_out, self.w_out,
                                                             raw, ang, v_now, w_now)
            self.ticks += 1; self.interventions += int(hit)
            if self.ticks % 100 == 0:
                self.get_logger().info(f'安全保护过去 10 秒介入 {self.interventions} 次')
                self.interventions = 0
        else:
            self.v_out += float(np.clip(min(v_cmd, self.v_limit) - self.v_out, -ACC_V * dt, ACC_V * dt))
            self.w_out += float(np.clip(w_cmd - self.w_out, -ACC_W * dt, ACC_W * dt))
        self.v_m, self.w_m = self.speed(self.v_m, self.w_m, self.v_out, self.w_out, dt)
        t = Twist(); t.linear.x, t.angular.z = self.v_out, self.w_out
        self.pub.publish(t)
        st.data = 'ACTIVE'; self.status.publish(st)


def main():
    rclpy.init(); rclpy.spin(RLController())


if __name__ == '__main__':
    main()
