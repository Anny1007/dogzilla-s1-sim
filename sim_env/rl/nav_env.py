"""2D 光达避障导航环境 (gymnasium)，用于训练 DOGZILLA 的强化学习局部控制器。

约定与 Gazebo 仿真 / rl_controller.py 保持一致，方便直接部署:
  * 光达 360 线, 0..2π 从机器人正前方逆时针, 量程 0.12..3.5 m, 无回波按最大量程处理
  * 观测里把 360 线按 10° 一组取最小值 -> 36 个方向
  * 瞄准点: 与部署相同, 先规划全局路径, 每步瞄准路径前方 LOOKAHEAD 处的点 (rl_policy.local_target)
  * 碰撞: 机器人中心离障碍物表面 < COLLIDE_DIST
  * 速度响应: 一阶滞后 + 增益损失 + 与 rl_controller 相同的加速度限制
  参数对应官方 DOGZILLA S1 URDF 模型 (description/dogzilla_s1_official.urdf.xacro), Gazebo 实测 (tools/gait_eval.py):
  指令 0.2 m/s -> 实际约 0.134 m/s, 原地转 0.8 rad/s -> 实际约 0.75 rad/s;
  边走边转时前进速度明显下降 (0.15 m/s + 0.5 rad/s -> 实际前进约 0.034 m/s), 用 TURN_SLOWDOWN 近似。

v2 (相对 v1, v1 原文件备份在 models/nav_env_v1_backup.py):
  1. 场景: 场地大小随机 (4~8 m x 3.5~6 m), 除了零散小障碍物, 还有长墙、从围墙伸出的隔墙、U 形陷阱、走廊;
     目标改为沿全局路径的瞄准点 (与部署相同), 进度奖励改用沿路径的距离 (绕路不再被扣分)
  2. 碰撞扣分 15 -> 50, 到达奖励 15 -> 30 (gamma 0.995 在 train.py)
  3. 允许小幅后退 (V_MIN -0.05 m/s); 25% 的回合从贴近障碍物处出发, 学习脱困
v3: NavEnv(shield=True) 在动作执行前加上安全保护 (safety_shield.py, 参考 2026_Steven_paper):
  RL 的意图投影到最近的安全候选速度; 介入时小幅扣分 (SHIELD_PENALTY, 本项目自加), 让 RL 学会直接提出安全动作
v4: 转向抖动扣分。Gazebo 里 v3 在前进中每 0.2~0.4 s 反转一次转向, 机身左右晃到翻倒 (tools/flip_monitor.py 纪录);
  2D 环境没有翻倒, 原本学不到要避开。前进中转向反转 (REVERSAL_PENALTY) 与转向变化量 x 速度 (STEER_RATE_PENALTY) 都扣分。
  两个权重是凭经验定的, 没有用翻倒的物理模型校准。
v5: 速度模型改用 Gazebo 实测 (rl_policy.speed_model_measured): 低速死区、不能后退、快速转弯明显减速;
  动作上限改为 0.15 m/s (0.2 m/s 加转向会翻倒), 最低 0 (不能后退)。v4 的转向扣分已停用。
  v1~v3 的权重用 NavEnv(v_min, v_max, speed_model='linear') 评估。
v6: 模型改成实机重量 (本体 875 g + LiDAR 45 g, LiDAR 后移到 x=-0.06, 关节 p=14), 速度模型重新实测
  (rl_policy.speed_model_875g, 含运动安全滤波的转弯降速、原地转时慢慢后退)。
  动作上限维持 0.15 m/s: 新重量下稳定走 0.2 m/s 加转向不再翻倒 (最大倾斜 7 度), 但 0.2 m/s 的急起急停会
  (tools/startstop_test.py, results/startstop_test_875g.txt: 0.20 时倾斜达 57~60 度, 0.15 时最大 10 度)。
  先试过上限 0.20 的版本, 2D 评估很好, 放进 Gazebo 在障碍物旁「停-冲」几次后翻倒 (results/training/flip_v6_vmax020.txt)。
  上限 0.15 从零训练 4M 步只到 70~87%, 比 v5 权重差; 改成以 v5 为起点接着训练 2M 步 (train.py --init=...), 采用这一版
  (models/s1_v6_finetune/, 用不含后退的 measured_875g_nocreep 模型训练)。v5 的权重用 NavEnv(speed_model='measured') 评估。
  手工方块模型 (s1_box) 版本的权重在 models/s1_box/。
  旧 CHAMP 放大模型的版本与权重在 models/champ_old/ (COLLIDE_DIST 0.12, V_MAX 0.3, V_GAIN 0.7, W_GAIN 0.9)
"""
import heapq
import math
import os

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from rl_policy import local_target, SPEED_MODELS, V_TAU, W_TAU
from safety_shield import SafetyShield

DT = 0.1
N_RAYS, N_BINS = 360, 36
R_MIN, R_MAX = 0.12, 3.5
COLLIDE_DIST = 0.16    # S1 机身 200 x 78 mm (含相机 216 mm), 原地转时机身前角扫过半径约 0.13 m, 再留余量
GOAL_RADIUS = 0.25
V_MAX, W_MAX = 0.20, 1.0      # 观测里速度的归一化常数 (与 rl_policy 相同, 不要改)
V_MIN, V_CMD_MAX = 0.0, 0.15  # 动作范围: 不能后退; 0.2 m/s 急起急停会翻倒 (见上面 v6 说明)
# 训练用的速度模型 (rl_policy.SPEED_MODELS); 环境变量 NAV_SPEED_MODEL 可以换 (例如 measured_875g = 多了原地转时慢慢后退)
SPEED_MODEL = os.environ.get('NAV_SPEED_MODEL', 'measured_875g_nocreep')
# 速度反应参数 V_GAIN / V_TAU / TURN_SLOWDOWN / W_GAIN / W_TAU 在 rl_policy.py (部署时也要用)
ACC_V, ACC_W = 0.5, 2.0       # 与 rl_controller.py 的输出加速度限制相同
SAFE_DIST = 0.40              # 离障碍物小于此距离开始扣分
COLLISION_PENALTY, GOAL_BONUS = 50.0, 30.0
NEAR_START_PROB = 0.25        # 从贴近障碍物处出发的回合比例
SHIELD_PENALTY = 0.1          # 安全保护介入时, 依意图与实际执行的差距扣分
# v4 试过 REVERSAL_PENALTY 0.3 / STEER_RATE_PENALTY 0.3: RL 学会站着不动 (速度 < REV_V 就不扣分), 到达 0/200, 已停用。
# 而且 v3 在 2D 环境本来就几乎不抖 (2.7 次/分), 抖动只在 Gazebo 出现, 问题不在这里。
REVERSAL_PENALTY = 0.0        # 前进中转向反转一次扣的分
STEER_RATE_PENALTY = 0.0      # 每步扣 |转向变化| (rad/s) x 速度/V_MAX
REV_W, REV_V = 0.05, 0.08     # 转向 > REV_W rad/s 且前进 > REV_V m/s 才算「前进中转向」
OBS_DIM = N_BINS + 1 + 2 + 2 + 2

# 全局路径规划 (模拟 Nav2 planner: 栅格 + 膨胀代价, 让路径离障碍物远一点)
GRID = 0.1
PLAN_BLOCK = 0.18             # 离障碍物小于此距离的格子不能走
INFLATION, INFLATION_COST = 0.45, 3.0

# 与 worlds/obstacle_test.world 一致的固定测试场地: ('box', cx, cy, sx, sy) / ('cyl', cx, cy, r)
TEST_WORLD = [
    ('box', 1.0, 0.3, 0.4, 0.4), ('box', 1.6, -0.9, 0.5, 0.3), ('cyl', 1.4, 0.9, 0.2),
    ('cyl', 0.5, -1.0, 0.15), ('box', -1.0, 1.2, 0.6, 0.3), ('cyl', -1.2, -1.0, 0.25),
]
WALLS = [('box', 0, 3, 6.1, 0.1), ('box', 0, -3, 6.1, 0.1), ('box', 3, 0, 0.1, 6.1), ('box', -3, 0, 0.1, 6.1)]
# 与 worlds/project3.world 一致 (project.pptx 第 1 页场景), 训练时不使用, 只用来评估
PROJECT3_WALLS = [('box', 2.9, 2.0, 7.1, 0.1), ('box', 2.9, -2.0, 7.1, 0.1),
                  ('box', -0.6, 0, 0.1, 4.1), ('box', 6.4, 0, 0.1, 4.1)]
PROJECT3 = [('box', 1.27, 0.0, 0.5, 1.5), ('box', 2.6, 1.4, 0.45, 0.45), ('box', 3.77, 1.1, 0.5, 1.7),
            ('cyl', 4.05, 0.05, 0.2), ('cyl', 2.65, -0.87, 0.28), ('cyl', 5.04, -0.72, 0.27)]
RAY_ANGLES = np.linspace(0, 2 * np.pi, N_RAYS, endpoint=False)


def clearance(x, y, obstacles):
    """点到最近障碍物表面的距离 (x, y 可以是 numpy 数组)。"""
    best = np.full(np.shape(x), 1e9)
    for o in obstacles:
        if o[0] == 'box':
            _, cx, cy, sx, sy = o
            dx, dy = np.abs(x - cx) - sx / 2, np.abs(y - cy) - sy / 2
            d = np.where((dx > 0) | (dy > 0), np.hypot(np.maximum(dx, 0), np.maximum(dy, 0)), np.maximum(dx, dy))
        else:
            _, cx, cy, r = o
            d = np.hypot(x - cx, y - cy) - r
        best = np.minimum(best, d)
    return best if np.ndim(best) else float(best)


def lidar_scan(x, y, yaw, obstacles):
    """返回 360 个距离 (无回波 = R_MAX)。"""
    ang = RAY_ANGLES + yaw
    dx, dy = np.cos(ang), np.sin(ang)
    best = np.full(N_RAYS, R_MAX)
    for o in obstacles:
        if o[0] == 'cyl':
            _, cx, cy, r = o
            fx, fy = x - cx, y - cy
            b = fx * dx + fy * dy
            c = fx * fx + fy * fy - r * r
            disc = b * b - c
            ok = disc >= 0
            t = -b - np.sqrt(np.where(ok, disc, 0))
            t = np.where(ok & (t > 0), t, np.inf)
            best = np.minimum(best, t)
        else:
            _, cx, cy, sx, sy = o
            with np.errstate(divide='ignore', invalid='ignore'):
                tx1, tx2 = (cx - sx / 2 - x) / dx, (cx + sx / 2 - x) / dx
                ty1, ty2 = (cy - sy / 2 - y) / dy, (cy + sy / 2 - y) / dy
            tmin = np.maximum(np.minimum(tx1, tx2), np.minimum(ty1, ty2))
            tmax = np.minimum(np.maximum(tx1, tx2), np.maximum(ty1, ty2))
            ok = (tmax >= np.maximum(tmin, 0)) & (tmin > 0)
            best = np.minimum(best, np.where(ok, tmin, np.inf))
    return np.clip(best, R_MIN, R_MAX)


def make_obs(scan, dist, rel_angle, prev_act, v, w):
    bins = scan.reshape(N_BINS, -1).min(axis=1) / R_MAX
    return np.concatenate([bins, [min(dist / 4.0, 1.0)], [math.sin(rel_angle), math.cos(rel_angle)],
                           prev_act, [v / V_MAX, w / W_MAX]]).astype(np.float32)


def arena_walls(w, h, cx=0.0, cy=0.0):
    return [('box', cx, cy + h / 2, w + 0.1, 0.1), ('box', cx, cy - h / 2, w + 0.1, 0.1),
            ('box', cx + w / 2, cy, 0.1, h + 0.1), ('box', cx - w / 2, cy, 0.1, h + 0.1)]


class Planner:
    """栅格 Dijkstra: 从终点往外算「沿路径的代价距离」场, 同时给出起点到终点的路径。"""

    def __init__(self, obstacles, walls, goal):
        xs = [o[1] for o in walls]; ys = [o[2] for o in walls]
        self.x0, self.y0 = min(xs), min(ys)
        nx, ny = int(math.ceil((max(xs) - self.x0) / GRID)) + 1, int(math.ceil((max(ys) - self.y0) / GRID)) + 1
        X, Y = np.meshgrid(self.x0 + GRID * np.arange(nx), self.y0 + GRID * np.arange(ny), indexing='ij')
        clr = clearance(X, Y, obstacles + walls)
        self.free = clr >= PLAN_BLOCK
        cost = 1.0 + INFLATION_COST * np.clip((INFLATION - clr) / (INFLATION - PLAN_BLOCK), 0, 1)
        self.field = np.full((nx, ny), np.inf)
        self.goal = goal
        g = self.cell(*goal)
        if not self.free[g]:
            return
        self.field[g] = 0.0
        heap = [(0.0, g)]
        nbr = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
               (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]
        while heap:
            d, (i, j) = heapq.heappop(heap)
            if d > self.field[i, j]:
                continue
            for di, dj, L in nbr:
                a, b = i + di, j + dj
                if 0 <= a < nx and 0 <= b < ny and self.free[a, b]:
                    nd = d + GRID * L * 0.5 * (cost[i, j] + cost[a, b])
                    if nd < self.field[a, b]:
                        self.field[a, b] = nd
                        heapq.heappush(heap, (nd, (a, b)))

    def cell(self, x, y):
        i = int(round((x - self.x0) / GRID)); j = int(round((y - self.y0) / GRID))
        return (min(max(i, 0), self.field.shape[0] - 1), min(max(j, 0), self.field.shape[1] - 1))

    def dist(self, x, y):
        """沿路径到终点的代价距离; 所在格子不可走 (贴近障碍物) 时取周围可走格子的最小值。"""
        i, j = self.cell(x, y)
        d = self.field[i, j]
        if np.isfinite(d):
            return float(d)
        win = self.field[max(i - 3, 0):i + 4, max(j - 3, 0):j + 4]
        return float(win.min() + 0.3) if np.isfinite(win).any() else math.inf

    def path(self, x, y):
        """从 (x, y) 沿代价场下降到终点的路径 [(x, y), ...]; 走不到时返回 None。"""
        i, j = self.cell(x, y)
        if not np.isfinite(self.field[i, j]):
            win = self.field[max(i - 3, 0):i + 4, max(j - 3, 0):j + 4]
            if not np.isfinite(win).any():
                return None
            a, b = np.unravel_index(np.argmin(win), win.shape)
            i, j = max(i - 3, 0) + a, max(j - 3, 0) + b
        pts = [(x, y)]
        for _ in range(self.field.size):
            if self.field[i, j] == 0.0:
                break
            best = min(((self.field[a, b], a, b) for a in range(i - 1, i + 2) for b in range(j - 1, j + 2)
                        if 0 <= a < self.field.shape[0] and 0 <= b < self.field.shape[1]))
            i, j = best[1], best[2]
            pts.append((self.x0 + GRID * i, self.y0 + GRID * j))
        pts.append(tuple(self.goal))
        return pts


class NavEnv(gym.Env):
    def __init__(self, seed=None, v_min=V_MIN, shield=False, v_max=V_CMD_MAX, speed_model=SPEED_MODEL):
        self.action_space = spaces.Box(-1.0, 1.0, (2,), np.float32)
        self.observation_space = spaces.Box(-1.0, 1.0, (OBS_DIM,), np.float32)
        self.rng = np.random.default_rng(seed)
        self.v_min, self.v_max = v_min, v_max     # 动作 -> 速度指令的范围; 评估旧权重时传它们自己的
        self.speed = SPEED_MODELS[speed_model]
        self.shield = SafetyShield(v_min, v_max, W_MAX, ACC_V, ACC_W, DT, tau_v=V_TAU, tau_w=W_TAU) if shield else None
        self.interventions = 0
        self.obstacles, self.walls = [], WALLS

    # ---- 场景生成 ----
    def _bar(self, x0, x1, y0, y1, length, thick, attach_prob=0.4):
        """一道长墙: 横或竖, 有一定机率从围墙伸出来 (像 project3 的隔墙)。"""
        horiz = self.rng.random() < 0.5
        if self.rng.random() < attach_prob:
            if horiz:
                side = x0 if self.rng.random() < 0.5 else x1
                cx = side + (length / 2 if side == x0 else -length / 2)
                return ('box', cx, self.rng.uniform(y0 + 0.8, y1 - 0.8), length, thick)
            side = y0 if self.rng.random() < 0.5 else y1
            cy = side + (length / 2 if side == y0 else -length / 2)
            return ('box', self.rng.uniform(x0 + 0.8, x1 - 0.8), cy, thick, length)
        cx, cy = self.rng.uniform(x0 + 0.5, x1 - 0.5), self.rng.uniform(y0 + 0.5, y1 - 0.5)
        return ('box', cx, cy, length, thick) if horiz else ('box', cx, cy, thick, length)

    def _scatter(self, n, x0, x1, y0, y1):
        obs = []
        for _ in range(n):
            cx, cy = self.rng.uniform(x0 + 0.3, x1 - 0.3), self.rng.uniform(y0 + 0.3, y1 - 0.3)
            if self.rng.random() < 0.5:
                obs.append(('cyl', cx, cy, self.rng.uniform(0.15, 0.3)))
            else:
                obs.append(('box', cx, cy, self.rng.uniform(0.3, 0.7), self.rng.uniform(0.3, 0.7)))
        return obs

    def _random_layout(self):
        w, h = self.rng.uniform(4.0, 8.0), self.rng.uniform(3.5, 6.0)
        walls = arena_walls(w, h)
        x0, x1, y0, y1 = -w / 2, w / 2, -h / 2, h / 2
        kind = self.rng.choice(['scatter', 'walls', 'trap'], p=[0.3, 0.45, 0.25])
        if kind == 'scatter':
            obs = self._scatter(self.rng.integers(3, 9), x0, x1, y0, y1)
        elif kind == 'walls':
            obs = [self._bar(x0, x1, y0, y1, self.rng.uniform(1.0, min(2.5, h - 1.2)), self.rng.uniform(0.1, 0.5))
                   for _ in range(self.rng.integers(1, 4))]
            obs += self._scatter(self.rng.integers(1, 6), x0, x1, y0, y1)
        else:
            # U 形陷阱 (开口朝随机方向) 或两道墙夹出的走廊
            cx, cy = self.rng.uniform(x0 + 1.2, x1 - 1.2), self.rng.uniform(y0 + 1.2, y1 - 1.2)
            s, t = self.rng.uniform(0.8, 1.4), self.rng.uniform(0.1, 0.3)
            if self.rng.random() < 0.6:
                sides = [('box', cx, cy + s / 2, s, t), ('box', cx, cy - s / 2, s, t),
                         ('box', cx + s / 2, cy, t, s), ('box', cx - s / 2, cy, t, s)]
                del sides[self.rng.integers(4)]
                obs = sides
            else:
                gap, L = self.rng.uniform(0.8, 1.4), self.rng.uniform(1.5, 3.0)
                obs = [('box', cx, cy + gap / 2 + t / 2, L, t), ('box', cx, cy - gap / 2 - t / 2, L, t)]
                if self.rng.random() < 0.5:   # 转成竖的走廊
                    obs = [('box', cx + (o[2] - cy), cy, o[4], o[3]) for o in obs]
            obs += self._scatter(self.rng.integers(1, 5), x0, x1, y0, y1)
        return obs, walls

    def _free_point(self, allo, lo, hi=9.0, avoid=None, min_sep=0.0):
        xs = [o[1] for o in self.walls]; ys = [o[2] for o in self.walls]
        for _ in range(300):
            p = np.array([self.rng.uniform(min(xs) + 0.25, max(xs) - 0.25), self.rng.uniform(min(ys) + 0.25, max(ys) - 0.25)])
            c = clearance(p[0], p[1], allo)
            if c < lo or c > hi:
                continue
            if avoid is not None and math.hypot(p[0] - avoid[0], p[1] - avoid[1]) < min_sep:
                continue
            return p
        return None

    def reset(self, seed=None, options=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        options = options or {}
        for _ in range(50):
            if options.get('obstacles') is not None:
                self.obstacles, self.walls = options['obstacles'], options.get('walls', WALLS)
            else:
                self.obstacles, self.walls = self._random_layout()
            allo = self.obstacles + self.walls
            s = options.get('start')
            if s is None:
                near = self.rng.random() < NEAR_START_PROB
                s = self._free_point(allo, 0.2, 0.35) if near else None
                s = s if s is not None else self._free_point(allo, 0.4)
            g = options.get('goal')
            g = g if g is not None else self._free_point(allo, 0.4, avoid=s, min_sep=2.0)
            if s is None or g is None:
                continue
            self.planner = Planner(self.obstacles, self.walls, g)
            self.path = self.planner.path(s[0], s[1])
            if self.path is not None or options.get('obstacles') is not None:
                break
        self.x, self.y = float(s[0]), float(s[1])
        self.path = self.path or [tuple(g)]
        self.path_idx = 0
        self.yaw = options.get('yaw', self.rng.uniform(-math.pi, math.pi))
        self.goal = np.array(g, dtype=float)
        self.v = self.w = 0.0
        self.v_out = self.w_out = 0.0
        self.prev_act = np.zeros(2)
        self.steps = 0
        self.gdist = self.planner.dist(self.x, self.y)
        path_len = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(self.path, self.path[1:]))
        v_top = 0.0
        for _ in range(100):                     # 速度模型在最高指令下的稳定速度
            v_top, _ = self.speed(v_top, 0.0, self.v_max, 0.0, DT)
        self.max_steps = int(min(2500, max(600, 2.0 * path_len / max(v_top, 0.05) / DT)))
        self.min_clear = 9.0
        self.interventions = 0
        self.reversals, self.last_turn = 0, 0   # 前进中转向反转次数, 上一次明显转向的方向
        return self._obs(), {}

    def _obs(self):
        allo = self.obstacles + self.walls
        scan = lidar_scan(self.x, self.y, self.yaw, allo) + self.rng.normal(0, 0.01, N_RAYS)
        scan = np.clip(scan, R_MIN, R_MAX)
        self.scan = scan
        (tx, ty), self.path_idx = local_target(self.path, self.path_idx, self.x, self.y)
        rel = math.atan2(ty - self.y, tx - self.x) - self.yaw
        rel = math.atan2(math.sin(rel), math.cos(rel))
        return make_obs(scan, math.hypot(tx - self.x, ty - self.y), rel, self.prev_act, self.v, self.w)

    def step(self, action):
        a = np.clip(action, -1, 1)
        v_cmd = self.v_min + (self.v_max - self.v_min) * (a[0] + 1) / 2
        w_cmd = W_MAX * a[1]
        shield_cost = 0.0
        w_prev = self.w_out
        if self.shield is not None:
            # 光达在机器人中心, 量程边界 (R_MAX) 视为无回波
            ranges = np.where(self.scan >= R_MAX - 1e-6, np.inf, self.scan)
            v_s, w_s, hit = self.shield.filter(v_cmd, w_cmd, self.v_out, self.w_out, ranges, RAY_ANGLES, self.v, self.w)
            if hit:
                self.interventions += 1
                v_i = float(np.clip(v_cmd, self.v_out - ACC_V * DT, self.v_out + ACC_V * DT))
                w_i = float(np.clip(w_cmd, self.w_out - ACC_W * DT, self.w_out + ACC_W * DT))
                shield_cost = math.hypot((v_i - v_s) / self.v_max, (w_i - w_s) / W_MAX)
            self.v_out, self.w_out = v_s, w_s
        else:
            self.v_out += float(np.clip(v_cmd - self.v_out, -ACC_V * DT, ACC_V * DT))
            self.w_out += float(np.clip(w_cmd - self.w_out, -ACC_W * DT, ACC_W * DT))
        v_cmd, w_cmd = self.v_out, self.w_out
        reversal = False
        if abs(self.w_out) > REV_W and self.v_out > REV_V:
            turn = 1 if self.w_out > 0 else -1
            reversal = self.last_turn != 0 and turn != self.last_turn
            self.reversals += int(reversal)
            self.last_turn = turn
        elif self.v_out <= REV_V:
            self.last_turn = 0      # 停下或原地转后重新开始算
        self.v, self.w = self.speed(self.v, self.w, v_cmd, w_cmd, DT)
        self.yaw += self.w * DT
        self.x += self.v * math.cos(self.yaw) * DT
        self.y += self.v * math.sin(self.yaw) * DT
        self.steps += 1
        new_dist = math.hypot(self.goal[0] - self.x, self.goal[1] - self.y)
        new_g = self.planner.dist(self.x, self.y)
        clr = clearance(self.x, self.y, self.obstacles + self.walls)
        self.min_clear = min(self.min_clear, clr)

        # 进度奖励用「沿路径到终点的距离」: 绕过长墙时直线距离会变大, 但沿路径距离仍在减少
        progress = self.gdist - new_g if np.isfinite(self.gdist) and np.isfinite(new_g) else 0.0
        reward = 8.0 * progress - 0.01 - 0.05 * float(np.abs(a - self.prev_act).sum()) - SHIELD_PENALTY * shield_cost
        reward -= REVERSAL_PENALTY * reversal + STEER_RATE_PENALTY * abs(self.w_out - w_prev) * max(self.v_out, 0.0) / V_MAX
        if clr < SAFE_DIST:
            reward -= 0.3 * (SAFE_DIST - clr) / SAFE_DIST
        self.gdist, self.prev_act = new_g, a.copy()
        terminated = truncated = False
        info = {}
        if clr < COLLIDE_DIST:
            reward, terminated, info['event'] = reward - COLLISION_PENALTY, True, 'collision'
        elif new_dist < GOAL_RADIUS:
            reward, terminated, info['event'] = reward + GOAL_BONUS, True, 'goal'
        elif self.steps >= self.max_steps:
            truncated, info['event'] = True, 'timeout'
        return self._obs(), float(reward), terminated, truncated, info
