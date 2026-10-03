"""2D 光达避障导航环境 (gymnasium)，用于训练 DOGZILLA 的强化学习局部控制器。

约定与 Gazebo 仿真保持一致，方便直接部署:
  * 光达 360 线, 0..2π 从机器人正前方逆时针, 量程 0.12..3.5 m, 无回波按最大量程处理
  * 观测里把 360 线按 10° 一组取最小值 -> 36 个方向
  * 碰撞: 机器人中心离障碍物表面 < COLLIDE_DIST
  * 速度响应: 一阶滞后 + 增益损失 + 与 rl_controller 相同的加速度限制
  参数对应官方 DOGZILLA S1 URDF 模型 (description/dogzilla_s1_official.urdf.xacro), Gazebo 实测 (tools/gait_eval.py):
  指令 0.2 m/s -> 实际约 0.134 m/s, 原地转 0.8 rad/s -> 实际约 0.75 rad/s;
  边走边转时前进速度明显下降 (0.15 m/s + 0.5 rad/s -> 实际前进约 0.034 m/s), 用 TURN_SLOWDOWN 近似。
  手工方块模型 (s1_box) 版本的权重在 models/s1_box/。
  旧 CHAMP 放大模型的版本与权重在 models/champ_old/ (COLLIDE_DIST 0.12, V_MAX 0.3, V_GAIN 0.7, W_GAIN 0.9)
"""
import math

import gymnasium as gym
import numpy as np
from gymnasium import spaces

DT = 0.1
ARENA = 3.0            # 围墙在 ±3 m
N_RAYS, N_BINS = 360, 36
R_MIN, R_MAX = 0.12, 3.5
COLLIDE_DIST = 0.16    # S1 机身 200 x 78 mm (含相机 216 mm), 原地转时机身前角扫过半径约 0.13 m, 再留余量
GOAL_RADIUS = 0.25
V_MAX, W_MAX = 0.20, 1.0   # S1 安全速度上限 0.2 m/s (config/gait_s1.yaml)
V_GAIN, V_TAU = 0.67, 0.5     # 实际速度 = 指令 * 增益, 一阶滞后时间常数
TURN_SLOWDOWN = 1.1           # 实际前进速度再乘 max(0.1, 1 - TURN_SLOWDOWN * |w|)
W_GAIN, W_TAU = 0.94, 0.3
ACC_V, ACC_W = 0.5, 2.0       # 与 rl_controller.py 的输出加速度限制相同
SAFE_DIST = 0.40              # 离障碍物小于此距离开始扣分
MAX_STEPS = 600
OBS_DIM = N_BINS + 1 + 2 + 2 + 2

# 与 worlds/obstacle_test.world 一致的固定测试场地: ('box', cx, cy, sx, sy) / ('cyl', cx, cy, r)
TEST_WORLD = [
    ('box', 1.0, 0.3, 0.4, 0.4), ('box', 1.6, -0.9, 0.5, 0.3), ('cyl', 1.4, 0.9, 0.2),
    ('cyl', 0.5, -1.0, 0.15), ('box', -1.0, 1.2, 0.6, 0.3), ('cyl', -1.2, -1.0, 0.25),
]
WALLS = [('box', 0, 3, 6.1, 0.1), ('box', 0, -3, 6.1, 0.1), ('box', 3, 0, 0.1, 6.1), ('box', -3, 0, 0.1, 6.1)]
RAY_ANGLES = np.linspace(0, 2 * np.pi, N_RAYS, endpoint=False)


def clearance(x, y, obstacles):
    best = 1e9
    for o in obstacles:
        if o[0] == 'box':
            _, cx, cy, sx, sy = o
            dx, dy = abs(x - cx) - sx / 2, abs(y - cy) - sy / 2
            d = math.hypot(max(dx, 0), max(dy, 0)) if (dx > 0 or dy > 0) else max(dx, dy)
        else:
            _, cx, cy, r = o
            d = math.hypot(x - cx, y - cy) - r
        best = min(best, d)
    return best


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


class NavEnv(gym.Env):
    def __init__(self, seed=None):
        self.action_space = spaces.Box(-1.0, 1.0, (2,), np.float32)
        self.observation_space = spaces.Box(-1.0, 1.0, (OBS_DIM,), np.float32)
        self.rng = np.random.default_rng(seed)
        self.obstacles = []

    # ---- 场景生成 ----
    def _random_layout(self):
        obs = []
        for _ in range(self.rng.integers(3, 9)):
            cx, cy = self.rng.uniform(-2.5, 2.5, 2)
            if self.rng.random() < 0.5:
                obs.append(('cyl', cx, cy, self.rng.uniform(0.15, 0.3)))
            else:
                obs.append(('box', cx, cy, self.rng.uniform(0.3, 0.7), self.rng.uniform(0.3, 0.7)))
        return obs

    def _free_point(self, obstacles, margin, avoid=None, min_sep=0.0):
        for _ in range(200):
            p = self.rng.uniform(-2.6, 2.6, 2)
            if clearance(p[0], p[1], obstacles + WALLS) < margin:
                continue
            if avoid is not None and math.hypot(p[0] - avoid[0], p[1] - avoid[1]) < min_sep:
                continue
            return p
        return np.zeros(2)

    def reset(self, seed=None, options=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        options = options or {}
        self.obstacles = options.get('obstacles', None) or self._random_layout()
        allo = self.obstacles + WALLS
        s = options.get('start')
        self.x, self.y = (s if s is not None else self._free_point(self.obstacles, 0.4))
        self.yaw = options.get('yaw', self.rng.uniform(-math.pi, math.pi))
        g = options.get('goal')
        self.goal = np.array(g if g is not None else self._free_point(self.obstacles, 0.4, (self.x, self.y), 2.0))
        self.v = self.w = 0.0
        self.v_out = self.w_out = 0.0
        self.prev_act = np.zeros(2)
        self.steps = 0
        self.dist = math.hypot(self.goal[0] - self.x, self.goal[1] - self.y)
        self.min_clear = 9.0
        return self._obs(), {}

    def _obs(self):
        scan = lidar_scan(self.x, self.y, self.yaw, self.obstacles + WALLS) + self.rng.normal(0, 0.01, N_RAYS)
        scan = np.clip(scan, R_MIN, R_MAX)
        rel = math.atan2(self.goal[1] - self.y, self.goal[0] - self.x) - self.yaw
        rel = math.atan2(math.sin(rel), math.cos(rel))
        return make_obs(scan, self.dist, rel, self.prev_act, self.v, self.w)

    def step(self, action):
        a = np.clip(action, -1, 1)
        v_cmd, w_cmd = V_MAX * (a[0] + 1) / 2, W_MAX * a[1]
        self.v_out += float(np.clip(v_cmd - self.v_out, -ACC_V * DT, ACC_V * DT))
        self.w_out += float(np.clip(w_cmd - self.w_out, -ACC_W * DT, ACC_W * DT))
        v_cmd, w_cmd = self.v_out, self.w_out
        self.v += (V_GAIN * v_cmd * max(0.1, 1 - TURN_SLOWDOWN * abs(self.w)) - self.v) * DT / V_TAU
        self.w += (W_GAIN * w_cmd - self.w) * DT / W_TAU
        self.yaw += self.w * DT
        self.x += self.v * math.cos(self.yaw) * DT
        self.y += self.v * math.sin(self.yaw) * DT
        self.steps += 1
        new_dist = math.hypot(self.goal[0] - self.x, self.goal[1] - self.y)
        clr = clearance(self.x, self.y, self.obstacles + WALLS)
        self.min_clear = min(self.min_clear, clr)

        reward = 8.0 * (self.dist - new_dist) - 0.01 - 0.05 * float(np.abs(a - self.prev_act).sum())
        if clr < SAFE_DIST:
            reward -= 0.3 * (SAFE_DIST - clr) / SAFE_DIST
        self.dist, self.prev_act = new_dist, a.copy()
        terminated = truncated = False
        info = {}
        if clr < COLLIDE_DIST:
            reward, terminated, info['event'] = reward - 15.0, True, 'collision'
        elif new_dist < GOAL_RADIUS:
            reward, terminated, info['event'] = reward + 15.0, True, 'goal'
        elif self.steps >= MAX_STEPS:
            truncated, info['event'] = True, 'timeout'
        return self._obs(), float(reward), terminated, truncated, info
