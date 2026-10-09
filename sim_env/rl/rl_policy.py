"""纯 numpy 推理: 部署到 ROS 容器时不需要 torch。obs 的构造与 nav_env.make_obs 完全一致。

local_target() 同时给 nav_env.py (训练) 与 rl_controller.py (部署) 使用, 保证两边瞄准的路径点算法相同。
"""
import math

import numpy as np

V_MAX, W_MAX = 0.20, 1.0   # 观测里的速度用它归一化 (固定, 不随动作上限改变); 旧权重的动作上限也是 0.2
# S1 步态对速度指令的反应 (一阶滞后 + 增益损失), Gazebo 实测 (tools/gait_eval.py)。
# 训练环境 nav_env.py 用它模拟机器狗; 部署时 rl_controller.py 用它推算观测里的「目前速度」:
# 四足走路时机身随步伐左右摆头, 腿部里程计的转向在 ±1.4 rad/s 间来回 (平均为 0), 直接当观测会让 RL 跟着抖。
V_GAIN, V_TAU = 0.67, 0.5
TURN_SLOWDOWN = 1.1           # 实际前进速度再乘 max(0.1, 1 - TURN_SLOWDOWN * |w|)
W_GAIN, W_TAU = 0.94, 0.3


def speed_model(v, w, v_cmd, w_cmd, dt):
    """v1~v3 用的线性模型: 由送出的指令推算下一刻的实际速度。"""
    v += (V_GAIN * v_cmd * max(0.1, 1 - TURN_SLOWDOWN * abs(w)) - v) * dt / V_TAU
    w += (W_GAIN * w_cmd - w) * dt / W_TAU
    return v, w


def speed_model_measured(v, w, v_cmd, w_cmd, dt):
    """v5 起用的模型, 依 Gazebo 实测 (tools/speed_response.py, results/speed_response_s1*.txt) 拟合:
    前进: 稳定速度 = max(0, 0.855 * 指令 - 0.0475), 低于约 0.056 m/s 不会前进, 也不能后退
          (实测 0.10 -> 0.038, 0.15 -> 0.08, 0.20 -> 0.12~0.13, -0.05 -> 0)
    转弯减速: 只在指令超过 0.1 m/s 时发生, 倍率 max(0.3, 1 - 26 * (指令 - 0.1) * |转向指令|)
          (实测 0.1 + 转向 0.4 不减速; 0.15 + 0.5 剩 0.39; 0.2 + 0.2 剩 0.45 且会翻倒)
    转向: 与线性模型相同 (实测增益 0.88~1.07)"""
    target = max(0.0, 0.855 * v_cmd - 0.0475) * max(0.3, 1 - 26 * max(0.0, v_cmd - 0.1) * abs(w_cmd))
    v += (target - v) * dt / V_TAU
    w += (W_GAIN * w_cmd - w) * dt / W_TAU
    return v, w


def speed_model_875g(v, w, v_cmd, w_cmd, dt, creep=True):
    """v6 起用的模型: 实机重量 (本体 875 g + LiDAR 45 g, LiDAR 在 x=-0.06) 的 Gazebo 实测
    (tools/speed_response.py, results/speed_response_s1_875g_run*.txt)。与 v5 模型不同的地方:
    * 包含运动安全滤波 (launch/cmd_vel_safety.py): RL 的输出先乘 max(0.15, 1 - |转向指令|) 才到步态控制器。
      新重量下步态本身边走边转已经不会减速 (直接送 /cmd_vel 0.15 + 0.5 rad/s 仍有 0.08 m/s), 减速全部来自滤波
    * 前进: 稳定速度 = 0.9 * (滤波后指令) - 0.051, 上限 0.11 (实测 0.10 -> 0.039, 0.15 -> 0.084, 0.20 -> 0.10~0.13;
      经滤波 0.10 + 0.2 -> 0.017, 0.10 + 0.4 -> 0.00, 0.15 + 0.5 -> 0.016)
    * 只要步态在踏步 (有前进或转向指令), 指令太小时机身会慢慢后退: 原地转实测 -0.05 m/s (creep=False 时当作 0)
    * 时间常数 0.35 s (实测), 转向增益 1.0 (实测经滤波 0.90~1.06)"""
    stepping = abs(v_cmd) > 1e-3 or abs(w_cmd) > 0.02
    target = min(0.11, 0.9 * v_cmd * max(0.15, 1 - abs(w_cmd)) - 0.051) if stepping else 0.0
    if not creep:
        target = max(0.0, target)
    v += (target - v) * dt / 0.35
    w += (w_cmd - w) * dt / W_TAU
    return v, w


SPEED_MODELS = {'linear': speed_model, 'measured': speed_model_measured, 'measured_875g': speed_model_875g,
                'measured_875g_nocreep': lambda *a: speed_model_875g(*a, creep=False)}
LOOKAHEAD = 0.8            # 沿全局路径向前瞄准的距离 (m)
WAYPOINT_ADVANCE = 0.4     # 离当前路径点小于此距离就换下一个


class Policy:
    def __init__(self, path):
        d = np.load(path)
        self.W = [d['W0'], d['W1'], d['W2']]; self.b = [d['b0'], d['b1'], d['b2']]
        # 动作 a[0] ∈ [-1, 1] 映射到 [v_min, v_max]。旧权重没有存这两个值, 是 [0, 0.2] (不能后退)
        self.v_min = float(d['v_min']) if 'v_min' in d.files else 0.0
        self.v_max = float(d['v_max']) if 'v_max' in d.files else V_MAX
        # 训练时有没有安全保护 (safety_shield.py); 有的话部署时也要开
        self.shield = bool(d['shield']) if 'shield' in d.files else False
        # 训练时用的速度模型 (部署时推算观测里的速度要用同一个); 旧权重是 linear
        self.speed_model = str(d['speed_model']) if 'speed_model' in d.files else 'linear'

    def act(self, obs):
        h = np.tanh(self.W[0] @ obs + self.b[0]); h = np.tanh(self.W[1] @ h + self.b[1])
        a = np.clip(self.W[2] @ h + self.b[2], -1, 1)          # 确定性动作 (均值)
        v = self.v_min + (self.v_max - self.v_min) * (a[0] + 1) / 2
        return v, W_MAX * a[1], a                              # v_cmd, w_cmd, 归一化动作(作为下一步的 prev_act)


def local_target(path, idx, x, y):
    """沿全局路径 path=[(x,y),...] 向前找 LOOKAHEAD 处的点。返回 (目标点, 新的 idx)。"""
    while idx < len(path) - 1 and math.hypot(path[idx][0] - x, path[idx][1] - y) < WAYPOINT_ADVANCE:
        idx += 1
    acc = 0.0
    for i in range(idx, len(path) - 1):
        (x1, y1), (x2, y2) = path[i], path[i + 1]
        seg = math.hypot(x2 - x1, y2 - y1)
        if acc + seg >= LOOKAHEAD:
            return path[i + 1], idx
        acc += seg
    return path[-1], idx
