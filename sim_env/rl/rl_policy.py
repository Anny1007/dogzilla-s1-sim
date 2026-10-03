"""纯 numpy 推理: 部署到 ROS 容器时不需要 torch。obs 的构造与 nav_env.make_obs 完全一致。

local_target() 同时给 nav_env.py (训练) 与 rl_controller.py (部署) 使用, 保证两边瞄准的路径点算法相同。
"""
import math

import numpy as np

V_MAX, W_MAX = 0.20, 1.0   # 须与 nav_env.py 一致 (S1); 观测里的速度也用它归一化
LOOKAHEAD = 0.8            # 沿全局路径向前瞄准的距离 (m)
WAYPOINT_ADVANCE = 0.4     # 离当前路径点小于此距离就换下一个


class Policy:
    def __init__(self, path):
        d = np.load(path)
        self.W = [d['W0'], d['W1'], d['W2']]; self.b = [d['b0'], d['b1'], d['b2']]
        # 动作 a[0] ∈ [-1, 1] 映射到 [v_min, v_max]。旧权重没有存这两个值, 是 [0, 0.2] (不能后退)
        self.v_min = float(d['v_min']) if 'v_min' in d.files else 0.0
        self.v_max = float(d['v_max']) if 'v_max' in d.files else V_MAX

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
