"""安全保护 (Safe RL 动作投影), 参考 2026_Steven_paper 的 Swarm_UAV_Safe_Env:
    RL 给出「想要的速度」(意图) -> 传统方法 (DWA 动态窗口 + 碰撞检查) 列出安全候选速度
    -> 执行候选里最接近意图的那一个。RL 选不出会撞的动作, 安全由传统方法保证。

训练 (nav_env.py) 与部署 (rl_controller.py) 共用本文件, 只用 numpy。

做法:
  1. 动态窗口: 依加速度限制, 列出下一步做得到的 (v, w) 网格, 再加上「意图截进窗口」这一个候选
  2. 碰撞检查: 每个候选走 HORIZON 秒, 算轨迹离光达点的最近距离。实际速度不会立刻变成指令值,
     而是从目前速度以一阶滞后 (时间常数 tau_v / tau_w) 逼近, 例如煞车时还会往前滑一段
     - 离障碍物 >= SAFE_DIST: 安全
     - 已经比 SAFE_DIST 近 (例如从贴墙处出发): 只要不比现在更靠近就算安全, 原地转与停下永远可行, 不会卡死
  3. 选择: 安全候选中离意图最近的 (速度与角速度各自按上限归一化); 意图本身安全时原样执行
  4. 都不安全 (例如高速冲向墙、窗口里没有 0): 选轨迹最近距离最大的那个
光达点以机器人中心为原点 (正前方 0 rad, 逆时针为正)。无回波 (inf) 的点忽略。
"""
import numpy as np

SAFE_DIST = 0.22      # 碰撞判定 0.16 m + 步态对速度指令约 0.5 s 延迟的余量
HORIZON, STEPS = 1.0, 10
N_V, N_W = 7, 9       # 动态窗口网格


class SafetyShield:
    def __init__(self, v_min, v_max, w_max, acc_v, acc_w, dt=0.1, safe_dist=SAFE_DIST, tau_v=0.0, tau_w=0.0):
        self.v_min, self.v_max, self.w_max = v_min, v_max, w_max
        self.acc_v, self.acc_w, self.dt = acc_v, acc_w, dt
        self.safe = safe_dist
        self.t = np.linspace(HORIZON / STEPS, HORIZON, STEPS)
        self.h = HORIZON / STEPS
        # 每个时间点「实际速度还差指令多少」的比例; tau = 0 表示指令立刻生效
        self.lag_v = np.exp(-self.t / tau_v) if tau_v > 0 else np.zeros(STEPS)
        self.lag_w = np.exp(-self.t / tau_w) if tau_w > 0 else np.zeros(STEPS)
        self.reach = max(abs(v_min), v_max) * HORIZON + safe_dist + 0.05   # 超过这个距离的点不可能撞到

    def candidates(self, v_now, w_now, v_int, w_int):
        lo_v, hi_v = max(self.v_min, v_now - self.acc_v * self.dt), min(self.v_max, v_now + self.acc_v * self.dt)
        lo_w, hi_w = max(-self.w_max, w_now - self.acc_w * self.dt), min(self.w_max, w_now + self.acc_w * self.dt)
        V, W = np.meshgrid(np.linspace(lo_v, hi_v, N_V), np.linspace(lo_w, hi_w, N_W), indexing='ij')
        intent = (float(np.clip(v_int, lo_v, hi_v)), float(np.clip(w_int, lo_w, hi_w)))
        return np.concatenate([[intent[0]], V.ravel()]), np.concatenate([[intent[1]], W.ravel()])

    def clearance(self, v, w, px, py, v_act=0.0, w_act=0.0):
        """每个候选 (v[k], w[k]) 的轨迹离光达点的最近距离, shape (K,)。v_act / w_act 是目前实际速度。"""
        vt = v[:, None] + (v_act - v[:, None]) * self.lag_v      # (K, S) 各时间点的实际速度
        wt = w[:, None] + (w_act - w[:, None]) * self.lag_w
        th = np.cumsum(wt * self.h, axis=1)
        x = np.cumsum(vt * np.cos(th) * self.h, axis=1)
        y = np.cumsum(vt * np.sin(th) * self.h, axis=1)
        d = np.hypot(x[:, :, None] - px, y[:, :, None] - py)      # (K, S, P)
        return d.min(axis=(1, 2))

    def filter(self, v_int, w_int, v_now, w_now, ranges, angles, v_act=None, w_act=None):
        """回传 (执行的 v, 执行的 w, 是否介入)。v_now / w_now 是上一步送出的指令 (动态窗口的中心),
        v_act / w_act 是目前实际速度 (没给就当作等于指令)。"""
        v, w = self.candidates(v_now, w_now, v_int, w_int)
        r = np.asarray(ranges, dtype=float)
        ok = np.isfinite(r)
        if not ok.any():
            return v[0], w[0], False
        cur = float(r[ok].min())
        near = ok & (r < self.reach)
        if not near.any():
            return v[0], w[0], False
        a = np.asarray(angles, dtype=float)[near]
        px, py = r[near] * np.cos(a), r[near] * np.sin(a)
        clr = self.clearance(v, w, px, py, v_now if v_act is None else v_act, w_now if w_act is None else w_act)
        safe = clr >= min(self.safe, cur - 0.005)
        if safe[0]:
            return v[0], w[0], False
        if not safe.any():
            k = int(np.argmax(clr))
        else:
            cost = ((v - v_int) / self.v_max) ** 2 + ((w - w_int) / self.w_max) ** 2
            k = int(np.argmin(np.where(safe, cost, np.inf)))
        return float(v[k]), float(w[k]), True
