"""在 2D 环境比较多个策略 (部署方式: 沿全局路径的瞄准点):
    python eval_compare.py [名称=权重.npz ...]      默认比较 models/policy.npz 与 models/s1_v2/policy.npz

三组场景:
  旧测试场地   worlds/obstacle_test.world, 7 段 x 30 轮
  投影片场地   worlds/project3.world (训练时从未见过), 与 benchmark_nav.py 相同的 7 段 x 30 轮
  随机新场景   v2 训练分布, 200 个训练评估没用过的种子
另附「追路径点」传统控制当基准: 朝瞄准点转向, 偏太多就原地转。
"""
import math
import sys

import numpy as np

from nav_env import DT, NavEnv, PROJECT3, PROJECT3_WALLS, TEST_WORLD, WALLS
from rl_policy import Policy

OLD_GOALS = [(2.2, 0.0), (-2.0, 0.3), (2.2, -1.8), (-2.0, -2.0), (2.2, 1.8), (-1.8, 1.8), (0.0, 0.0)]
P3_GOALS = [(5.9, 1.1), (0.0, 0.0), (5.6, -1.5), (0.3, 1.5), (3.1, 0.6), (1.5, -1.5), (5.9, 1.1)]


class PurePursuit:
    v_min = 0.0

    def act(self, o):
        ang = math.atan2(o[37], o[38])
        a = np.array([1.0 if abs(ang) < 0.6 else -1.0, np.clip(2 * ang, -1, 1)], dtype=np.float32)
        return 0, 0, a


def run_episode(e, pol, options=None, seed=None):
    o, _ = e.reset(seed=seed, options=options)
    while True:
        _, _, a = pol.act(o)
        o, _, te, tr, i = e.step(a)
        if te or tr:
            return i['event'], e.steps * DT, e.min_clear


def fixed_world(pol, obstacles, walls, goals, start, rounds=30):
    e = NavEnv(seed=42, v_min=pol.v_min); res = []
    for _ in range(rounds):
        s = start
        for g in goals:
            res.append(run_episode(e, pol, dict(obstacles=obstacles, walls=walls, start=s, goal=g,
                                                yaw=e.rng.uniform(-math.pi, math.pi))))
            s = g
    return res


def random_worlds(pol, n=200):
    e = NavEnv(v_min=pol.v_min)
    return [run_episode(e, pol, seed=50000 + k) for k in range(n)]


def summary(res):
    n = len(res); ev = [r[0] for r in res]
    t = [r[1] for r in res if r[0] == 'goal']
    return (f"到达 {ev.count('goal'):3d}/{n} ({100 * ev.count('goal') / n:3.0f}%)  碰撞 {ev.count('collision'):3d}  "
            f"超时 {ev.count('timeout'):3d}  成功段平均 {np.mean(t) if t else float('nan'):5.1f}s  "
            f"最近离障 {min(r[2] for r in res):.2f}m")


if __name__ == '__main__':
    specs = sys.argv[1:] or ['旧(v1)=models/policy.npz', '新(v2)=models/s1_v2/policy.npz']
    pols = [(s.split('=')[0], Policy(s.split('=')[1])) for s in specs] + [('追路径点', PurePursuit())]
    for title, fn in [('旧测试场地 (7 段 x 30 轮)', lambda p: fixed_world(p, TEST_WORLD, WALLS, OLD_GOALS, (0.0, 0.0))),
                      ('投影片场地 project3 (训练没看过, 7 段 x 30 轮)', lambda p: fixed_world(p, PROJECT3, PROJECT3_WALLS, P3_GOALS, (0.0, 0.0))),
                      ('随机新场景 (200 个)', random_worlds)]:
        print(f'== {title}')
        for name, p in pols:
            print(f'  {name:8s} {summary(fn(p))}', flush=True)
