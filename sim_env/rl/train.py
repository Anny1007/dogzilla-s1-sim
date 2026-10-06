"""PPO 训练: python train.py [总步数] [输出目录] [--shield]  -> 模型存 <输出目录>/ppo_nav.zip, 部署权重存 <输出目录>/policy.npz
输出目录默认 models (会覆盖部署用的权重)。--shield: 训练时加上安全保护 (safety_shield.py), 权重档会记录, 部署时自动开启。
评估到达数最高的模型另存 <输出目录>/best/ (ppo_nav.zip + policy.npz)。训练曲线 (loss / entropy / KL / 平均回报 ...) 存 <输出目录>/progress.csv,
每 10 万步的到达/碰撞评估存 <输出目录>/eval.csv, 用 tools/plot_rl_training.py 画图。"""
import os, sys, time
import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.logger import configure
from stable_baselines3.common.vec_env import SubprocVecEnv
from nav_env import NavEnv, V_MIN, V_CMD_MAX, SPEED_MODEL

torch.set_num_threads(1)
SHIELD = '--shield' in sys.argv
ARGS = [a for a in sys.argv[1:] if a != '--shield']
TOTAL = int(float(ARGS[0])) if len(ARGS) > 0 else 2_000_000
OUT = ARGS[1] if len(ARGS) > 1 else 'models'


def evaluate(model, n=200):
    e = NavEnv(shield=SHIELD); ev = {'goal': 0, 'collision': 0, 'timeout': 0}; steps = 0
    for k in range(n):
        o, _ = e.reset(seed=1000 + k)          # 与冒烟测试相同的 200 个场景
        while True:
            a, _ = model.predict(o, deterministic=True)
            o, r, te, tr, i = e.step(a); steps += 1
            if te or tr: ev[i['event']] += 1; break
    return ev


class EvalCB(BaseCallback):
    def __init__(self, every=100_000):
        super().__init__(); self.every, self.next, self.t0 = every, every, time.time()
        self.best = -1
    def _on_step(self):
        if self.num_timesteps >= self.next:
            self.next += self.every
            ev = evaluate(self.model)
            print(f"[{self.num_timesteps:>9}] {time.time()-self.t0:5.0f}s  到达 {ev['goal']}/200  碰撞 {ev['collision']}  超时 {ev['timeout']}", flush=True)
            with open(f'{OUT}/eval.csv', 'a') as f:
                f.write(f"{self.num_timesteps},{ev['goal']},{ev['collision']},{ev['timeout']}\n")
            self.model.save(f'{OUT}/ppo_nav')
            if ev['goal'] > self.best:              # 保留评估最好的模型, 不会被后面变差的版本盖掉
                self.best = ev['goal']
                os.makedirs(f'{OUT}/best', exist_ok=True)
                self.model.save(f'{OUT}/best/ppo_nav'); export_npz(self.model, f'{OUT}/best/policy.npz')
                print(f"            -> 新的最佳模型 (到达 {ev['goal']}/200), 存到 {OUT}/best/", flush=True)
        return True


def export_npz(model, path):
    sd = model.policy.state_dict()
    keys = ['mlp_extractor.policy_net.0', 'mlp_extractor.policy_net.2', 'action_net']
    out = {}
    for i, k in enumerate(keys):
        out[f'W{i}'] = sd[k + '.weight'].cpu().numpy(); out[f'b{i}'] = sd[k + '.bias'].cpu().numpy()
    out['v_min'], out['v_max'] = np.float32(V_MIN), np.float32(V_CMD_MAX)   # 动作 -> 速度的映射, rl_policy.Policy 读取
    out['speed_model'] = np.array(SPEED_MODEL)                              # 部署时推算观测速度用同一个模型
    out['shield'] = np.int8(SHIELD)                                     # 训练时有没有安全保护, 部署时照着开
    np.savez(path, **out)


if __name__ == '__main__':
    os.makedirs(OUT, exist_ok=True)
    with open(f'{OUT}/eval.csv', 'w') as f:
        f.write('timesteps,goal,collision,timeout\n')
    env = make_vec_env(NavEnv, n_envs=8, vec_env_cls=SubprocVecEnv, env_kwargs={'shield': SHIELD})
    model = PPO('MlpPolicy', env, n_steps=512, batch_size=512, learning_rate=3e-4, gamma=0.995, gae_lambda=0.95,
                ent_coef=0.005, policy_kwargs=dict(net_arch=dict(pi=[64, 64], vf=[64, 64]), activation_fn=torch.nn.Tanh),
                seed=0, verbose=0, device='cpu')
    model.set_logger(configure(OUT, ['csv']))     # 每次 rollout 后写一行 progress.csv
    model.learn(TOTAL, callback=EvalCB())
    model.save(f'{OUT}/ppo_nav'); export_npz(model, f'{OUT}/policy.npz')
    ev = evaluate(model); print(f"FINAL 到达 {ev['goal']}/200 碰撞 {ev['collision']} 超时 {ev['timeout']}", flush=True); print("== DONE")
