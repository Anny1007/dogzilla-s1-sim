import numpy as np, math
from stable_baselines3 import PPO
from nav_env import *
from rl_policy import Policy
m=PPO.load('models/ppo_nav', device='cpu'); p=Policy('models/policy.npz')
e=NavEnv(seed=5); o,_=e.reset(); mx=0
for _ in range(300):
    a1,_=m.predict(o,deterministic=True); _,_,a2=p.act(o); mx=max(mx,float(np.abs(a1-a2).max()))
    o,r,te,tr,i=e.step(a1)
    if te or tr: o,_=e.reset()
print("numpy 推理与 SB3 模型最大动作差: %.2e"%mx)

GOALS=[(2.2,0.0),(-2.0,0.3),(2.2,-1.8),(-2.0,-2.0),(2.2,1.8),(-1.8,1.8),(0.0,0.0)]
def run_policy(pol, name, rounds=30):
    ev={'goal':0,'collision':0,'timeout':0}; times=[]; clears=[]
    e=NavEnv(seed=42)
    for r in range(rounds):
        start=(0.0,0.0); yaw=e.rng.uniform(-math.pi,math.pi)
        for g in GOALS:
            o,_=e.reset(options=dict(obstacles=TEST_WORLD,start=start,goal=g,yaw=yaw))
            while True:
                _,_,a=pol(o); o,_,te,tr,i=e.step(a)
                if te or tr: break
            ev[i['event']]+=1; clears.append(e.min_clear)
            if i['event']=='goal': times.append(e.steps*DT)
            start=(e.x,e.y) if i['event']=='goal' else (0.0,0.0); yaw=e.yaw
    n=sum(ev.values())
    print(f"{name:10s} 到达 {ev['goal']}/{n} ({100*ev['goal']/n:.0f}%)  碰撞 {ev['collision']}  超时 {ev['timeout']}  成功段平均用时 {np.mean(times):.1f}s  最近离障 {min(clears):.2f}m")
def greedy(o):
    ang=math.atan2(o[37],o[38]); return 0,0,np.array([1.0 if abs(ang)<0.6 else -0.5, np.clip(2*ang,-1,1)],dtype=np.float32)
if __name__=='__main__':
    print("固定测试场地(训练时从未见过), 7 段 x 30 轮 = 210 段:")
    run_policy(p.act,"RL(PPO)"); run_policy(greedy,"直奔目标")
