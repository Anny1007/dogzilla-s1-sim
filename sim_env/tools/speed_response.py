"""量 S1 在 Gazebo 裡對速度指令的實際反應 (用真值), 給 rl/nav_env.py 的速度模型校準用。
用法 (容器內, 模擬以空場地啟動且沒有其他節點送速度指令, 例如 ./run.sh sim rl world:=/usr/share/gazebo-11/worlds/empty.world):
    python3 /tools/speed_response.py [每段秒數]

量測項目:
  1. 速度表: 固定 (v, w) 指令, 直接送 /cmd_vel, 量穩定後的實際前進速度 (沿機身方向)、轉向速率、最大傾斜
  2. 經過運動安全濾波: 同樣的指令改送 /cmd_vel_raw (cmd_vel_safety.py 會降速、限制轉向變化)
  3. 反應時間: 靜止 -> 前進 0.15 m/s, 實際速度到達穩定值 63% 所需時間
  4. 轉向抖動: 前進 0.15 m/s, 轉向在 ±0.2 rad/s 間以 2.5 Hz 切換 (RL 在 Gazebo 裡出現的情況)
機身傾斜超過 45° 視為翻倒, 立即停止 (之後的量測無效)。
"""
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

T = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0
SETTLE = 2.0          # 每段前幾秒是加速段, 不算進穩定速度
rclpy.init()
n = rclpy.create_node('speed_response')
buf = []


def cb(m):
    p = m.pose.pose; q = p.orientation
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (q.x * q.x + q.y * q.y)))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    buf.append((time.time(), p.position.x, p.position.y, yaw, tilt))


n.create_subscription(Odometry, '/demo/odom/ground_truth', cb, 50)
pubs = {'/cmd_vel': n.create_publisher(Twist, '/cmd_vel', 10), '/cmd_vel_raw': n.create_publisher(Twist, '/cmd_vel_raw', 10)}
flipped = False


def send(topic, v, w):
    t = Twist(); t.linear.x = float(v); t.angular.z = float(w); pubs[topic].publish(t)


def hold(topic, cmd, dur):
    """cmd(t) -> (v, w), 每 0.1 s 送一次 (与 RL 控制频率相同)。回传这段时间的真值纪录。"""
    global flipped
    t0 = time.time(); start = len(buf); nxt = 0.0
    while time.time() - t0 < dur:
        if time.time() - t0 >= nxt:
            send(topic, *cmd(time.time() - t0)); nxt += 0.1
        rclpy.spin_once(n, timeout_sec=0.01)
        if buf and buf[-1][4] > 45:
            flipped = True; break
    return np.array(buf[start:]), t0


def steady(a, t0):
    a = a[a[:, 0] > t0 + SETTLE]
    if len(a) < 10:
        return float('nan'), float('nan')
    dt = a[-1, 0] - a[0, 0]
    yaw_unwrap = np.unwrap(a[:, 3])
    # 沿机身方向的前进速度: 每一小段位移投影到当时的朝向
    dx, dy = np.diff(a[:, 1]), np.diff(a[:, 2])
    fwd = np.sum(dx * np.cos(a[:-1, 3]) + dy * np.sin(a[:-1, 3])) / dt
    return fwd, (yaw_unwrap[-1] - yaw_unwrap[0]) / dt


def rest(topic):
    hold(topic, lambda t: (0.0, 0.0), 2.0)


def table(topic, cases):
    print(f'\n== 速度表 (指令送 {topic}, 每段 {T:.0f} 秒, 去掉前 {SETTLE:.0f} 秒)')
    print(f"{'指令 v':>7s} {'指令 w':>7s} | {'实际前进':>8s} {'实际转向':>8s} | {'前进/指令':>8s} {'转向/指令':>8s} | 最大倾斜")
    rows = []
    for v, w in cases:
        if flipped:
            break
        a, t0 = hold(topic, lambda t, v=v, w=w: (v, w), T)
        fv, fw = steady(a, t0)
        rv = fv / v if abs(v) > 1e-6 else float('nan'); rw = fw / w if abs(w) > 1e-6 else float('nan')
        print(f'{v:+7.2f} {w:+7.2f} | {fv:+8.3f} {fw:+8.2f} | {rv:8.2f} {rw:8.2f} | {a[:, 4].max():5.1f}°' + ('  ← 翻倒' if flipped else ''),
              flush=True)
        rows.append((v, w, fv, fw))
        rest(topic)
    return rows


while not buf:
    rclpy.spin_once(n, timeout_sec=0.1)
rest('/cmd_vel')

direct = table('/cmd_vel', [(0.05, 0), (0.10, 0), (0.15, 0), (0.20, 0), (0, 0.3), (0, 0.6), (0, 1.0),
                            (0.10, 0.2), (0.10, 0.4), (0.20, 0.2), (0.15, 0.5), (-0.05, 0), (-0.05, 0.3)])
safety = table('/cmd_vel_raw', [(0.20, 0), (0.10, 0.2), (0.10, 0.4), (0.15, 0.5)])

if not flipped:
    print('\n== 反应时间 (/cmd_vel, 静止 -> 前进 0.15 m/s)')
    a, t0 = hold('/cmd_vel', lambda t: (0.15, 0.0), 6.0)
    t = a[:, 0] - t0
    # 沿机身方向的位移, 用 0.5 s 窗口 (约一个步态周期) 算速度
    s = np.concatenate([[0], np.cumsum(np.diff(a[:, 1]) * np.cos(a[:-1, 3]) + np.diff(a[:, 2]) * np.sin(a[:-1, 3]))])
    tt = np.arange(0.5, t[-1], 0.1)
    vv = np.array([(np.interp(x, t, s) - np.interp(x - 0.5, t, s)) / 0.5 for x in tt])
    v_ss = vv[tt > 3].mean()
    t63 = tt[np.argmax(vv >= 0.63 * v_ss)] - 0.25      # 窗口中心
    print(f'稳定速度 {v_ss:.3f} m/s, 到达 63% 约 {t63:.2f} s (一阶滞后的时间常数), ' +
          '速度曲线 ' + ' '.join(f'{x:.2f}' for x in vv[::5]))
    rest('/cmd_vel')

if not flipped:
    print('\n== 转向抖动 (前进 0.15 m/s, 转向 ±0.2 rad/s, 2.5 Hz 切换)')
    for topic in ('/cmd_vel', '/cmd_vel_raw'):
        a, t0 = hold(topic, lambda t: (0.15, 0.2 if int(t * 5) % 2 == 0 else -0.2), T)
        fv, fw = steady(a, t0)
        print(f'  送 {topic:12s} 实际前进 {fv:+.3f} m/s, 转向 {fw:+.2f} rad/s, 最大倾斜 {a[:, 4].max():.1f}°' + ('  ← 翻倒' if flipped else ''))
        rest(topic)
        if flipped:
            break

send('/cmd_vel', 0, 0)
print('\n== 翻倒, 后面的量测无效' if flipped else '\n== 完成')
