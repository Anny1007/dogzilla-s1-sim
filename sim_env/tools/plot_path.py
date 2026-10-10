#!/usr/bin/env python3
"""把 tools/path_record.py 錄的真值路徑畫在 project3 場地上 (簡報用):
    python3 plot_path.py <path.csv> <輸出.png> [開始秒] [結束秒] [起點 x,y] [目標 x,y]
場地與 worlds/project3.world、benchmark_nav.py 的 WORLDS['project3'] 相同。"""
import csv, sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Circle, Rectangle

for f in ('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc', '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc'):
    try: font_manager.fontManager.addfont(f)
    except Exception: pass
plt.rcParams['font.family'] = ['Noto Sans CJK TC', 'Noto Sans CJK JP', 'sans-serif']
WALLS = [('box', 2.9, 2.0, 7.1, 0.1), ('box', 2.9, -2.0, 7.1, 0.1), ('box', -0.6, 0, 0.1, 4.1), ('box', 6.4, 0, 0.1, 4.1)]
OBST = [('box', 1.27, 0.0, 0.5, 1.5), ('box', 2.6, 1.4, 0.45, 0.45), ('box', 3.77, 1.1, 0.5, 1.7),
        ('cyl', 4.05, 0.05, 0.2), ('cyl', 2.65, -0.87, 0.28), ('cyl', 5.04, -0.72, 0.27)]
rows = [tuple(map(float, r)) for r in list(csv.reader(open(sys.argv[1])))[1:]]
t0 = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0; t1 = float(sys.argv[4]) if len(sys.argv) > 4 else 1e9
start = tuple(map(float, sys.argv[5].split(','))) if len(sys.argv) > 5 else None
goal = tuple(map(float, sys.argv[6].split(','))) if len(sys.argv) > 6 else None
seg = [r for r in rows if t0 <= r[0] <= t1]
fig, ax = plt.subplots(figsize=(7.2, 4.6), dpi=220)
for o in WALLS + OBST:
    fc, ec = ('#B9BEC5', '#8A9099') if o in WALLS else ('#D9DDE2', '#6B7480')
    ax.add_patch(Rectangle((o[1] - o[3] / 2, o[2] - o[4] / 2), o[3], o[4], fc=fc, ec=ec, lw=0.8) if o[0] == 'box'
                 else Circle((o[1], o[2]), o[3], fc=fc, ec=ec, lw=0.8))
ax.plot([r[1] for r in seg], [r[2] for r in seg], color='#00549D', lw=1.8, label='機器狗走過的路徑（模擬器真實位置）')
s = start or (seg[0][1], seg[0][2])
ax.plot(*s, 'o', color='#0B0B0B', ms=7, label='起點'); ax.text(s[0] + 0.12, s[1] - 0.28, '起點', fontsize=9)
if goal:
    ax.plot(*goal, '*', color='#C0392B', ms=15, label='目標'); ax.text(goal[0] - 0.15, goal[1] + 0.22, f'目標 ({goal[0]}, {goal[1]})', fontsize=9, ha='right')
ax.plot([], [], 's', color='#D9DDE2', mec='#6B7480', ms=9, label='障礙物')
ax.set_xlim(-0.9, 6.7); ax.set_ylim(-2.3, 2.3); ax.set_aspect('equal')
ax.set_xlabel('x (m)', fontsize=9); ax.set_ylabel('y (m)', fontsize=9); ax.tick_params(labelsize=8)
for sp in ('top', 'right'): ax.spines[sp].set_visible(False)
ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.13), ncol=4, frameon=False, fontsize=8.5)
fig.tight_layout(); fig.savefig(sys.argv[2], facecolor='white'); print('saved', sys.argv[2], len(seg), '點')
