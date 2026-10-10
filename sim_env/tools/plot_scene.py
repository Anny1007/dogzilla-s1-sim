#!/usr/bin/env python3
"""画 rl/scenes.py 里的测试场地 (障碍物、7 个目标点的顺序), 可叠上 tools/path_record.py 录的真值路径:
    python3 tools/plot_scene.py <场地名称> <输出.png> [path.csv]"""
import csv, os, sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Circle, Rectangle
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '../rl'))
from scenes import SCENES
for f in ('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',):
    try: font_manager.fontManager.addfont(f)
    except Exception: pass
plt.rcParams['font.family'] = ['Noto Sans CJK TC', 'Noto Sans CJK JP', 'sans-serif']
s = SCENES[sys.argv[1]]
_w = [o for o in s['walls']]; _ar = (max(o[2] for o in _w) - min(o[2] for o in _w) + 0.6) / (max(o[1] for o in _w) - min(o[1] for o in _w) + 0.6)
fig, ax = plt.subplots(figsize=(7.2, max(4.4, 7.2 * _ar * 0.92)), dpi=170)
for o in s['walls'] + s['obstacles']:
    fc, ec = ('#B9BEC5', '#8A9099') if o in s['walls'] else ('#D9DDE2', '#6B7480')
    ax.add_patch(Rectangle((o[1] - o[3] / 2, o[2] - o[4] / 2), o[3], o[4], fc=fc, ec=ec, lw=0.8) if o[0] == 'box' else Circle((o[1], o[2]), o[3], fc=fc, ec=ec, lw=0.8))
if len(sys.argv) > 3:
    r = [tuple(map(float, x)) for x in list(csv.reader(open(sys.argv[3])))[1:]]
    ax.plot([a[1] for a in r], [a[2] for a in r], color='#00549D', lw=1.0, alpha=0.9, label='機器狗走過的路徑（模擬器真實位置）')
ax.plot(0, 0, 'o', color='#0B0B0B', ms=7); ax.text(0.08, -0.3, '起點', fontsize=9)
seen = {}
for i, (gx, gy) in enumerate(s['goals'], 1):
    seen.setdefault((gx, gy), []).append(str(i))
for (gx, gy), ids in seen.items():
    if (gx, gy) != (0.0, 0.0):
        ax.plot(gx, gy, '*', color='#C0392B', ms=13)
    ax.text(gx + 0.1, gy + 0.12, '目標 ' + '、'.join(ids), fontsize=8.5, color='#C0392B')
wx = [o[1] for o in s['walls']]; wy = [o[2] for o in s['walls']]
ax.set_xlim(min(wx) - 0.3, max(wx) + 0.3); ax.set_ylim(min(wy) - 0.3, max(wy) + 0.3); ax.set_aspect('equal'); ax.set_xlabel('x (m)', fontsize=9); ax.set_ylabel('y (m)', fontsize=9); ax.tick_params(labelsize=8)
for sp in ('top', 'right'): ax.spines[sp].set_visible(False)
if len(sys.argv) > 3: ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.13), frameon=False, fontsize=8.5)
fig.tight_layout(); fig.savefig(sys.argv[2], facecolor='white'); print('saved', sys.argv[2])
