#!/usr/bin/env python3
"""畫 LiDAR 安裝位置比較的側視示意圖 (簡報用): python3 plot_lidar_positions.py <輸出.png>

機身輪廓取自官方 URDF 網格範圍 (description/make_s1_official.py), 腿、質心與支撐中心來自 tools/com_check.py (CHAMP 站姿),
各位置的結果來自 results/sweeps/tune_875g_round2_lidar_x.txt 與 round3。座標為 base_link, 單位 mm, 右邊是機頭。"""
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Polygon, Rectangle

for f in ('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc', '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc'):
    try: font_manager.fontManager.addfont(f)
    except Exception: pass
plt.rcParams['font.family'] = ['Noto Sans CJK TC', 'Noto Sans CJK JP', 'sans-serif']
BLUE, RED, AMBER, INK, GREY, BODY = '#00549D', '#C0392B', '#C77700', '#0B0B0B', '#8A8A8A', '#D9DDE2'

fig, ax = plt.subplots(figsize=(4.75, 3.82), dpi=300)
GROUND = -105.3
ax.axhline(GROUND, color=GREY, lw=1.2)
# 機身: 下層 (整個機身長度)、上層平台、相機外殼
for x0, x1, z0, z1 in [(-106, 103, -2, 36), (-36, 56, 36, 68), (38, 56, 68, 80)]:
    ax.add_patch(Rectangle((x0, z0), x1 - x0, z1 - z0, fc=BODY, ec='#6B7480', lw=1.0))
# 腿 (大腿關節 -> 膝 -> 腳掌), 前後各一; CHAMP 站姿: 腳掌在大腿關節正下方
for hip, knee, foot in [((75, 14.5), (35.5, -30.5), (75, GROUND)), ((-75, 14.5), (-114.5, -30.5), (-75, GROUND))]:
    ax.plot(*zip(hip, knee, foot), color='#6B7480', lw=5, solid_capstyle='round')
# LiDAR: 官方位置 (虛線) 與採用位置 (實心), 外殼 x -19.3 ~ +27, z 68 ~ 103
ax.add_patch(Rectangle((-16.7 - 19.3, 68), 46.3, 35, fc='none', ec=AMBER, lw=1.3, ls='--'))
ax.add_patch(Rectangle((-60 - 19.3, 68), 46.3, 35, fc=BLUE, ec=BLUE, lw=1.3, alpha=0.9))
ax.add_patch(Rectangle((-60 - 12, 36), 24, 32, fc='none', ec=BLUE, lw=1.0, ls=':'))
ax.text(-60 + 4, 85, 'LiDAR\n45 g', color='white', ha='center', va='center', fontsize=8.5, fontweight='bold')
ax.text(-106, 54, '支架\n約 32 mm', color=BLUE, ha='center', va='center', fontsize=7.5)
ax.text(34, 96, '官方位置', color=AMBER, ha='left', va='center', fontsize=8)
# 各測試位置 (LiDAR 原點), 標在上方
pos = [(30, RED, '翻倒'), (4, RED, ''), (-16.7, AMBER, '大晃'), (-40, BLUE, ''), (-50, BLUE, ''), (-60, BLUE, '採用'),
       (-70, BLUE, ''), (-86, BLUE, '穩')]
for x, c, lab in pos:
    big = abs(x + 60) < 1
    ax.plot([x], [128], 'o', ms=10 if big else 6.5, color=c, mec='white', mew=0.8, zorder=5)
    if lab:
        ax.text(x, 139, lab, color=c, ha='center', va='bottom', fontsize=9 if big else 8, fontweight='bold' if big else 'normal')
ax.text(-138, 128, '測試的\n位置', color=INK, ha='center', va='center', fontsize=8)
# 整機質心 (LiDAR 在採用位置時) 與支撐中心
ax.plot([-5.0], [14.8], marker='o', ms=9, color=INK, zorder=6); ax.plot([-5.0], [14.8], marker='+', ms=7, color='white', zorder=7)
ax.annotate('整機質心 (920 g)', (-5.0, 14.8), (-72, -66), fontsize=8.5, color=INK, arrowprops=dict(arrowstyle='-', color=INK, lw=0.8))
ax.add_patch(Polygon([(0.3, GROUND), (-7.7, GROUND - 15.5), (8.3, GROUND - 15.5)], fc=BLUE, ec=BLUE))
ax.text(0.3, GROUND - 21, '四腳支撐中心', color=BLUE, ha='center', va='top', fontsize=8.5)
ax.annotate('', (135, 20), (105, 20), arrowprops=dict(arrowstyle='->', color=GREY, lw=1.5))
ax.text(120, 28, '前', color=GREY, ha='center', fontsize=9)
ax.set_xlim(-160, 145); ax.set_ylim(-150, 165); ax.set_aspect('equal'); ax.axis('off')
fig.subplots_adjust(0, 0, 1, 1)
fig.savefig(sys.argv[1], facecolor='white'); print('saved', sys.argv[1])
