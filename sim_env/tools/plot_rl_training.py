#!/usr/bin/env python3
"""把 rl/train.py 的訓練日誌畫成學習曲線: python3 plot_rl_training.py <輸出.png> <日誌.log>...

日誌每 10 萬步一行 "[步數] 秒數  到达 x/200  碰撞 y  超时 z" (200 個固定隨機場景的評估),
最後若有 eval_offline.py 的結果 ("RL(PPO) 到达 a/210 ...", "直奔目标 到达 b/210 ...") 也一併畫成長條圖。
"""
import os
import re
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

for f in ('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc', '/fonts/NotoSansCJK-Regular.ttc'):
    if os.path.exists(f):
        font_manager.fontManager.addfont(f)
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=f).get_name()
        break

SERIES = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100']   # 類別色, 依序指派
INK, MUTED, GRID, SURFACE = '#0b0b0b', '#52514e', '#e4e3df', '#fcfcfb'
STEP = re.compile(r'\[\s*(\d+)\]\s+\d+s\s+到达 (\d+)/(\d+)\s+碰撞 (\d+)\s+超时 (\d+)')
OFFLINE = re.compile(r'^(RL\(PPO\)|直奔目标)\s+到达 (\d+)/(\d+)')


def parse(path):
    steps, reach, coll, offline = [], [], [], {}
    for line in open(path, encoding='utf-8'):
        m = STEP.search(line)
        if m:
            s, g, n, c, _ = map(int, m.groups())
            steps.append(s / 1e6); reach.append(100 * g / n); coll.append(100 * c / n)
        m = OFFLINE.search(line.strip())
        if m:
            offline[m.group(1)] = 100 * int(m.group(2)) / int(m.group(3))
    return steps, reach, coll, offline


def main():
    out, logs = sys.argv[1], sys.argv[2:]
    runs = [(os.path.splitext(os.path.basename(p))[0], *parse(p)) for p in logs]
    has_offline = any(r[4] for r in runs)
    fig, axes = plt.subplots(1, 3 if has_offline else 2, figsize=(16 if has_offline else 11, 4.6),
                             gridspec_kw={'width_ratios': [3, 3, 2] if has_offline else [1, 1]})
    fig.patch.set_facecolor(SURFACE)
    for ax in axes:
        ax.set_facecolor(SURFACE)
        for s in ('top', 'right'):
            ax.spines[s].set_visible(False)
        for s in ('left', 'bottom'):
            ax.spines[s].set_color(GRID)
        ax.tick_params(colors=MUTED, labelsize=9)
        ax.grid(axis='y', color=GRID, linewidth=0.8)
        ax.set_axisbelow(True)

    for k, (ax, title, idx) in enumerate([(axes[0], '到達率 (%)', 2), (axes[1], '碰撞率 (%)', 3)]):
        for i, r in enumerate(runs):
            ax.plot(r[1], r[idx], color=SERIES[i], linewidth=2, marker='o', markersize=3.5, label=r[0])
            ax.annotate(f'{r[idx][-1]:.0f}%', (r[1][-1], r[idx][-1]), xytext=(6, 0), textcoords='offset points',
                        va='center', fontsize=9, color=INK)
        ax.set_title(title, loc='left', fontsize=12, color=INK)
        ax.set_xlabel('訓練步數 (百萬)', color=MUTED, fontsize=9)
        ax.set_ylim(0, 100)
    axes[0].legend(frameon=False, fontsize=9, loc='lower right', labelcolor=INK)

    if has_offline:
        ax = axes[2]
        labels, vals, cols = [], [], []
        for i, r in enumerate(runs):
            if 'RL(PPO)' in r[4]:
                labels.append(r[0]); vals.append(r[4]['RL(PPO)']); cols.append(SERIES[i])
        base = next(r[4]['直奔目标'] for r in runs if '直奔目标' in r[4])
        labels.append('直奔目標 (基準)'); vals.append(base); cols.append('#9c9b96')
        y = range(len(labels))[::-1]
        ax.barh(list(y), vals, color=cols, height=0.55)
        for yy, v in zip(y, vals):
            ax.text(v + 1.5, yy, f'{v:.0f}%', va='center', fontsize=9, color=INK)
        ax.set_yticks(list(y)); ax.set_yticklabels(labels, fontsize=9, color=INK)
        ax.set_xlim(0, 110); ax.grid(axis='y', visible=False); ax.grid(axis='x', color=GRID, linewidth=0.8)
        ax.set_title('固定測試場地到達率 (訓練沒看過, 210 段)', loc='left', fontsize=12, color=INK)

    fig.suptitle('RL (PPO) 避障訓練結果 — 每 10 萬步在 200 個隨機場景評估', x=0.01, ha='left', fontsize=14, color=INK)
    fig.tight_layout()
    fig.savefig(out, dpi=150, facecolor=SURFACE)
    print('saved', out)


if __name__ == '__main__':
    main()
