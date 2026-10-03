#!/usr/bin/env python3
"""畫 PPO 訓練過程的 loss / reward / 成功率: python3 plot_ppo_metrics.py <訓練輸出目錄> <輸出.png>

讀 rl/train.py 產生的 <目錄>/progress.csv (SB3 每次 rollout 一行) 與 <目錄>/eval.csv (每 10 萬步評估)。
淡色細線是原始值, 深色線是移動平均。
"""
import csv
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager

for f in ('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc', '/fonts/NotoSansCJK-Regular.ttc'):
    if os.path.exists(f):
        font_manager.fontManager.addfont(f)
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=f).get_name()
        break

BLUE, ORANGE = '#2a78d6', '#eb6834'
INK, MUTED, GRID, SURFACE = '#0b0b0b', '#52514e', '#e4e3df', '#fcfcfb'

# (欄位, 標題, 說明)
PANELS = [
    ('rollout/ep_rew_mean', '平均回合回報 (reward)', '越高越好'),
    (None, '到達率 / 碰撞率 (%)', '「accuracy」: 200 個固定隨機場景'),
    ('rollout/ep_len_mean', '平均回合長度 (步)', '越短 = 越快到達或越早撞'),
    ('train/loss', '總 loss', 'policy + 0.5·value + entropy 項'),
    ('train/value_loss', 'Value loss', 'critic 預測回報的誤差'),
    ('train/policy_gradient_loss', 'Policy gradient loss', 'actor 的 clipped surrogate'),
    ('train/entropy_loss', 'Entropy loss', '越接近 0 = 動作越確定'),
    ('train/explained_variance', 'Explained variance', 'critic 解釋回報的比例, 1 最好'),
    ('train/approx_kl', 'Approx KL', '每次更新策略改變多少'),
]


def read_csv(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return {k: np.array([float(r[k]) if r[k] not in ('', None) else np.nan for r in rows]) for k in rows[0]}


def smooth(y, n=15):
    out = np.full_like(y, np.nan)
    for i in range(len(y)):
        w = y[max(0, i - n + 1):i + 1]
        w = w[np.isfinite(w)]
        if len(w):
            out[i] = w.mean()
    return out


def style(ax):
    ax.set_facecolor(SURFACE)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'):
        ax.spines[s].set_color(GRID)
    ax.tick_params(colors=MUTED, labelsize=8)
    ax.grid(color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


def main():
    d, out = sys.argv[1], sys.argv[2]
    p = read_csv(f'{d}/progress.csv')
    ev = read_csv(f'{d}/eval.csv')
    x = p['time/total_timesteps'] / 1e6
    fig, axes = plt.subplots(3, 3, figsize=(15, 10))
    fig.patch.set_facecolor(SURFACE)
    for ax, (key, title, note) in zip(axes.flat, PANELS):
        style(ax)
        ax.set_title(title, loc='left', fontsize=11, color=INK, pad=18)
        ax.text(0, 1.02, note, transform=ax.transAxes, fontsize=8, color=MUTED, va='bottom', ha='left')
        if key is None:
            ex = ev['timesteps'] / 1e6
            for col, c, lab in (('goal', BLUE, '到達'), ('collision', ORANGE, '碰撞')):
                y = 100 * ev[col] / 200
                ax.plot(ex, y, color=c, linewidth=2, marker='o', markersize=3.5, label=lab)
                ax.annotate(f'{lab} {y[-1]:.0f}%', (ex[-1], y[-1]), xytext=(-4, 8), textcoords='offset points',
                            ha='right', fontsize=8, color=INK)
            ax.set_ylim(0, 100)
            ax.legend(frameon=False, fontsize=8, loc='center right', labelcolor=INK)
        else:
            y = p[key]
            ax.plot(x, y, color=BLUE, linewidth=0.8, alpha=0.3)
            ax.plot(x, smooth(y), color=BLUE, linewidth=2)
            if key == 'train/explained_variance':
                ax.set_ylim(min(0, np.nanmin(y)), 1)
        if ax in axes[-1]:
            ax.set_xlabel('訓練步數 (百萬)', color=MUTED, fontsize=9)
    fig.suptitle('PPO 訓練過程 — loss 與成功率', x=0.01, ha='left', fontsize=15, color=INK)
    fig.tight_layout(h_pad=2.2)
    fig.savefig(out, dpi=140, facecolor=SURFACE)
    print('saved', out)


if __name__ == '__main__':
    main()
