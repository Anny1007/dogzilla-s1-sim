#!/usr/bin/env python3
"""依 rl/scenes.py 的定义产生 Gazebo 场地 worlds/<名称>.world (地面、灯光等沿用 worlds/project3.world 的开头):
    python3 tools/make_world.py [名称 ...]      不给名称就全部重产。障碍物高 1 m; 第 1 段的目标放一个绿色标记 (只有外观, 没有碰撞)。"""
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '../rl'))
from scenes import SCENES

head = open(os.path.join(HERE, '../worlds/project3.world'), encoding='utf-8').read()
head = head[:head.index('<!-- project.pptx')]
MAT = lambda c: f'<material><ambient>{c} 1</ambient><diffuse>{c} 1</diffuse></material>'


def model(name, o, color):
    geo = (f'<box><size>{o[3]} {o[4]} 1.0</size></box>' if o[0] == 'box' else f'<cylinder><radius>{o[3]}</radius><length>1.0</length></cylinder>')
    return (f"<model name='{name}'><static>1</static><pose>{o[1]} {o[2]} 0.5 0 0 0</pose><link name='l'>\n"
            f"<collision name='c'><geometry>{geo}</geometry></collision>\n"
            f"<visual name='v'><geometry>{geo}</geometry>{MAT(color)}</visual></link></model>\n")


for name in sys.argv[1:] or SCENES:
    s = SCENES[name]
    # Gazebo 视窗的初始镜头: 从场地南侧斜上方看向场地中心 (project3.world 原本的镜头是给 7 m x 4 m 场地用的)
    xs = [o[1] for o in s['walls']]; ys = [o[2] for o in s['walls']]
    cx, cy, span = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2, max(max(xs) - min(xs), max(ys) - min(ys))
    cam = f"<pose frame=''>{cx:.2f} {cy - 0.85 * span:.2f} {0.75 * span:.2f} 0 0.72 1.5708</pose>" if span > 7.5 else None
    h = head.replace("<pose frame=''>2.9 -5.0 4.2 0 0.72 1.5708</pose>", cam) if cam else head
    out = h + f'<!-- 由 tools/make_world.py 依 rl/scenes.py 的 {name} 产生, 请勿手动修改 -->\n'
    for i, o in enumerate(s['walls']):
        out += model(f'wall_{i}', o, '0.6 0.6 0.6')
    for i, o in enumerate(s['obstacles']):
        out += model(f'obs_{i}', o, '0.85 0.45 0.1' if o[0] == 'cyl' else '0.72 0.72 0.72')
    gx, gy = s['goals'][0]
    out += (f"<model name='goal_marker'><static>1</static><pose>{gx} {gy} 0 0 0 0</pose><link name='l'>\n"
            f"<visual name='pad'><pose>0 0 0.005 0 0 0</pose><geometry><cylinder><radius>0.2</radius><length>0.01</length></cylinder></geometry>{MAT('0.15 0.6 0.25')}</visual>\n"
            f"<visual name='pole'><pose>0 0 0.3 0 0 0</pose><geometry><cylinder><radius>0.03</radius><length>0.6</length></cylinder></geometry>{MAT('0.15 0.6 0.25')}</visual></link></model>\n"
            '</world>\n</sdf>\n')
    path = os.path.join(HERE, f'../worlds/{name}.world'); open(path, 'w', encoding='utf-8').write(out); print('写出', os.path.normpath(path), len(s['obstacles']), '个障碍物')
