"""站姿下的整機質心與四腳支撐中心 (靜態計算, 不需要 Gazebo), 用來看 LiDAR 放在不同前後位置時質心偏多少。
站姿用 CHAMP 步態控制器實際下的關節角 (大腿 0.72 rad, 膝 -1.48 rad: 腳掌在大腿關節正下方), 不是模型生成時的初始角。
Gazebo 實測走路時四腳的平均位置 (tf base_link -> *_foot_link): 前腳 x = +76 mm, 後腳 x = -67 mm, 中心約 +5 mm。
用法 (主機, sim_env/ 下): python3 tools/com_check.py [LiDAR x ...]     讀 description/dogzilla_s1_official.urdf.xacro"""
import math, os, re, sys
import xml.etree.ElementTree as ET
import numpy as np

src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../description/dogzilla_s1_official.urdf.xacro'), encoding='utf-8').read()
tu, tl = 0.72, -1.48      # CHAMP 站姿 (/joint_group_effort_controller/joint_trajectory 靜止時的指令)
src = re.sub(r'<xacro:if.*?</xacro:if>', '', src, flags=re.S); src = re.sub(r'<xacro:[^>]*?/>', '', src)
root = ET.fromstring(src.replace('xmlns:xacro="http://ros.org/wiki/xacro"', '').encode())
J = {j.find('child').get('link'): j for j in root.findall('joint')}
ang = {'upper_leg_joint': tu, 'lower_leg_joint': tl}


def pose(link):
    if link == 'base_link':
        return np.eye(3), np.zeros(3)
    j = J[link]; R, p = pose(j.find('parent').get('link'))
    a = ang.get(j.get('name')[3:], 0.0); c, s = math.cos(a), math.sin(a)
    return R @ np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]), p + R @ np.array([float(v) for v in j.find('origin').get('xyz').split()])


parts = {}
for l in root.findall('link'):
    i = l.find('inertial')
    if i is None:
        continue
    o = i.find('origin'); c = np.array([float(v) for v in o.get('xyz').split()]) if o is not None else np.zeros(3)
    R, p = pose(l.get('name')); parts[l.get('name')] = (float(i.find('mass').get('value')), p + R @ c)
feet = np.mean([pose(f'{g}_foot_link')[1] for g in ('lf', 'rf', 'lh', 'rh')], axis=0)
lm, lc = parts.pop('laser_link')
bm = sum(v[0] for v in parts.values()); bc = sum(v[0] * v[1] for v in parts.values()) / bm
lx0 = float(J['laser_link'].find('origin').get('xyz').split()[0])
print(f'四腳支撐中心 x = {feet[0]*1000:+.1f} mm (base_link 座標, + 為前方)')
print(f'本體 {bm*1000:.0f} g, 質心 x = {bc[0]*1000:+.1f} mm, 高度 z = {bc[2]*1000:+.1f} mm;  LiDAR {lm*1000:.0f} g, 質心高度 z = {lc[2]*1000:+.1f} mm')
for lx in [float(a) for a in sys.argv[1:]] or [lx0]:
    c = (bm * bc + lm * (lc + np.array([lx - lx0, 0, 0]))) / (bm + lm)
    print(f'  LiDAR x = {lx*1000:+6.1f} mm -> 整機質心 x = {c[0]*1000:+.2f} mm (在支撐中心後方 {(feet[0]-c[0])*1000:.2f} mm), 高度 z = {c[2]*1000:+.2f} mm')
