#!/usr/bin/env python3
"""把模擬用的 S1 模型 (description/dogzilla_s1_official.urdf.xacro 的外觀網格) 畫成一張圖 (簡報用), 只需要 numpy 與 Pillow:
    python3 render_model.py <輸出.png> [方位角=-58] [仰角=24]
站姿用 CHAMP 的關節角 (大腿 0.72, 膝 -1.48 rad)。LiDAR 畫在模型裡的位置 (make_s1_official.py 的 LIDAR_X);
LiDAR 下方的支架模型裡沒有, 這裡畫一個方塊示意。作法: 正交投影 + z-buffer + 平面著色。"""
import math, os, re, struct, sys
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
MESH = os.path.join(HERE, '../../vendor/Program/yahboomcar_ws_ros2/src/champ/champ_description/meshes/XGO')
src = open(os.path.join(HERE, '../description/dogzilla_s1_official.urdf.xacro'), encoding='utf-8').read()
src = re.sub(r'<xacro:if.*?</xacro:if>', '', src, flags=re.S); src = re.sub(r'<xacro:[^>]*?/>', '', src)
root = ET.fromstring(src.replace('xmlns:xacro="http://ros.org/wiki/xacro"', '').encode())
J = {j.find('child').get('link'): j for j in root.findall('joint')}
ANG = {'upper_leg_joint': 0.72, 'lower_leg_joint': -1.48}
GREY = {'base_link': 0.60, 'hip': 0.30, 'upper': 0.52, 'lower': 0.47, 'camera_link': 0.10, 'laser_link': 0.13}


def pose(link):
    if link == 'base_link':
        return np.eye(3), np.zeros(3)
    j = J[link]; R, p = pose(j.find('parent').get('link'))
    a = ANG.get(j.get('name')[3:], 0.0); c, s = math.cos(a), math.sin(a)
    return R @ np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]), p + R @ np.array([float(v) for v in j.find('origin').get('xyz').split()])


def stl(path):
    d = open(path, 'rb').read(); n = struct.unpack('<I', d[80:84])[0]
    return np.frombuffer(d[84:84 + 50 * n], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))['v'].reshape(-1, 3, 3).astype(float)


def box(lo, hi):
    (x0, y0, z0), (x1, y1, z1) = lo, hi
    c = np.array([[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0], [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]])
    q = [(0, 1, 2, 3), (4, 5, 6, 7), (0, 1, 5, 4), (2, 3, 7, 6), (1, 2, 6, 5), (0, 3, 7, 4)]
    return np.array([[c[a], c[b], c[d]] for a, b, d, e in q] + [[c[a], c[d], c[e]] for a, b, d, e in q])


tris, shade = [], []
for l in root.findall('link'):
    m = l.find('visual/geometry/mesh')
    if m is None:
        continue
    name = l.get('name'); R, p = pose(name)
    t = stl(os.path.join(MESH, os.path.basename(m.get('filename')))) @ R.T + p
    g = GREY.get(name) or next(v for k, v in GREY.items() if k in name)
    tris.append(t); shade.append(np.full(len(t), g))
lx = float(J['laser_link'].find('origin').get('xyz').split()[0])
if lx < -0.04:      # LiDAR 已移到後段低平台上方: 畫支架示意 (平台 z=0.036, LiDAR 底面 z=0.068)
    t = box((lx - 0.012, -0.014, 0.036), (lx + 0.016, 0.014, 0.0685)); tris.append(t); shade.append(np.full(len(t), 0.42))
T = np.concatenate(tris); G = np.concatenate(shade)

az, el = math.radians(float(sys.argv[2]) if len(sys.argv) > 2 else -58), math.radians(float(sys.argv[3]) if len(sys.argv) > 3 else 24)
view = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])     # 由模型指向相機
right = np.cross([0, 0, 1], view); right /= np.linalg.norm(right); up = np.cross(view, right)
P = np.stack([T @ right, T @ up, T @ view], axis=-1)                                              # (N, 3, 3): 畫面 x, y, 深度
nrm = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]); ln = np.linalg.norm(nrm, axis=1); ok = ln > 1e-14
nrm[ok] /= ln[ok, None]
light = np.array([0.35, -0.45, 0.82]); light = light / np.linalg.norm(light)
lum = G * (0.42 + 0.48 * np.abs(nrm @ light) + 0.22 * np.abs(nrm @ view))

SS, W, H = 3, 770, 617
lo, hi = P[..., :2].reshape(-1, 2).min(0), P[..., :2].reshape(-1, 2).max(0)
sc = min((W * SS * 0.96) / (hi[0] - lo[0]), (H * SS * 0.96) / (hi[1] - lo[1]))
off = np.array([W * SS / 2, H * SS / 2]) - sc * (lo + hi) / 2
X = P[..., 0] * sc + off[0]; Y = H * SS - (P[..., 1] * sc + off[1]); Z = P[..., 2]
zbuf = np.full((H * SS, W * SS), -1e9); img = np.zeros((H * SS, W * SS)); mask = np.zeros((H * SS, W * SS), bool)
for i in np.nonzero(ok)[0]:
    x, y, z = X[i], Y[i], Z[i]
    x0, x1 = max(int(math.floor(x.min())), 0), min(int(math.ceil(x.max())), W * SS - 1)
    y0, y1 = max(int(math.floor(y.min())), 0), min(int(math.ceil(y.max())), H * SS - 1)
    if x1 < x0 or y1 < y0:
        continue
    den = (y[1] - y[2]) * (x[0] - x[2]) + (x[2] - x[1]) * (y[0] - y[2])
    if abs(den) < 1e-12:
        continue
    px, py = np.meshgrid(np.arange(x0, x1 + 1) + 0.5, np.arange(y0, y1 + 1) + 0.5)
    a = ((y[1] - y[2]) * (px - x[2]) + (x[2] - x[1]) * (py - y[2])) / den
    b = ((y[2] - y[0]) * (px - x[2]) + (x[0] - x[2]) * (py - y[2])) / den
    c = 1 - a - b
    zz = a * z[0] + b * z[1] + c * z[2]
    sub = zbuf[y0:y1 + 1, x0:x1 + 1]
    hit = (a >= -1e-6) & (b >= -1e-6) & (c >= -1e-6) & (zz > sub)
    sub[hit] = zz[hit]; img[y0:y1 + 1, x0:x1 + 1][hit] = lum[i]; mask[y0:y1 + 1, x0:x1 + 1][hit] = True
rgba = np.zeros((H * SS, W * SS, 4), np.uint8)
v = np.clip(img, 0, 1) * 255
rgba[..., 0] = rgba[..., 1] = rgba[..., 2] = v
rgba[..., 3] = mask * 255
Image.fromarray(rgba, 'RGBA').resize((W, H), Image.LANCZOS).save(sys.argv[1]); print('saved', sys.argv[1], len(T), '個三角形, LiDAR x =', lx)
