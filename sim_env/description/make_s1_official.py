#!/usr/bin/env python3
"""由 Yahboom 官方 DOGZILLA S1 URDF 產生模擬用模型 dogzilla_s1_official.urdf.xacro

官方模型: vendor/Program/yahboomcar_ws_ros2/src/champ/champ_description/urdf/xgo_rviz.xacro
(SolidWorks 1:1 匯出, Yahboom 教材「Rviz simulation in ros2 environment」用的就是它, 網格在 meshes/XGO/)

沿用官方的部分 (不更動): 所有連桿的外觀網格、關節位置 (origin)、質量、質心與慣量 (右側髖部除外, 見 7)。
為了能在 Gazebo + CHAMP 裡模擬而做的修正:
  1. 左前膝關節名稱打錯 (官方寫成 lf_lower_leg_link, 與連桿同名), 改為 lf_lower_leg_joint。
  2. 關節旋轉軸正負號統一成 CHAMP 的慣例 (髖 +x, 大腿/膝 +y)。官方各腿方向不一 (例如前腿髖 -x、後腿 +x),
     CHAMP 的逆運動學假設四條腿同向, 不統一的話步態會亂掉。只是角度正負號的定義, 機構本身不變。
  3. 加上腳掌座標系 (foot_link): 前後/高度取小腿網格的腳底中心; 側向則放在大腿關節的平面上 (y 位移 0),
     而不是網格裡真正的腳 (往內 18 mm)。CHAMP 假設小腿與腳掌和大腿關節同一平面, 站姿只算到「髖 + 大腿」的
     側向位移; 若 foot_link 照實放在內側 18 mm, CHAMP 的逆運動學會用髖關節往外轉來補, 四條腿撇開約 10°
     (實測腳掌在 ±0.072 m, 應為 ±0.053 m)。腳掌的碰撞 box 仍在網格的真實位置, 物理接觸點不變。
  4. 碰撞形狀由 STL 改為 box (依各網格實際範圍): STL 碰撞會讓 gzserver 段錯誤 (LiDAR 射線與 trimesh,
     laser_link.STL 有 5 萬多個三角形)。LiDAR / 相機外殼不設碰撞。小腿用斜的 box + 平底腳掌 box (球形腳會打滑)。
  5. 關節上限改為實機舵機規格 (官方填的 effort 25 / velocity 1.5 是預設值): 0.44 N·m (4.5 kg·cm)、10.5 rad/s (0.1 s/60°);
     並加上關節阻尼 0.01 N·m·s/rad (ODE implicitSpringDamper 隱式求解), 抑制輕量腿上的 PID 顫振 (config/ros_control_s1.yaml)。
  6. 加上 Gazebo 需要的東西: LiDAR / IMU / (可選) 相機感測器、ros2_control、真值外掛、腳掌摩擦,
     以及以站姿生成用的關節初始角 (initial_value)。
  7. 右側髖部 (rf/rh_hip_link) 的質量與慣量改成與左側鏡像相同。官方右側髖部是簡化過的網格 (三角形數只有左側一半),
     質量 9.8 g 只有左側 19.6 g 的一半; 實機左右對稱。不修的話 0.2 m/s 前進加左轉時機身傾斜達 30°, 右轉只有 7°
     (tools/combo_test.py), 修正後左右都在 8.4° 內。
  8. 質量改為實機重量: S1 本體 (含相機, 不含 LiDAR) 共 875 g (實機秤重約 870~880 g), 加裝的 DOGZILLA S2 LiDAR 45 g。
     官方 URDF 的本體只有 611 g (沒算進電池等), 各連桿的質量與慣量等比例放大到 875 g (質心位置不變);
     LiDAR 連桿 (官方 14 g) 的質量與慣量放大到 45 g。
  9. LiDAR 安裝位置 (LIDAR_X): 見下方常數的說明。

用法 (在 sim_env/description/ 下): python3 make_s1_official.py
"""
import math
import os

import numpy as np
import re
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
OFFICIAL = os.path.join(HERE, '../../vendor/Program/yahboomcar_ws_ros2/src/champ/champ_description/urdf/xgo_rviz.xacro')
OUT = os.path.join(HERE, 'dogzilla_s1_official.urdf.xacro')

SERVO_EFFORT, SERVO_VEL = 0.44, 10.5
JOINT_DAMPING = float(os.environ.get('S1_JOINT_DAMPING', 0.01))   # N·m·s/rad, ODE 隱式求解 (implicitSpringDamper)
S1_MASS = float(os.environ.get('S1_MASS', 0.875))        # kg, S1 本體 (不含 LiDAR), 實機秤重約 870~880 g
LIDAR_MASS = float(os.environ.get('S1_LIDAR_MASS', 0.045))   # kg, DOGZILLA S2 的 LiDAR
# LiDAR 前後位置 (laser_link 原點在 base_link 的 x, m; 官方 URDF 是 -0.016732, 即上層平台的最後緣)。
# 站姿下本體質心在四腳支撐中心前方約 17 mm (tools/com_check.py), 所以 LiDAR 越往後放, 整機質心越接近支撐中心。
# Gazebo 掃描 (tools/tune_sweep.sh, results/sweeps/tune_875g_round2_lidar_x.txt 與 round3):
#   比官方位置更前面走路會翻倒, 官方位置 4 次測試有 1 次大幅晃動 (28 度); -0.04 ~ -0.07 都穩 (前進晃動 RMS 約 2.2~2.6 度, 與沒裝 LiDAR 時的 1.9 度接近),
#   差異小於量測雜訊; -0.086 (貼齊機尾) 後腿膝關節扭力飽和升到 21%。
# 取平台區中間的 -0.06: 膝關節飽和最低 (0~11%), 安裝誤差 1~2 cm 也不影響。左右置中 (y 沿用官方 0), 高度沿用官方
# (LiDAR 底面 z = 68 mm = 上層平台高度, 掃描平面 z = 93 mm 高於相機外殼 80 mm, 再低前方會被機身擋住)。
# 這個位置在機身後段較低的平台 (z = 36 mm) 上方, 實機需要約 32 mm 高的支架。
LIDAR_X = float(os.environ.get('S1_LIDAR_X', -0.06))
STAND_H = float(os.environ.get('S1_STAND_H', 0.105))            # 生成時的站高 (髖關節到腳底), 須與 config/gait_s1.yaml 的 nominal_height 一致
LEGS = ['lf', 'rf', 'lh', 'rh']

# 網格範圍 (m, 各連桿座標系), 由 meshes/XGO/*.STL 量得
BASE_BOX = ((-0.1061, -0.0302, -0.0018), (0.1029, 0.0302, 0.0801))
HIP_BOX = {'l': ((-0.0366, -0.0047, -0.0075), (0.0, 0.0493, 0.0222)),
           'r': ((-0.0366, -0.0493, -0.0075), (0.0, 0.0047, 0.0222))}
UPPER_BOX = {'l': ((-0.013, -0.0312, -0.0657), (0.0096, -0.0062, 0.0059)),
             'r': ((-0.013, 0.0062, -0.0657), (0.0096, 0.0312, 0.0059))}
FOOT_X, FOOT_Y, FOOT_Z = -0.0226, 0.0183, -0.0815     # 腳底中心 (左腿 y 取負)


def box_xml(lo, hi):
    c = [(a + b) / 2 for a, b in zip(lo, hi)]
    s = [b - a for a, b in zip(lo, hi)]
    return (f'<collision><origin xyz="{c[0]:.5f} {c[1]:.5f} {c[2]:.5f}" rpy="0 0 0" />'
            f'<geometry><box size="{s[0]:.5f} {s[1]:.5f} {s[2]:.5f}" /></geometry></collision>')


def lower_leg_collisions(side):
    y = -FOOT_Y if side == 'l' else FOOT_Y
    # 小腿: 從膝關節斜向腳掌, 底端留 1.5 cm 給平底腳掌
    fx, fz = FOOT_X, FOOT_Z + 0.015
    length = math.hypot(fx, fz)
    pitch = math.atan2(-fx, -fz)       # 繞 y 軸轉 pitch 後, box 的 z 軸沿著 (膝 -> 腳掌) 方向
    shin = (f'<collision><origin xyz="{fx/2:.5f} {y:.5f} {fz/2:.5f}" rpy="0 {pitch:.5f} 0" />'
            f'<geometry><box size="0.012 0.012 {length:.5f}" /></geometry></collision>')
    foot = (f'<collision name="foot"><origin xyz="{FOOT_X:.5f} {y:.5f} {FOOT_Z + 0.009:.5f}" rpy="0 0 0" />'
            f'<geometry><box size="0.016 0.012 0.018" /></geometry></collision>')
    return shin + foot


def stand_angles(upper_xyz, knee_xyz):
    """腳掌在大腿關節正下方 STAND_H 處 (x 與零角度時相同) 的大腿/膝關節角, 膝蓋朝後 (CHAMP '>>')。"""
    ux, uz = upper_xyz[0], upper_xyz[2]
    kx, kz = knee_xyz[0], knee_xyz[2]
    zero_foot_x = ux + kx + FOOT_X
    tu, tl = np.meshgrid(np.arange(0, 1.6, 0.0005), -np.arange(0, 2.4, 0.0005), indexing='ij')
    rot = lambda x, z, t: (x * np.cos(t) + z * np.sin(t), -x * np.sin(t) + z * np.cos(t))   # 繞 +y 軸
    k = rot(kx, kz, tu); f = rot(FOOT_X, FOOT_Z, tu + tl)
    e = (ux + k[0] + f[0] - zero_foot_x) ** 2 + (uz + k[1] + f[1] + STAND_H) ** 2
    i = np.unravel_index(np.argmin(e), e.shape)
    assert e[i] < 1e-7, '找不到站姿解'
    return float(tu[i]), float(tl[i])


def main():
    src = open(OFFICIAL, encoding='utf-8').read()
    src = src.replace('${-pi}', '-3.14159').replace('${pi}', '3.14159')
    src = re.sub(r'<xacro:property[^>]*/>', '', src)
    root = ET.fromstring(src.encode('utf-8'))

    # --- 1. 關節更名 / 2. 軸 / 5. 上限 ---
    joints = {j.get('name'): j for j in root.findall('joint')}
    joints['lf_lower_leg_link'].set('name', 'lf_lower_leg_joint')
    joints = {j.get('name'): j for j in root.findall('joint')}
    for leg in LEGS:
        for part, axis in [('hip', '1 0 0'), ('upper_leg', '0 1 0'), ('lower_leg', '0 1 0')]:
            j = joints[f'{leg}_{part}_joint']
            j.find('axis').set('xyz', axis)
            lim = j.find('limit')
            lim.set('effort', str(SERVO_EFFORT)); lim.set('velocity', str(SERVO_VEL))
            lim.set('lower', '-3.14159'); lim.set('upper', '3.14159')
            ET.SubElement(j, 'dynamics', damping=str(JOINT_DAMPING))

    # --- 7. 左右髖部質量對稱化 (設 S1_SYMMETRIC_HIP=0 可關閉) ---
    if os.environ.get('S1_SYMMETRIC_HIP', '1') == '1':
        lf = root.find("link[@name='lf_hip_link']/inertial")
        for leg in ('rf', 'rh'):
            inert = root.find(f"link[@name='{leg}_hip_link']/inertial")
            inert.find('mass').set('value', lf.find('mass').get('value'))
            for k, v in lf.find('inertia').attrib.items():
                inert.find('inertia').set(k, v if k not in ('ixy', 'iyz') else str(-float(v)))
            o = lf.find('origin').get('xyz').split()
            inert.find('origin').set('xyz', f"{o[0]} {-float(o[1])} {o[2]}")

    # --- 8. 質量 / 9. LiDAR 位置 ---
    inertials = {l.get('name'): l.find('inertial') for l in root.findall('link') if l.find('inertial') is not None}
    def scale(inert, k):
        inert.find('mass').set('value', f"{float(inert.find('mass').get('value')) * k:.6g}")
        for a, v in inert.find('inertia').attrib.items():
            inert.find('inertia').set(a, f'{float(v) * k:.6g}')
    body_mass = sum(float(i.find('mass').get('value')) for n, i in inertials.items() if n != 'laser_link')
    for n, i in inertials.items():
        scale(i, LIDAR_MASS / float(i.find('mass').get('value')) if n == 'laser_link' else S1_MASS / body_mass)
    o = joints['laser_Joint'].find('origin')
    xyz = o.get('xyz').split()
    o.set('xyz', f'{LIDAR_X} {xyz[1]} {xyz[2]}')

    # --- 4. 碰撞 ---
    links = {l.get('name'): l for l in root.findall('link')}
    for l in links.values():
        for c in l.findall('collision'):
            l.remove(c)
    def add(link, xml):
        for e in ET.fromstring(f'<x>{xml}</x>'):
            links[link].append(e)
    add('base_link', box_xml(*BASE_BOX))
    for leg in LEGS:
        side = 'l' if leg in ('lf', 'lh') else 'r'
        add(f'{leg}_hip_link', box_xml(*HIP_BOX[side]))
        add(f'{leg}_upper_leg_link', box_xml(*UPPER_BOX[side]))
        add(f'{leg}_lower_leg_link', lower_leg_collisions(side))

    # 官方的 joint_state_publisher 外掛保留, 其他 Gazebo 東西在下面另外加
    body = ET.tostring(root, encoding='unicode')
    body = body[body.index('>') + 1: body.rindex('</robot>')]

    # --- 3. 腳掌 / 6. Gazebo ---
    extra = []
    stand = {}
    for leg in LEGS:
        side = 'l' if leg in ('lf', 'lh') else 'r'
        y = -FOOT_Y if side == 'l' else FOOT_Y
        extra.append(f'''
  <link name="{leg}_foot_link" />
  <joint name="{leg}_foot_joint" type="fixed">
    <parent link="{leg}_lower_leg_link" /><child link="{leg}_foot_link" />
    <origin xyz="{FOOT_X} 0 {FOOT_Z}" rpy="0 0 0" />
  </joint>
  <gazebo reference="{leg}_lower_leg_link">
    <mu1>1.5</mu1><mu2>1.5</mu2><kp>1000000.0</kp><kd>50.0</kd><minDepth>0.001</minDepth><maxVel>0.1</maxVel>
  </gazebo>''')
        u = [float(v) for v in joints[f'{leg}_upper_leg_joint'].find('origin').get('xyz').split()]
        k = [float(v) for v in joints[f'{leg}_lower_leg_joint'].find('origin').get('xyz').split()]
        stand[leg] = stand_angles(u, k)
    for leg in LEGS:
        tu, tl = stand[leg]
        for part, init in [('hip', 0.0), ('upper_leg', tu), ('lower_leg', tl)]:
            extra.append(f'''
  <gazebo reference="{leg}_{part}_joint"><implicitSpringDamper>1</implicitSpringDamper></gazebo>''')
            extra.append(f'''
  <ros2_control name="{leg}_{part}_joint" type="system">
    <hardware><plugin>gazebo_ros2_control/GazeboSystem</plugin></hardware>
    <joint name="{leg}_{part}_joint">
      <command_interface name="effort" />
      <state_interface name="position"><param name="initial_value">{init:.4f}</param></state_interface>
      <state_interface name="velocity" />
    </joint>
  </ros2_control>''')

    header = f'''<?xml version="1.0"?>
<!-- 由 make_s1_official.py 自動產生, 請勿手動修改 (改產生器後重跑)。
  來源 Yahboom 官方 DOGZILLA S1 URDF (champ_description/urdf/xgo_rviz.xacro, SolidWorks 1:1 匯出)。
  修正內容與原因見 make_s1_official.py 開頭說明。生成站姿 (站高 {STAND_H} m) 大腿 {stand["lf"][0]:.3f} rad, 膝 {stand["lf"][1]:.3f} rad。
  注意：註解內不要用「半形冒號+空格」, robot_description 會被 launch 當 YAML 解析而失敗。 -->
<robot name="champ" xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:arg name="camera" default="false" />
'''
    gazebo = '''
  <!-- ===== Gazebo 感測器與外掛 ===== -->
  <gazebo reference="base_link"><material>Gazebo/Grey</material></gazebo>
  <!-- LiDAR (官方 laser_link, 網格頂面在 z=0, 射線從頂面下 1 cm 的掃描頭發出) -->
  <gazebo reference="laser_link">
    <sensor name="laser_sensor" type="ray">
      <always_on>true</always_on><visualize>true</visualize><update_rate>10</update_rate>
      <pose>0 0 -0.01 0 0 0</pose>
      <ray>
        <scan><horizontal><samples>360</samples><resolution>1</resolution><min_angle>0</min_angle><max_angle>6.28</max_angle></horizontal></scan>
        <range><min>0.12</min><max>3.5</max><resolution>0.015</resolution></range>
        <noise><type>gaussian</type><mean>0.0</mean><stddev>0.01</stddev></noise>
      </ray>
      <plugin name="laserscan" filename="libgazebo_ros_ray_sensor.so">
        <ros><remapping>~/out:=scan</remapping></ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>laser_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>
  <xacro:if value="$(arg camera)">
    <gazebo reference="camera_link">
      <sensor name="camera" type="camera">
        <update_rate>15</update_rate>
        <camera><horizontal_fov>1.8</horizontal_fov><image><width>640</width><height>480</height></image><clip><near>0.02</near><far>10</far></clip></camera>
        <plugin name="camera_plugin" filename="libgazebo_ros_camera.so"><camera_name>cam</camera_name><frame_name>camera_link</frame_name></plugin>
      </sensor>
    </gazebo>
  </xacro:if>
  <!-- IMU (官方模型沒有 imu_link, 加在機身質心附近) -->
  <link name="imu_link"><inertial><mass value="0.001" /><inertia ixx="1e-8" ixy="0" ixz="0" iyy="1e-8" iyz="0" izz="1e-8" /></inertial></link>
  <joint name="imu_joint" type="fixed"><parent link="base_link" /><child link="imu_link" /><origin xyz="0 0 0.028" /></joint>
  <gazebo reference="imu_link">
    <sensor name="yahboom_imu" type="imu">
      <always_on>true</always_on><update_rate>100</update_rate>
      <imu>
        <angular_velocity>
          <x><noise type="gaussian"><mean>0</mean><stddev>2e-4</stddev></noise></x>
          <y><noise type="gaussian"><mean>0</mean><stddev>2e-4</stddev></noise></y>
          <z><noise type="gaussian"><mean>0</mean><stddev>2e-4</stddev></noise></z>
        </angular_velocity>
        <linear_acceleration>
          <x><noise type="gaussian"><mean>0</mean><stddev>1.7e-2</stddev></noise></x>
          <y><noise type="gaussian"><mean>0</mean><stddev>1.7e-2</stddev></noise></y>
          <z><noise type="gaussian"><mean>0</mean><stddev>1.7e-2</stddev></noise></z>
        </linear_acceleration>
      </imu>
      <plugin name="yahboom_imu_plugin" filename="libgazebo_ros_imu_sensor.so">
        <initial_orientation_as_reference>false</initial_orientation_as_reference>
        <frame_name>imu_link</frame_name>
        <ros><namespace>/imu</namespace><remapping>~/out:=data</remapping></ros>
      </plugin>
    </sensor>
  </gazebo>
  <gazebo>
    <!-- 真值位姿 (只給 benchmark 評分與 RL 節點用) -->
    <plugin name="p3d_base_controller" filename="libgazebo_ros_p3d.so">
      <ros><namespace>/demo</namespace><argument>odom:=odom/ground_truth</argument></ros>
      <body_name>base_link</body_name><frame_name>world</frame_name>
      <update_rate>10.0</update_rate><gaussian_noise>0.01</gaussian_noise>
    </plugin>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <parameters>/config/ros_control_s1.yaml</parameters>
    </plugin>
  </gazebo>
'''
    out = header + body + ''.join(extra) + gazebo + '</robot>\n'
    # robot_description 會被當 YAML 解析: 官方註解/內容裡的 ": " 換成全形
    out = re.sub(r'<!--.*?-->', lambda m: m.group(0).replace(': ', '：'), out, flags=re.S)
    open(OUT, 'w', encoding='utf-8').write(out)
    print('寫出', OUT)
    for leg in LEGS:
        print(f'  {leg} 站姿 大腿 {stand[leg][0]:+.3f} rad, 膝 {stand[leg][1]:+.3f} rad')


if __name__ == '__main__':
    main()
