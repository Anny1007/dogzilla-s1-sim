#!/usr/bin/env python3
"""LiDAR 地面點過濾: /scan + /imu/data -> /scan_filtered

四足步態走路時機身俯仰/側滾可達 6~8°, 2D LiDAR 的掃描平面跟著傾斜, 在幾公尺外打到地面,
SLAM 與代價地圖就會出現不存在的障礙物。這個節點依 IMU 的 roll/pitch 算出每條射線會在多遠
碰到地面, 回波距離超過 (地面交點 * MARGIN) 的點視為地面, 改成 inf (無回波)。
一圈掃描要 0.1 s, 期間機身還在晃, 所以取最近 history 秒內所有 IMU 姿態中最先碰到地面的那個 (保守)。
機身水平時所有射線都不會碰到地面, 掃描原封不動。實機一樣可用 (DOGZILLA 有 IMU)。

參數:
  scan_height  掃描平面離地高度 (m), 模擬中約 0.39
  margin       地面交點距離的安全係數, 回波 > 交點 * margin 即丟棄。0.5 相當於把離地低於
               (1 - 0.5) * scan_height 的點都當地面; 掃描平面低時射線幾乎貼地擦過, 傾角低估 1° 在 1.5 m 外
               就差 2.6 cm, 所以要保守 (S1 掃描平面只有約 0.17 m)
  history      取多久內的 IMU 姿態 (s)
  self_box     機身自身遮擋區 [x_min, x_max, y_half] (m, laser_link 座標), 落在裡面的回波丟棄。
               S1 的 LiDAR 射線原點只比機身頂高約 2 cm, 又在機身前緣後方 0.13 m, 機頭朝下超過約 9° 時
               掃描平面就打到自己的機身前緣 (0.12~0.13 m 的回波), 代價地圖把機器狗自己標成障礙物,
               Nav2 規劃失敗、恢復行為回報 Collision Ahead。預設 [0, 0, 0] = 不過濾。
"""
import collections
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan


class ScanGroundFilter(Node):
    def __init__(self):
        super().__init__('scan_ground_filter')
        self.h = self.declare_parameter('scan_height', 0.39).value
        self.margin = self.declare_parameter('margin', 0.5).value
        self.history = self.declare_parameter('history', 0.3).value
        self.declare_parameter('self_box', [0.0, 0.0, 0.0])
        self.tilts = collections.deque()   # (時間戳 s, roll, pitch)
        self.dropped = self.self_hits = self.total = 0
        self.create_subscription(Imu, '/imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(LaserScan, '/scan_filtered', qos_profile_sensor_data)
        self.create_timer(30.0, self.report)

    def on_imu(self, m):
        q = m.orientation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.tilts.append((t, roll, pitch))
        while self.tilts and self.tilts[0][0] < t - self.history:
            self.tilts.popleft()

    def on_scan(self, m):
        # 每次重讀參數, 可用 ros2 param set 即時調整
        self.h = self.get_parameter('scan_height').value
        self.margin = self.get_parameter('margin').value
        self.history = self.get_parameter('history').value
        r = np.array(m.ranges, dtype=np.float32)
        a = m.angle_min + m.angle_increment * np.arange(len(r))
        x0, x1, yh = self.get_parameter('self_box').value
        with np.errstate(invalid='ignore'):
            body = (r * np.cos(a) > x0) & (r * np.cos(a) < x1) & (np.abs(r * np.sin(a)) < yh)
        self.self_hits += int(body.sum())
        r[body] = np.inf
        m.ranges = r.tolist()
        # 射線在機身座標 (cos a, sin a, 0), 經 roll(x 軸) 再 pitch(y 軸) 旋轉後的世界 z 分量:
        #   z = -sin(pitch) * cos(a) + cos(pitch) * sin(roll) * sin(a)
        # (pitch > 0 = 機頭朝下, REP-103)
        if not self.tilts:
            self.pub.publish(m)
            return
        _, roll, pitch = np.array(self.tilts).T[:, :, None]      # 每個歷史姿態一列, 對所有射線廣播
        dz = (-np.sin(pitch) * np.cos(a) + np.cos(pitch) * np.sin(roll) * np.sin(a)).min(axis=0)
        with np.errstate(divide='ignore'):
            ground = np.where(dz < -1e-3, self.h / -dz, np.inf)
        hit_ground = np.isfinite(r) & (r > ground * self.margin)
        self.dropped += int(hit_ground.sum()); self.total += len(r)
        r[hit_ground] = np.inf
        m.ranges = r.tolist()
        self.pub.publish(m)

    def report(self):
        if self.total:
            self.get_logger().info(f'過去 30 秒濾掉 {100.0 * self.dropped / self.total:.1f}% 的回波 (判定為地面), '
                                   f'{100.0 * self.self_hits / self.total:.1f}% (打到自己機身)')
        self.dropped = self.self_hits = self.total = 0


def main():
    rclpy.init()
    rclpy.spin(ScanGroundFilter())


if __name__ == '__main__':
    main()
