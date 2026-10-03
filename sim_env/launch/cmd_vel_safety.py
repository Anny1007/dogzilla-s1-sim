#!/usr/bin/env python3
"""運動安全濾波: /cmd_vel_raw (Nav2 / RL) -> /cmd_vel (CHAMP 步態控制器)

官方 DOGZILLA S1 模型 + CHAMP 步態實測 (tools/gait_eval.py, tools/flip_monitor.py):
  * 邊走邊轉幾乎走不動: 指令 0.15 m/s + 0.5 rad/s 時實際前進只有約 0.034 m/s。
  * DWB 不知道這點, 會一直送「0.2 m/s + 大角速度」並在 +0.8 / -0.66 rad/s 之間來回修正,
    機器狗原地踏步, 最後在高速中反轉轉向時翻倒 (傾斜 9° -> 21° -> 40° -> 85°, 0.6 s 內)。
所以在指令進步態控制器前:
  1. 轉彎降速: v *= max(MIN_SCALE, 1 - |w| / W_FULL_STOP)  (反正邊轉邊走也走不動, 先轉再走較穩)
  2. 限制角速度變化率 W_ACC (rad/s²), 避免高速中急速反轉
  3. 前進速度上限 V_MAX (與 config/gait_s1.yaml 一致)
實機一樣適用: 把 driver_node 訂閱的話題接到本節點輸出即可。
"""
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelSafety(Node):
    def __init__(self):
        super().__init__('cmd_vel_safety')
        self.v_max = self.declare_parameter('v_max', 0.20).value
        self.w_full_stop = self.declare_parameter('w_full_stop', 1.0).value
        self.min_scale = self.declare_parameter('min_scale', 0.15).value
        self.w_acc = self.declare_parameter('w_acc', 1.5).value
        self.w_out, self.t_last = 0.0, None
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Twist, '/cmd_vel_raw', self.on_cmd, 10)

    def on_cmd(self, m):
        now = time.monotonic()
        dt = 0.05 if self.t_last is None else min(max(now - self.t_last, 0.0), 0.1)
        self.t_last = now
        w = m.angular.z
        self.w_out += max(-self.w_acc * dt, min(self.w_acc * dt, w - self.w_out))
        if w == 0.0 and m.linear.x == 0.0 and abs(self.w_out) < 0.05:
            self.w_out = 0.0                                  # 停止指令時直接歸零
        scale = max(self.min_scale, 1.0 - abs(self.w_out) / self.w_full_stop)
        out = Twist()
        out.linear.x = max(-self.v_max, min(self.v_max, m.linear.x)) * scale
        out.linear.y = m.linear.y * scale
        out.angular.z = self.w_out
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(CmdVelSafety())


if __name__ == '__main__':
    main()
