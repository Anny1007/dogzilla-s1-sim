#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

class KeyboardCtrlNode(Node):
    def __init__(self):
        super().__init__('keyboard_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.get_logger().info('鍵盤控制節點已啟動！請使用 W/A/S/D/Q/E/X 控制，按 Ctrl+C 退出。')

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        msg = Twist()
        try:
            while rclpy.ok():
                key = self.get_key()
                
                # 判斷按鍵並設定對應的線速度 (linear) 或角速度 (angular)
                if key == "w":
                    msg.linear.x = 1.0; msg.linear.y = 0.0; msg.angular.z = 0.0
                elif key == "s":
                    msg.linear.x = -1.0; msg.linear.y = 0.0; msg.angular.z = 0.0
                elif key == "a":
                    msg.linear.x = 0.0; msg.linear.y = 1.0; msg.angular.z = 0.0
                elif key == "d":
                    msg.linear.x = 0.0; msg.linear.y = -1.0; msg.angular.z = 0.0
                elif key == "q":
                    msg.linear.x = 0.0; msg.linear.y = 0.0; msg.angular.z = 1.0
                elif key == "e":
                    msg.linear.x = 0.0; msg.linear.y = 0.0; msg.angular.z = -1.0
                elif key == "x":
                    msg.linear.x = 0.0; msg.linear.y = 0.0; msg.angular.z = 0.0
                elif key == "\x03":  # Ctrl+C
                    break
                else:
                    continue # 如果按了其他鍵，就不發布

                self.publisher_.publish(msg)
                
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardCtrlNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()