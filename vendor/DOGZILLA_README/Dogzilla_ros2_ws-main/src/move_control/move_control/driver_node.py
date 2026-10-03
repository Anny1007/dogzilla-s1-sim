#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from DOGZILLALib import DOGZILLA

class DogzillaDriverNode(Node):
    def __init__(self):
        super().__init__('driver_node')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # 初始化機器狗硬體
        self.dog = DOGZILLA()
        self.get_logger().info('DOGZILLA 硬體驅動節點已啟動，等待指令...')

    def cmd_vel_callback(self, msg):
        # 每次收到新指令時，先停止目前的動作
        self.dog.stop()

        # 根據收到的 Twist 訊息，決定呼叫哪個硬體函式
        if msg.linear.x > 0:
            self.dog.forward(10)
        elif msg.linear.x < 0:
            self.dog.back(10)
        elif msg.linear.y > 0:
            self.dog.left(10)
        elif msg.linear.y < 0:
            self.dog.right(10)
        elif msg.angular.z > 0:
            self.dog.turnleft(10)
        elif msg.angular.z < 0:
            self.dog.turnright(10)
        else:
            # 如果所有數值都是 0 (對應按鍵 'x')，就保持停止
            pass

def main(args=None):
    rclpy.init(args=args)
    node = DogzillaDriverNode()
    try:
        rclpy.spin(node) # 讓節點保持運行，持續監聽主題
    except KeyboardInterrupt:
        pass
    finally:
        node.dog.stop() # 程式結束前確保機器狗停止
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()