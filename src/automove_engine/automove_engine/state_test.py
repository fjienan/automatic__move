#!/usr/bin/env python3
"""
测试状态管理系统的脚本。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class StateTestNode(Node):
    """状态测试节点。"""

    def __init__(self):
        super().__init__('state_test_node')
        
        # 创建命令发布器
        self.command_publisher = self.create_publisher(String, 'robot_command', 10)
        
        # 创建状态订阅器
        self.state_subscriber = self.create_subscription(
            String, 'robot_state', self.state_callback, 10)
        
        self.current_state = None
        self.get_logger().info('状态测试节点已启动')

    def state_callback(self, msg):
        """状态消息回调函数。"""
        self.current_state = msg.data
        self.get_logger().info(f'当前状态: {self.current_state}')

    def send_command(self, command):
        """发送命令。"""
        msg = String()
        msg.data = command
        self.command_publisher.publish(msg)
        self.get_logger().info(f'发送命令: {command}')


def main(args=None):
    """主函数。"""
    rclpy.init(args=args)
    
    test_node = StateTestNode()
    
    # 等待节点初始化
    time.sleep(1)
    
    try:
        # 测试序列
        commands = [
            ('move', 3),      # 移动3秒
            ('stop', 2),      # 停止2秒
            ('navigate', 5),  # 导航5秒
            ('obstacle', 2),  # 模拟障碍物2秒
            ('idle', 2)       # 空闲2秒
        ]
        
        for command, duration in commands:
            test_node.send_command(command)
            
            # 旋转节点指定时间
            start_time = time.time()
            while time.time() - start_time < duration:
                rclpy.spin_once(test_node, timeout_sec=0.1)
        
        test_node.get_logger().info('测试完成')
        
    except KeyboardInterrupt:
        test_node.get_logger().info('测试中断')
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
