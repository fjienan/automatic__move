#!/usr/bin/env python3
"""
ROS2 状态管理节点，用于机器人状态转换。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from .state import State
from .states import IdleState


class StateManagerNode(Node):
    """管理机器人状态转换的 ROS2 节点。"""

    def __init__(self):
        """初始化状态管理器节点。"""
        super().__init__('state_manager_node')
        
        # 状态管理
        self.current_state = None
        self.state_context = {}  # 状态间共享数据

        # ROS2 发布器和订阅器
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state_publisher = self.create_publisher(String, 'robot_state', 10)
        self.command_subscriber = self.create_subscription(
            String, 'robot_command', self.command_callback, 10)
        
        # 定时器用于状态更新
        self.timer = self.create_timer(0.1, self.update_state)  # 10Hz 更新频率
        
        # 在 context 中存储 ROS2 相关对象
        self.state_context['node'] = self
        self.state_context['cmd_vel_pub'] = self.cmd_vel_publisher
        self.state_context['logger'] = self.get_logger()
        self.state_context['state_publisher'] = self.state_publisher

        self.get_logger().info('状态管理器节点已启动')

    def set_initial_state(self, state=None):
        """
        设置机器人的初始状态。

        Args:
            state (State): 初始状态。如果为 None，默认为 IdleState。
        """
        if state is None:
            state = IdleState(self.state_context)

        self.transition_to(state)

    def transition_to(self, new_state):
        """
        转换到新状态。

        Args:
            new_state (State): 要转换到的状态。
        """
        if self.current_state:
            self.current_state.exit()

        self.current_state = new_state
        self.current_state.state_context = self.state_context
        self.current_state.enter()
        
        # # 发布状态变化
        # state_msg = String()
        # state_msg.data = type(new_state).__name__
        # self.state_publisher.publish(state_msg)
        # self.get_logger().info(f'状态转换到: {type(new_state).__name__}')

    def update_state(self):
        """
        更新当前状态。
        这个方法由定时器定期调用来执行状态逻辑。
        """
        if self.current_state:
            self.get_logger().debug(f'当前状态: {type(self.current_state).__name__}')
            next_state = self.current_state.execute()
            if next_state and next_state != self.current_state:
                self.transition_to(next_state)
        else:
            self.get_logger().warn('当前没有活动状态。')
    def command_callback(self, msg):
        """
        处理外部命令的回调函数。

        Args:
            msg (String): 接收到的命令消息。
        """
        command = msg.data
        self.get_logger().info(f'收到命令: {command}')
        
        if self.current_state:
            next_state = self.current_state.handle_event(command)
            if next_state and next_state != self.current_state:
                self.transition_to(next_state)

    def get_current_state(self):
        """
        获取当前状态。

        Returns:
            State: 当前状态。
        """
        return self.current_state


def main(args=None):
    """主函数，启动状态管理器节点。"""
    rclpy.init(args=args)
    
    # 创建状态管理器节点
    state_manager_node = StateManagerNode()
    
    # 设置初始状态
    state_manager_node.set_initial_state()
    
    try:
        rclpy.spin(state_manager_node)
    except KeyboardInterrupt:
        state_manager_node.get_logger().info('正在关闭状态管理器节点...')
    finally:
        state_manager_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()