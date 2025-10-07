#!/usr/bin/env python3
"""
机器人状态管理的基础状态类。
"""

class State:
    """所有机器人状态的基类。"""

    def __init__(self, context=None):
        """
        初始化状态。

        Args:
            context: 上下文对象，保存状态间共享的数据
        """
        self.context = context
        # 🚨 确保这里从 context 中获取了 logger 和 cmd_vel_pub
        if self.context:
            self.node = self.context.get('node')
            self.logger = self.context.get('logger')
            self.state_publisher = self.context.get('node').state_publisher
            self.cmd_vel_pub = self.context.get('cmd_vel_pub')
        else:
            # 防止 context 为空时后续代码出错
            self.logger = None 
            self.cmd_vel_pub = None
            self.state_publisher = None

    def enter(self):
        """
        进入此状态时调用。
        在子类中重写此方法来执行初始化操作。
        """
        pass

    def execute(self):
        """
        执行此状态的主要逻辑。
        在子类中重写此方法来实现状态行为。

        Returns:
            State: 要转换到的下一个状态，或 None 表示保持当前状态
        """
        return None

    def exit(self):
        """
        退出此状态时调用。
        在子类中重写此方法来执行清理操作。
        """
        pass

    def handle_event(self, event):
        """
        处理在此状态下发生的事件。
        在子类中重写此方法来处理特定事件。

        Args:
            event: 要处理的事件

        Returns:
            State: 要转换到的下一个状态，或 None 表示保持当前状态
        """
        return None