#!/usr/bin/env python3
"""
机器人行为的具体状态类。
"""

from geometry_msgs.msg import Twist
from std_msgs.msg import String # 引入 String 用于发布移动命令
from .state import State
import sys
import re
# -------------------------- 状态定义和路点 --------------------------

# 状态列表 (用于参考)
state_list = ["idle", "attain_spearhead", "spear_comb","forest", "conflict", "2robot_comb"]

# 目标路点字典 (供 AutoMoveEngine 使用)
state_position = {
    "attain_spearhead": [[5.2, 1.0]],
    "spear_comb": [[5.2, 0.5], [3.5, 0.5]],
    "forest": [[3.5, 2.0],[3.0,2.0],[3.0,8.7]],
    "conflict": [[0.5, 8.7],[0.5, 11.3],[5.0, 11.3]],
}

# -------------------------- 状态类字典 --------------------------
# 用于 MoveToNextStateWaypoint 动态查找下一个状态类
# 我们将把所有状态类添加到这个字典中
STATE_CLASSES = {}

# -------------------------- 通用移动过渡状态 --------------------------
# 辅助函数：将 CamelCase 转换为 snake_case
def camel_to_snake(name):
    name = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
    return re.sub('([a-z0-9])([A-Z])', r'\1_\2', name).lower()

class MoveToNextStateWaypoint(State):
    """
    通用移动过渡状态：
    负责将机器人移动到下一个目标状态（FinalState）对应的起始路点。
    """
    def __init__(self, context, final_state_name):
        super().__init__(context)
        self.final_state_name = final_state_name
        # 构造 AutoMoveEngine 识别的路点名称，使用目标状态名的小写
        # 例如：AttainSpearheadState -> 'attain_spearhead'
        name_without_state = final_state_name.replace('State', '')
        self.move_topic_name = camel_to_snake(name_without_state)
    def enter(self):
        super().enter()
        self.logger.info(f"进入移动过渡状态，目标是 {self.final_state_name} 的起始点。")
        
        # 1. 告诉 AutoMoveEngine 开始移动
        # 发布到 'robot_state' 话题，AutoMoveEngine 会查找对应的路点
        move_cmd_msg = String(data=self.move_topic_name) 
        self.state_publisher.publish(move_cmd_msg)
        
    def handle_event(self, event):
        """处理任务完成事件，然后切换到目标状态。"""
        
        # 检查是否收到 AutoMoveEngine 发来的完成信号（我们在 StateManagerNode 中定义为 'TASK_FINISHED'）
        if event == 'TASK_FINISHED' :
            self.logger.info(f"移动任务完成。正式切换到目标状态: {self.final_state_name}。")
            
            # 2. 动态加载并返回目标状态的实例
            TargetClass = STATE_CLASSES.get(self.final_state_name)
            if TargetClass:
                return TargetClass(self.context)
            else:
                self.logger.error(f"无法找到目标状态类: {self.final_state_name}。返回 IdleState。")
                return IdleState(self.context)
        
        return None
STATE_CLASSES['MoveToNextStateWaypoint'] = MoveToNextStateWaypoint

# -------------------------- 核心行为状态 --------------------------

class IdleState(State):
    """机器人空闲状态，等待命令。"""
    
    # 辅助函数：停止机器人并通知 AutoMoveEngine 进入 IDLE
    def _send_idle_cmd(self):
        cmd_vel_pub = self.context.get('cmd_vel_pub')
        if cmd_vel_pub:
            stop_msg = Twist()
            cmd_vel_pub.publish(stop_msg)
        
        # 通知 AutoMoveEngine 停止移动并清理其状态
        stop_cmd_msg = String(data='IDLE') 
        self.state_publisher.publish(stop_cmd_msg)

    def enter(self):
        super().enter()
        self.logger.info("进入空闲状态")
        self._send_idle_cmd()

    def execute(self):
        return None

    def exit(self):
        self.logger.info("退出空闲状态")

    def handle_event(self, event):
        """
        处理事件。
        在切换到任何任务状态前，都先切换到 **移动过渡状态**。
        """
        # 注意：这里我们使用状态类名作为目标名称
        if event == "attain_spearhead":
            return MoveToNextStateWaypoint(self.context, 'AttainSpearheadState')
        if event =="spear_comb":
            return MoveToNextStateWaypoint(self.context, 'SpearCombState')
        if event =="forest":
            return MoveToNextStateWaypoint(self.context, 'ForestState')
        if event =="conflict":
            return MoveToNextStateWaypoint(self.context, 'ConflictState')
        if event =="2robot_comb":
            return MoveToNextStateWaypoint(self.context, 'TwoRobotCombState')
        return None
STATE_CLASSES['IdleState'] = IdleState


class AttainSpearheadState(State):
    """机器人达到矛头状态。"""

    def enter(self):
        super().enter()
        self.logger.info("进入达到矛头状态。机器人已就位。开始执行任务...")
        # 注意：此时机器人已经移动到目标位置，无需再次发送停止或移动指令。

    def handle_event(self, event):
        """处理事件。"""
        # 假设任务完成后，收到 'TASK_DONE_AS' 事件
         # 注意：这里我们使用状态类名作为目标名称
        if event == "idle":
            return IdleState(self.context)
        if event =="spear_comb":
            return MoveToNextStateWaypoint(self.context, 'SpearCombState')
        if event =="forest":
            return MoveToNextStateWaypoint(self.context, 'ForestState')
        if event =="conflict":
            return MoveToNextStateWaypoint(self.context, 'ConflictState')
        if event =="2robot_comb":
            return MoveToNextStateWaypoint(self.context, 'TwoRobotCombState')
        return None
STATE_CLASSES['AttainSpearheadState'] = AttainSpearheadState


class SpearCombState(State):
    """机器人矛组合状态。"""

    def enter(self):
        super().enter()
        self.logger.info("进入矛组合状态。机器人已就位。开始执行任务...")

    def handle_event(self, event):
        """处理事件。"""
         # 注意：这里我们使用状态类名作为目标名称
        if event == "idle":
            return IdleState(self.context)
        if event == "attain_spearhead":
            return MoveToNextStateWaypoint(self.context, 'AttainSpearheadState')
        if event =="forest":
            return MoveToNextStateWaypoint(self.context, 'ForestState')
        if event =="conflict":
            return MoveToNextStateWaypoint(self.context, 'ConflictState')
        if event =="2robot_comb":
            return MoveToNextStateWaypoint(self.context, 'TwoRobotCombState')
        return None
STATE_CLASSES['SpearCombState'] = SpearCombState


class ForestState(State):
    """机器人森林状态。"""

    def enter(self):
        super().enter()
        self.logger.info("进入森林状态。机器人已就位。开始执行任务...")

    def handle_event(self, event):
        """处理事件。"""
        if event == "attain_spearhead":
            return MoveToNextStateWaypoint(self.context, 'AttainSpearheadState')
        if event =="spear_comb":
            return MoveToNextStateWaypoint(self.context, 'SpearCombState')
        if event =="idle":
            return IdleState(self.context)
        if event =="conflict":
            return MoveToNextStateWaypoint(self.context, 'ConflictState')
        if event =="2robot_comb":
            return MoveToNextStateWaypoint(self.context, 'TwoRobotCombState')
        return None
STATE_CLASSES['ForestState'] = ForestState


class ConflictState(State):
    """机器人冲突状态。"""

    def enter(self):
        super().enter()
        self.logger.info("进入冲突状态。机器人已就位。开始执行任务...")

    def handle_event(self, event):
        """处理事件。"""
        if event == "attain_spearhead":
            return MoveToNextStateWaypoint(self.context, 'AttainSpearheadState')
        if event =="spear_comb":
            return MoveToNextStateWaypoint(self.context, 'SpearCombState')
        if event =="forest":
            return MoveToNextStateWaypoint(self.context, 'ForestState')
        if event =="idle":
            return IdleState(self.context)
        if event =="2robot_comb":
            return MoveToNextStateWaypoint(self.context, 'TwoRobotCombState')
        return None
STATE_CLASSES['ConflictState'] = ConflictState


class TwoRobotCombState(State):
    """机器人双机组合状态。"""

    def enter(self):
        super().enter()
        self.logger.info("进入双机组合状态。机器人已就位。开始执行任务...")

    def handle_event(self, event):
        """处理事件。"""
        if event == "TASK_DONE_2R" or event == "idle":
            return IdleState(self.context)
        return None
STATE_CLASSES['TwoRobotCombState'] = TwoRobotCombState