#!/usr/bin/env python3
"""
启动自主移动机器人状态管理系统的 launch 文件。
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """生成 launch 描述。"""
    return LaunchDescription([
        Node(
            package='automove_engine',
            executable='state_manager',
            name='state_manager_node',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }],
            remappings=[
                ('cmd_vel', 'cmd_vel'),
                ('robot_state', 'robot_state'),
                ('robot_command', 'robot_command')
            ]
        )
    ])
