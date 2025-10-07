# AutoMove Engine 使用指南

## 简介

AutoMove Engine 是一个基于 ROS2 的机器人状态管理系统，提供了灵活的状态机框架来管理机器人的各种行为状态。

## 快速开始

### 1. 构建项目

```bash
cd ~/automatic_move
colcon build --packages-select automove_engine
source install/setup.bash
```

### 2. 启动状态管理器

```bash
# 方法1: 使用 launch 文件（推荐）
ros2 launch automove_engine state_manager_launch.py

# 方法2: 直接运行节点
ros2 run automove_engine state_manager
```

### 3. 监控系统状态

在新的终端中：

```bash
# 监控机器人当前状态
ros2 topic echo /robot_state

# 监控速度命令输出
ros2 topic echo /cmd_vel
```

### 4. 控制机器人

在另一个终端中发送命令：

```bash
# 开始移动
ros2 topic pub /robot_command std_msgs/String "data: 'move'" --once

# 停止机器人
ros2 topic pub /robot_command std_msgs/String "data: 'stop'" --once

# 开始导航
ros2 topic pub /robot_command std_msgs/String "data: 'navigate'" --once

# 触发避障行为
ros2 topic pub /robot_command std_msgs/String "data: 'obstacle'" --once

# 返回空闲状态
ros2 topic pub /robot_command std_msgs/String "data: 'idle'" --once
```

### 5. 运行自动化测试

```bash
ros2 run automove_engine state_test
```

## 状态说明

### IdleState（空闲状态）
- **行为**: 机器人静止等待
- **转换到**: MovingState, NavigationState
- **触发命令**: `move`, `navigate`

### MovingState（移动状态）
- **行为**: 机器人以 0.2 m/s 的速度前进
- **持续时间**: 5秒后自动转换到停止状态
- **转换到**: StoppedState, AvoidingObstacleState, IdleState
- **触发命令**: `stop`, `obstacle`, `idle`

### StoppedState（停止状态）
- **行为**: 机器人完全停止
- **转换到**: MovingState, IdleState
- **触发命令**: `move`, `resume`, `idle`

### AvoidingObstacleState（避障状态）
- **行为**: 机器人先后退1秒，然后转向2秒
- **持续时间**: 3秒后自动转换到移动状态
- **转换到**: MovingState, StoppedState, IdleState
- **触发命令**: `obstacle_cleared`, `stop`, `idle`

### NavigationState（导航状态）
- **行为**: 机器人前进并周期性改变方向
- **持续时间**: 10秒后自动转换到空闲状态
- **转换到**: AvoidingObstacleState, StoppedState, IdleState
- **触发命令**: `obstacle`, `stop`, `idle`

## 系统架构

```
StateManagerNode (ROS2 Node)
├── Publishers
│   ├── /cmd_vel (geometry_msgs/Twist)
│   └── /robot_state (std_msgs/String)
├── Subscribers
│   └── /robot_command (std_msgs/String)
├── Timer (10Hz 状态更新)
└── State Machine
    ├── IdleState
    ├── MovingState
    ├── StoppedState
    ├── AvoidingObstacleState
    └── NavigationState
```

## 扩展系统

### 添加新状态

1. 在 `states.py` 中添加新的状态类：

```python
class YourNewState(State):
    def __init__(self, context):
        super().__init__(context)
        # 初始化状态特定的变量
    
    def enter(self):
        # 进入状态时的操作
        pass
    
    def execute(self):
        # 状态主逻辑
        # 返回下一个状态或 None
        return None
    
    def exit(self):
        # 退出状态时的操作
        pass
    
    def handle_event(self, event):
        # 处理外部事件
        if event == "your_command":
            return AnotherState(self.context)
        return None
```

2. 在其他状态的 `handle_event()` 方法中添加到新状态的转换。

### 修改机器人行为

你可以通过修改各状态的 `execute()` 方法来改变机器人的行为：

- 修改速度值来改变移动速度
- 修改计时器来改变状态持续时间
- 添加传感器数据处理来实现真实的避障

## 故障排除

### 常见问题

1. **节点无法启动**
   - 确保已正确 source ROS2 环境
   - 检查是否有其他同名节点运行

2. **没有速度输出**
   - 检查 `/cmd_vel` 话题是否有订阅者
   - 确保机器人处于正确的状态

3. **状态转换不正确**
   - 检查命令格式是否正确
   - 使用 `ros2 topic echo /robot_state` 监控状态变化

### 调试技巧

```bash
# 查看所有话题
ros2 topic list

# 查看话题信息
ros2 topic info /robot_command

# 查看节点信息
ros2 node info /state_manager_node

# 实时监控日志
ros2 run rqt_console rqt_console
```

## 与仿真集成

要在 Gazebo 仿真中使用状态管理系统，确保：

1. Gazebo 中的机器人订阅了 `/cmd_vel` 话题
2. 启动了必要的控制器和驱动
3. 正确配置了机器人的 URDF 模型

```bash
# 示例：启动 Gazebo 和状态管理器
ros2 launch your_robot_package gazebo.launch.py &
ros2 launch automove_engine state_manager_launch.py
```
