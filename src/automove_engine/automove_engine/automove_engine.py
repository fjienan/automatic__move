import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math
import numpy as np 
from time import sleep

# 假设 states 和 PID_Controller 已经准备好
from . import states
from .PID import PID_Controller

class AutoMoveEngine(Node):
    def __init__(self):
        super().__init__('auto_move_engine')
        self.get_logger().info('Auto Move Engine Node has been started.')
        
        # 位置: [x, y, z] 或 [x, y, yaw] - 只需要前两个维度
        self.pose = np.array([0.0, 0.0, 0.0])  
        self.current_state = 'IDLE' 
        self.next_state = None
        self.state_list = ["attain_spearhead", "spear_comb","forest", "conflict"]
        self.state_index = 0      
        self.goal_waypoints = []    
        self.current_waypoint_index = 0 
        
        # 时间记录，用于计算 dt (Time step)
        self.last_time = self.get_clock().now()
        
        # 实例化您的 PID 控制器
        self.pid_controller = PID_Controller(kp=2.5, ki=0.05, kd=0.03, dimention=2,vx_max = 2.5, vy_max = 2.5)

        # 限制参数
        self.MAX_SPEED = 10.0  # 最大线速度 (m/s)            
        self.WAYPOINT_TOLERANCE = 0.2   
        
        # --- ROS 2 通信 ---
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.state_subscriber = self.create_subscription(String, 'robot_state', self.state_callback, 10)
        
        # Publisher for cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 【新增】Publisher for robot_command (用于发送反馈给 StateManagerNode)
        self.command_publisher = self.create_publisher(String, 'robot_command', 10) 
        
        # --- 主控制定时器 (非阻塞) ---
        timer_period = 0.05  # 20Hz
        self.timer = self.create_timer(timer_period, self.main_control_loop)
        self.get_logger().info('Control timer started.')

    def odom_callback(self, msg):
        """回调函数：更新 x, y 坐标。"""
        x = msg.pose.pose.position.x 
        y = msg.pose.pose.position.y
        self.pose[0] = x + 6.0
        self.pose[1] = 6.0 - y
        self.pose[2] = 0.0
    
    
    def state_callback(self, msg):
        """状态回调函数：设置目标状态和路点。"""
        new_state = msg.data
        if new_state in states.state_position:
            self.next_state = new_state
            # 确保目标点格式正确，并转换为 numpy 数组列表
            self.goal_waypoints = [np.array(p) for p in states.state_position[new_state]]
            self.current_waypoint_index = 0
            self.get_logger().info(f'Set state {self.next_state} with {len(self.goal_waypoints)} waypoints.')
        elif new_state == 'IDLE':
             self.current_state = 'IDLE'
             self.next_state = None
             self.stop_robot()
        
    
    def main_control_loop(self):
        """主控制循环，周期性调用，处理运动逻辑。"""
        
        # 计算时间间隔 dt
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if self.next_state is not None:
            self.current_state = self.next_state 
            self.move_to_waypoint(dt) 
        else:
            self.stop_robot()
            self.get_logger().info('Waiting for next state...')
            sleep(0.1)
            if self.state_index < len(self.state_list):
                self.next_state = self.state_list[self.state_index]
                self._publish_command(self.state_list[self.state_index])
                self.state_index += 1


    def move_to_waypoint(self, dt):
        """非阻塞式地向当前目标点移动一步。"""
        
        if not self.goal_waypoints or self.current_waypoint_index >= len(self.goal_waypoints):
            # 所有目标点完成
            self.get_logger().info(f'Finished all waypoints for state: {self.current_state}')
            
            self._publish_command('TASK_FINISHED') 


            self.current_state = 'IDLE'
            self.next_state = None
            self.stop_robot()
            return

        # 当前目标点 (仅取 x, y)
        target_point = self.goal_waypoints[self.current_waypoint_index][:2]
        current_position = self.pose[:2]
        
        # 检查是否到达当前目标点
        distance = np.linalg.norm(target_point - current_position)
        
        if distance < self.WAYPOINT_TOLERANCE:
            self.get_logger().info(f'Reached waypoint {self.current_waypoint_index+1}/{len(self.goal_waypoints)}: ({target_point[0]:.2f}, {target_point[1]:.2f})')
            self.current_waypoint_index += 1
            return 

        # --- PID 控制计算 (二维) ---
        vel_vector = self.pid_controller.compute_vel(target_point, current_position, dt)
        linear_x = vel_vector[0]
        linear_y = -vel_vector[1]

        # --- 限制速度 (强制安全) ---
        current_speed = np.linalg.norm(vel_vector)
        if current_speed > self.MAX_SPEED:
            scale_factor = self.MAX_SPEED / current_speed
            linear_x *= scale_factor
            linear_y *= scale_factor
        
        # --- 发布速度指令 ---
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.angular.z = 0.0 
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info(f'Moving to ({target_point[0]:.2f},{target_point[1]:.2f}). Dist:{distance:.2f}. Cmd: vx={linear_x:.2f}, vy={linear_y:.2f}')

    # --- 辅助函数 ---
    
    def _publish_command(self, data):
        """发布自定义机器人命令消息。"""
        msg = String()
        msg.data = data
        self.command_publisher.publish(msg)

    def stop_robot(self):
        """发布零速度指令停止机器人。"""
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(stop_msg)


def main(args=None):
    rclpy.init(args=args)
    auto_move_engine = AutoMoveEngine()
    try:
        rclpy.spin(auto_move_engine)
    except KeyboardInterrupt:
        pass
    finally:
        auto_move_engine.stop_robot()
        auto_move_engine.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()