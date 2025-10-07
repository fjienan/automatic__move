import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import numpy as np

class QuickTest(Node):
    def __init__(self):
        super().__init__('quick_test')
        self.get_logger().info('Quick test node started')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_ = self.create_timer(0.2, self.publish_twist)

    def publish_twist(self):
        twist = Twist()
        twist.linear.x = np.cos(2*time.time())
        twist.linear.y = np.sin(2*time.time())
        self.get_logger().info('Publishing twist x: %f' % twist.linear.x)
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = QuickTest()
    rclpy.spin(node)
    rclpy.shutdown()
