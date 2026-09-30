#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from reward_calculator import RewardCalculator
import math

class GazeboEnvNode(Node):
    def __init__(self):
        super().__init__('gazebo_env_node')
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        
        self.reward_calc = RewardCalculator()
        self.current_scan = None
        self.current_v = 0.0
        self.current_w = 0.0

    def scan_cb(self, msg):
        valid = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r) and r > 0.01]
        self.current_scan = min(valid) if valid else 10.0

    def odom_cb(self, msg):
        self.current_v = msg.twist.twist.linear.x
        self.current_w = msg.twist.twist.angular.z

    def step(self, action_idx):
        actions = [
            (0.2, 0.0),   # Forward
            (0.0, 0.5),   # Turn Left
            (0.0, -0.5),  # Turn Right
            (0.0, 0.0)    # Stop
        ]
        
        cmd = Twist()
        cmd.linear.x = actions[action_idx][0]
        cmd.angular.z = actions[action_idx][1]
        self.cmd_pub.publish(cmd)
        
        rclpy.spin_once(self, timeout_sec=0.1)
        
        reward, done = self.reward_calc.compute_reward(
            self.current_v, self.current_w, self.current_scan
        )
        return reward, done

def main(args=None):
    rclpy.init(args=args)
    node = GazeboEnvNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()