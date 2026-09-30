#!/usr/bin/env python3
"""
lidar_state.py

Reads raw LiDAR (/scan) data from the robot and converts it into a
simplified "state" — this is what the RL agent will use as its "eyes".

Instead of 360 raw numbers, we reduce it to 5 zones:
  front, front-left, front-right, left, right
Each zone is labeled: "close", "medium", or "far"

Run this WHILE Gazebo + the robot are already running.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


# Distance thresholds (in meters) — tweak these later based on testing
CLOSE_THRESHOLD = 0.5
MEDIUM_THRESHOLD = 1.2


def classify_distance(distance):
    """Turns a raw distance number into a simple label."""
    if distance < CLOSE_THRESHOLD:
        return "close"
    elif distance < MEDIUM_THRESHOLD:
        return "medium"
    else:
        return "far"


class LidarStateNode(Node):
    def __init__(self):
        super().__init__('lidar_state_node')

        # Subscribe to the LiDAR topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.get_logger().info("Lidar State Node started. Listening to /scan...")

    def scan_callback(self, msg):
        ranges = msg.ranges  # list of ~360 distance readings
        num_readings = len(ranges)

        if num_readings == 0:
            return

        # Helper: get the minimum (closest) distance within a range of angles
        def min_in_range(start_deg, end_deg):
            start_idx = int((start_deg / 360.0) * num_readings)
            end_idx = int((end_deg / 360.0) * num_readings)
            segment = ranges[start_idx:end_idx]
            # Filter out invalid readings (inf, nan, 0)
            valid = [r for r in segment if not math.isinf(r) and not math.isnan(r) and r > 0.01]
            if not valid:
                return 10.0  # nothing detected = treat as "far away"
            return min(valid)

        # LiDAR angle convention: 0° = directly ahead, going counter-clockwise
        front_dist = min(min_in_range(0, 15), min_in_range(345, 360))
        front_left_dist = min_in_range(15, 60)
        left_dist = min_in_range(60, 120)
        front_right_dist = min_in_range(300, 345)
        right_dist = min_in_range(240, 300)

        state = {
            "front": classify_distance(front_dist),
            "front_left": classify_distance(front_left_dist),
            "front_right": classify_distance(front_right_dist),
            "left": classify_distance(left_dist),
            "right": classify_distance(right_dist),
        }

        # Print the simplified state (this is what RL will eventually read)
        self.get_logger().info(
            f"STATE -> front:{state['front']:6s} | "
            f"front_left:{state['front_left']:6s} | "
            f"front_right:{state['front_right']:6s} | "
            f"left:{state['left']:6s} | "
            f"right:{state['right']:6s}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = LidarStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
