#!/usr/bin/env python3
"""
record_sensor_data.py
=============================================================================
Records real-time sensor data from TurtleBot3 in Gazebo into a CSV file.

Captured topics:
  - /odom  (Position x, y, and linear/angular velocities)
  - /scan  (LiDAR ranges & 5-zone sector distances: front, left, right, etc.)
  - /imu   (Linear accelerations and angular velocities)

Usage:
  python3 scripts/record_sensor_data.py [--output filename.csv] [--hz rate]

Press Ctrl+C to stop recording and save the dataset.
=============================================================================
"""

import os
import sys
import math
import time
import csv
import argparse
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry


class SensorDataRecorder(Node):
    def __init__(self, output_path, sample_hz=10.0):
        super().__init__('sensor_data_recorder')

        self.output_path = output_path
        self.sample_period = 1.0 / sample_hz
        self.record_count = 0

        # Data buffers
        self.latest_odom = None
        self.latest_scan = None
        self.latest_imu = None

        # Subscriptions
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', self.imu_cb, qos_profile_sensor_data)

        # Ensure directory exists
        os.makedirs(os.path.dirname(os.path.abspath(self.output_path)), exist_ok=True)

        # Open CSV file and write header
        self.csv_file = open(self.output_path, mode='w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'timestamp_sec',
            'pos_x', 'pos_y', 'pos_z',
            'linear_velocity', 'angular_velocity',
            'imu_accel_x', 'imu_accel_y', 'imu_accel_z',
            'imu_gyro_z',
            'lidar_min_dist',
            'lidar_front', 'lidar_front_left', 'lidar_front_right',
            'lidar_left', 'lidar_right'
        ])

        # Periodic timer for sampling and writing
        self.timer = self.create_timer(self.sample_period, self.record_sample)

        self.get_logger().info(f"Sensor recorder started! Saving to: {self.output_path}")
        self.get_logger().info("Drive the robot or run simulation. Press Ctrl+C to stop recording.")

    def odom_cb(self, msg):
        self.latest_odom = msg

    def scan_cb(self, msg):
        self.latest_scan = msg

    def imu_cb(self, msg):
        self.latest_imu = msg

    def _extract_lidar_zones(self, ranges):
        n = len(ranges)
        if n == 0:
            return 10.0, 10.0, 10.0, 10.0, 10.0, 10.0

        def min_in_range(start_deg, end_deg):
            start_idx = int((start_deg / 360.0) * n)
            end_idx = int((end_deg / 360.0) * n)
            segment = ranges[start_idx:end_idx]
            valid = [r for r in segment if not math.isinf(r) and not math.isnan(r) and r > 0.01]
            return min(valid) if valid else 10.0

        front = min(min_in_range(0, 15), min_in_range(345, 360))
        front_left = min_in_range(15, 60)
        left = min_in_range(60, 120)
        front_right = min_in_range(300, 345)
        right = min_in_range(240, 300)

        all_valid = [r for r in ranges if not math.isinf(r) and not math.isnan(r) and r > 0.01]
        overall_min = min(all_valid) if all_valid else 10.0

        return overall_min, front, front_left, front_right, left, right

    def record_sample(self):
        # Only record if we have received at least odometry or scan data
        if self.latest_odom is None and self.latest_scan is None:
            return

        now = self.get_clock().now().nanoseconds / 1e9

        # Odom extraction
        pos_x = pos_y = pos_z = lin_v = ang_w = 0.0
        if self.latest_odom is not None:
            pos = self.latest_odom.pose.pose.position
            pos_x, pos_y, pos_z = pos.x, pos.y, pos.z
            lin_v = self.latest_odom.twist.twist.linear.x
            ang_w = self.latest_odom.twist.twist.angular.z

        # IMU extraction
        acc_x = acc_y = acc_z = gyro_z = 0.0
        if self.latest_imu is not None:
            acc = self.latest_imu.linear_acceleration
            acc_x, acc_y, acc_z = acc.x, acc.y, acc.z
            gyro_z = self.latest_imu.angular_velocity.z

        # LiDAR extraction
        min_d = f_d = fl_d = fr_d = l_d = r_d = 10.0
        if self.latest_scan is not None:
            min_d, f_d, fl_d, fr_d, l_d, r_d = self._extract_lidar_zones(self.latest_scan.ranges)

        self.writer.writerow([
            f"{now:.3f}",
            f"{pos_x:.3f}", f"{pos_y:.3f}", f"{pos_z:.3f}",
            f"{lin_v:.3f}", f"{ang_w:.3f}",
            f"{acc_x:.3f}", f"{acc_y:.3f}", f"{acc_z:.3f}",
            f"{gyro_z:.3f}",
            f"{min_d:.3f}",
            f"{f_d:.3f}", f"{fl_d:.3f}", f"{fr_d:.3f}",
            f"{l_d:.3f}", f"{r_d:.3f}"
        ])

        self.record_count += 1
        if self.record_count % 50 == 0:
            self.csv_file.flush()
            self.get_logger().info(
                f"[{self.record_count} samples] Pos: ({pos_x:.2f}, {pos_y:.2f}) | "
                f"V: {lin_v:.2f} m/s | Min LiDAR: {min_d:.2f} m"
            )

    def close(self):
        self.csv_file.flush()
        self.csv_file.close()
        self.get_logger().info(f"Saved total of {self.record_count} samples to {self.output_path}")


def main():
    parser = argparse.ArgumentParser(description="Record robot sensor data to CSV")
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    default_filename = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'data',
        f'sensor_data_{timestamp_str}.csv'
    )
    parser.add_argument('--output', '-o', default=default_filename, help="Output CSV path")
    parser.add_argument('--hz', type=float, default=10.0, help="Sampling frequency in Hz (default: 10)")
    args = parser.parse_args()

    rclpy.init()
    node = SensorDataRecorder(args.output, args.hz)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
