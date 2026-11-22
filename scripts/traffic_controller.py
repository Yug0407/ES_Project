#!/usr/bin/env python3
"""
traffic_controller.py  --  moves the npc_* cars and pedestrians in Gazebo
=============================================================================
Started automatically by launch/duckietown_world.launch.py.

How it works:  TrafficSim (traffic_sim.py) works out where every car/person
should be right now; this node tells Gazebo "put model X at this pose" about
30 times per SIMULATION second (so it stays in step with the robot even when
Gazebo runs slower than real time).

It also watches the TurtleBot, so cars slow down and queue behind it.
"""
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
try:
    from ros_gz_interfaces.srv import SetEntityPose
    from ros_gz_interfaces.msg import Entity
except ImportError:          # package missing -> use the plain `gz service` fallback only
    SetEntityPose = Entity = None

from traffic_sim import TrafficSim

import math


class TrafficController(Node):
    def __init__(self):
        super().__init__('traffic_controller')
        self.declare_parameter('world', 'duckietown_city')
        self.declare_parameter('robot_model', 'waffle_pi')
        self.declare_parameter('rate_hz', 30.0)
        world = self.get_parameter('world').value
        self.robot_name = self.get_parameter('robot_model').value
        self.period = 1.0 / float(self.get_parameter('rate_hz').value)

        self.sim = TrafficSim()
        self.world = world
        self.cli = self.create_client(SetEntityPose, f'/world/{world}/set_pose') if SetEntityPose else None
        self.use_cli = SetEntityPose is None   # fallback: call `gz service` directly
        self.cli_proc = None
        self.start_wall = time.monotonic()
        self.pending = {}                     # model name -> (future, time sent)
        self.robot = None                     # (x, y) of the TurtleBot
        self.last_clock = None
        self.last_step_clock = None
        self.clock_seen = False
        self.wall_last = time.monotonic()
        self.warned = set()

        self.create_subscription(Clock, '/clock', self.on_clock, 10)
        self.create_subscription(TFMessage, f'/world/{world}/dynamic_pose/info', self.on_poses, 10)
        self.create_timer(2.0, self.health_check)          # wall-clock timer
        self.create_timer(self.period, self.on_wall_tick)  # only used if /clock is missing
        self.get_logger().info(f'Traffic controller up. Waiting for /world/{world}/set_pose ...')

    # ---------------------------------------------------------------- inputs
    def on_poses(self, msg):
        for tf in msg.transforms:
            if tf.child_frame_id == self.robot_name:
                t = tf.transform.translation
                self.robot = (t.x, t.y)

    def on_clock(self, msg):
        now = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.clock_seen = True
        if self.last_clock is not None and now < self.last_clock - 0.5:
            self.get_logger().info('Simulation was reset -> traffic restarts from its start positions.')
            self.sim = TrafficSim()
            self.last_step_clock = None
        self.last_clock = now
        if self.last_step_clock is None:
            self.last_step_clock = now
            return
        dt = now - self.last_step_clock
        if dt >= self.period:
            self.last_step_clock = now
            self.advance(dt)

    def on_wall_tick(self):
        """Fallback when no /clock is bridged: use real time."""
        t = time.monotonic()
        dt, self.wall_last = t - self.wall_last, t
        if not self.clock_seen:
            self.advance(dt)

    # ---------------------------------------------------------------- output
    def ros_ready(self):
        return self.cli is not None and self.cli.service_is_ready()

    def advance(self, dt):
        if not (self.ros_ready() or self.use_cli):
            return
        poses = self.sim.step(dt, self.robot)
        if self.ros_ready():
            self.send_ros(poses)
        else:
            self.send_cli(poses)

    def send_ros(self, poses):
        now = time.monotonic()
        for name, (x, y, z, yaw) in poses.items():
            old = self.pending.get(name)
            if old is not None and not old[0].done() and now - old[1] < 1.0:
                continue                          # previous request still running
            req = SetEntityPose.Request()
            req.entity.name = name
            req.entity.type = getattr(Entity, 'MODEL', 2)
            req.pose.position.x, req.pose.position.y, req.pose.position.z = x, y, z
            req.pose.orientation.z = math.sin(yaw / 2.0)
            req.pose.orientation.w = math.cos(yaw / 2.0)
            self.pending[name] = (self.cli.call_async(req), now)

    def send_cli(self, poses):
        """Fallback: one `gz service ... set_pose_vector` call moves everything."""
        if self.cli_proc is not None and self.cli_proc.poll() is None:
            return
        body = ' '.join(
            f'pose {{ name: "{n}" position {{ x: {x:.4f} y: {y:.4f} z: {z:.4f} }} '
            f'orientation {{ z: {math.sin(yaw / 2):.5f} w: {math.cos(yaw / 2):.5f} }} }}'
            for n, (x, y, z, yaw) in poses.items())
        try:
            self.cli_proc = subprocess.Popen(
                ['gz', 'service', '-s', f'/world/{self.world}/set_pose_vector',
                 '--reqtype', 'gz.msgs.Pose_V', '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '1000', '--req', body],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except FileNotFoundError:
            self.get_logger().error('`gz` command not found - source your Gazebo/ROS environment.',
                                    throttle_duration_sec=10.0)

    def health_check(self):
        if self.ros_ready():
            self.use_cli = False
        elif not self.use_cli and time.monotonic() - self.start_wall > 12.0:
            self.use_cli = True
            self.get_logger().warn(
                'ROS set_pose service never appeared -> switching to the plain `gz service` fallback '
                '(works, but motion is less smooth). Check the traffic_bridge output in the launch log.')
        if not self.clock_seen and (self.ros_ready() or self.use_cli):
            self.get_logger().warn('No /clock yet - using real time (cars may look too fast if Gazebo runs slow).',
                                   throttle_duration_sec=20.0)
        if self.robot is None:
            self.get_logger().info('Robot pose not seen yet - cars will not brake for it until it appears.',
                                   throttle_duration_sec=30.0)


def main():
    rclpy.init(args=sys.argv)
    node = TrafficController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
