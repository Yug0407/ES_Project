#!/usr/bin/env python3
import math
import time
import random
import numpy as np
import gymnasium as gym
from gymnasium import spaces

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped

from gz.transport13 import Node as GzNode
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean
from stable_baselines3 import SAC

WORLD_NAME = "obstacle_world"
ROBOT_NAME = "waffle_pi"
MODE = "stage1"  # Options: "stage1", "stage2"


def quaternion_to_euler(x, y, z, w):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class TurtleBotRLEnv(gym.Env):
    def __init__(
        self,
        world_name,
        robot_name,
        safe_spawns,
        arena_bound=6.0,
        dt=0.1,
        log_file="training_log.txt",
    ):
        super().__init__()
        self.world_name = world_name
        self.robot_name = robot_name
        self.safe_spawns = safe_spawns
        self.arena_bound = arena_bound
        self.dt = dt
        self.log_file = log_file

        # Waffle Pi physical limits & acceleration caps
        self.v_min, self.v_max = 0.0, 0.26      # m/s
        self.w_min, self.w_max = -1.82, 1.82    # rad/s
        self.a_max = 0.5                        # m/s^2
        self.alpha_max = 2.5                    # rad/s^2
        self.num_sectors = 16
        self.max_lidar_range = 3.5

        # Action: normalized [v, w] in [-1, 1]
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)

        # Observation (40 dims): 16 LiDAR distances + 16 closing rates + 8 kinematics/stability
        obs_dim = self.num_sectors * 2 + 8
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32
        )

        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("rl_sac_trainer_node")
        self.cmd_pub = self.node.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.node.create_subscription(LaserScan, "/scan", self._scan_cb, qos_profile_sensor_data)
        self.node.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.node.create_subscription(Imu, "/imu", self._imu_cb, qos_profile_sensor_data)

        self.gz_node = GzNode()

        self.lidar_sectors = np.full(self.num_sectors, self.max_lidar_range, dtype=np.float32)
        self.prev_lidar_sectors = np.full(self.num_sectors, self.max_lidar_range, dtype=np.float32)
        self.lidar_rates = np.zeros(self.num_sectors, dtype=np.float32)

        self.x = 0.0
        self.y = 0.0
        self.v = 0.0
        self.w = 0.0
        self.prev_v = 0.0
        self.prev_w = 0.0
        self.a = 0.0
        self.alpha = 0.0

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.prev_roll = 0.0
        self.prev_pitch = 0.0
        self.tilt_rate = 0.0

        self.step_count = 0
        self.max_steps = 500
        self.episode_num = 0
        self.episode_reward = 0.0

    def _scan_cb(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges = np.nan_to_num(
            ranges, nan=self.max_lidar_range, posinf=self.max_lidar_range, neginf=0.0
        )
        ranges = np.clip(ranges, 0.01, self.max_lidar_range)
        chunks = np.array_split(ranges, self.num_sectors)
        self.lidar_sectors = np.array([np.min(c) for c in chunks], dtype=np.float32)

    def _odom_cb(self, msg: Odometry):
        self.v = float(msg.twist.twist.linear.x)
        self.w = float(msg.twist.twist.angular.z)

    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        self.roll, self.pitch, self.yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)

    def _wait_step_dt(self, duration: float):
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.005)

    def _publish_cmd(self, v_cmd: float, w_cmd: float):
        msg = TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(v_cmd)
        msg.twist.angular.z = float(w_cmd)
        self.cmd_pub.publish(msg)

    def _teleport_random_pose(self):
        self._publish_cmd(0.0, 0.0)
        bx, by = random.choice(self.safe_spawns)
        rx = bx + random.uniform(-0.6, 0.6)
        ry = by + random.uniform(-0.6, 0.6)
        ryaw = random.uniform(-math.pi, math.pi)

        req = Pose()
        req.name = self.robot_name
        req.position.x = rx
        req.position.y = ry
        req.position.z = 0.05
        req.orientation.z = math.sin(ryaw / 2.0)
        req.orientation.w = math.cos(ryaw / 2.0)

        service = f"/world/{self.world_name}/set_pose"
        ok, _ = self.gz_node.request(service, req, Pose, Boolean, 2000)
        if not ok:
            print(f"[WARNING] Failed to call {service} for model '{self.robot_name}'")
        else:
            self.x = rx
            self.y = ry
            self.yaw = ryaw

    def _get_obs(self):
        norm_lidar = self.lidar_sectors / self.max_lidar_range
        norm_rates = np.clip(self.lidar_rates / 1.5, -1.0, 1.0)
        centripetal = (self.v * self.w) / (self.v_max * self.w_max)
        kinematics = np.array(
            [
                self.v / self.v_max,
                self.w / self.w_max,
                np.clip(self.a / self.a_max, -1.5, 1.5),
                np.clip(self.alpha / self.alpha_max, -1.5, 1.5),
                centripetal,
                self.roll,
                self.pitch,
                np.clip(self.tilt_rate / 2.0, -1.5, 1.5),
            ],
            dtype=np.float32,
        )
        return np.concatenate([norm_lidar, norm_rates, kinematics])

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        if self.episode_num > 0:
            with open(self.log_file, "a") as f:
                f.write(f"{self.episode_num},{self.episode_reward:.4f}\n")

        self.episode_num += 1
        self.episode_reward = 0.0
        self.step_count = 0

        for _ in range(5):
            self._teleport_random_pose()
            self._wait_step_dt(0.25)
            if float(np.min(self.lidar_sectors)) > 0.45:
                break

        self.prev_lidar_sectors = self.lidar_sectors.copy()
        self.lidar_rates = np.zeros(self.num_sectors, dtype=np.float32)
        self.prev_v = self.v
        self.prev_w = self.w
        self.a = 0.0
        self.alpha = 0.0
        self.prev_roll = self.roll
        self.prev_pitch = self.pitch
        self.tilt_rate = 0.0

        return self._get_obs(), {}

    def _compute_smooth_reward(self, crashed: bool, toppled: bool) -> float:
        if crashed or toppled:
            speed_ratio = self.v / self.v_max
            return -15.0 - 10.0 * (speed_ratio ** 2)

        front_dist = float(min(self.lidar_sectors[0], self.lidar_sectors[-1], self.lidar_sectors[1]))
        min_dist = float(np.min(self.lidar_sectors))
        min_closing_rate = float(np.min(self.lidar_rates))

        clearance_factor = float(np.clip((front_dist - 0.25) / 1.0, 0.0, 1.0))
        r_progress = 2.5 * (self.v / self.v_max) * clearance_factor
        r_stall = -0.40 if (self.v < 0.05 and front_dist > 0.80) else 0.0

        r_clearance = 0.0
        if min_dist < 0.70:
            r_clearance -= 1.2 * ((0.70 - min_dist) / 0.70) ** 2
        if min_dist < 0.80 and min_closing_rate < 0.0:
            r_clearance -= 0.4 * abs(min_closing_rate)

        r_dynamics = (
            -0.15 * abs(self.a / self.a_max)
            - 0.15 * abs(self.alpha / self.alpha_max)
            - 0.30 * abs((self.v * self.w) / (self.v_max * self.w_max))
            - 0.10 * abs(self.w / self.w_max)
        )
        r_stability = -2.0 * (self.roll ** 2 + self.pitch ** 2) - 0.5 * (self.tilt_rate ** 2)

        return r_progress + r_stall + r_clearance + r_dynamics + r_stability

    def step(self, action):
        self.step_count += 1

        v_target = self.v_min + (float(action[0]) + 1.0) * 0.5 * (self.v_max - self.v_min)
        w_target = float(action[1]) * self.w_max

        v_cmd = np.clip(v_target, self.v - self.a_max * self.dt, self.v + self.a_max * self.dt)
        w_cmd = np.clip(w_target, self.w - self.alpha_max * self.dt, self.w + self.alpha_max * self.dt)

        self._publish_cmd(v_cmd, w_cmd)
        self._wait_step_dt(self.dt)

        # Track position relative to teleport origin to enforce arena geofence
        self.x += self.v * math.cos(self.yaw) * self.dt
        self.y += self.v * math.sin(self.yaw) * self.dt

        self.lidar_rates = (self.lidar_sectors - self.prev_lidar_sectors) / self.dt
        self.prev_lidar_sectors = self.lidar_sectors.copy()

        self.a = (self.v - self.prev_v) / self.dt
        self.alpha = (self.w - self.prev_w) / self.dt
        self.prev_v = self.v
        self.prev_w = self.w

        d_roll = (self.roll - self.prev_roll) / self.dt
        d_pitch = (self.pitch - self.prev_pitch) / self.dt
        self.tilt_rate = math.hypot(d_roll, d_pitch)
        self.prev_roll = self.roll
        self.prev_pitch = self.pitch

        min_dist = float(np.min(self.lidar_sectors))
        crashed = min_dist < 0.21
        toppled = (abs(self.roll) + abs(self.pitch)) > 0.55
        out_of_bounds = (
            (abs(self.x) > self.arena_bound or abs(self.y) > self.arena_bound)
            if self.arena_bound is not None
            else False
        )

        reward = self._compute_smooth_reward(crashed, toppled)
        terminated = bool(crashed or toppled)
        truncated = bool(self.step_count >= self.max_steps or out_of_bounds)
        self.episode_reward += reward

        return self._get_obs(), float(reward), terminated, truncated, {}


if __name__ == "__main__":
    stage1_spawns = [(0.0, 0.0), (2.2, 2.2), (-2.2, 2.2), (2.2, -2.2), (-2.2, -2.2)]
    stage2_spawns = [(-9.0, -9.0), (-4.5, 3.0), (0.0, 0.0), (4.0, -4.0), (-15.0, 0.0)]

    if MODE == "stage1":
        env = TurtleBotRLEnv(
            world_name=WORLD_NAME,
            robot_name=ROBOT_NAME,
            safe_spawns=stage1_spawns,
            arena_bound=6.0,
        )
        model = SAC(
            "MlpPolicy",
            env,
            learning_rate=3e-4,
            buffer_size=100_000,
            batch_size=256,
            policy_kwargs=dict(net_arch=[256, 256]),
            verbose=1,
        )
        model.learn(total_timesteps=40_000)
        model.save("sac_stage1_obstacle")
        print("Saved Stage 1 model: sac_stage1_obstacle.zip")

    elif MODE == "stage2":
        env = TurtleBotRLEnv(
            world_name=WORLD_NAME,
            robot_name=ROBOT_NAME,
            safe_spawns=stage2_spawns,
            arena_bound=None,  # Full road/city map in Stage 2
        )
        model = SAC.load("sac_stage1_obstacle", env=env)
        model.learn(total_timesteps=40_000, reset_num_timesteps=False)
        model.save("sac_final_model")
        print("Saved final curriculum model: sac_final_model.zip")
