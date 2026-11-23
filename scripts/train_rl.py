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

from reward_calculator import RewardCalculator

# Verified from gz service -l and gz model --list
WORLD_NAME = "obstacle_world"
ROBOT_NAME = "waffle_pi"
MODE = "smoke_test"  # Options: "smoke_test", "stage1", "stage2"


def quaternion_to_euler(x, y, z, w):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    return roll, pitch


class TurtleBotRLEnv(gym.Env):
    def __init__(self, world_name, robot_name, safe_spawns, dt=0.1, log_file="training_log.txt"):
        super().__init__()
        self.world_name = world_name
        self.robot_name = robot_name
        self.safe_spawns = safe_spawns
        self.dt = dt
        self.log_file = log_file

        # Waffle Pi kinematic and acceleration envelopes
        self.v_min, self.v_max = 0.0, 0.26      # m/s
        self.w_min, self.w_max = -1.82, 1.82    # rad/s
        self.a_max = 0.5                        # m/s^2
        self.alpha_max = 2.5                    # rad/s^2
        self.num_sectors = 16
        self.max_lidar_range = 3.5

        # Action: normalized [v, w] in [-1, 1]
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)

        # State: 16 LiDAR sectors + [v, w, a, alpha, roll, pitch] = 22 features
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(self.num_sectors + 6,), dtype=np.float32
        )

        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("rl_sac_trainer_node")
        self.cmd_pub = self.node.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.node.create_subscription(LaserScan, "/scan", self._scan_cb, qos_profile_sensor_data)
        self.node.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.node.create_subscription(Imu, "/imu", self._imu_cb, qos_profile_sensor_data)

        self.gz_node = GzNode()
        self.reward_calc = RewardCalculator()

        self.lidar_sectors = np.full(self.num_sectors, self.max_lidar_range, dtype=np.float32)
        self.v = 0.0
        self.w = 0.0
        self.prev_v = 0.0
        self.prev_w = 0.0
        self.a = 0.0
        self.alpha = 0.0
        self.roll = 0.0
        self.pitch = 0.0

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
        self.roll, self.pitch = quaternion_to_euler(q.x, q.y, q.z, q.w)

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

    def _get_obs(self):
        norm_lidar = self.lidar_sectors / self.max_lidar_range
        kinematics = np.array(
            [
                self.v / self.v_max,
                self.w / self.w_max,
                self.a / self.a_max,
                self.alpha / self.alpha_max,
                self.roll,
                self.pitch,
            ],
            dtype=np.float32,
        )
        return np.concatenate([norm_lidar, kinematics])

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        if self.episode_num > 0:
            with open(self.log_file, "a") as f:
                f.write(f"{self.episode_num},{self.episode_reward:.4f}\n")

        self.episode_num += 1
        self.episode_reward = 0.0
        self.step_count = 0

        self.prev_v = 0.0
        self.prev_w = 0.0
        self.a = 0.0
        self.alpha = 0.0
        self.reward_calc.last_v = 0.0
        self.reward_calc.last_w = 0.0

        for _ in range(5):
            self._teleport_random_pose()
            self._wait_step_dt(0.25)
            if float(np.min(self.lidar_sectors)) > 0.45:
                break

        return self._get_obs(), {}

    def step(self, action):
        self.step_count += 1

        v_target = self.v_min + (float(action[0]) + 1.0) * 0.5 * (self.v_max - self.v_min)
        w_target = float(action[1]) * self.w_max

        # Enforce hard acceleration bounds (a_max, alpha_max)
        v_cmd = np.clip(v_target, self.v - self.a_max * self.dt, self.v + self.a_max * self.dt)
        w_cmd = np.clip(w_target, self.w - self.alpha_max * self.dt, self.w + self.alpha_max * self.dt)

        self._publish_cmd(v_cmd, w_cmd)
        self._wait_step_dt(self.dt)

        self.a = (self.v - self.prev_v) / self.dt
        self.alpha = (self.w - self.prev_w) / self.dt
        self.prev_v = self.v
        self.prev_w = self.w

        min_dist = float(np.min(self.lidar_sectors))

        reward, crashed = self.reward_calc.compute_reward(
            self.v, self.w, min_dist, dt=self.dt
        )

        toppled = (abs(self.roll) + abs(self.pitch)) > 0.55  # ~31 deg
        if toppled and not crashed:
            reward = -100.0

        if not (crashed or toppled):
            if min_dist < 0.50:
                reward -= 1.5 * (0.50 - min_dist)
            reward -= 1.5 * (self.roll**2 + self.pitch**2)

        terminated = bool(crashed or toppled)
        truncated = bool(self.step_count >= self.max_steps)
        self.episode_reward += reward

        return self._get_obs(), float(reward), terminated, truncated, {}


if __name__ == "__main__":
    stage1_spawns = [(0.0, 0.0), (2.5, 2.5), (-2.5, 2.5), (2.5, -2.5), (-2.5, -2.5)]
    stage2_spawns = [(-9.0, -9.0), (-4.5, 3.0), (0.0, 0.0), (4.0, -4.0), (-15.0, 0.0)]

    spawns = stage2_spawns if MODE == "stage2" else stage1_spawns
    env = TurtleBotRLEnv(world_name=WORLD_NAME, robot_name=ROBOT_NAME, safe_spawns=spawns)

    if MODE == "smoke_test":
        model = SAC("MlpPolicy", env, learning_rate=3e-4, batch_size=64, verbose=1)
        model.learn(total_timesteps=300)
        print("Smoke test complete!")

    elif MODE == "stage1":
        model = SAC(
            "MlpPolicy",
            env,
            learning_rate=3e-4,
            buffer_size=100_000,
            batch_size=256,
            policy_kwargs=dict(net_arch=[256, 256]),
            verbose=1,
        )
        model.learn(total_timesteps=50_000)
        model.save("sac_stage1_obstacle")
        print("Saved Stage 1 model: sac_stage1_obstacle.zip")

    elif MODE == "stage2":
        model = SAC.load("sac_stage1_obstacle", env=env)
        model.learn(total_timesteps=50_000, reset_num_timesteps=False)
        model.save("sac_final_model")
        print("Saved final curriculum model: sac_final_model.zip")
