#!/usr/bin/env python3
import os
import math
import time
import random
import numpy as np
import gymnasium as gym
from gymnasium import spaces

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Imu, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from tf2_msgs.msg import TFMessage

from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean
try:
    from gz.msgs10.pose_v_pb2 import Pose_V
except ImportError:
    Pose_V = None

from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import CheckpointCallback

from traffic_sim import LanePath, ROUTES

WORLD_NAME = "duckietown_city"
ROBOT_NAME = "waffle_pi"


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


class DuckietownRLEnv(gym.Env):
    def __init__(self, world_name=WORLD_NAME, robot_name=ROBOT_NAME, dt=0.1, log_file="training_log.txt"):
        super().__init__()
        self.world_name = world_name
        self.robot_name = robot_name
        self.dt = dt
        self.log_file = log_file

        # Right-hand lane trajectories (0.30 m right of road centerline)
        self.spawn_lanes = [
            LanePath(ROUTES["big"], reverse=False),
            LanePath(ROUTES["big"], reverse=True),
            LanePath(ROUTES["west"], reverse=False),
            LanePath(ROUTES["west"], reverse=True),
        ]

        pts_list, yaw_list = [], []
        for lane in self.spawn_lanes:
            step_idx = max(1, int(0.05 / lane.ds))
            pts_list.extend(lane.pts[::step_idx])
            yaw_list.extend(lane.yaw[::step_idx])
        self.lane_xy = np.array(pts_list, dtype=np.float32)
        self.lane_yaw = np.array(yaw_list, dtype=np.float32)

        # Waffle Pi physical limits & acceleration caps
        self.v_min, self.v_max = 0.0, 0.26      # m/s
        self.w_min, self.w_max = -1.82, 1.82    # rad/s
        self.a_max = 0.5                        # m/s^2
        self.alpha_max = 2.5                    # rad/s^2
        self.num_sectors = 16
        self.max_lidar_range = 3.5

        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)

        # Observation (64 dims — 100% onboard sensors):
        # 16 LiDAR distances + 16 LiDAR closing rates + 24 camera features (Near + Lookahead) + 8 kinematics/IMU
        obs_dim = self.num_sectors * 2 + 24 + 8
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32
        )

        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("duckietown_sac_trainer")
        self.cmd_pub = self.node.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.node.create_subscription(LaserScan, "/scan", self._scan_cb, qos_profile_sensor_data)
        self.node.create_subscription(Image, "/camera/image_raw", self._image_cb, qos_profile_sensor_data)
        self.node.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.node.create_subscription(Imu, "/imu", self._imu_cb, qos_profile_sensor_data)
        self.node.create_subscription(
            TFMessage,
            f"/world/{self.world_name}/dynamic_pose/info",
            self._gt_pose_cb,
            qos_profile_sensor_data,
        )

        self.set_pose_cli = self.node.create_client(SetEntityPose, f"/world/{self.world_name}/set_pose")
        self.gz_node = GzNode()
        time.sleep(0.5)
        if Pose_V is not None:
            self.gz_node.subscribe(
                Pose_V,
                f"/world/{self.world_name}/dynamic_pose/info",
                self._gz_pose_cb,
            )

        self.lidar_sectors = np.full(self.num_sectors, self.max_lidar_range, dtype=np.float32)
        self.prev_lidar_sectors = np.full(self.num_sectors, self.max_lidar_range, dtype=np.float32)
        self.lidar_rates = np.zeros(self.num_sectors, dtype=np.float32)

        self.cam_features = np.zeros(24, dtype=np.float32)
        self.lookahead_grass = 0.0
        self._last_cam_time = 0.0

        self.gt_x = -0.9
        self.gt_y = 0.3
        self.gt_z = 0.0
        self.gt_yaw = 0.0
        self.gt_pose_received = False

        self.v = 0.0
        self.w = 0.0
        self.prev_v = 0.0
        self.prev_w = 0.0
        self.a = 0.0
        self.alpha = 0.0

        self.roll = 0.0
        self.pitch = 0.0
        self.prev_roll = 0.0
        self.prev_pitch = 0.0
        self.tilt_rate = 0.0

        self.stop_steps = 0
        self.step_count = 0
        self.max_steps = 600
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

    def _extract_band_features(self, band: np.ndarray):
        strips = np.array_split(band, 3, axis=1)  # Left, Center, Right
        feats = []
        grass_vals = []
        for s in strips:
            r, g, b = s[:, :, 0], s[:, :, 1], s[:, :, 2]
            brightness = (r + g + b) / 3.0
            max_c = np.maximum(np.maximum(r, g), b)
            min_c = np.minimum(np.minimum(r, g), b)
            sat = np.divide(max_c - min_c, max_c, out=np.zeros_like(max_c), where=max_c > 1e-4)

            asphalt = float(np.mean((brightness >= 0.06) & (brightness <= 0.42) & (sat < 0.24)))
            yellow = float(np.mean((r > 0.48) & (g > 0.40) & (b < 0.35)))
            white = float(np.mean((brightness > 0.68) & (sat < 0.22)))
            grass = float(np.mean((g > r + 0.05) & (g > b + 0.05) & (g > 0.18)))

            feats.extend([asphalt, yellow, white, grass])
            grass_vals.append(grass)
        return feats, float(np.mean(grass_vals))

    def _image_cb(self, msg: Image):
        now = time.time()
        if now - self._last_cam_time < 0.08:
            return
        self._last_cam_time = now

        if msg.encoding not in ("rgb8", "bgr8"):
            return

        img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        if msg.encoding == "bgr8":
            img = img[:, :, ::-1]

        norm_img = img.astype(np.float32) / 255.0
        h = msg.height
        lookahead_band = norm_img[int(h * 0.42):int(h * 0.68), :, :]
        near_band = norm_img[int(h * 0.68):int(h * 0.95), :, :]

        near_feats, _ = self._extract_band_features(near_band)
        look_feats, look_grass = self._extract_band_features(lookahead_band)

        self.cam_features = np.array(near_feats + look_feats, dtype=np.float32)
        self.lookahead_grass = look_grass

    def _odom_cb(self, msg: Odometry):
        self.v = float(msg.twist.twist.linear.x)
        self.w = float(msg.twist.twist.angular.z)

    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        self.roll, self.pitch, _ = quaternion_to_euler(q.x, q.y, q.z, q.w)

    def _gt_pose_cb(self, msg: TFMessage):
        matched = None
        for t in msg.transforms:
            fid = t.child_frame_id
            if fid == self.robot_name or fid.endswith("::" + self.robot_name) or fid.endswith("/" + self.robot_name):
                matched = t
                break
            if self.robot_name in fid and matched is None:
                matched = t
        if matched is not None:
            self.gt_x = float(matched.transform.translation.x)
            self.gt_y = float(matched.transform.translation.y)
            self.gt_z = float(matched.transform.translation.z)
            q = matched.transform.rotation
            _, _, self.gt_yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
            if not self.gt_pose_received:
                print(f"[INFO] Ground-truth pose locked via ROS TF: '{matched.child_frame_id}'")
            self.gt_pose_received = True

    def _gz_pose_cb(self, msg):
        matched = None
        for p in msg.pose:
            if p.name == self.robot_name or p.name.endswith("::" + self.robot_name):
                matched = p
                break
            if self.robot_name in p.name and matched is None:
                matched = p
        if matched is not None:
            self.gt_x = float(matched.position.x)
            self.gt_y = float(matched.position.y)
            self.gt_z = float(matched.position.z)
            q = matched.orientation
            _, _, self.gt_yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
            if not self.gt_pose_received:
                print(f"[INFO] Ground-truth pose locked via Gz transport: '{matched.name}'")
            self.gt_pose_received = True

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

    def _compute_lane_errors(self):
        heading_diff = np.arctan2(
            np.sin(self.gt_yaw - self.lane_yaw), np.cos(self.gt_yaw - self.lane_yaw)
        )
        valid_mask = np.cos(heading_diff) > 0.0
        if np.any(valid_mask):
            valid_idx = np.where(valid_mask)[0]
            dx = self.gt_x - self.lane_xy[valid_idx, 0]
            dy = self.gt_y - self.lane_xy[valid_idx, 1]
            h_diff = heading_diff[valid_idx]
            cost = (dx * dx + dy * dy) + 0.15 * (1.0 - np.cos(h_diff))
            idx = int(valid_idx[int(np.argmin(cost))])
        else:
            dx_all = self.gt_x - self.lane_xy[:, 0]
            dy_all = self.gt_y - self.lane_xy[:, 1]
            idx = int(np.argmin(dx_all * dx_all + dy_all * dy_all))

        dx_best = self.gt_x - float(self.lane_xy[idx, 0])
        dy_best = self.gt_y - float(self.lane_xy[idx, 1])
        lyaw = float(self.lane_yaw[idx])

        e_lat = float(-dx_best * math.sin(lyaw) + dy_best * math.cos(lyaw))
        e_psi = float(heading_diff[idx])
        return e_lat, e_psi

    def _teleport_random_lane_pose(self):
        self._publish_cmd(0.0, 0.0)
        lane = random.choice(self.spawn_lanes)
        x, y, yaw = lane.pose(random.uniform(0.0, lane.length))

        lat_offset = random.uniform(-0.05, 0.05)
        rx = x - lat_offset * math.sin(yaw)
        ry = y + lat_offset * math.cos(yaw)
        ryaw = yaw + random.uniform(-0.12, 0.12)

        moved = False
        if self.set_pose_cli.service_is_ready():
            ros_req = SetEntityPose.Request()
            ros_req.entity.name = self.robot_name
            ros_req.entity.type = getattr(Entity, "MODEL", 2)
            ros_req.pose.position.x = rx
            ros_req.pose.position.y = ry
            ros_req.pose.position.z = 0.05
            ros_req.pose.orientation.z = math.sin(ryaw / 2.0)
            ros_req.pose.orientation.w = math.cos(ryaw / 2.0)
            fut = self.set_pose_cli.call_async(ros_req)
            deadline = time.time() + 1.5
            while not fut.done() and time.time() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.01)
            if fut.done() and fut.result() is not None and fut.result().success:
                moved = True

        if not moved:
            req = Pose()
            req.name = self.robot_name
            req.position.x = rx
            req.position.y = ry
            req.position.z = 0.05
            req.orientation.z = math.sin(ryaw / 2.0)
            req.orientation.w = math.cos(ryaw / 2.0)
            service = f"/world/{self.world_name}/set_pose"
            ok, rep = self.gz_node.request(service, req, Pose, Boolean, 5000)
            moved = bool(ok and (rep is None or getattr(rep, "data", True)))

        if not moved:
            print(f"[WARNING] Failed to teleport '{self.robot_name}' via ROS and Gz set_pose")
        else:
            self.gt_x = rx
            self.gt_y = ry
            self.gt_yaw = ryaw

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
        return np.concatenate([norm_lidar, norm_rates, self.cam_features, kinematics])

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        if self.episode_num > 0:
            with open(self.log_file, "a") as f:
                f.write(f"{self.episode_num},{self.episode_reward:.4f}\n")

        self.episode_num += 1
        self.episode_reward = 0.0
        self.step_count = 0
        self.stop_steps = 0

        for _ in range(10):
            self._teleport_random_lane_pose()
            self._wait_step_dt(0.30)
            e_lat, _ = self._compute_lane_errors()
            if float(np.min(self.lidar_sectors)) > 0.60 and abs(e_lat) < 0.12 and self.gt_z > -0.10:
                break

        if not self.gt_pose_received and self.episode_num == 1:
            print("[WARNING] Ground-truth pose stream not yet received; check dynamic_pose/info topic.")

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

    def _compute_reward(self, crashed: bool, toppled: bool, lane_violation: bool, e_lat: float, e_psi: float) -> float:
        if crashed or toppled or lane_violation:
            speed_ratio = self.v / self.v_max
            return -22.0 - 10.0 * (speed_ratio ** 2)

        front_dist = float(min(self.lidar_sectors[0], self.lidar_sectors[15], self.lidar_sectors[1]))
        rear_dist = float(min(self.lidar_sectors[6], self.lidar_sectors[7], self.lidar_sectors[8], self.lidar_sectors[9]))
        rear_closing = float(min(self.lidar_rates[6], self.lidar_rates[7], self.lidar_rates[8], self.lidar_rates[9]))
        min_dist = float(np.min(self.lidar_sectors))
        min_closing_rate = float(np.min(self.lidar_rates))

        alignment = max(0.0, math.cos(e_psi))
        lane_gauss = math.exp(-(e_lat ** 2) / (2.0 * (0.10 ** 2)))
        g_lane = lane_gauss * alignment
        g_clear = float(np.clip((front_dist - 0.60) / 0.70, 0.0, 1.0))
        r_drive = 2.0 * (self.v / self.v_max) * g_clear * (0.15 + 0.85 * g_lane)

        r_bound = 0.0
        if e_lat > 0.13:
            # Continuous recovery penalty across yellow line (+0.13 m -> +0.45 m)
            r_bound -= 2.2 * ((e_lat - 0.13) / 0.22) ** 2
        elif e_lat < -0.13:
            r_bound -= 2.2 * ((-0.13 - e_lat) / 0.15) ** 2

        r_grass = -1.2 * max(0.0, self.lookahead_grass - 0.15)

        r_stop = 0.0
        if 0.28 <= front_dist <= 0.75:
            if self.v < 0.03 and self.stop_steps <= 45:
                r_stop = 0.35 * g_lane
            elif self.stop_steps > 45:
                r_stop = -0.25
        elif front_dist > 0.85 and self.stop_steps > 10:
            r_stop = -0.65

        rear_threat = (rear_dist < 1.35) and (rear_closing < -0.15)
        r_evade = 0.0
        if rear_threat and front_dist > 0.85:
            r_evade = 1.2 * (self.v / self.v_max) * g_lane

        r_obst = 0.0
        if min_dist < 0.60:
            r_obst -= 2.0 * ((0.60 - min_dist) / 0.60) ** 2
        if min_dist < 0.80 and min_closing_rate < 0.0:
            prox_weight = (0.80 - min_dist) / 0.80
            r_obst -= 0.5 * abs(min_closing_rate) * prox_weight

        accel_pen = 0.0 if rear_threat else 0.10 * abs(self.a / self.a_max)
        r_dyn = (
            -accel_pen
            - 0.12 * abs(self.alpha / self.alpha_max)
            - 0.08 * abs((self.v * self.w) / (self.v_max * self.w_max))
        )
        r_stab = -2.5 * (self.roll ** 2 + self.pitch ** 2) - 0.4 * (self.tilt_rate ** 2)

        return r_drive + r_bound + r_grass + r_stop + r_evade + r_obst + r_dyn + r_stab

    def step(self, action):
        self.step_count += 1

        v_target = self.v_min + (float(action[0]) + 1.0) * 0.5 * (self.v_max - self.v_min)
        w_target = float(action[1]) * self.w_max

        v_cmd = np.clip(v_target, self.v - self.a_max * self.dt, self.v + self.a_max * self.dt)
        w_cmd = np.clip(w_target, self.w - self.alpha_max * self.dt, self.w + self.alpha_max * self.dt)

        self._publish_cmd(v_cmd, w_cmd)
        self._wait_step_dt(self.dt)

        if self.v < 0.03:
            self.stop_steps += 1
        else:
            self.stop_steps = 0

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

        e_lat, e_psi = self._compute_lane_errors()
        min_dist = float(np.min(self.lidar_sectors))

        crashed = min_dist < 0.21
        toppled = (abs(self.roll) + abs(self.pitch)) > 0.55
        lane_violation = (e_lat > 0.45 or e_lat < -0.28) if self.gt_pose_received else False
        fell_off_map = self.gt_z < -0.20

        reward = self._compute_reward(crashed, toppled, lane_violation, e_lat, e_psi)
        terminated = bool(crashed or toppled or lane_violation)
        truncated = bool(self.step_count >= self.max_steps or fell_off_map)
        self.episode_reward += reward

        return self._get_obs(), float(reward), terminated, truncated, {}


if __name__ == "__main__":
    os.makedirs("checkpoints", exist_ok=True)
    checkpoint_cb = CheckpointCallback(
        save_freq=5_000,
        save_path="./checkpoints/",
        name_prefix="sac_duckietown_v3",
    )

    env = DuckietownRLEnv()
    ckpt_10k = "./checkpoints/sac_duckietown_v3_10000_steps.zip"

    if os.path.exists(ckpt_10k):
        print(f"Warm-starting from {ckpt_10k} with +0.45m corner recovery buffer...")
        model = SAC.load(ckpt_10k, env=env)
        model.log_ent_coef.data.fill_(math.log(0.25))
        model.learn(total_timesteps=25_000, reset_num_timesteps=False, callback=checkpoint_cb)
    else:
        model = SAC(
            "MlpPolicy",
            env,
            learning_rate=3e-4,
            buffer_size=100_000,
            batch_size=256,
            policy_kwargs=dict(net_arch=[256, 256]),
            verbose=1,
        )
        model.learn(total_timesteps=35_000, callback=checkpoint_cb)

    model.save("sac_duckietown_final")
    print("Saved presentation model: sac_duckietown_final.zip")
