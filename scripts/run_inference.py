#!/usr/bin/env python3
import argparse
import csv
import datetime
import math
import os
import time
import numpy as np

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Imu, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from stable_baselines3 import SAC


def quaternion_to_euler(x, y, z, w):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    return roll, pitch


class DuckietownPresenter:
    def __init__(self, model_path: str, dt: float = 0.1, device: str = "cpu", max_steps: int = 0):
        self.model_path = model_path
        self.dt = dt
        self.max_steps = max_steps
        self.model = SAC.load(model_path, device=device)

        self.v_min, self.v_max = 0.0, 0.26
        self.w_max = 1.82
        self.a_max = 0.5
        self.alpha_max = 2.5
        self.num_sectors = 16
        self.max_lidar_range = 3.5

        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("duckietown_sac_presenter")
        self.cmd_pub = self.node.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.node.create_subscription(LaserScan, "/scan", self._scan_cb, qos_profile_sensor_data)
        self.node.create_subscription(Image, "/camera/image_raw", self._image_cb, qos_profile_sensor_data)
        self.node.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.node.create_subscription(Imu, "/imu", self._imu_cb, qos_profile_sensor_data)

        self.lidar_sectors = np.full(self.num_sectors, self.max_lidar_range, dtype=np.float32)
        self.prev_lidar_sectors = np.full(self.num_sectors, self.max_lidar_range, dtype=np.float32)
        self.lidar_rates = np.zeros(self.num_sectors, dtype=np.float32)

        self.cam_features = np.zeros(24, dtype=np.float32)
        self._last_cam_time = 0.0

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

    def _scan_cb(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges = np.nan_to_num(
            ranges, nan=self.max_lidar_range, posinf=self.max_lidar_range, neginf=0.0
        )
        ranges = np.clip(ranges, 0.01, self.max_lidar_range)
        chunks = np.array_split(ranges, self.num_sectors)
        self.lidar_sectors = np.array([np.min(c) for c in chunks], dtype=np.float32)

    def _extract_band_features(self, band: np.ndarray):
        strips = np.array_split(band, 3, axis=1)
        feats = []
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
        return feats

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
        self.cam_features = np.array(
            self._extract_band_features(near_band) + self._extract_band_features(lookahead_band),
            dtype=np.float32,
        )

    def _odom_cb(self, msg: Odometry):
        self.v = float(msg.twist.twist.linear.x)
        self.w = float(msg.twist.twist.angular.z)

    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        self.roll, self.pitch = quaternion_to_euler(q.x, q.y, q.z, q.w)

    def _build_obs(self) -> np.ndarray:
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

    def _publish_cmd(self, v_cmd: float, w_cmd: float):
        msg = TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(v_cmd)
        msg.twist.angular.z = float(w_cmd)
        self.cmd_pub.publish(msg)

    def run(self):
        os.makedirs("logs", exist_ok=True)
        run_ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        trace_path = os.path.join("logs", f"inference_trace_{run_ts}.csv")
        summary_path = os.path.join("logs", "inference_summary.csv")

        warmup_end = time.time() + 0.5
        while time.time() < warmup_end:
            rclpy.spin_once(self.node, timeout_sec=0.01)
        self.prev_lidar_sectors = self.lidar_sectors.copy()

        print(f"[INFO] Running deterministic SAC policy -> logging to {trace_path} (Ctrl+C to stop)...")
        step = 0
        speeds, yaw_rates, min_clearances, asphalt_ratios = [], [], [], []

        with open(trace_path, "w", newline="") as trace_file:
            writer = csv.writer(trace_file)
            writer.writerow([
                "step", "time_s", "v_cmd", "w_cmd", "v", "w",
                "front_lidar", "min_lidar", "near_asphalt", "near_yellow", "lookahead_grass",
                "roll", "pitch"
            ])

            try:
                while rclpy.ok():
                    if self.max_steps > 0 and step >= self.max_steps:
                        break
                    t0 = time.time()
                    rclpy.spin_once(self.node, timeout_sec=0.01)

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

                    obs = self._build_obs()
                    action, _ = self.model.predict(obs, deterministic=True)

                    v_target = self.v_min + (float(action[0]) + 1.0) * 0.5 * (self.v_max - self.v_min)
                    w_target = float(action[1]) * self.w_max

                    v_cmd = float(np.clip(v_target, self.v - self.a_max * self.dt, self.v + self.a_max * self.dt))
                    w_cmd = float(np.clip(w_target, self.w - self.alpha_max * self.dt, self.w + self.alpha_max * self.dt))
                    self._publish_cmd(v_cmd, w_cmd)

                    step += 1
                    t_sim = round(step * self.dt, 2)
                    front_dist = float(min(self.lidar_sectors[0], self.lidar_sectors[15], self.lidar_sectors[1]))
                    min_dist = float(np.min(self.lidar_sectors))
                    near_asphalt = float(np.mean([self.cam_features[0], self.cam_features[4], self.cam_features[8]]))
                    near_yellow = float(np.mean([self.cam_features[1], self.cam_features[5], self.cam_features[9]]))
                    look_grass = float(np.mean([self.cam_features[15], self.cam_features[19], self.cam_features[23]]))

                    writer.writerow([
                        step, t_sim, round(v_cmd, 4), round(w_cmd, 4), round(self.v, 4), round(self.w, 4),
                        round(front_dist, 3), round(min_dist, 3), round(near_asphalt, 3),
                        round(near_yellow, 3), round(look_grass, 3), round(self.roll, 4), round(self.pitch, 4)
                    ])

                    speeds.append(self.v)
                    yaw_rates.append(abs(self.w))
                    min_clearances.append(min_dist)
                    asphalt_ratios.append(near_asphalt)

                    if step % 20 == 0:
                        print(
                            f"[t={t_sim:5.1f}s] v={self.v:.3f} m/s | w={self.w:+.3f} rad/s | "
                            f"front_lidar={front_dist:.2f} m | near_asphalt={near_asphalt:.2f}"
                        )

                    elapsed = time.time() - t0
                    if elapsed < self.dt:
                        time.sleep(self.dt - elapsed)
            except KeyboardInterrupt:
                pass
            finally:
                self._publish_cmd(0.0, 0.0)
                self.node.destroy_node()

        if speeds:
            duration_s = round(step * self.dt, 1)
            mean_v = float(np.mean(speeds))
            max_v = float(np.max(speeds))
            mean_w = float(np.mean(yaw_rates))
            min_clr = float(np.min(min_clearances))
            near_cols = int(sum(d < 0.21 for d in min_clearances))
            mean_asph = float(np.mean(asphalt_ratios))

            write_header = not os.path.exists(summary_path)
            with open(summary_path, "a", newline="") as sum_file:
                s_writer = csv.writer(sum_file)
                if write_header:
                    s_writer.writerow([
                        "timestamp", "model", "duration_s", "mean_speed_mps", "max_speed_mps",
                        "mean_abs_yaw_radps", "min_clearance_m", "near_collisions", "mean_near_asphalt", "trace_file"
                    ])
                s_writer.writerow([
                    run_ts, self.model_path, duration_s, round(mean_v, 4), round(max_v, 4),
                    round(mean_w, 4), round(min_clr, 3), near_cols, round(mean_asph, 3), trace_path
                ])

            print(
                f"\n=== RUNTIME SUMMARY ({step} steps / {duration_s}s) ===\n"
                f"Mean speed:        {mean_v:.3f} m/s (max {max_v:.3f} m/s)\n"
                f"Mean |yaw rate|:   {mean_w:.3f} rad/s\n"
                f"Min clearance:     {min_clr:.3f} m\n"
                f"Near collisions:   {near_cols}\n"
                f"Mean near asphalt: {mean_asph:.2f}\n"
                f"Saved trace:       {trace_path}\n"
                f"Appended summary:  {summary_path}"
            )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="sac_duckietown_final.zip", help="Path to trained SAC .zip model")
    parser.add_argument("--dt", type=float, default=0.1, help="Control loop period (s)")
    parser.add_argument("--max-steps", type=int, default=0, help="Stop after N steps (0 = run until Ctrl+C)")
    args = parser.parse_args()
    DuckietownPresenter(model_path=args.model, dt=args.dt, max_steps=args.max_steps).run()


if __name__ == "__main__":
    main()
