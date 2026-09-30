#!/usr/bin/env python3
import math

class RewardCalculator:
    def __init__(self):
        self.last_v = 0.0
        self.last_w = 0.0

    def compute_reward(self, v, w, min_lidar_dist, dt=0.1):
        # Calculate accelerations
        lin_acc = (v - self.last_v) / dt
        ang_acc = (w - self.last_w) / dt
        
        self.last_v = v
        self.last_w = w

        # Check for collision
        if min_lidar_dist < 0.2:
            return -100.0, True

        # Calculate reward factors
        reward = 0.0
        reward += v * 2.0               # Reward forward velocity
        reward -= abs(w) * 0.5          # Penalize excessive turning
        reward -= abs(lin_acc) * 0.1     # Penalize linear acceleration
        reward -= abs(ang_acc) * 0.1     # Penalize angular acceleration

        return reward, False