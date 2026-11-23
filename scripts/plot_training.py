#!/usr/bin/env python3
import os
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_episode_log(path="training_log.txt"):
    episodes, rewards = [], []
    if not os.path.exists(path):
        return np.array(episodes), np.array(rewards)
    with open(path, "r") as f:
        reader = csv.reader(f)
        for row in reader:
            if len(row) >= 2:
                try:
                    episodes.append(int(row[0]))
                    rewards.append(float(row[1]))
                except ValueError:
                    continue
    return np.array(episodes), np.array(rewards)


def rolling_mean(arr, window=10):
    if len(arr) < window:
        return arr
    kernel = np.ones(window) / window
    return np.convolve(arr, kernel, mode="valid")


def main():
    # Rollout metrics recorded during v4 training
    timesteps = np.array([10443, 12067, 15050, 17433, 18513, 22204, 23810, 27401, 28995, 29741, 31313, 34063, 37421])
    ep_len_mean = np.array([117, 130, 155, 173, 181, 215, 229, 261, 273, 279, 290, 313, 340])
    ep_rew_mean = np.array([-72.7, -71.4, -83.8, -92.4, -98.2, -109.0, -124.0, -126.0, -130.0, -136.0, -130.0, -138.0, -148.0])
    actor_loss = np.array([20.1, 21.5, 25.1, 27.5, 26.9, 31.4, 32.2, 36.2, 41.1, 42.9, 43.9, 45.1, 48.8])
    critic_loss = np.array([4.79, 3.36, 4.06, 6.66, 9.22, 7.61, 10.5, 12.0, 17.3, 14.1, 12.3, 10.2, 8.67])
    ent_coef = np.array([0.182, 0.108, 0.0613, 0.0642, 0.0685, 0.0763, 0.0731, 0.0741, 0.0722, 0.0714, 0.0697, 0.0656, 0.0714])

    episodes, ep_rewards = load_episode_log("training_log.txt")

    fig, axes = plt.subplots(2, 2, figsize=(13, 8.5), dpi=150)
    fig.suptitle("SAC Duckietown Autonomous Driving — Training Diagnostics", fontsize=14, fontweight="bold")

    # 1. Episode Survival Length vs Timesteps
    ax = axes[0, 0]
    ax.plot(timesteps / 1000.0, ep_len_mean, marker="o", color="#1f77b4", linewidth=2)
    ax.set_title("Mean Episode Survival Length (ep_len_mean)")
    ax.set_xlabel("Environment Timesteps (k)")
    ax.set_ylabel("Steps (10 Hz)")
    ax.grid(True, alpha=0.3)

    # 2. Episode Return (Raw Log + Rollout Mean)
    ax = axes[0, 1]
    if len(ep_rewards) > 0:
        ax.scatter(episodes, ep_rewards, s=14, alpha=0.45, color="#7f7f7f", label="Episode Return (raw)")
        if len(ep_rewards) >= 8:
            sm = rolling_mean(ep_rewards, window=8)
            ax.plot(episodes[7:], sm, color="#d62728", linewidth=2, label="8-Ep Moving Avg")
        ax.set_xlabel("Episode")
        ax.legend(loc="best")
    else:
        ax.plot(timesteps / 1000.0, ep_rew_mean, marker="s", color="#d62728", linewidth=2)
        ax.set_xlabel("Environment Timesteps (k)")
    ax.set_title("Episode Return Progression")
    ax.set_ylabel("Cumulative Reward")
    ax.grid(True, alpha=0.3)

    # 3. Actor & Critic Losses
    ax = axes[1, 0]
    ax.plot(timesteps / 1000.0, critic_loss, marker="o", color="#2ca02c", linewidth=2, label="Critic Loss (MSE)")
    ax.set_xlabel("Environment Timesteps (k)")
    ax.set_ylabel("Critic Loss", color="#2ca02c")
    ax.tick_params(axis="y", labelcolor="#2ca02c")
    ax.grid(True, alpha=0.3)

    ax_twin = ax.twinx()
    ax_twin.plot(timesteps / 1000.0, actor_loss, marker="^", linestyle="--", color="#ff7f0e", linewidth=2, label="Actor Loss")
    ax_twin.set_ylabel("Actor Loss", color="#ff7f0e")
    ax_twin.tick_params(axis="y", labelcolor="#ff7f0e")
    ax.set_title("SAC Actor & Critic Losses")

    # 4. Entropy Temperature (Exploration Decay)
    ax = axes[1, 1]
    ax.plot(timesteps / 1000.0, ent_coef, marker="d", color="#9467bd", linewidth=2)
    ax.set_title("Entropy Regularization Coefficient (ent_coef)")
    ax.set_xlabel("Environment Timesteps (k)")
    ax.set_ylabel("Alpha (ent_coef)")
    ax.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.95])
    out_path = "plots/training_curves.png"
    plt.savefig(out_path)
    print(f"Saved training curves to {out_path}")


if __name__ == "__main__":
    main()
