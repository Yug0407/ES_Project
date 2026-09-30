#!/usr/bin/env python3
import matplotlib.pyplot as plt
import os

def plot_results(log_file='training_log.txt'):
    if not os.path.exists(log_file):
        print("No log file found yet.")
        return

    episodes, rewards = [], []
    with open(log_file, 'r') as f:
        for line in f:
            parts = line.strip().split(',')
            if len(parts) == 2:
                episodes.append(int(parts[0]))
                rewards.append(float(parts[1]))

    plt.figure(figsize=(10, 5))
    plt.plot(episodes, rewards, label='Reward per Episode')
    plt.xlabel('Episode')
    plt.ylabel('Total Reward')
    plt.title('RL Training Performance')
    plt.grid(True)
    plt.savefig('training_curve.png')
    plt.show()

if __name__ == '__main__':
    plot_results()