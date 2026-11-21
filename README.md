# RL-Based Object Detection & Obstacle Avoidance — Gazebo Simulation

**Course:** Embedded Systems — Group Project
**Group:** Group 3

## Problem Statement

Reinforcement Learning (RL) based object detection on the Gazebo simulator, focused on obstacle avoidance and object toppling. The model learns to distinguish obstacles from a valid path using a reward/penalty system that factors in the robot's speed, angular speed, acceleration, and angular acceleration.

## Tech Stack

| Component | Choice | Why |
|---|---|---|
| OS | Ubuntu 24.04 (via WSL2 on Windows) | Required for ROS2 Jazzy compatibility |
| Robot Middleware | ROS2 Jazzy | Latest LTS-adjacent ROS2 release matching Ubuntu 24.04 |
| Simulator | Gazebo (new Gazebo / `ros-jazzy-ros-gz`) | Ships with ROS2 Jazzy; industry-standard robotics simulator |
| Robot Platform | TurtleBot3 (Burger model) | Lightweight, well-documented, widely used for RL/navigation research |
| Obstacle Sensing | LiDAR (`/scan` topic) | Simpler and more reliable for RL state input than camera-based detection |
| RL Algorithm | Q-Learning / lightweight DQN *(planned)* | Feasible to train within our project timeline, unlike full deep RL |
| Language | Python 3 | Standard for ROS2 nodes and RL implementation |
| Version Control | Git + GitHub | Team collaboration |

## Why These Choices

- **TurtleBot3 over other robots:** Most tutorials/community support exist for it, and its LiDAR + differential-drive setup fits our obstacle-avoidance problem directly.
- **LiDAR over camera-based detection:** True vision-based object detection (e.g. YOLO) is a much larger, separate project. Our problem statement asks for obstacle detection via RL rewards/penalties — LiDAR distance readings satisfy this requirement and are far more reliable to get working within our timeframe.
- **Q-Learning/small DQN over deep RL (PPO, SAC, etc.):** Deep RL requires significantly more training time and tuning than we have. A discretized Q-table or small DQN can still demonstrate the core RL concept (state → action → reward) required by the assignment.

## Progress So Far

- [x] Installed ROS2 Jazzy on Ubuntu 24.04 (WSL2)
- [x] Installed Gazebo simulator
- [x] Built TurtleBot3 packages (Burger model) from source (Jazzy branch)
- [x] Verified simulation: robot spawns in a Gazebo world with obstacles
- [x] Verified sensors: LiDAR (`/scan`), odometry (`/odom`), IMU (`/imu`) all publishing correctly
- [x] Verified manual control: robot drives via keyboard teleop
- [x] Set up shared GitHub repo for team collaboration
- [ ] Build custom Gazebo world with a "toppling" object
- [ ] Write Python node to convert LiDAR data into RL state representation
- [ ] Define reward function (based on speed, angular speed, acceleration, collision/toppling)
- [ ] Implement and train Q-Learning / DQN agent
- [ ] Evaluate trained agent's obstacle-avoidance performance
- [ ] Record results and prepare final report/demo

## Setup Instructions

See [`SETUP.md`](./SETUP.md) for the full step-by-step environment setup guide (copy-paste commands for ROS2, Gazebo, and TurtleBot3).

## Simulation Worlds

All worlds live in `worlds/` and run on **Gazebo Harmonic** (SDFormat 1.9).

| World file | Description | Launch command |
|---|---|---|
| `obstacle_world.sdf` | Minimal 20×20 m RL arena — topple-able cylinder | `gz sim -r worlds/obstacle_world.sdf` |
| `road_world.sdf` | 40×20 m road with slope, speed bumps, sliding pedestrians | `gz sim -r worlds/road_world.sdf` |
| `rl_arena.sdf` | 24×24 m full urban city grid (main RL training world) | `gz sim -r worlds/rl_arena.sdf` |
| `city_world.sdf` | High-fidelity city referencing Gazebo Fuel models | `gz sim -r worlds/city_world.sdf` |
| `duckietown_city.sdf` | Duckietown-style city grid (ported from [theconstructcore/riabot](https://bitbucket.org/theconstructcore/riabot)) | `gz sim -r worlds/duckietown_city.sdf` |

### Running the TurtleBot3 in any world

```bash
# Source ROS2 and the TurtleBot3 workspace first (in WSL):
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi

# Duckietown city (dedicated launcher — robot spawns on the road):
ros2 launch launch/duckietown_world.launch.py

# Duckietown city at a specific road position:
ros2 launch launch/duckietown_world.launch.py x_pose:=-4.5 y_pose:=3.0

# RL arena:
ros2 launch launch/road_world_robot.launch.py \
  world_file:=$(pwd)/worlds/rl_arena.sdf

# Obstacle world:
ros2 launch launch/road_world_robot.launch.py \
  world_file:=$(pwd)/worlds/obstacle_world.sdf

# Road world:
ros2 launch launch/road_world_robot.launch.py \
  world_file:=$(pwd)/worlds/road_world.sdf

# City world (default):
ros2 launch launch/road_world_robot.launch.py

# Full RL pipeline (world + robot + gym env node):
ros2 launch launch/rl_environment.launch.py
```

### Duckietown city — notes

- World origin: `(0, 0)` — marked by a green disc on the ground.
- Road rows run east-west at **y = 3**, **y = 9**, **y = 15**.
- To start the robot on the main street: `x_pose:=-4.5 y_pose:=3.0`.
- Road segment textures are in `models/duckietown_models/`.
- Original source: `duckietown_gazebo/worlds/main.world` from [theconstructcore/riabot](https://bitbucket.org/theconstructcore/riabot) (SDF 1.5, classic Gazebo), converted to SDF 1.9 / Gazebo Harmonic.

## Team

- (Add team member names/GitHub usernames here)
