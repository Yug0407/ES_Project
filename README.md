# RL-Based Obstacle Avoidance & Object Detection in Gazebo

**Course:** Embedded Systems — Group Project  
**Platform:** ROS 2 Jazzy + Gazebo Harmonic + TurtleBot3 (Waffle Pi)  
**Host Environment:** Windows 10/11 with WSL2 (Ubuntu 24.04 LTS)

---

## 📌 Project Overview

This project implements a Reinforcement Learning (RL) pipeline for autonomous mobile robot navigation, obstacle detection, and collision/toppling avoidance using simulated sensor data (LiDAR + Odometry).

The simulation features multiple custom Gazebo worlds (urban city grids, road networks with speed bumps and slopes, obstacle arenas, and Duckietown city tracks) running on **Gazebo Harmonic (SDF 1.9)** and **ROS 2 Jazzy**.

---

## 💻 Tech Stack & Requirements

| Component | Specification | Description |
|---|---|---|
| **Host OS** | Windows 10 (Build 19044+) or Windows 11 | Supports WSL2 and WSLg (GUI acceleration) |
| **Linux Environment** | Ubuntu 24.04 LTS (WSL2) | Required for ROS 2 Jazzy |
| **Middleware** | ROS 2 Jazzy Jalisco | Core robotics communication framework |
| **Simulator** | Gazebo Harmonic (`ros-jazzy-ros-gz`) | Modern Gazebo simulation engine |
| **Robot Model** | TurtleBot3 (Waffle Pi) | Differential drive robot equipped with 360° LiDAR, IMU, and Camera |
| **Programming** | Python 3.12 | Nodes, RL environments, and data analysis |

---

## 🚀 Complete Installation Guide

Follow these steps sequentially inside your **WSL Ubuntu 24.04 terminal**.

### 1. Update Ubuntu System Packages
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y software-properties-common curl git build-essential python3-pip
```

### 2. Install ROS 2 Jazzy
Add the official ROS 2 Jazzy apt repository:
```bash
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-jazzy-desktop
```

### 3. Install Gazebo Harmonic & ROS-Gazebo Bridge
```bash
sudo apt install -y ros-jazzy-ros-gz ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-image
```

### 4. Install TurtleBot3 Packages
Install the pre-built ROS 2 Jazzy TurtleBot3 suite:
```bash
sudo apt install -y ros-jazzy-turtlebot3* ros-jazzy-dynamixel-sdk
```

### 5. Clone This Repository
```bash
cd ~
git clone https://github.com/Yug0407/ES_Project.git
cd ~/ES_Project
```
*(If you are working directly from Windows, your repo path in WSL is `/mnt/c/Users/<YourUsername>/OneDrive/Desktop/ES_Project`)*.

### 6. Configure Environment Variables Automatically
Add environment configuration to your `~/.bashrc` so ROS 2 and TurtleBot3 settings load automatically every time you open a terminal:
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
source ~/.bashrc
```

Verify the installation:
```bash
ros2 pkg list | grep turtlebot3
```
You should see packages like `turtlebot3_gazebo`, `turtlebot3_teleop`, and `turtlebot3_msgs`.

---

## 🎮 How to Start and Run the Simulation

Always launch the simulation in **Terminal 1** and your controller/scripts in **Terminal 2**.

### Step 1: Launch Gazebo & Spawn the Car (Terminal 1)

Navigate to the project root:
```bash
cd ~/ES_Project
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
```

Choose any of the following worlds to launch:

#### Option A: Duckietown City World (Recommended)
Spawns the TurtleBot3 directly on the central roadway:
```bash
ros2 launch launch/duckietown_world.launch.py x_pose:=-4.5 y_pose:=3.0
```

#### Option B: Full Urban RL Arena
24×24 m city grid with perimeter walls and buildings:
```bash
ros2 launch launch/road_world_robot.launch.py \
  world_file:=$(pwd)/worlds/rl_arena.sdf \
  x_pose:=-9.0 y_pose:=-9.0
```

#### Option C: Road World with Slopes & Speed Bumps
40×20 m road featuring speed bumps, incline slopes, and obstacles:
```bash
ros2 launch launch/road_world_robot.launch.py \
  world_file:=$(pwd)/worlds/road_world.sdf \
  x_pose:=-15.0 y_pose:=0.0
```

#### Option D: Basic Obstacle Arena
20×20 m open arena with topple-able cylinders:
```bash
ros2 launch launch/road_world_robot.launch.py \
  world_file:=$(pwd)/worlds/obstacle_world.sdf \
  x_pose:=0.0 y_pose:=0.0
```

#### Option E: City World (Default)
High-fidelity urban environment:
```bash
ros2 launch launch/road_world_robot.launch.py \
  world_file:=$(pwd)/worlds/city_world.sdf \
  x_pose:=0.0 y_pose:=0.0
```

> [!IMPORTANT]
> **Gazebo Play Button:** In Gazebo Sim, make sure the simulation is active. If the timer in the bottom bar is paused, click the **Play (▶)** button on the bottom-left corner so physics and sensors run.

---

### Step 2: Drive and Control the Car (Terminal 2)

Open a **second WSL terminal** while Gazebo is running:

```bash
cd ~/ES_Project
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
```

#### 🕹️ Manual Keyboard Teleoperation (Drive with keys):
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```
* **`w`**: Move forward  
* **`x`**: Move backward  
* **`a`**: Turn left  
* **`d`**: Turn right  
* **`s`**: Emergency stop  

> [!TIP]
> **Click inside Terminal 2:** You must click inside the terminal window running `teleop_keyboard` to focus it before pressing keys.

---

### Step 3: Monitor Sensors & RL Nodes (Terminal 2 or 3)

#### 📡 5-Sector LiDAR Observation Monitor:
Converts 360 raw LiDAR rays into directional proximity zones (`front`, `front_left`, `front_right`, `left`, `right`):
```bash
python3 scripts/lidar_state.py
```

#### 🤖 RL Environment Step Node:
Runs the gym step loop, calculates speed/acceleration rewards, and sends discrete actions:
```bash
python3 scripts/gazebo_env_node.py
```

#### 📊 Inspect Raw ROS 2 Topics:
```bash
# Print LiDAR messages
ros2 topic echo /scan

# Print robot position & velocity
ros2 topic echo /odom

# List all active simulation topics
ros2 topic list
```

---

## 🛠️ Troubleshooting & Common Fixes

### 1. The Car Is Not Moving
* **Click into the Teleop Terminal:** If the Gazebo 3D window is focused, pressing `W/A/S/D` will only move the 3D camera. Click into the teleop terminal.
* **Check the Play button:** Ensure Gazebo's physics engine is running (Play ▶ icon at the bottom-left).
* **ROS 2 Jazzy Twist format:** ROS 2 Jazzy requires `geometry_msgs/msg/TwistStamped` over `/cmd_vel` rather than un-stamped `Twist`.

### 2. Gazebo Freezes or Says "Found additional publishers"
If a previous simulation wasn't stopped cleanly, kill all background simulation processes and restart:
```bash
killall -9 gz-sim-server gz-sim-gui ruby ros2 parameter_bridge image_bridge 2>/dev/null
```

### 3. Gazebo Window Does Not Open
* Ensure WSLg is up to date in Windows PowerShell (Admin):
  ```powershell
  wsl --update
  ```
* Inside WSL, check that display forwarding is active:
  ```bash
  echo $DISPLAY
  ```
  *(Should print `:0`)*.

---

## 📂 Repository Structure

```
ES_Project/
├── launch/
│   ├── duckietown_world.launch.py   # Dedicated Duckietown launcher + robot spawner
│   ├── road_world_robot.launch.py   # Generic launcher for any world + robot spawner
│   ├── city_simulation.launch.py    # City world launcher
│   └── rl_environment.launch.py     # World + robot + Gym RL node pipeline
├── models/
│   └── duckietown_models/           # Road plane & intersection textures (road3.png, etc.)
├── scripts/
│   ├── gazebo_env_node.py           # RL environment node (action step & reward calculation)
│   ├── lidar_state.py               # 5-zone LiDAR state classifier
│   ├── reward_calculator.py         # Multi-factor reward function (v, w, a_lin, a_ang)
│   └── plot_training.py             # Script to visualize RL training rewards
├── worlds/
│   ├── duckietown_city.sdf          # Converted Duckietown city grid (SDF 1.9)
│   ├── rl_arena.sdf                 # Urban city grid (main RL training arena)
│   ├── road_world.sdf               # Road with slope, speed bumps, and obstacles
│   ├── obstacle_world.sdf           # Minimal 20x20m obstacle arena
│   └── city_world.sdf               # High-fidelity urban city environment
├── README.md                        # Project documentation & setup instructions
└── SETUP.md                         # Detailed environment setup notes
```
