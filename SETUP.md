# Environment Setup Guide

Follow these steps in order, in your **WSL Ubuntu 24.04 terminal**. Copy-paste each block, wait for it to finish, then move to the next.

> Note: If your Ubuntu version is different from 24.04, some package names below (`jazzy`) will differ. Check with `lsb_release -a` first and ask the team lead if it's not 24.04.

## 1. Update Ubuntu

```bash
sudo apt update && sudo apt upgrade -y
```

## 2. Add the ROS2 apt repository

```bash
sudo apt install software-properties-common -y
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

## 3. Install ROS2 Jazzy (this takes 15-30 min)

```bash
sudo apt update
sudo apt install ros-jazzy-desktop -y
```

## 4. Source ROS2 automatically every time you open a terminal

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Check it worked:
```bash
ros2 pkg list
```
You should see a long list of package names print out.

## 5. Install Gazebo

```bash
sudo apt install ros-jazzy-ros-gz -y
```

Test it:
```bash
gz sim shapes.sdf
```
A window with basic 3D shapes should open. Close it once confirmed.

## 6. Set up the TurtleBot3 workspace

```bash
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
```

## 7. Install build tools and dependencies

```bash
sudo apt install python3-colcon-common-extensions ros-jazzy-xacro -y
```

## 8. Build the workspace

```bash
cd ~/turtlebot3_ws
colcon build --symlink-install
```

This takes 5-15 minutes. You should see `Summary: 15 packages finished` at the end with **no red "Failed" lines**. If you see errors, screenshot and share with the team.

## 9. Source the workspace and set robot model

```bash
echo "source ~/turtlebot3_ws/install/setup.bash" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

## 10. Final test — launch the simulation

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

A Gazebo window should open showing the TurtleBot3 robot in a world with obstacles. Leave this running.

In a **new terminal window** (open a fresh WSL terminal, don't close the first one):

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

Click into this terminal and press `w`, `a`, `s`, `d` — you should see the robot move in the Gazebo window. If it moves, your setup is complete!

## 11. Clone the project code repo

```bash
cd ~
git clone https://github.com/Yug0407/ES_Project.git
cd ES_Project
```

This is where our actual RL code, custom worlds, and scripts will live going forward.

---

### Common issues

- **`ros2: command not found`** → run `source ~/.bashrc` or open a fresh terminal.
- **`colcon build` fails on `dynamixel_sdk`** → make sure you cloned `DynamixelSDK` in step 6; rerun step 8.
- **Gazebo window doesn't open at all** → make sure you're on Windows 11 or updated Windows 10 with WSLg support; try `wsl --update` in PowerShell, then restart your terminal.
