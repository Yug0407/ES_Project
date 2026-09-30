# World Creation Guide: `rl_arena.sdf`

This document details the step-by-step evolution and technical design of the `rl_arena.sdf` world file for **Gazebo Sim (Harmonic / ROS 2 Jazzy)**, created for testing and Reinforcement Learning (RL) navigation with the **TurtleBot3 Burger**.

---

## Table of Contents
1. [Simulation Foundation & Core Plugins](#1-simulation-foundation--core-plugins)
2. [Stage 1: Base 10m x 10m Enclosed Arena](#2-stage-1-base-10m-x-10m-enclosed-arena)
3. [Stage 2: Static Obstacle Integration & Clearance Verification](#3-stage-2-static-obstacle-integration--clearance-verification)
4. [Stage 3: Realistic Urban Road Track Features](#4-stage-3-realistic-urban-road-track-features)
5. [Stage 4: Expansion to 20m x 20m with Dynamic Actors](#5-stage-4-expansion-to-20m-x-20m-with-dynamic-actors)
6. [Stage 5: Comprehensive 24m x 24m Urban City Grid & Infinite Traffic](#6-stage-5-comprehensive-24m-x-24m-urban-city-grid--infinite-traffic)
7. [World Inspection & Launch Instructions](#7-world-inspection--launch-instructions)

---

## 1. Simulation Foundation & Core Plugins

Every stage of `rl_arena.sdf` uses **SDFormat 1.9** with the following foundational parameters:

- **Physics Engine**: Open Dynamics Engine (ODE) configured for real-time discrete time-stepping:
  - `<max_step_size>`: `0.001` (1 ms per step)
  - `<real_time_update_rate>`: `1000` (1 kHz simulation frequency)
  - `<gravity>`: `0 0 -9.8`
- **Essential Gazebo Sim Plugins**:
  - `gz::sim::systems::Physics` (`gz-sim-physics-system`): Executes rigid body dynamics, joint constraints, and collision response.
  - `gz::sim::systems::UserCommands` (`gz-sim-user-commands-system`): Handles GUI interactions, spawning models, and user modifications.
  - `gz::sim::systems::SceneBroadcaster` (`gz-sim-scene-broadcaster-system`): Streams visual state and transforms to the Gazebo GUI.
  - `gz::sim::systems::Contact` (`gz-sim-contact-system`): Computes contact points and collision forces for sensor feedback.
- **Lighting**: Directional sunlight source (`sun`) positioned high above the scene (`z = 18m - 20m`) with cast shadows enabled and natural diffuse/specular balance.

---

## 2. Stage 1: Base 10m x 10m Enclosed Arena

### Objective
Create a flat, clean, enclosed perimeter to ensure robots cannot escape the boundary.

### Implementation Details
- **Coordinate Space**: Bounded between $X \in [-5.0\text{m}, +5.0\text{m}]$ and $Y \in [-5.0\text{m}, +5.0\text{m}]$.
- **Ground**: Flat ground plane model with plane normal `(0, 0, 1)` and size `20m x 20m`.
- **Perimeter Boundary Walls**:
  - 4 static box models (`wall_north`, `wall_south`, `wall_east`, `wall_west`).
  - Height: $0.5\,\text{m}$ (vertical center $z = 0.25\,\text{m}$ so walls rest exactly on the ground).
  - Thickness: $0.15\,\text{m}$.
  - Overlapping length ($10.15\,\text{m}$ on outer walls) ensures seamless corner closure with no gaps.
- **Robot Clearance**: Center $(0, 0, 0)$ kept open and obstacle-free for robot initialization.

---

## 3. Stage 2: Static Obstacle Integration & Clearance Verification

### Objective
Introduce symmetric static geometry to test basic obstacle avoidance and path planning.

### Implementation Details
- **4 Cylindrical Pillars**:
  - Radius: $0.3\,\text{m}$, Height: $0.6\,\text{m}$ ($z = 0.3\,\text{m}$).
  - Symmetrically placed at $(\pm 2.0\,\text{m}, \pm 2.0\,\text{m})$.
  - Clearance calculation: Euclidean distance from $(0, 0)$ is $\sqrt{2^2 + 2^2} \approx 2.83\,\text{m}$. Subtracting radius ($0.3\,\text{m}$) leaves $\approx 2.53\,\text{m}$ clearance to the robot.
- **2 Rectangular Barrier Blocks**:
  - Dimensions: $0.3\,\text{m}\,(X) \times 2.0\,\text{m}\,(Y) \times 0.5\,\text{m}\,(Z)$.
  - Placed at $(0.0, +2.5\,\text{m})$ and $(0.0, -2.5\,\text{m})$.
  - Closest edge to center is at $y = \pm 1.5\,\text{m}$, satisfying the minimum $1.5\,\text{m}$ spawn clearance rule.
- **Continuous Corridors**:
  - Preserved a wide $3.0\,\text{m}$ unobstructed East-West corridor through the center and perimeter passages exceeding $2.5\,\text{m}$ width.

---

## 4. Stage 3: Realistic Urban Road Track Features

### Objective
Transform abstract primitive obstacles into a road test track testing TurtleBot3 sensors (IMU pitch, LiDAR, jerk, braking).

### Implementation Details
- **Dark Asphalt Road Base**:
  - Ground material tinted to asphalt RGB `(0.15, 0.15, 0.15)` for visual realism and high contrast with lane markings.
- **Painted Markings ($Z = 0.002\,\text{m}$)**:
  - Thin flat boxes raised $1\,\text{mm}$ above ground level to eliminate z-fighting:
    - Double yellow central dividing line (`1 0.8 0`) separating driving lanes.
    - Solid white outer edge lines (`1 1 1`).
    - White zebra crosswalk stripes at $x = -1.5\,\text{m}$.
- **Incline Ramp, Plateau & Descent (Elevation Test)**:
  - Tests pitch angles and IMU acceleration without bottoming out the Burger's low ground clearance ($15\text{--}20\,\text{mm}$):
    - Incline: Length $2.0\,\text{m}$, Width $1.2\,\text{m}$, tilted pitch $\approx 7.1^\circ$ ($0.122\,\text{rad}$), rising from $z = 0$ to $z = 0.25\,\text{m}$.
    - Plateau: Length $1.0\,\text{m}$, Width $1.2\,\text{m}$, elevated at $z = 0.25\,\text{m}$.
    - Descent: Length $2.0\,\text{m}$, Width $1.2\,\text{m}$, tilted pitch $\approx -7.1^\circ$ ($-0.122\,\text{rad}$).
- **Speed Breaker (IMU Vertical Jerk & Braking Test)**:
  - Arched profile modeled using a submerged horizontal cylinder ($R = 0.2\,\text{m}$) where only the top $0.04\,\text{m}$ (height) and $0.25\,\text{m}$ (width) emerges above ground.
  - Finished with bright yellow paint and contrast hazard striping across the lane.
- **Urban Road Props**:
  - `pedestrian_target`: Standing silhouette target ($0.4\,\text{m} \times 0.3\,\text{m} \times 1.7\,\text{m}$) with distinct torso clothing and head geometry.
  - `jersey_barrier_east` and `jersey_barrier_west`: Heavy highway barricades with reflective stripes flanking the turns.

---

## 5. Stage 4: Expansion to 20m x 20m with Dynamic Actors

### Objective
Double the arena area to $20\text{m} \times 20\text{m}$ and incorporate dynamic moving obstacles.

### Implementation Details
- **Boundary Wall Expansion**: Wall spans increased to $20\,\text{m}$ ($X, Y \in [-10\,\text{m}, +10\,\text{m}]$), ground base enlarged to $30\,\text{m} \times 30\,\text{m}$.
- **Dynamic Moving Pedestrian (`moving_pedestrian`)**:
  - Cylindrical body ($H = 1.6\,\text{m}, R = 0.25\,\text{m}$) with head geometry and full collision/visual enabled for both GPU and 2D planar LiDAR.
  - Scripted via `gz::sim::systems::TrajectoryFollower`:
    - Waypoints between $(2.0, -2.5)$ and $(2.0, 2.5)$ at walking velocity $\sim 0.6\,\text{m/s}$.
    - Set `<gravity>false</gravity>` to eliminate ground friction drag and tipping.
    - Set `<loop>true</loop>` for infinite reciprocating crossing.
- **Dual Speed Bumps**:
  - Bump 1 placed along the main straight at $(-1.0, -7.0)$.
  - Bump 2 placed approaching the turn at $(7.0, 4.5)$ to enforce cornering deceleration.
- **Elevated Bridge**:
  - $7\,\text{m}$ total flyover structure along $y = 7.0\,\text{m}$ ($2\,\text{m}$ incline, $3\,\text{m}$ bridge deck at $z = 0.25\,\text{m}$ with side guardrails, $2\,\text{m}$ descent).
- **Chicane Road Work Bottlenecks**:
  - Two staggered construction barricades on the west corridor forcing single-lane navigation.
- **Stationary Props**:
  - Curbside vehicle silhouette ($3\,\text{m} \times 1.5\,\text{m} \times 1.1\,\text{m}$) and industrial metal dumpster.
- **Robot Spawn**: Initialized at $(-7.0, -7.0, 0.05)$ facing forward along the $+X$ lane.

---

## 6. Stage 5: Comprehensive 24m x 24m Urban City Grid & Infinite Traffic

### Objective
Create a structured city layout with defined city blocks, dual intersections, and continuous infinite-loop dynamic traffic.

### Implementation Details

### 1. Urban Grid Architecture ($24\,\text{m} \times 24\,\text{m}$)
- **Boundary Walls**: Spans $X, Y \in [-12.0\,\text{m}, +12.0\,\text{m}]$ with height $0.5\,\text{m}$ and thickness $0.15\,\text{m}$.
- **Ground Base**: $34\,\text{m} \times 34\,\text{m}$ dark asphalt plane.
- **Two Central City Blocks (Raised Pedestrian Islands)**:
  - **Block A (North Island)**: Box $7.0\,\text{m}\,(X) \times 5.0\,\text{m}\,(Y) \times 0.15\,\text{m}\,(Z)$ centered at $(0.0, 4.5, 0.075)$. Concrete material (`0.6 0.6 0.6`).
  - **Block B (South Island)**: Box $7.0\,\text{m}\,(X) \times 5.0\,\text{m}\,(Y) \times 0.15\,\text{m}\,(Z)$ centered at $(0.0, -4.5, 0.075)$. Concrete material (`0.6 0.6 0.6`).
  - **Urban Structures**: Added central kiosks/pavilions atop both islands to serve as 3D visual landmarks and occlusion obstacles for LiDAR.
- **Roadway Network**:
  - Outer perimeter ring road ($3.5\,\text{m}$ drivable lane width).
  - Central East-West cross avenue between the two city blocks ($Y \in [-1.5\,\text{m}, +1.5\,\text{m}]$), creating dual 4-way intersections at $x = -3.5\,\text{m}$ and $x = +3.5\,\text{m}$.
  - Markings: Solid white outer curb lines at $\pm 10.8\,\text{m}$, dashed yellow lane dividers along ring roads at $\pm 9.0\,\text{m}$ and along cross avenue at $y = 0.0\,\text{m}$, with 8 painted crosswalk stripes across the intersections.

### 2. Infinite Loop Dynamic Obstacles
- **Actor 1: Figure-8 Dynamic Pedestrian (`dynamic_pedestrian_1`)**:
  - Implemented using native SDF `<actor>` with `<loop>true</loop>` and `<auto_start>true</auto_start>`.
  - Collision cylinder ($R = 0.25\,\text{m}, L = 1.6\,\text{m}$) and multi-part visual body/head.
  - Scripted trajectory with 2-second waypoint intervals completing a closed figure-8:
    $$\begin{aligned}
    t = 0.0\text{s} &: (-3.0, 0.0, 0.85), \text{yaw} = +0.4636\,\text{rad} \\
    t = 2.0\text{s} &: (0.0, 1.5, 0.85), \text{yaw} = -0.4636\,\text{rad} \\
    t = 4.0\text{s} &: (3.0, 0.0, 0.85), \text{yaw} = -2.6779\,\text{rad} \\
    t = 6.0\text{s} &: (0.0, -1.5, 0.85), \text{yaw} = +2.6779\,\text{rad} \\
    t = 8.0\text{s} &: (-3.0, 0.0, 0.85), \text{yaw} = +0.4636\,\text{rad}
    \end{aligned}$$
  - Start and end waypoints match identically in position and heading, ensuring jerk-free looping.
- **Model 2: Autonomous Patrol Vehicle Dummy (`automated_patrol_vehicle`)**:
  - Patrol cart model ($1.8\,\text{m} \times 0.9\,\text{m} \times 0.6\,\text{m}$) equipped with amber warning flasher and headlights.
  - Controlled by `gz::sim::systems::TrajectoryFollower` with `<loop>true</loop>` cycling the 4 perimeter corners:
    $$(9.0, 9.0) \to (9.0, -9.0) \to (-9.0, -9.0) \to (-9.0, 9.0) \to (9.0, 9.0)$$
  - Starts at $(9.0, 9.0)$, safely opposite the robot spawn point.

### 3. Structured Elevation & Speed Challenges
- **Speed Bump 1**: Central cross avenue at $(-1.5\,\text{m}, 0.0\,\text{m})$, $3.0\,\text{m}$ width, $0.3\,\text{m}$ length, $0.04\,\text{m}$ arched profile with black-and-yellow striping.
- **Speed Bump 2**: East ring road at $(9.0\,\text{m}, 0.0\,\text{m})$, $3.0\,\text{m}$ width, $0.3\,\text{m}$ length, $0.04\,\text{m}$ arched profile.
- **North Flyover Bridge ($y = 9.0\,\text{m}$)**:
  - Incline ramp ($x \in [-3.5\,\text{m}, -1.5\,\text{m}]$, $3\,\text{m}$ lane width, $\approx 7.1^\circ$ pitch, $0.25\,\text{m}$ rise).
  - Elevated straight span ($x \in [-1.5\,\text{m}, +1.5\,\text{m}]$ at $z = 0.25\,\text{m}$) with safety guard rails.
  - Descent ramp ($x \in [+1.5\,\text{m}, +3.5\,\text{m}]$, slopes smoothly to ground level).

### 4. Spawn Zone
- Position: `(-9.0, -9.0, 0.05)`.
- Heading: North ($+Y$) along the open outer lane.
- Completely clear of obstacles, speed bumps, and dynamic traffic on startup.

---

## 7. Bug Fixes & Physics Stability Optimization

### 1. Mesh Manager Error Resolution (`Invalid mesh filename extension: __default__`)
- **Root Cause**: When an SDFormat `<actor>` element is declared without an explicit `<skin><filename>`, SDFormat's internal Actor DOM assigns `__default__` as the skin filename. Gazebo Sim's `MeshManager` subsequently attempts to resolve this filename, fails to find a valid 3D mesh extension (e.g., `.dae` or `.obj`), and throws `[Err] [MeshManager.cc:150] Invalid mesh filename extension: __default__`.
- **Solution**:
  - Replaced the `<actor>` tag with a native `<model name="dynamic_pedestrian_1">`.
  - Constructed the pedestrian using only native primitives: `<cylinder>` for the torso/legs ($R = 0.25\,\text{m}, L = 1.6\,\text{m}$) and `<sphere>` for the head ($R = 0.14\,\text{m}$).
  - Retained the figure-8 loop path using the verified `gz::sim::systems::TrajectoryFollower` plugin with `<loop>true</loop>`.

### 2. Elimination of Flying / Ejected Dynamic Vehicles
  - **Re-Enabling Trajectory Animations**:
    - After ensuring strict Z-axis placement, the `gz-sim-trajectory-follower-system` plugins were successfully re-enabled for both the `dynamic_pedestrian_1` and `automated_patrol_vehicle`.
    - Both models have `<static>false</static>`, `<kinematic>true</kinematic>`, and `<gravity>false</gravity>` to allow smooth scripted movement without gravity-induced drift or collision ejection.
  - **Exact Ground Placement (Zero Interpenetration)**:
    - To prevent any contact issues with the ground plane, Z-axis origins are set strictly to `Height / 2` so bases rest perfectly flush.
    - `automated_patrol_vehicle`: Center positioned at $z = 0.4\,\text{m}$.
    - `dynamic_pedestrian_1`: Cylinder Length $1.6\,\text{m}$, center positioned at $z = 0.8\,\text{m}$.
  - **Visual-Only Road Markings**:
    - All yellow and white lane stripes, dividing dashes, and crosswalk markings are strictly `<static>true</static>` with `<visual>` tags ONLY (all `<collision>` tags stripped), eliminating any surface collision snags.
  - **Stationary Models**:
    - All walls (`wall_*`), city blocks (`city_block_*`), speed bumps, and flyover ramps strictly enforce `<static>true</static>`.

---

## 8. World Inspection & Launch Instructions

To launch and test the world in **WSL (Ubuntu 24.04)**:

```bash
cd /mnt/c/Users/neelm/OneDrive/Desktop/ES_Project
gz sim -r worlds/rl_arena.sdf
```

> **Note**: Both `worlds/rl_arena.sdf` and `rl_arena.sdf` (root) are kept synchronized for convenience.


---

## 9. Stage 6: Modular Structure & High-Fidelity City Simulation

### Objective
Restructure the project for scalability using professional ROS 2 directory conventions and upgrade the visual quality by transitioning from primitive shapes to high-fidelity Gazebo Fuel models.

### Implementation Details
- **Modular Directory Architecture**:
  - `worlds/`: Holds the distinct environments (`city_world.sdf`, `rl_arena.sdf`).
  - `models/`: Destination directory for all downloaded meshes and Fuel assets.
  - `scripts/`: Holds utility scripts like `download_city_assets.sh`.
  - `launch/`: Contains the ROS 2 / Python launch files (e.g., `city_simulation.launch.py`).
  - `src/`: Prepared for future RL and Gym training nodes.
- **High-Fidelity Assets (`worlds/city_world.sdf`)**:
  - Replaced primitive city blocks with official OpenRobotics models pulled directly via `<uri>` links (e.g., `Gas Station`, `House 1`, `House 2`, `SUV`, and `Pine Tree`).
- **Required Track Features Retained**:
  - A clean 2-lane asphalt loop for continuous navigation.
  - Standardized Yellow Speed Bump placed on the West Road.
  - Incline/Decline Slope Ramp placed on the East Road.
  - A completely open, flat intersection origin `(0,0,0)` reserved for the robot's safe initial spawn.
- **Launch Integration**:
  - Created `scripts/download_city_assets.sh` to properly format the local `GZ_SIM_RESOURCE_PATH`.
  - Authored `launch/city_simulation.launch.py` using standard `LaunchDescription` and `ExecuteProcess` to programmatically start Gazebo Sim with the correct environment variables pointing to `worlds/city_world.sdf`.
