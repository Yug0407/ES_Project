#!/usr/bin/env python3
"""
road_world_robot.launch.py
===========================================================================
Generic launcher: Gazebo + TurtleBot3 (waffle_pi).
Accepts any ES_Project SDF world via the `world_file` argument.

Usage (default — city_world.sdf):
    ros2 launch launch/road_world_robot.launch.py

Usage (explicit world):
    ros2 launch launch/road_world_robot.launch.py world_file:=<path>

Available worlds in this project
---------------------------------
  worlds/obstacle_world.sdf    – Minimal 20×20 m RL obstacle arena
  worlds/road_world.sdf        – 40×20 m road with slope, speed bumps,
                                 sliding pedestrians
  worlds/rl_arena.sdf          – 24×24 m full urban city grid (RL world)
  worlds/city_world.sdf        – High-fidelity city (Gazebo Fuel models)
  worlds/duckietown_city.sdf   – Duckietown-style city grid (ported from
                                 theconstructcore/riabot on Bitbucket)

Quick commands
--------------
  # Duckietown city (dedicated launcher with road-centre spawn):
  ros2 launch launch/duckietown_world.launch.py

  # Duckietown city via this generic launcher:
  ros2 launch launch/road_world_robot.launch.py \
    world_file:=$(pwd)/worlds/duckietown_city.sdf

  # Bare Gazebo (no ROS2, no robot):
  gz sim -r worlds/duckietown_city.sdf
  gz sim -r worlds/rl_arena.sdf
  gz sim -r worlds/road_world.sdf
  gz sim -r worlds/obstacle_world.sdf

Convention: the robot always spawns at (ROAD_START_X, ROAD_START_Y).
For the Duckietown world use duckietown_world.launch.py instead, which
places the robot on the actual road surface (y=3).
===========================================================================
"""

import os
os.environ['TURTLEBOT3_MODEL'] = 'waffle_pi'

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


# ---- Standard starting point convention for ALL road worlds in this project ----
# Override per world with:  x_pose:=<x> y_pose:=<y>
# (spawn_turtlebot3.launch.py accepts x_pose/y_pose only; no yaw argument)
ROAD_START_X = '0.0'
ROAD_START_Y = '0.0'


def generate_launch_description():

    # Default world file = our new city_world.sdf
    project_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    default_world = os.path.join(project_dir, 'worlds', 'city_world.sdf')

    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=default_world,
        description='Full path to the .sdf world file to load'
    )

    x_pose_arg = DeclareLaunchArgument(
        'x_pose',
        default_value=ROAD_START_X,
        description='Robot spawn X. obstacle=0, road_world=-15, rl_arena=-9, '
                    'city_world=0, duckietown=0 (or -4.5 to sit on the road)'
    )

    y_pose_arg = DeclareLaunchArgument(
        'y_pose',
        default_value=ROAD_START_Y,
        description='Robot spawn Y. obstacle=0, road_world=0, rl_arena=-9, '
                    'city_world=0, duckietown=0 (or 3 to sit on the road)'
    )

    world_file = LaunchConfiguration('world_file')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')

    # Let Gazebo resolve model textures/meshes under models/
    # (needed by duckietown_city.sdf road-segment textures and city_world.sdf)
    models_dir = os.path.join(project_dir, 'models')
    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=models_dir
    )

    # --- Start Gazebo with the given world file ---
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={'gz_args': ['-r ', world_file]}.items()
    )

    # --- Spawn the TurtleBot3 robot at the start of the road ---
    # Reuses TurtleBot3's own spawn logic (respects $TURTLEBOT3_MODEL, e.g. waffle_pi)
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch',
                'spawn_turtlebot3.launch.py'
            ])
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
        }.items()
    )

    return LaunchDescription([
        world_file_arg,
        x_pose_arg,
        y_pose_arg,
        set_resource_path,
        gz_sim,
        spawn_robot,
    ])
