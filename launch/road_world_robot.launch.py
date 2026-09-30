#!/usr/bin/env python3
"""
road_world_robot.launch.py

Launches Gazebo with our custom road world AND spawns the TurtleBot3 robot
at the BEGINNING of the road, facing forward.

Usage (default — uses worlds/road_world.sdf):
    ros2 launch road_world_robot.launch.py

Usage (with a different world file, e.g. a teammate's version):
    ros2 launch road_world_robot.launch.py world_file:=/full/path/to/their_world.sdf

Convention: every road world should place the start of the road around
x = -15, y = 0, with the road running along the +X direction. This way,
the robot always spawns at the correct starting point regardless of whose
world file is being used.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


# ---- Standard starting point convention for ALL road worlds in this project ----
ROAD_START_X = '-15.0'
ROAD_START_Y = '0.0'
ROAD_START_YAW = '0.0'   # facing along +X (down the road)


def generate_launch_description():

    # Default world file = our own road_world.sdf, sitting in ~/rl-project/worlds/
    default_world = os.path.join(
        os.path.expanduser('~/rl-project/worlds'),
        'road_world.sdf'
    )

    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=default_world,
        description='Full path to the .sdf world file to load'
    )

    world_file = LaunchConfiguration('world_file')

    # --- Start Gazebo with the given world file ---
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={'gz_args': world_file}.items()
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
            'x_pose': ROAD_START_X,
            'y_pose': ROAD_START_Y,
        }.items()
    )

    return LaunchDescription([
        world_file_arg,
        gz_sim,
        spawn_robot,
    ])
