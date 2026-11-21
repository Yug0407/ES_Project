#!/usr/bin/env python3
"""
duckietown_world.launch.py
===========================================================================
Launches Gazebo Harmonic with the Duckietown city world AND spawns the
TurtleBot3 (waffle_pi) at the origin of the city grid.

The Duckietown city is ported from:
  https://bitbucket.org/theconstructcore/riabot  (duckietown_gazebo package)
and converted to SDF 1.9 / Gazebo Harmonic format.

Usage (default — Duckietown city):
    ros2 launch launch/duckietown_world.launch.py

Usage (override spawn position):
    ros2 launch launch/duckietown_world.launch.py x_pose:=-5.0 y_pose:=3.0

The robot spawns at (0, 0) — the green disc in the world marks this point.
To drive on the main road, move it to e.g. x=-4.5, y=3.0 (centre of the
long straight road_plane_3 segment in the middle of the city grid).
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


# ---- Robot spawn position in the Duckietown city ----
# (0,0) is the centre / spawn marker.
# The main east-west road runs at y=3, y=9, y=15.
# Place the robot on the road by setting y_pose to one of those values.
DEFAULT_X = '0.0'
DEFAULT_Y = '0.0'
# Note: spawn_turtlebot3.launch.py only accepts x_pose / y_pose.
# The robot keeps the default heading defined in the waffle_pi model.sdf.


def generate_launch_description():

    project_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    duckietown_world = os.path.join(project_dir, 'worlds', 'duckietown_city.sdf')

    # Let Gazebo find the road-segment textures in models/duckietown_models/
    models_dir = os.path.join(project_dir, 'models')
    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=models_dir
    )

    x_pose_arg = DeclareLaunchArgument(
        'x_pose',
        default_value=DEFAULT_X,
        description='Robot spawn X position in the Duckietown city'
    )

    y_pose_arg = DeclareLaunchArgument(
        'y_pose',
        default_value=DEFAULT_Y,
        description='Robot spawn Y position in the Duckietown city'
    )

    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')

    # --- Start Gazebo with the Duckietown city world ---
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={'gz_args': ['-r ', duckietown_world]}.items()
    )

    # --- Spawn TurtleBot3 ---
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
        x_pose_arg,
        y_pose_arg,
        set_resource_path,
        gz_sim,
        spawn_robot,
    ])
