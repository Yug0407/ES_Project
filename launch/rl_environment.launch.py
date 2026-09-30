#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_rl_project = os.path.expanduser('~/rl-project')
    
    # 1. Launch Road World + Robot Spawn
    road_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rl_project, 'launch', 'road_world_robot.launch.py')
        )
    )

    # 2. Start the Environment Gym Node
    env_node = ExecuteProcess(
        cmd=['python3', os.path.join(pkg_rl_project, 'scripts', 'gazebo_env_node.py')],
        output='screen'
    )

    return LaunchDescription([
        road_world_launch,
        env_node,
    ])