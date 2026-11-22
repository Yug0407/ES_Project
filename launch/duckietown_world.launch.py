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
    ros2 launch launch/duckietown_world.launch.py x_pose:=-5.0 y_pose:=0.3

Traffic (cars + pedestrians) is driven by traffic_controller.py, started below.

The robot spawns at (0, 0) — the green disc in the world marks this point.
To drive on the main road, move it to e.g. x=-4.5, y=3.0 (centre of the
long straight road_plane_3 segment in the middle of the city grid).
===========================================================================
"""

import os
os.environ['TURTLEBOT3_MODEL'] = 'waffle_pi'

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# ---- Robot spawn position in the Duckietown city ----
# (0,0) is the centre / spawn marker.
# Main east-west road centre line is y = 0.6, lanes are 0.3 m either side.
# The robot faces east, so its (right-hand) lane is y = 0.3  -> same lane the
# traffic uses, so cars behind it will slow down and follow it.
DEFAULT_X = '-0.9'
DEFAULT_Y = '0.3'
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

    # Optional modern GUI theme (created by: python3 scripts/beautify_city.py --gui)
    gui_config = os.path.join(project_dir, 'config', 'duckietown_gui.config')
    gz_args = ['-r ', duckietown_world]
    if os.path.exists(gui_config):
        gz_args += [' --gui-config ', gui_config]

    # --- Start Gazebo with the Duckietown city world ---
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={'gz_args': gz_args}.items()
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

    # --- Traffic: bridge Gazebo's set_pose service + robot pose into ROS 2 ---
    world_name = 'duckietown_city'
    traffic_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='traffic_bridge',
        output='screen',
        arguments=[
            f'/world/{world_name}/set_pose@ros_gz_interfaces/srv/SetEntityPose',
            f'/world/{world_name}/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
    )

    # --- Traffic: the node that drives the cars and pedestrians ---
    # Find traffic_controller.py (scripts/ folder or project root)
    controller_path = next(
        (p for p in (os.path.join(project_dir, 'scripts', 'traffic_controller.py'),
                     os.path.join(project_dir, 'traffic_controller.py')) if os.path.exists(p)),
        None)
    if controller_path is None:
        raise RuntimeError('traffic_controller.py not found in scripts/ or the project root')
    traffic_controller = TimerAction(
        period=4.0,   # give Gazebo a moment to start
        actions=[ExecuteProcess(
            cmd=['python3', controller_path],
            output='screen',
        )],
    )

    # Spawn the robot a few seconds after Gazebo starts, so the (decorated) world is fully
    # loaded first. Without the wait the spawn can time out and the robot never appears.
    spawn_robot_later = TimerAction(period=8.0, actions=[spawn_robot])

    return LaunchDescription([
        x_pose_arg,
        y_pose_arg,
        set_resource_path,
        gz_sim,
        spawn_robot_later,
        traffic_bridge,
        traffic_controller,
    ])