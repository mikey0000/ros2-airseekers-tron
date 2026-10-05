# SPDX-License-Identifier: Apache-2.0
"""Launch the mower_docking action server.

  ros2 launch mower_docking docking.launch.py
  ros2 launch mower_docking docking.launch.py skip_nav_to_approach:=true   # bench
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('mower_docking'), 'config', 'docking.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=cfg),
        DeclareLaunchArgument('skip_nav_to_approach', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='mower_docking',
            executable='docking_server',
            name='mower_docking',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'skip_nav_to_approach': ParameterValue(
                    LaunchConfiguration('skip_nav_to_approach'), value_type=bool),
                 'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool)},
            ],
        ),
    ])
