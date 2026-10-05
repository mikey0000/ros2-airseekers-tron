# Copyright 2026 michael
# SPDX-License-Identifier: Apache-2.0
"""Coverage planning: mower_coverage /coverage/plan + the /plan_coverage adapter.

  ros2 launch mower_coverage_bridge coverage_bridge.launch.py
  ros2 launch mower_coverage_bridge coverage_bridge.launch.py start_planner:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    start_planner = LaunchConfiguration('start_planner')
    transit_gap = LaunchConfiguration('transit_gap_m')
    return LaunchDescription([
        DeclareLaunchArgument('start_planner', default_value='true',
                              description='also start mower_coverage_node (/coverage/plan)'),
        DeclareLaunchArgument('transit_gap_m', default_value='0.6',
                              description='sub-path split gap; keep == kSegmentTransitGapM'),
        Node(
            package='mower_coverage',
            executable='mower_coverage_node',
            name='mower_coverage_node',
            output='screen',
            condition=IfCondition(start_planner),
        ),
        Node(
            package='mower_coverage_bridge',
            executable='coverage_action_server',
            name='coverage_server',
            output='screen',
            parameters=[{'transit_gap_m': ParameterValue(transit_gap, value_type=float)}],
        ),
    ])
