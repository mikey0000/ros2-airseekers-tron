# Copyright 2026 michael
# SPDX-License-Identifier: Apache-2.0
"""Coverage planning: mower_coverage /coverage/plan + the /plan_coverage adapter.

  ros2 launch mower_coverage_bridge coverage_bridge.launch.py
  ros2 launch mower_coverage_bridge coverage_bridge.launch.py start_planner:=false
  ros2 launch mower_coverage_bridge coverage_bridge.launch.py cut_width_m:=0.22

Swath spacing (``operation_width``) = ``cut_width_m - swath_overlap_m``. The cut width
defaults to the URDF cutter disc (radius 0.10 m -> 0.20 m); TODO: measure the blade.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    start_planner = LaunchConfiguration('start_planner')
    transit_gap = LaunchConfiguration('transit_gap_m')
    cut_width = LaunchConfiguration('cut_width_m')
    overlap = LaunchConfiguration('swath_overlap_m')
    boundary_inset = LaunchConfiguration('boundary_inset_m')
    turn_radius = LaunchConfiguration('min_turn_radius_m')
    border_inset = LaunchConfiguration('border_inset_m')
    operation_width = PythonExpression(['float(', cut_width, ') - float(', overlap, ')'])
    return LaunchDescription([
        DeclareLaunchArgument('start_planner', default_value='true',
                              description='also start mower_coverage_node (/coverage/plan)'),
        DeclareLaunchArgument('transit_gap_m', default_value='0.6',
                              description='sub-path split gap; keep == kSegmentTransitGapM'),
        DeclareLaunchArgument('cut_width_m', default_value='0.20',
                              description='blade cut width [m] (URDF cutter disc 2 x 0.10; '
                                          'TODO: measure)'),
        DeclareLaunchArgument('swath_overlap_m', default_value='0.02',
                              description='overlap between neighbouring swaths [m]'),
        DeclareLaunchArgument('boundary_inset_m', default_value='0.15',
                              description='outer ring centreline sits operation_width/2 + '
                                          'this inside the boundary [m]'),
        DeclareLaunchArgument('min_turn_radius_m', default_value='0.0',
                              description='0 = pivot between swaths (diff-drive); > 0 = '
                                          'F2C v3 Dubins turns of this radius'),
        # 0.0: the outer ring centreline sits at op_width/2 + boundary_inset_m (the per-area
        # edge margin, default 0.05) inside the recorded line. The former 0.20 left a ~0.19 m
        # band uncut all round (owner: "no gaps"); tracking margin now comes from the edge
        # margin (docs/analysis/2026-10-06_ring_drift.md for the history).
        DeclareLaunchArgument('border_inset_m', default_value='0.0',
                              description='outer ring centreline pull-back inside the '
                                          'recorded boundary [m]'),
        Node(
            package='mower_coverage',
            executable='mower_coverage_node',
            name='mower_coverage_node',
            output='screen',
            parameters=[{
                'cut_width_m': ParameterValue(cut_width, value_type=float),
                'swath_overlap_m': ParameterValue(overlap, value_type=float),
                'boundary_inset_m': ParameterValue(boundary_inset, value_type=float),
                'min_turn_radius_m': ParameterValue(turn_radius, value_type=float),
                'border_inset_m': ParameterValue(border_inset, value_type=float),
            }],
            condition=IfCondition(start_planner),
        ),
        Node(
            package='mower_coverage_bridge',
            executable='coverage_action_server',
            name='coverage_server',
            output='screen',
            parameters=[{
                'transit_gap_m': ParameterValue(transit_gap, value_type=float),
                'operation_width': ParameterValue(operation_width, value_type=float),
            }],
        ),
    ])
