"""Mission layer: /behavior_tree_node (mower_mission).

Run gui_bridge with ``serve_high_level:=false`` alongside it, otherwise both
nodes serve /behavior_tree_node/*.

    ros2 launch mower_mission mission.launch.py
    ros2 launch mower_mission mission.launch.py params_file:=/path/to/mission.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory('mower_mission'), 'config',
                                  'mission.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='mission parameters (thresholds, names)'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        Node(
            package='mower_mission',
            executable='mission_node',
            respawn=True, respawn_delay=2.0,  # docs/crash_recovery.md
            name='behavior_tree_node',
            output='screen',
            emulate_tty=True,
            parameters=[LaunchConfiguration('params_file'),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            # SIGINT first so the node can send the blade-off before it exits.
            sigterm_timeout='5',
        ),
    ])
