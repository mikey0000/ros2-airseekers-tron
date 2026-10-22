"""Theft, lift and incident alerts: /alert_node (mower_alerts).

    ros2 launch mower_alerts alerts.launch.py
    ros2 launch mower_alerts alerts.launch.py params_file:=/path/to/alerts.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
try:  # Python node hosts (mower_bringup/node_host.py, compose_nodes:=true)
    from mower_bringup.node_host_launch import hosted_node
except ImportError:  # mower_bringup not installed: plain processes
    def hosted_node(_group, **kwargs):
        return Node(**kwargs)


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory('mower_alerts'), 'config',
                                  'alerts.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='alert thresholds'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('anchor_path', default_value='~/.ros/mower_alerts/parked_anchor.json',
                              description='parked GPS position, kept across restarts'),
        hosted_node(
            'housekeeping', package='mower_alerts',
            executable='alert_node',
            name='alert_node',
            output='screen',
            emulate_tty=True,
            respawn=True, respawn_delay=2.0,
            parameters=[LaunchConfiguration('params_file'),
                        {'use_sim_time': LaunchConfiguration('use_sim_time'),
                         'anchor_path': LaunchConfiguration('anchor_path')}],
        ),
    ])
