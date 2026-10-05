# SPDX-License-Identifier: GPL-3.0-or-later
"""Start mower_map map_server_node.

    ros2 launch mower_map map_server.launch.py maps_dir:=/tmp/maps \
        robot_yaml_path:=/ros2_ws/config/gui/mowgli_robot.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ARGS = [
    # name, default, description, type
    ('maps_dir', '/ros2_ws/maps', 'directory holding areas.dat and dock_pose.yaml', str),
    ('robot_yaml_path', '', "mowgli_robot.yaml to mirror dock_pose_* into ('' = off)", str),
    ('map_frame', 'map', 'frame of areas, masks and dock pose', str),
    ('base_frame', 'base_footprint', 'robot frame for TF fallbacks', str),
    ('blade_frame', 'blade_link', 'cutter frame for mow progress', str),
    ('resolution', '0.1', 'keepout / progress grid resolution (m)', float),
    ('mask_margin', '2.0', 'lethal border around the areas in the mask (m)', float),
    ('odom_topic', '/odometry/filtered_map', 'map-frame odometry', str),
    ('dock_gates_override', 'false', 'skip set_docking_point gates (bench only)', bool),
    ('datum_lat', '0.0', 'datum stamp written into areas.dat', float),
    ('datum_lon', '0.0', 'datum stamp written into areas.dat', float),
]


def generate_launch_description():
    params_file = os.path.join(get_package_share_directory('mower_map'), 'config',
                               'map_server.yaml')
    overrides = {name: ParameterValue(LaunchConfiguration(name), value_type=typ)
                 for name, _, _, typ in ARGS}
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default, description=desc)
         for name, default, desc, _ in ARGS]
        + [Node(package='mower_map', executable='map_server_node', name='map_server_node',
                output='screen', parameters=[params_file, overrides])])
