# SPDX-License-Identifier: GPL-3.0-or-later
"""Start mower_map obstacle_tracker_node (auto obstacle marking).

    ros2 launch mower_map obstacle_tracker.launch.py auto_remove:=false

Normally launched by bringup/mower.launch.py; this is the standalone form for
bench / bag-replay testing. Also start mower_map/map_server.launch.py so the
promote / remove / get_mowing_area services exist.
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
    ('auto_promote', 'true', 'promote stable clusters into the map keepout', bool),
    ('auto_remove', 'true', 'remove a keepout once the object is verifiably gone', bool),
    ('only_while_mowing', 'false', 'only promote while the high-level state is mowing', bool),
    ('obstacle_step_rate', '1.0', 'tracker update / decision rate (Hz)', float),
    ('obstacle_points_topic', '/stereo_depth/points_map', 'ground-removed stereo cloud', str),
    ('extra_obstacle_points_topic', '/ai/det/obstacle_points_map',
     "YOLO-ranged detections, map frame ('' = off)", str),
    ('clear_points_topic', '/stereo_depth/clear_points_map', "clearing cloud ('' = off)", str),
    # Named map_odom_topic (not odom_topic): launch arguments share one context across
    # includes (nav2.launch.py declares odom_topic=/odom for the EKF). Same default as
    # mower_map/map_server.launch.py so a shared value never diverges.
    ('map_odom_topic', '/odometry/filtered_map', 'map-frame odometry', str),
    # Named obstacle_robot_yaml_path (not robot_yaml_path): launch arguments share
    # one context across includes (map_server.launch.py declares robot_yaml_path);
    # the node parameter itself is robot_yaml_path.
    ('obstacle_robot_yaml_path', '', 'mowgli_robot.yaml to read the GUI-bound auto_promote from ("" = off)', str),
    # Obstacle snapshots (whole annotated front frame -> GUI). snapshot_dir must sit
    # under the maps volume shared with the GUI container (/ros2_ws/maps).
    ('snapshot_enabled', 'true', 'save a front-frame JPEG per new obstacle for the GUI', bool),
    ('snapshot_topic', '/vio/right/image_annotated', 'annotated front camera image', str),
    ('snapshot_dir', '/ros2_ws/maps/obstacle_snapshots', 'snapshot JPEG output directory', str),
    ('obstacle_snapshot_quality', '80', 'snapshot JPEG quality (1-100)', int),
]


def generate_launch_description():
    params_file = os.path.join(get_package_share_directory('mower_map'), 'config',
                               'obstacle_tracker.yaml')
    overrides = {name: ParameterValue(LaunchConfiguration(name), value_type=typ)
                 for name, _, _, typ in ARGS}
    overrides['step_rate'] = overrides.pop('obstacle_step_rate')
    overrides['odom_topic'] = overrides.pop('map_odom_topic')
    # Prefixed like obstacle_step_rate so the (generic) launch-argument namespace
    # cannot collide with another include; the node param itself is snapshot_quality.
    overrides['snapshot_quality'] = overrides.pop('obstacle_snapshot_quality')
    # GUI settings (config/gui/mowgli_robot.yaml): at startup the node reads the
    # bound key obstacle_auto_promote from here. The GUI also pushes the same
    # parameter live; see launch/robot_settings.py BINDINGS.
    overrides['robot_yaml_path'] = overrides.pop('obstacle_robot_yaml_path')
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default, description=desc)
         for name, default, desc, _ in ARGS]
        + [Node(package='mower_map', executable='obstacle_tracker', name='obstacle_tracker',
                respawn=True, respawn_delay=2.0,  # docs/crash_recovery.md
                output='screen', parameters=[params_file, overrides])])
