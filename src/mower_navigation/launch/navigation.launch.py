# Copyright 2026 ROS 2 port team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Nav2 navigation stack for the Airseekers Tron (ROS 2 Humble).

Starts controller_server, planner_server, behavior_server, bt_navigator,
velocity_smoother and lifecycle_manager_navigation. With ``use_keepout:=true``
it also starts costmap_filter_info_server under its own lifecycle manager.

Expects (from launch/nav2.launch.py + the URDF): TF map -> odom -> base_link,
and /odometry/filtered. No map_server, no AMCL.

Velocity chain (all geometry_msgs/Twist, unstamped):
    controller_server  cmd_vel -> /cmd_vel_nav_raw
    velocity_smoother  /cmd_vel_nav_raw -> /cmd_vel_nav   (twist_mux nav lane)
    behavior_server    cmd_vel -> /cmd_vel_nav
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

LIFECYCLE_NODES = [
    "controller_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "velocity_smoother",
]


def generate_launch_description() -> LaunchDescription:
    share = FindPackageShare("mower_navigation")
    params_file = LaunchConfiguration("params_file")
    autostart = ParameterValue(LaunchConfiguration("autostart"), value_type=bool)
    log_level = LaunchConfiguration("log_level")

    common = {
        "output": "screen",
        "respawn": False,
        "arguments": ["--ros-args", "--log-level", log_level],
    }
    tf_remaps = [("/tf", "tf"), ("/tf_static", "tf_static")]

    args = [
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([share, "config", "nav2_params.yaml"]),
            description="Full path to the Nav2 parameter file."),
        DeclareLaunchArgument(
            "use_keepout", default_value="false",
            description="Start costmap_filter_info_server (keepout filter info on "
                        "/costmap_filter_info, mask /keepout_mask). The global "
                        "costmap's keepout_filter is configured either way."),
        DeclareLaunchArgument(
            "autostart", default_value="true",
            description="Configure + activate the lifecycle nodes automatically."),
        DeclareLaunchArgument("log_level", default_value="info"),
    ]

    nodes = [
        Node(
            package="nav2_controller", executable="controller_server",
            name="controller_server",
            parameters=[params_file],
            remappings=tf_remaps + [
                ("cmd_vel", "/cmd_vel_nav_raw"),
                ("odom", "/odometry/filtered"),
            ],
            **common),
        Node(
            package="nav2_planner", executable="planner_server",
            name="planner_server",
            parameters=[params_file],
            remappings=tf_remaps,
            **common),
        Node(
            package="nav2_behaviors", executable="behavior_server",
            name="behavior_server",
            parameters=[params_file],
            remappings=tf_remaps + [("cmd_vel", "/cmd_vel_nav")],
            **common),
        Node(
            package="nav2_bt_navigator", executable="bt_navigator",
            name="bt_navigator",
            parameters=[params_file, {
                "default_nav_to_pose_bt_xml": PathJoinSubstitution(
                    [share, "behavior_trees",
                     "navigate_to_pose_w_replanning_and_recovery.xml"]),
                "default_nav_through_poses_bt_xml": PathJoinSubstitution(
                    [share, "behavior_trees",
                     "navigate_through_poses_w_replanning_and_recovery.xml"]),
            }],
            remappings=tf_remaps,
            **common),
        Node(
            package="nav2_velocity_smoother", executable="velocity_smoother",
            name="velocity_smoother",
            parameters=[params_file],
            remappings=tf_remaps + [
                ("cmd_vel", "/cmd_vel_nav_raw"),
                ("cmd_vel_smoothed", "/cmd_vel_nav"),
            ],
            **common),
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            parameters=[params_file,
                        {"autostart": autostart, "node_names": LIFECYCLE_NODES}],
            **common),
    ]

    keepout = GroupAction(
        condition=IfCondition(LaunchConfiguration("use_keepout")),
        actions=[
            Node(
                package="nav2_map_server", executable="costmap_filter_info_server",
                name="costmap_filter_info_server",
                parameters=[params_file],
                **common),
            Node(
                package="nav2_lifecycle_manager", executable="lifecycle_manager",
                name="lifecycle_manager_costmap_filters",
                parameters=[params_file, {
                    "autostart": autostart,
                    "node_names": ["costmap_filter_info_server"],
                }],
                **common),
        ])

    return LaunchDescription(args + nodes + [keepout])
