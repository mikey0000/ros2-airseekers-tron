"""Run the OpenVINS VIO replacement: stereo_vio_bridge producer + ov_msckf consumer."""
import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    stack_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return LaunchDescription([
        Node(
            package="stereo_vio_bridge",
            executable="stereo_vio_bridge",
            name="stereo_vio_bridge",
            output="screen",
        ),
        Node(
            package="ov_msckf",
            executable="run_subscribe_msckf",
            name="run_subscribe_msckf",
            namespace="ov_msckf",
            output="screen",
            parameters=[{
                "config_path": os.path.join(
                    stack_root, "..", "ros2_port_handoff", "14_vio_replacement",
                    "open_vins", "estimator_config.yaml"),
            }],
        ),
    ])
