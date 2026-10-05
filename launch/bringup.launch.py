"""Top-level bringup for the Airseekers Tron ROS 2 (Humble) driver stack.

Starts the three hardware drivers plus the bumper safety-routing controller.
Planner/task nodes (MowgliNext-derived) are layered on top once routing/docking
nodes land.

Note: bumper_controller subscribes /mower_base/status (MowerBaseDevStatus), which
is published by the not-yet-ported mower_base node; it starts dormant until that
lands (the MCU still enforces the instant bumper cutoff regardless).
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mower_mcu_driver',
            executable='mcu_node',
            name='mower_mcu_driver',
            parameters=[{
                'port': '/dev/serial_mower',
                'baud': 115200,
                'cmd_vel_timeout': 0.5,
            }],
            output='screen',
        ),
        Node(
            package='wit_imu_driver',
            executable='wit_node',
            name='wit_imu_driver',
            parameters=[{
                'port': '/dev/serial_imu',
                'rate': 100.0,
                'frame_id': 'imu_link',
            }],
            output='screen',
        ),
        Node(
            package='um960_gps_driver',
            executable='um960_node',
            name='um960_gps_driver',
            parameters=[{
                'port': '/dev/serial_rtk',
                'baud': 115200,
                'frame_id': 'gps',
            }],
            output='screen',
        ),
        Node(
            package='bumper_controller',
            executable='bumper_controller_node',
            name='bumper_controller',
            parameters=[{
                'pid_kp': 0.8,
                'pid_ki': 0.0,
                'pid_kd': 0.05,
                'pid_max_angular': 0.5,
                'pid_tolerance': 0.03,
            }],
            output='screen',
        ),
    ])
