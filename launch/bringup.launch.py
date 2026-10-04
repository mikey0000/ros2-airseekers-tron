"""Top-level bringup for the Airseekers Tron ROS 2 (Humble) driver stack.

Starts the three hardware drivers. Planner/task nodes (MowgliNext-derived)
are layered on top once routing/docking nodes land.
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
    ])
