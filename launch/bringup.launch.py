"""Top-level bringup for the Airseekers Tron ROS 2 (Humble) driver stack.

Starts the three hardware drivers plus the bumper safety-routing controller.
Planner/task nodes (MowgliNext-derived) are layered on top once routing/docking
nodes land.

All four drivers respawn (2 s delay) so a missing/unplugged serial port does
not take the stack down; the drivers log the open failure and retry.

Note: bumper_controller subscribes /mower_base/status (MowerBaseDevStatus), which
is published by the not-yet-ported mower_base node; it starts dormant until that
lands (the MCU still enforces the instant bumper cutoff regardless).

base_keys (``keys:=true``, default) reads the top-panel buttons from the "key input"
evdev device and maps them to /clear_estop and HighLevelControl (docs/buttons.md).

mower_lights (``lights:=true``, default) drives the WS2812 status LEDs on /dev/spidev3.0
like the vendor mower_light_sound node: serves /light_control and maps the mission state
to vendor light modes (docs/lights.md). ``lights_dry_run:=true`` computes frames only.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('keys', default_value='true',
                              description='Start the top-panel button node (base_keys).'),
        DeclareLaunchArgument('lights', default_value='true',
                              description='Start the status-LED node (mower_lights).'),
        DeclareLaunchArgument('lights_dry_run', default_value='false',
                              description='mower_lights: compute frames but do not write SPI.'),
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
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package='wit_imu_driver',
            executable='wit_node',
            name='wit_imu_driver',
            parameters=[{
                'port': '/dev/serial_imu',
                'rate': 100.0,
                'frame_id': 'imu_link',
                'imu_topic': '/imu/data',   # driver default; consumed by ekf/imu_cal
            }],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package='um960_gps_driver',
            executable='um960_node',
            name='um960_gps_driver',
            parameters=[{
                'port': '/dev/serial_rtk',
                'baud': 115200,
                'frame_id': 'gps_link',   # URDF child link (driver default is 'gps')
            }],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
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
                # twist_mux input (mower_teleop/config/twist_mux.yaml), not /cmd_vel.
                'cmd_vel_topic': '/cmd_vel_bumper',
            }],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package='base_keys',
            executable='base_keys_node',
            name='base_keys',
            condition=IfCondition(LaunchConfiguration('keys')),
            parameters=[{
                'device_name': 'key input',     # /dev/input/event5 (vendor: /dev/keyboard)
                'long_press_ms': 3000,
                'debounce_ms': 50,
                'power_long_action': 'log_only',
            }],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package='mower_lights',
            executable='light_controller',
            name='light_controller',
            condition=IfCondition(LaunchConfiguration('lights')),
            parameters=[{
                'auto_mode': True,
                'dry_run': ParameterValue(LaunchConfiguration('lights_dry_run'), value_type=bool),
                'spi_device': '/dev/spidev3.0',     # WS2812 chain (vendor libws_2812.so)
            }],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
