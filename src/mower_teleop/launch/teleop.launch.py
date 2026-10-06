from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mux_params = PathJoinSubstitution(
        [FindPackageShare('mower_teleop'), 'config', 'twist_mux.yaml'])

    args = [
        ('port', '8766', 'WebSocket listen port', int),
        ('bind', '0.0.0.0', 'WebSocket bind address', str),
        ('max_linear', '0.5', 'Linear velocity clamp (m/s)', float),
        ('max_angular', '1.0', 'Angular velocity clamp (rad/s)', float),
        ('replay_interval', '0.05', 'Replay period of last command (s)', float),
        ('command_lease', '0.25', 'Max age of a command that is replayed (s)', float),
    ]
    decls = [DeclareLaunchArgument(n, default_value=d, description=desc)
             for n, d, desc, _ in args]
    relay_params = {n: ParameterValue(LaunchConfiguration(n), value_type=t)
                    for n, _, _, t in args}

    return LaunchDescription(decls + [
        Node(
            package='twist_mux',
            executable='twist_mux',
            respawn=True, respawn_delay=2.0,  # docs/crash_recovery.md
            name='twist_mux',
            output='screen',
            parameters=[mux_params],
            remappings=[('cmd_vel_out', '/cmd_vel_raw')],
        ),
        Node(
            package='mower_teleop',
            executable='cmd_vel_ws_relay',
            respawn=True, respawn_delay=2.0,  # docs/crash_recovery.md
            name='cmd_vel_ws_relay',
            output='screen',
            parameters=[relay_params],
        ),
    ])
