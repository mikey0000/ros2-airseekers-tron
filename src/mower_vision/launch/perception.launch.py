"""NPU perception + safety consumer: det_ros, seg_ros, obstacle_guard.

Needs the camera topics from ``mower_bringup/launch/cameras.launch.py``.

Args:
  det / seg / obstacle_guard   enable each node (seg off by default: nothing consumes it yet)
  models_dir                   fallback directory for the .rknn files when the device path
                               /userdata/ros2/models/<file> is missing ('' = none)
  dry_run                      det/seg stay alive and publish nothing if rknnlite / model
                               is missing (default: exit 1 with a clear error)
  stop_on_close                obstacle_guard sends the zero burst + /cutter_off
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

LC = LaunchConfiguration


def generate_launch_description():
    args = [
        DeclareLaunchArgument('det', default_value='true'),
        DeclareLaunchArgument('seg', default_value='false'),
        DeclareLaunchArgument('obstacle_guard', default_value='true'),
        DeclareLaunchArgument('models_dir', default_value=''),
        DeclareLaunchArgument('dry_run', default_value='false'),
        DeclareLaunchArgument('stop_on_close', default_value='false'),
    ]
    npu_overrides = {
        'models_dir': LC('models_dir'),
        'dry_run': ParameterValue(LC('dry_run'), value_type=bool),
    }
    det = Node(
        package='det_ros', executable='det_ros', name='det_ros', output='screen',
        condition=IfCondition(LC('det')),
        parameters=[PathJoinSubstitution([FindPackageShare('det_ros'), 'config', 'det.yaml']),
                    npu_overrides],
    )
    seg = Node(
        package='seg_ros', executable='seg_ros', name='seg_ros', output='screen',
        condition=IfCondition(LC('seg')),
        parameters=[PathJoinSubstitution([FindPackageShare('seg_ros'), 'config', 'seg.yaml']),
                    npu_overrides],
    )
    guard = Node(
        package='mower_vision', executable='obstacle_guard', name='obstacle_guard',
        output='screen', condition=IfCondition(LC('obstacle_guard')),
        parameters=[PathJoinSubstitution([FindPackageShare('mower_vision'), 'config',
                                          'obstacle_guard.yaml']),
                    {'stop_on_close': ParameterValue(LC('stop_on_close'), value_type=bool)}],
    )
    return LaunchDescription(args + [det, seg, guard])
