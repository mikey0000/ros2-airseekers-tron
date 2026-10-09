"""NPU perception + safety consumer: det_ros, seg_ros, obstacle_guard.

Needs the camera topics from ``mower_bringup/launch/cameras.launch.py``.

Args:
  det_backend                  cpp (det_ros_cpp, default) | python (det_ros fallback);
                               both run as node 'det_ros' with det_ros/config/det.yaml
  det / seg / obstacle_guard   enable each node (seg off by default; nongrass turns it on)
  nongrass                     grass segmentation -> non-grass memory (2026-10-09,
                               docs/grass_segmentation.md): runs seg_ros (front stereo right
                               colour eye, NPU core 1) + seg_ros/nongrass_projector ->
                               /ai/seg/ground_cells -> mower_map map_server_node -> Nav2
                               global nongrass_layer (soft). Default false until verified live.
  models_dir                   fallback directory for the .rknn files when the device path
                               /userdata/ros2/models/<file> is missing ('' = none)
  dry_run                      det/seg stay alive and publish nothing if rknnlite / model
                               is missing (default: exit 1 with a clear error)
  stop_on_close                obstacle_guard sends the zero burst + /cutter_off
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

LC = LaunchConfiguration


def _have_det_cpp():
    import os
    try:
        from ament_index_python.packages import get_package_prefix
        exe = os.path.join(get_package_prefix('det_ros_cpp'), 'lib', 'det_ros_cpp', 'det_ros_cpp')
        return os.access(exe, os.X_OK)
    except Exception:  # noqa: BLE001 - PackageNotFoundError and friends
        return False


def generate_launch_description():
    args = [
        DeclareLaunchArgument('det', default_value='true'),
        DeclareLaunchArgument('det_backend', default_value='cpp'),
        DeclareLaunchArgument('seg', default_value='false'),
        DeclareLaunchArgument('nongrass', default_value='false'),
        DeclareLaunchArgument('obstacle_guard', default_value='true'),
        DeclareLaunchArgument('models_dir', default_value=''),
        DeclareLaunchArgument('dry_run', default_value='false'),
        DeclareLaunchArgument('stop_on_close', default_value='false'),
    ]
    npu_overrides = {
        'models_dir': LC('models_dir'),
        'dry_run': ParameterValue(LC('dry_run'), value_type=bool),
    }
    det_params = [PathJoinSubstitution([FindPackageShare('det_ros'), 'config', 'det.yaml']),
                  npu_overrides]

    def det_if(backend):
        return IfCondition(PythonExpression([
            "'", LC('det'), "'.lower() == 'true' and '", LC('det_backend'), "' == '", backend, "'"]))
    det = Node(
        package='det_ros', executable='det_ros', name='det_ros', output='screen',
        respawn=True, respawn_delay=2.0,  # docs/crash_recovery.md
        condition=det_if('python'), parameters=det_params,
    )
    det_cpp = Node(
        package='det_ros_cpp', executable='det_ros_cpp', name='det_ros', output='screen',
        respawn=True, respawn_delay=2.0,  # docs/crash_recovery.md
        condition=det_if('cpp'), parameters=det_params,
    )
    # det_backend:=cpp but det_ros_cpp not built/installed -> run the python node instead
    # (never reference a missing package: launch would abort the whole stack).
    if not _have_det_cpp():
        det = Node(
            package='det_ros', executable='det_ros', name='det_ros', output='screen',
            respawn=True, respawn_delay=2.0,  # docs/crash_recovery.md
            condition=IfCondition(PythonExpression(["'", LC('det'), "'.lower() == 'true'"])),
            parameters=det_params,
        )
        det_cpp = LogInfo(msg='det_ros_cpp not installed: det_backend falls back to python det_ros')
    # seg runs when asked for directly OR when its consumer (nongrass) is on.
    seg_on = IfCondition(PythonExpression([
        "'", LC('seg'), "'.lower() == 'true' or '", LC('nongrass'), "'.lower() == 'true'"]))
    seg = Node(
        package='seg_ros', executable='seg_ros', name='seg_ros', output='screen',
        respawn=True, respawn_delay=2.0,  # docs/crash_recovery.md
        condition=seg_on,
        parameters=[PathJoinSubstitution([FindPackageShare('seg_ros'), 'config', 'seg.yaml']),
                    npu_overrides],
    )
    nongrass = Node(
        package='seg_ros', executable='nongrass_projector', name='nongrass_projector',
        output='screen', respawn=True, respawn_delay=2.0,  # docs/crash_recovery.md
        condition=IfCondition(LC('nongrass')),
        parameters=[PathJoinSubstitution([FindPackageShare('seg_ros'), 'config',
                                          'nongrass_projector.yaml'])],
    )
    guard = Node(
        package='mower_vision', executable='obstacle_guard', name='obstacle_guard',
        respawn=True, respawn_delay=2.0,  # docs/crash_recovery.md
        output='screen', condition=IfCondition(LC('obstacle_guard')),
        parameters=[PathJoinSubstitution([FindPackageShare('mower_vision'), 'config',
                                          'obstacle_guard.yaml']),
                    {'stop_on_close': ParameterValue(LC('stop_on_close'), value_type=bool)}],
    )
    return LaunchDescription(args + [det, det_cpp, seg, nongrass, guard])
