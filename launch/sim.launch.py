"""Headless simulation of the Airseekers Tron stack (no mower, no hardware).

    ros2 launch mower_bringup sim.launch.py [world:=<yaml>] [arg:=value ...]

``mower_sim/sim_node`` replaces the hardware drivers (mcu_node, wit_node,
um960_node, stereo_depth, rear camera) and integrates /cmd_vel against a world
YAML (src/mower_sim/worlds/: lawn polygon, dock pose, static obstacles and
scripted moving ones). Everything above the drivers is the real code:

    robot_state_publisher (URDF), static map -> odom, twist_mux (lanes of
    mower_teleop/config/twist_mux.yaml -> /cmd_vel_raw), cmd_vel_slew
    (/cmd_vel_raw -> /cmd_vel), slip_detector, bumper_controller (/cmd_vel_bumper,
    /bumper_cloud), Nav2 (mower_navigation), mower_map map_server_node,
    coverage (planner + /plan_coverage), mower_docking, mower_mission
    (/behavior_tree_node), gui_bridge (serve_high_level false while the mission runs).

``localization``:
    sim (default)  the sim publishes odom -> base_link, /odometry/filtered and an
                   aligned /heading_aligner/status (ground truth, deterministic).
    ekf            the real gps_gate / navsat_transform / heading_aligner / ekf_node
                   (launch/nav2.launch.py) fuse the sim's /odom, /imu/data and /fix.

The world's lawn and dock are written to a scratch ``maps_dir`` (fresh temp dir
unless given) as areas.dat / dock_pose.yaml for map_server_node; GUI-owned files
(robot yaml, datum.env) also go there, never into config/gui.

Not started (ports / hardware): foxglove_bridge (``foxglove:=true`` to watch it
with the GUI or Foxglove Studio), the teleop WebSocket relay (``teleop_relay:=true``),
cameras, perception, det_range, base_keys, lights, supervisor, mow_recorder.
"""

import os
import sys
import tempfile

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, SetLaunchConfiguration)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import robot_settings  # noqa: E402  (launch/robot_settings.py, installed alongside)


def _share(package, *rel):
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory(package), *rel)


def _prepare_world(context):
    """Load the world, write areas.dat / dock_pose.yaml, expose its datum."""
    from mower_sim.maps import write_maps
    from mower_sim.world import load_world
    path = LaunchConfiguration('world').perform(context).strip()
    if not os.path.isabs(path) and not os.path.isfile(path):
        path = _share('mower_sim', 'worlds', path if path.endswith('.yaml') else path + '.yaml')
    world = load_world(path)
    maps_dir = LaunchConfiguration('maps_dir').perform(context).strip()
    if not maps_dir:
        maps_dir = tempfile.mkdtemp(prefix='mower_sim_maps_')
    keep = LaunchConfiguration('keep_maps').perform(context).strip().lower() in ('1', 'true')
    if keep and os.path.isfile(os.path.join(maps_dir, 'areas.dat')):
        # keep_maps (2026-10-09): reuse areas / paths drawn in the GUI on a previous run (the
        # world's sensors still model the world file's obstacles, not the drawn map's).
        pass
    else:
        write_maps(world, maps_dir)
    actions = [SetLaunchConfiguration('world_file', path),
               SetLaunchConfiguration('maps_dir', maps_dir),
               SetLaunchConfiguration('datum_lat', '%.9f' % world.datum_lat),
               SetLaunchConfiguration('datum_lon', '%.9f' % world.datum_lon),
               LogInfo(msg='mower_sim: world %s (%s), maps_dir %s, datum %.7f,%.7f'
                           % (world.name, path, maps_dir, world.datum_lat, world.datum_lon))]
    if not LaunchConfiguration('docking_params_file').perform(context).strip():
        actions.append(SetLaunchConfiguration(
            'docking_params_file', _share('mower_docking', 'config', 'docking.yaml')))
    nav2 = LaunchConfiguration('nav2_params_file').perform(context).strip()
    if not nav2:
        nav2 = _share('mower_navigation', 'config', 'nav2_params.yaml')
    if LaunchConfiguration('stereo_costmap').perform(context).strip().lower() not in ('true', '1'):
        import yaml
        with open(nav2, encoding='utf-8') as f:
            doc = robot_settings.without_stereo_sources(yaml.safe_load(f))
        nav2 = robot_settings.write_params_file(doc, maps_dir, 'nav2_params_no_stereo.yaml')
    actions.append(SetLaunchConfiguration('nav2_params_file', nav2))
    return actions


def _localization(context):
    mode = LaunchConfiguration('localization').perform(context).strip().lower()
    if mode == 'sim':
        return []
    if mode != 'ekf':
        raise RuntimeError("localization must be 'sim' or 'ekf', got %r" % mode)
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    maps_dir = LaunchConfiguration('maps_dir').perform(context)
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'nav2.launch.py')),
        launch_arguments={
            'urdf_file': LaunchConfiguration('urdf_file'),
            'use_robot_state_publisher': 'false',
            'use_nav2_core': 'false',
            # explicit: launch configurations are global and map_server.launch.py has
            # already declared base_frame:=base_footprint (the EKF would then publish
            # odom -> base_footprint and split the TF tree)
            'map_frame': 'map', 'odom_frame': 'odom', 'base_frame': 'base_link',
            'odom_topic': '/odom', 'raw_imu_topic': '/imu/data', 'fix_topic': '/fix',
            'datum_lat': LaunchConfiguration('datum_lat'),
            'datum_lon': LaunchConfiguration('datum_lon'),
            'datum_yaw': '0.0',
            'heading_offset_file': os.path.join(maps_dir, 'heading_offset.yaml'),
        }.items())]


def generate_launch_description() -> LaunchDescription:
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    stack_root = os.path.dirname(launch_dir)
    default_urdf = os.path.join(stack_root, 'config', 'urdf', 'mower.urdf.xacro')

    def arg(name, default, description):
        return DeclareLaunchArgument(name, default_value=default, description=description)

    def enabled(name):
        return IfCondition(LaunchConfiguration(name))

    arguments = [
        arg('world', 'garden.yaml', 'World YAML: a path, or a file name under '
            'share/mower_sim/worlds/ (garden, transit_box).'),
        arg('localization', 'sim', 'sim (ground-truth TF + /odometry/filtered) | ekf (real '
            'gps_gate/navsat/heading_aligner/ekf on the simulated sensors).'),
        arg('maps_dir', '', 'areas.dat / dock_pose.yaml directory ("" = fresh temp dir).'),
        arg('keep_maps', 'false', 'true: keep an existing maps_dir/areas.dat (GUI-drawn areas and paths) '
            'instead of rewriting it from the world.'),
        arg('navigation', 'true', 'Nav2 (mower_navigation/navigation.launch.py).'),
        arg('nav2_params_file', '', 'Nav2 params ("" = mower_navigation/config/nav2_params.yaml).'),
        arg('stereo_costmap', 'true', 'Keep the stereo sources in the costmaps (false = '
            'bumper-only, as mower.launch.py stereo_costmap:=false).'),
        arg('map_server', 'true', 'mower_map map_server_node on maps_dir.'),
        arg('coverage', 'true', 'mower_coverage_bridge (planner + /plan_coverage).'),
        arg('docking', 'true', 'mower_docking (dock/undock actions, vision on the rendered '
            'rear-camera marker).'),
        arg('docking_params_file', '', 'mower_docking params ("" = its config/docking.yaml).'),
        arg('rear_marker_size', '0.04', 'sim: printed size of the rendered dock ArUco (m); '
            'mower_docking must use the same marker_size.'),
        arg('mission', 'true', 'mower_mission /behavior_tree_node.'),
        arg('mission_stub', 'false', 'A scenario client (mower_sim scenario --stub-state) '
            'stands in for the mission (/motion_enabled + high_level_status): gui_bridge then '
            'does not serve its own stub high-level state either. Use with mission:=false.'),
        arg('gui_bridge', 'true', 'mower_gui_bridge (/hardware_bridge/*, /gps/status, '
            '/odometry/filtered_map; the mission needs it).'),
        arg('foxglove', 'false', 'foxglove_bridge (all topics) for the GUI / Foxglove Studio.'),
        arg('foxglove_port', '8765', 'foxglove_bridge port.'),
        arg('teleop_relay', 'false', 'cmd_vel_ws_relay WebSocket on :8766 (GUI joystick).'),
        arg('stereo', 'true', 'sim: synthetic /stereo_depth/points + clear_points.'),
        arg('stereo_noise_m', '0.0', 'sim: gaussian stereo range noise (m).'),
        arg('min_wheel_speed_mps', '0.06', 'sim: MCU wheel speed floor (0 = off).'),
        arg('urdf_file', default_urdf, 'xacro robot description.'),
        arg('log_level', 'info', 'Log level of the sim node.'),
    ]

    sim = Node(
        package='mower_sim', executable='sim_node', name='mower_sim', output='screen',
        parameters=[{
            'world_file': LaunchConfiguration('world_file'),
            'localization': LaunchConfiguration('localization'),
            'stereo': ParameterValue(LaunchConfiguration('stereo'), value_type=bool),
            'stereo_noise_m': ParameterValue(LaunchConfiguration('stereo_noise_m'),
                                             value_type=float),
            'min_wheel_speed_mps': ParameterValue(LaunchConfiguration('min_wheel_speed_mps'),
                                                  value_type=float),
            'rear_marker_size': ParameterValue(LaunchConfiguration('rear_marker_size'),
                                               value_type=float),
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    core = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'robot_description': ParameterValue(
                 Command(['xacro ', LaunchConfiguration('urdf_file')]), value_type=str)}]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='static_map_odom',
             output='screen',
             arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0',
                        '--yaw', '0', '--frame-id', 'map', '--child-frame-id', 'odom']),
        Node(package='twist_mux', executable='twist_mux', name='twist_mux', output='screen',
             parameters=[PathJoinSubstitution(
                 [FindPackageShare('mower_teleop'), 'config', 'twist_mux.yaml'])],
             remappings=[('cmd_vel_out', '/cmd_vel_raw')]),
        Node(package='mower_teleop', executable='cmd_vel_ws_relay', name='cmd_vel_ws_relay',
             output='screen', condition=enabled('teleop_relay')),
        Node(package='mower_control', executable='cmd_vel_slew', name='cmd_vel_slew',
             output='screen',
             # the GUI settings file of this checkout is device-owned: do not read it
             parameters=[{'robot_settings_file': ''}]),
        # costmaps read the odom/map copies of /bumper_cloud and /ai/det/obstacle_points
        # (2026-10-09, mower_control/fixed_frame_relay.py)
        Node(package='mower_control', executable='fixed_frame_relay', name='fixed_frame_relay',
             output='screen'),
        Node(package='mower_control', executable='slip_detector', name='slip_detector',
             output='screen'),
        Node(package='bumper_controller', executable='bumper_controller_node',
             name='bumper_controller', output='screen',
             parameters=[PathJoinSubstitution([FindPackageShare('bumper_controller'), 'config',
                                               'bumper_controller.yaml']), {
                 'pid_kp': 0.8, 'pid_ki': 0.0, 'pid_kd': 0.05, 'pid_max_angular': 0.5,
                 'pid_tolerance': 0.03, 'cmd_vel_topic': '/cmd_vel_bumper'}]),
    ]

    gui_bridge = Node(
        package='mower_gui_bridge', executable='gui_bridge', name='gui_bridge', output='screen',
        parameters=[{
            # its stub high-level state machine only when the real mission is off
            'serve_high_level': PythonExpression(
                ["'", LaunchConfiguration('mission'), "' != 'true' and '",
                 LaunchConfiguration('mission_stub'), "' != 'true'"]),
            'robot_yaml_path': PathJoinSubstitution([LaunchConfiguration('maps_dir'),
                                                     'mowgli_robot.yaml']),
            'datum_env_path': PathJoinSubstitution([LaunchConfiguration('maps_dir'),
                                                    'datum.env']),
            'datum_lat': ParameterValue(LaunchConfiguration('datum_lat'), value_type=float),
            'datum_lon': ParameterValue(LaunchConfiguration('datum_lon'), value_type=float),
        }],
        condition=enabled('gui_bridge'),
    )

    def include(package, rel, condition, **launch_args):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare(package)] + rel.split('/'))),
            launch_arguments=launch_args.items(), condition=enabled(condition))

    stack = [
        include('mower_navigation', 'launch/navigation.launch.py', 'navigation',
                params_file=LaunchConfiguration('nav2_params_file')),
        include('mower_map', 'launch/map_server.launch.py', 'map_server',
                maps_dir=LaunchConfiguration('maps_dir'),
                robot_yaml_path='',
                datum_lat=LaunchConfiguration('datum_lat'),
                datum_lon=LaunchConfiguration('datum_lon'),
                map_odom_topic='/odometry/filtered_map'),
        include('mower_coverage_bridge', 'launch/coverage_bridge.launch.py', 'coverage'),
        include('mower_docking', 'launch/docking.launch.py', 'docking',
                params_file=LaunchConfiguration('docking_params_file')),
        include('mower_mission', 'launch/mission.launch.py', 'mission'),
        Node(package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge',
             output='screen', condition=enabled('foxglove'),
             parameters=[{'port': ParameterValue(LaunchConfiguration('foxglove_port'),
                                                 value_type=int),
                          'send_buffer_limit': 10000000, 'use_compression': False,
                          'sysinfo': False}]),
    ]

    return LaunchDescription(
        arguments
        + [SetLaunchConfiguration('world_file', ''), SetLaunchConfiguration('datum_lat', '0.0'),
           SetLaunchConfiguration('datum_lon', '0.0'),
           OpaqueFunction(function=_prepare_world)]
        + [sim] + core + [gui_bridge] + stack
        + [OpaqueFunction(function=_localization)]
    )
