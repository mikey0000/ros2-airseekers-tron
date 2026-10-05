"""Single entry point for the Airseekers Tron ROS 2 (Humble) stack.

    ros2 launch mower_bringup mower.launch.py [arg:=value ...]

Run ``ros2 launch mower_bringup mower.launch.py --show-args`` for the full list.
Groups (each a launch argument, ``true``/``false``):

    drivers       true   bringup.launch.py: mcu_node, wit_node, um960_node,
                         bumper_controller (respawn, 2 s)
    (always)             robot_state_publisher (config/urdf/mower.urdf.xacro)
    publish_static_map_odom
                  true   identity map -> odom (Phase A: map == odom, until a
                         GPS-anchored map -> odom publisher lands)
    control       true   mower_control: cmd_vel_slew (/cmd_vel_raw -> /cmd_vel),
                         slip_detector, imu_cal
    teleop        true   mower_teleop/launch/teleop.launch.py (twist_mux +
                         cmd_vel_ws_relay; twist_mux outputs /cmd_vel_raw)
    gui_bridge    true   mower_gui_bridge/gui_bridge
    foxglove      true   foxglove_bridge on :8765
    localization  true   nav2.launch.py localization only: gps_gate,
                         navsat_transform_node, ekf_node (its own
                         robot_state_publisher and placeholder Nav2 nodes are
                         switched off here)
    navigation    false  mower_navigation/launch/navigation.launch.py (Nav2)

Map origin: ``datum_lat``/``datum_lon`` default to '' (unset). Unset values are
read from ``datum_env_file`` (DATUM_LAT=/DATUM_LON=, written by the GUI's
"set datum from GPS" via gui_bridge set_datum) at launch time; if that file is
missing too they become 0.0 (= unset, first fix is the origin).

Velocity chain: teleop/Nav2/bumper -> twist_mux -> /cmd_vel_raw -> cmd_vel_slew
-> /cmd_vel -> mower_mcu_driver.

Optional packages (mower_teleop, mower_gui_bridge, mower_navigation) are
resolved with FindPackageShare / Node only when their group is enabled, so the
stack still launches with ``teleop:=false gui_bridge:=false`` on a workspace
where they are not built.
"""

import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, SetLaunchConfiguration)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


DEFAULT_DATUM_ENV = '/userdata/ros2/datum.env'


def read_datum_env(path):
    """(lat, lon) strings from a datum.env (DATUM_LAT=.., DATUM_LON=..), or None.
    Same format as mower_gui_bridge.datum.format_datum_env / parse_datum_env."""
    vals = {}
    try:
        with open(path, encoding='utf-8') as f:
            lines = f.read().splitlines()
    except OSError:
        return None
    for raw in lines:
        line = raw.strip()
        if not line or line.startswith('#'):
            continue
        if line.startswith('export '):
            line = line[len('export '):].strip()
        key, sep, value = line.partition('=')
        value = value.strip().strip('"\'')
        if not sep:
            continue
        try:
            float(value)
        except ValueError:
            continue
        vals[key.strip()] = value
    if 'DATUM_LAT' not in vals or 'DATUM_LON' not in vals:
        return None
    return vals['DATUM_LAT'], vals['DATUM_LON']


def _resolve_datum(context):
    """Fill unset datum_lat/datum_lon from datum_env_file, else 0.0."""
    lat = LaunchConfiguration('datum_lat').perform(context).strip()
    lon = LaunchConfiguration('datum_lon').perform(context).strip()
    if lat and lon:
        return [LogInfo(msg='map datum %s,%s (launch args)' % (lat, lon))]
    path = LaunchConfiguration('datum_env_file').perform(context).strip()
    found = read_datum_env(path) if path else None
    if found is not None:
        lat, lon = lat or found[0], lon or found[1]
        source = path
    else:
        lat, lon = lat or '0.0', lon or '0.0'
        source = 'default (unset; %s not found)' % (path or 'no datum_env_file')
    return [SetLaunchConfiguration('datum_lat', lat),
            SetLaunchConfiguration('datum_lon', lon),
            LogInfo(msg='map datum %s,%s from %s' % (lat, lon, source))]


def generate_launch_description() -> LaunchDescription:
    # Sibling launch files and the URDF: resolve relative to this file so it
    # works both from the source tree and from share/mower_bringup/.
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    stack_root = os.path.dirname(launch_dir)
    default_urdf = os.path.join(stack_root, 'config', 'urdf', 'mower.urdf.xacro')

    def arg(name, default, description):
        return DeclareLaunchArgument(name, default_value=default, description=description)

    def enabled(name):
        return IfCondition(LaunchConfiguration(name))

    arguments = [
        arg('drivers', 'true', 'Start the serial drivers + bumper_controller.'),
        arg('publish_static_map_odom', 'true',
            'Publish identity map -> odom (Phase A: map == odom).'),
        arg('control', 'true', 'Start mower_control (cmd_vel_slew, slip_detector, imu_cal).'),
        arg('teleop', 'true', 'Include mower_teleop/teleop.launch.py (twist_mux + relay).'),
        arg('gui_bridge', 'true', 'Start mower_gui_bridge/gui_bridge.'),
        arg('foxglove', 'true', 'Start foxglove_bridge.'),
        arg('foxglove_port', '8765', 'foxglove_bridge WebSocket port.'),
        arg('localization', 'true', 'Include nav2.launch.py localization (gps_gate, navsat, ekf).'),
        arg('navigation', 'false', 'Include mower_navigation/navigation.launch.py (Nav2).'),
        arg('map_server', 'true', 'Include mower_map/map_server.launch.py (zones, keepout mask, dock pose).'),
        arg('maps_dir', '/ros2_ws/maps', 'areas.dat / dock_pose.yaml directory (compose mounts /userdata/ros2/maps).'),
        arg('coverage', 'true', 'Include mower_coverage_bridge (planner + /plan_coverage action).'),
        arg('cut_width_m', '0.20', 'Blade cut width, m (URDF cutter disc radius 0.10; TODO: measure '
            'the blade). Swath spacing = cut_width_m - swath_overlap_m.'),
        arg('swath_overlap_m', '0.02', 'Overlap between neighbouring coverage swaths, m.'),
        arg('docking', 'true', 'Include mower_docking/docking.launch.py (dock/undock actions).'),
        arg('mission', 'true', 'Include mower_mission/mission.launch.py (the /behavior_tree_node mission layer). '
            'When true, gui_bridge runs with serve_high_level:=false.'),
        arg('cameras', 'true', 'Include cameras.launch.py (OA + rear v4l2_camera).'),
        arg('video', 'true', 'web_video_server MJPEG on :8080 (GUI camera page via /api/cameras; source for the RTSP relay).'),
        arg('perception', 'false', 'Include mower_vision/perception.launch.py (det_ros + obstacle_guard).'),
        arg('stop_on_close', 'false', 'obstacle_guard: zero burst + cutter off on a close obstacle.'),
        arg('datum_lat', '', 'Map origin (GPS datum) latitude, deg. Dock position; must match '
            'config/gui/mowgli_robot.yaml. Empty = DATUM_LAT from datum_env_file, else 0.0 '
            '(0.0/0.0 = unset: first fix).'),
        arg('datum_lon', '', 'Map origin (GPS datum) longitude, deg (see datum_lat).'),
        arg('datum_env_file', DEFAULT_DATUM_ENV,
            'DATUM_LAT=/DATUM_LON= file written by gui_bridge set_datum; supplies unset '
            'datum_lat/datum_lon.'),
        arg('datum_yaw', '0.0', 'Map origin heading, rad ENU (0 = east).'),
        arg('urdf_file', default_urdf, 'xacro robot description.'),
        arg('imu_calibration_file', '/userdata/ros2/calibration/imu_calibration.yaml',
            'imu_cal persisted bias file (on the device /userdata partition).'),
    ]

    drivers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup.launch.py')),
        condition=enabled('drivers'),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', LaunchConfiguration('urdf_file')]), value_type=str),
        }],
    )

    static_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_odom',
        output='screen',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'map', '--child-frame-id', 'odom'],
        condition=enabled('publish_static_map_odom'),
    )

    control = [
        Node(package='mower_control', executable='cmd_vel_slew', name='cmd_vel_slew',
             output='screen', condition=enabled('control')),
        Node(package='mower_control', executable='slip_detector', name='slip_detector',
             output='screen', condition=enabled('control')),
        Node(package='mower_control', executable='imu_cal', name='imu_cal',
             output='screen', condition=enabled('control'),
             parameters=[{'calibration_file': LaunchConfiguration('imu_calibration_file')}]),
    ]

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('mower_teleop'), 'launch', 'teleop.launch.py'])),
        condition=enabled('teleop'),
    )

    # gui_bridge's stub high-level state machine is only used when the real mission node is off.
    gui_bridge = Node(
        package='mower_gui_bridge',
        executable='gui_bridge',
        name='gui_bridge',
        output='screen',
        parameters=[{
            'serve_high_level': PythonExpression(
                ["'", LaunchConfiguration('mission'), "' != 'true'"]),
            # set_datum persists here (GUI settings file + launch datum defaults).
            'robot_yaml_path': os.path.join(stack_root, 'config', 'gui', 'mowgli_robot.yaml'),
            'datum_env_path': LaunchConfiguration('datum_env_file'),
            'datum_lat': ParameterValue(LaunchConfiguration('datum_lat'), value_type=float),
            'datum_lon': ParameterValue(LaunchConfiguration('datum_lon'), value_type=float),
        }],
        condition=enabled('gui_bridge'),
    )

    mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('mower_mission'), 'launch', 'mission.launch.py'])),
        condition=enabled('mission'),
    )

    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': ParameterValue(LaunchConfiguration('foxglove_port'), value_type=int),
            'send_buffer_limit': 10000000,
            'use_compression': False,
        }],
        condition=enabled('foxglove'),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'nav2.launch.py')),
        launch_arguments={
            'urdf_file': LaunchConfiguration('urdf_file'),
            # robot_state_publisher runs above; Nav2 lives in mower_navigation.
            'use_robot_state_publisher': 'false',
            'use_nav2_core': 'false',
            # Pinned map origin shared with the GUI (config/gui/mowgli_robot.yaml).
            'datum_lat': LaunchConfiguration('datum_lat'),
            'datum_lon': LaunchConfiguration('datum_lon'),
            'datum_yaw': LaunchConfiguration('datum_yaw'),
        }.items(),
        condition=enabled('localization'),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('mower_navigation'), 'launch', 'navigation.launch.py'])),
        condition=enabled('navigation'),
    )

    map_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('mower_map'), 'launch', 'map_server.launch.py'])),
        launch_arguments={
            'maps_dir': LaunchConfiguration('maps_dir'),
            # Keeps the GUI's dock_pose_x/y/yaw in sync (config/gui is mounted into the GUI container).
            'robot_yaml_path': os.path.join(stack_root, 'config', 'gui', 'mowgli_robot.yaml'),
            'datum_lat': LaunchConfiguration('datum_lat'),
            'datum_lon': LaunchConfiguration('datum_lon'),
        }.items(),
        condition=enabled('map_server'),
    )

    coverage = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('mower_coverage_bridge'), 'launch', 'coverage_bridge.launch.py'])),
        launch_arguments={
            'cut_width_m': LaunchConfiguration('cut_width_m'),
            'swath_overlap_m': LaunchConfiguration('swath_overlap_m'),
        }.items(),
        condition=enabled('coverage'),
    )

    docking = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('mower_docking'), 'launch', 'docking.launch.py'])),
        condition=enabled('docking'),
    )

    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'cameras.launch.py')),
        launch_arguments={'web_video_server': LaunchConfiguration('video')}.items(),
        condition=enabled('cameras'),
    )

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('mower_vision'), 'launch', 'perception.launch.py'])),
        launch_arguments={'stop_on_close': LaunchConfiguration('stop_on_close')}.items(),
        condition=enabled('perception'),
    )

    return LaunchDescription(
        arguments
        + [OpaqueFunction(function=_resolve_datum)]
        + [drivers, robot_state_publisher, static_map_odom]
        + control
        + [teleop, gui_bridge, foxglove, localization, navigation,
           map_server, coverage, docking, mission, cameras, perception]
    )
