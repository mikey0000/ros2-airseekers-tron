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
    navigation    true   mower_navigation/launch/navigation.launch.py (Nav2)
    stereo_costmap
                  true   stereo depth + det_range as local-costmap obstacle
                         sources; false = bumper-only obstacle layer (safe-off)

Map origin: ``datum_lat``/``datum_lon`` default to '' (unset). Unset values are
read from ``datum_env_file`` (DATUM_LAT=/DATUM_LON=, written by the GUI's
"set datum from GPS" via gui_bridge set_datum) at launch time; if that file is
missing too they become 0.0 (= unset, first fix is the origin).

GUI settings: ``robot_settings_file`` (default config/gui/mowgli_robot.yaml, the
file the MowgliNext GUI saves) is read at launch; the settings bound to ROS
parameters for this robot (launch/robot_settings.py: speeds, cut width, mission
thresholds, docking approach, NTRIP caster) override the package defaults of
the mission, Nav2, docking, coverage and um960 nodes. Keys missing from the
file keep the package defaults; ``robot_settings_file:=''`` disables this.

Velocity chain: teleop/Nav2/bumper -> twist_mux -> /cmd_vel_raw -> cmd_vel_slew
-> /cmd_vel -> mower_mcu_driver.

Optional packages (mower_teleop, mower_gui_bridge, mower_navigation) are
resolved with FindPackageShare / Node only when their group is enabled, so the
stack still launches with ``teleop:=false gui_bridge:=false`` on a workspace
where they are not built.
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, SetLaunchConfiguration)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import robot_settings  # noqa: E402  (launch/robot_settings.py, installed alongside)


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


# Settings target -> (package whose share holds the base params file, its
# path below share/, launch configuration handed to the include as params_file).
SETTINGS_PARAM_FILES = {
    'behavior_tree_node': ('mower_mission', 'config/mission.yaml', 'mission_params_file'),
    'mower_docking': ('mower_docking', 'config/docking.yaml', 'docking_params_file'),
    'controller_server': ('mower_navigation', 'config/nav2_params.yaml', 'nav2_params_file'),
}
# Targets that get an overlay file instead (the node has no params_file).
SETTINGS_OVERLAYS = {'um960_gps_driver': 'um960_params_file'}
DEFAULT_CUT_WIDTH_M = '0.20'


def _share_file(package, rel):
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    try:
        return os.path.join(get_package_share_directory(package), rel)
    except PackageNotFoundError:
        return None


def _apply_robot_settings(context):
    """Map the GUI's bound settings onto node parameters (see robot_settings.py)."""
    path = LaunchConfiguration('robot_settings_file').perform(context).strip()
    settings = robot_settings.load_settings(path)
    values, warnings = robot_settings.bound_values(settings)
    actions = [LogInfo(msg='robot settings: ' + w) for w in warnings]

    # Defaults: the packages' own files, no um960 overlay.
    for target, (package, rel, config) in SETTINGS_PARAM_FILES.items():
        actions.append(SetLaunchConfiguration(config, _share_file(package, rel) or ''))
    for config in SETTINGS_OVERLAYS.values():
        actions.append(SetLaunchConfiguration(config, ''))

    cut_width = LaunchConfiguration('cut_width_m').perform(context).strip()
    if not cut_width:
        launch_values = values.pop('launch', {})
        cut_width = str(launch_values.get('cut_width_m', DEFAULT_CUT_WIDTH_M))
        actions.append(SetLaunchConfiguration('cut_width_m', cut_width))
    else:
        values.pop('launch', None)

    if not values:
        actions.append(LogInfo(msg='robot settings: none bound in %s (package defaults)'
                                   % (path or 'no robot_settings_file')))
        return actions
    out_dir = robot_settings.make_out_dir()
    for target, params in values.items():
        if target in SETTINGS_PARAM_FILES:
            package, rel, config = SETTINGS_PARAM_FILES[target]
            base = _share_file(package, rel)
            if base is None or not os.path.isfile(base):
                actions.append(LogInfo(msg='robot settings: %s not installed, %s ignored'
                                           % (package, sorted(params))))
                continue
            doc = robot_settings.merged_params_document(base, target, params)
        elif target in SETTINGS_OVERLAYS:
            config = SETTINGS_OVERLAYS[target]
            doc = robot_settings.overlay_document(target, params)
        else:
            continue
        written = robot_settings.write_params_file(doc, out_dir, target + '.yaml')
        actions.append(SetLaunchConfiguration(config, written))
        shown = {k: ('***' if 'password' in k else v) for k, v in params.items()}
        actions.append(LogInfo(msg='robot settings: /%s %s (from %s)' % (target, shown, path)))
    return actions


def _apply_stereo_costmap(context):
    """stereo_costmap:=false -> nav2 params without the stereo/det_range costmap sources."""
    if LaunchConfiguration('stereo_costmap').perform(context).strip().lower() in ('true', '1'):
        return []
    import yaml
    base = LaunchConfiguration('nav2_params_file').perform(context)
    if not base or not os.path.isfile(base):
        return [LogInfo(msg='stereo_costmap:=false: no nav2 params file, nothing to change')]
    with open(base, encoding='utf-8') as f:
        doc = robot_settings.without_stereo_sources(yaml.safe_load(f))
    written = robot_settings.write_params_file(doc, robot_settings.make_out_dir(),
                                               'nav2_params_no_stereo.yaml')
    return [SetLaunchConfiguration('nav2_params_file', written),
            LogInfo(msg='stereo_costmap:=false: local costmap obstacle_layer is bumper-only '
                        '(%s)' % written)]


def generate_launch_description() -> LaunchDescription:
    # Sibling launch files and the URDF: resolve relative to this file so it
    # works both from the source tree and from share/mower_bringup/.
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    stack_root = os.path.dirname(launch_dir)
    default_urdf = os.path.join(stack_root, 'config', 'urdf', 'mower.urdf.xacro')
    # The live GUI settings file (realpath: an install tree may hold a copy).
    default_robot_settings = os.path.join(
        os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
        'config', 'gui', 'mowgli_robot.yaml')

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
        arg('navigation', 'true', 'Include mower_navigation/navigation.launch.py (Nav2: bt_navigator, controller/planner/behavior servers, velocity_smoother). Needed by mission and docking (/navigate_to_pose, /follow_path).'),
        arg('map_server', 'true', 'Include mower_map/map_server.launch.py (zones, keepout mask, dock pose).'),
        arg('maps_dir', '/ros2_ws/maps', 'areas.dat / dock_pose.yaml directory (compose mounts /userdata/ros2/maps).'),
        arg('coverage', 'true', 'Include mower_coverage_bridge (planner + /plan_coverage action).'),
        arg('cut_width_m', '', 'Blade cut width, m. Empty = the GUI tool_width from '
            'robot_settings_file, else 0.20 (URDF cutter disc radius 0.10; TODO: measure the '
            'blade). Swath spacing = cut_width_m - swath_overlap_m.'),
        arg('swath_overlap_m', '0.02', 'Overlap between neighbouring coverage swaths, m.'),
        arg('docking', 'true', 'Include mower_docking/docking.launch.py (dock/undock actions).'),
        arg('mission', 'true', 'Include mower_mission/mission.launch.py (the /behavior_tree_node mission layer). '
            'When true, gui_bridge runs with serve_high_level:=false.'),
        arg('cameras', 'true', 'Include cameras.launch.py (OA + rear v4l2_camera).'),
        arg('stereo', 'true', 'cameras.launch.py: front Metoak stereo via mower_cameras/stereo_cam '
            '(/vio/{left,right}/image_raw, 5 Hz, on demand). Off when vio is true.'),
        arg('stereo_costmap', 'true', 'Feed the front stereo depth (/stereo_depth/points) and '
            'det_range into the Nav2 local costmap obstacle_layer. false = bumper-only obstacle '
            'layer (safe-off switch if the stereo marks the lawn); stereo_depth/det_range keep '
            'running for the GUI/obstacle_guard.'),
        arg('vio', 'false', 'stereo_vio_bridge owns the stereo device (run launch/vio.launch.py); '
            'cameras.launch.py then does not start stereo_cam.'),
        arg('video', 'true', 'web_video_server MJPEG on :8080 (GUI camera page via /api/cameras; source for the RTSP relay).'),
        arg('perception', 'true', 'Include mower_vision/perception.launch.py (det_ros on both OA cameras + obstacle_guard; seg off). '
            'det_ros publishes /ai/det/detections and /<camera_ns>/image_annotated (GUI Perception page).'),
        arg('det_range', 'true', 'det_range: range det_ros detections of the front stereo right eye '
            'with the stereo hardware depth -> /ai/det/detections_ranged (obstacle_guard prefers it), '
            '/ai/det/obstacles, /ai/det/obstacle_points (local costmap source).'),
        arg('det_backend', 'cpp', 'det_ros implementation: cpp (det_ros_cpp, RKNN C API) or python '
            '(det_ros, rknn-toolkit-lite2 fallback). Same node name, params, topics.'),
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
        arg('robot_settings_file', default_robot_settings,
            'GUI settings (mowgli_robot.yaml); bound keys override package defaults '
            '(launch/robot_settings.py). Empty = package defaults only.'),
        arg('imu_calibration_file', '/userdata/ros2/calibration/imu_calibration.yaml',
            'imu_cal persisted bias file (on the device /userdata partition).'),
    ]

    drivers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup.launch.py')),
        # NTRIP caster / correction source from the GUI settings (empty = none).
        launch_arguments={'um960_params_file': LaunchConfiguration('um960_params_file')}.items(),
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
        # mission.yaml with the GUI's bound settings merged in (_apply_robot_settings).
        launch_arguments={'params_file': LaunchConfiguration('mission_params_file')}.items(),
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
            # /foxglove_bridge/sysinfo (2 Hz /proc scan) has no subscriber here.
            # num_threads stays at the default (one per core): 2 threads measured the same
            # total CPU, and the GUI's blocking parameter requests would then hold up a
            # larger share of message delivery. The bridge's cost is ~3 ms per delivered
            # message (most likely each executor wake walking the ~200 parameter/service
            # clients created for the GUI's parameter requests), so it scales with the
            # subscribed rate: the GUI's 50 Hz /wheel_odom and 30 Hz /odometry/filtered_map
            # are ~2/3 of it.
            'sysinfo': False,
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
        launch_arguments={'params_file': LaunchConfiguration('nav2_params_file')}.items(),
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
            # Explicit: the boundary monitor needs the map-frame EKF pose (not /odom).
            'map_odom_topic': '/odometry/filtered_map',
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
        launch_arguments={'params_file': LaunchConfiguration('docking_params_file')}.items(),
        condition=enabled('docking'),
    )

    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'cameras.launch.py')),
        launch_arguments={'web_video_server': LaunchConfiguration('video'),
                          'stereo': LaunchConfiguration('stereo'),
                          'vio': LaunchConfiguration('vio')}.items(),
        condition=enabled('cameras'),
    )

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('mower_vision'), 'launch', 'perception.launch.py'])),
        launch_arguments={'stop_on_close': LaunchConfiguration('stop_on_close'),
                          'det_backend': LaunchConfiguration('det_backend')}.items(),
        condition=enabled('perception'),
    )

    det_range = Node(
        package='det_range',
        executable='det_range',
        name='det_range',
        output='screen',
        parameters=[PathJoinSubstitution([FindPackageShare('det_range'), 'config',
                                          'det_range.yaml'])],
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration('det_range'), "' == 'true' and '",
             LaunchConfiguration('perception'), "' == 'true'"])),
    )

    return LaunchDescription(
        arguments
        + [OpaqueFunction(function=_resolve_datum),
           OpaqueFunction(function=_apply_robot_settings),
           OpaqueFunction(function=_apply_stereo_costmap)]
        + [drivers, robot_state_publisher, static_map_odom]
        + control
        + [teleop, gui_bridge, foxglove, localization, navigation,
           map_server, coverage, docking, mission, cameras, perception, det_range]
    )
