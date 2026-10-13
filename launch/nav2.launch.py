"""Nav2 bringup for the Airseekers Tron: slam-less localization + the Nav2 controller stack.

Layer on top of ``bringup.launch.py`` (the three serial drivers). Start those first — this
file consumes their topics:

    mower_mcu_driver -> /odom                     50 Hz  SpeedData dead reckoning
    wit_imu_driver   -> /imu/data                 100 Hz JY61P, frame imu_link
    um960_gps_driver -> /fix, /fix_status         10 Hz  NavSatFix + quality summary

======================================================================================
Required apt packages
======================================================================================

Localization (this file's core):

    ros-jazzy-robot-localization    ekf_node, navsat_transform_node
    ros-jazzy-nav2-bringup          Nav2 params + lifecycle bringup
    ros-jazzy-nav2-controller       RegulatedPurePursuit, RotationShim
    ros-jazzy-nav2-planner          SmacPlanner2D
    ros-jazzy-nav2-costmap-2d       static + obstacle layers, keepout filter
    ros-jazzy-nav2-behaviors        spin, back up, wait
    ros-jazzy-nav2-smoother         costmap smoothing
    ros-jazzy-nav2-velocity-smoother velocity limits, collision_monitor chain
    ros-jazzy-nav2-map-server       static layer source
    ros-jazzy-nav2-lifecycle-manager
    ros-jazzy-robot-state-publisher URDF frames -> /tf (entry 0 below)
    ros-jazzy-xacro                 robot_description preprocessor
    ros-jazzy-tf2-ros, ros-jazzy-tf2-geometry-msgs, ros-jazzy-geometry2_msgs

NOT installed, deliberately: ``ros-jazzy-slam-toolbox``. This is a slam-less stack.

``docker/Dockerfile.jazzy`` already installs ``ros-jazzy-ros-base``,
``ros-jazzy-nav2-bringup`` and ``ros-jazzy-control-toolbox``, so the only genuinely new
line for this file is:

    apt-get install -y ros-jazzy-robot-localization

That belongs in the image build, not on the device: the host is Ubuntu 20.04/Noetic and
cannot apt-install Jazzy (docs/deployment.md).

``ros-jazzy-robot-state-publisher`` and ``ros-jazzy-xacro`` (entry 0 below)
are likewise image dependencies, not yet in ``docker/Dockerfile.jazzy`` —
``ros-jazzy-nav2-bringup`` may pull robot_state_publisher in transitively,
but install both explicitly to be sure.

======================================================================================
Frame tree (REP-105) and who owns each transform
======================================================================================

    map                                                   <- ekf_map (localization_mode:=dual)
      |        2026-10-09, REP-105 dual EKF (default): ekf_map (world_frame map,
      |        config/ekf_dual.yaml) fuses wheel + aligned IMU + /odometry/gps (RTK,
      |        covariance by fix class) and publishes map -> odom and
      |        /odometry/filtered_map. The GPS correction lives in map -> odom; odom
      |        stays continuous. localization_mode:=single is the old fallback: one
      |        GPS-fused filter in odom (config/ekf.yaml) + the static identity
      |        map -> odom from mower.launch.py (static_map_odom), and gui_bridge
      |        relays /odometry/filtered as /odometry/filtered_map.
      |        Map origin = the navsat datum (datum_lat/lon/yaw) in both modes.
      |
    odom                                                  <- ekf_node (publish_tf: true)
      |        Sole publisher of odom -> base_link. Dual mode: no GPS in this filter.
      |
    base_link                                             <- URDF: config/urdf/mower.urdf.xacro
      |        Centre of the REAR DRIVE AXLE, not the chassis centre
      |        (MowgliNext default chassis_center_x = 0.18 m).
      |        Published by the robot_state_publisher entry below
      |        (mower_mcu_driver's /odom carries it as child_frame_id).
      |
      +-- imu_link   WIT JY61P. The URDF owns base_link -> imu_link
      |             (identity today); wit_imu_driver's publish_tf
      |             parameter defaults to false and bringup does not
      |             override it. TODO(calib): set the real mounting
      |             offset in the xacro.
      +-- gps_link   UM960 antenna. um960_gps_driver stamps /fix with
      |             frame_id "gps_link" (launch/bringup.launch.py
      |             overrides the driver's built-in "gps" default so
      |             the published frame matches this URDF child link)
      |             and publishes NO transform, so the frame must
      |             exist in the URDF or navsat_transform_node logs
      |             "Unable to obtain base_link -> gps_link transform"
      |             and assumes the receiver sits at the robot origin
      |             — a silent lever-arm error.
      +-- bumper     bumper strip; mower_mcu_driver reports state in
      |             MowerSensorInfo (no TF from the driver).
      +-- cutter_link cutter, if a costmap ever needs it.
      +-- (no lidar_link: the Tron has no 2D LiDAR. Obstacle input is Metoak
          stereo depth — planned metoak_stereo_driver, see
          docs/localization_control_plan.md.)

Sole TF ownership (MowgliNext Invariant 2, kept here): no driver broadcasts TF —
``wit_imu_driver``'s ``publish_tf`` defaults to False — so ``robot_state_publisher``
owns the URDF's frames (including the IMU's static mount) and ``ekf_node`` is the
only publisher of ``odom -> base_link``.

======================================================================================
RESOLVED: navsat_transform_node needs an IMU orientation that we do not publish
======================================================================================

This used to be the top blocker: ``wit_imu_driver`` left ``Imu.orientation`` at all
zeros and reported all-zero covariances, so ``navsat_transform_node`` built its
GPS-to-world transform from a zero quaternion, ``ekf_node`` fused zeros as ``imu0``
pose yaw, and robot_localization treated every IMU quantity as a perfect measurement.

It is fixed in the driver: ``wit_node._publish_typed()`` fills the quaternion from the
JY61P 0x53 roll/pitch/yaw frame and reports ``orientation_rp_covariance`` (0.02) /
``orientation_yaw_covariance`` (1.0). The remaining limitation is physical, not a code
defect — the JY61P is a 6-axis unit, so its yaw is gyro-integrated and drifts. That is
what ``heading_aligner`` (below, ``/imu/data`` -> ``/imu/data_aligned``) is for: it puts
yaw in ENU from course-over-ground / dock pose / a persisted offset, and
``ekf_node`` consumes the aligned topic, not the raw one.

Still worth knowing: ``navsat_transform_node`` can also be pointed at the odometry
heading with ``use_odometry_yaw: true`` if the IMU heading ever needs bypassing. See
docs/localization_control_plan.md section 3.

======================================================================================
The gate between the receiver and the filter
======================================================================================

``navsat_transform_node`` fuses a single-point fix without complaint, and a 1-2 m error in
the projected pose becomes a metre-scale error in where Nav2 thinks the robot is.
``mower_localization/gps_gate`` republishes ``/fix`` as ``/fix_gated`` only when the fix is
worth fusing: ``NavSatFix.status.status`` at or above ``min_fix_status``, a ``used_fixes``
whitelist, and a position covariance inside ``max_position_covariance``. The decision reads
the message itself, never a filter that already consumed the fix.

``navsat_transform_node`` subscribes to ``gps/fix``, so the remap below is what points it at
the gated stream.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _float(argument: str) -> ParameterValue:
    """Force a launch argument to float.

    Without this a YAML/launch string such as "100.0" reaches a double-typed ROS parameter as
    a string and the node rejects it at load time.
    """
    return ParameterValue(LaunchConfiguration(argument), value_type=float)


def _int(argument: str) -> ParameterValue:
    """Force a launch argument to int (same reason as :func:`_float`)."""
    return ParameterValue(LaunchConfiguration(argument), value_type=int)


def _navsat_transform(context, navsat_config):
    """Build navsat_transform_node, pinning the map origin (GPS datum) when one is given.

    ``datum_lat``/``datum_lon`` both 0.0 is the "unset" placeholder: keep
    ``wait_for_datum: false`` so the first fix becomes the origin (it moves every boot, so
    the GUI map, which uses a fixed datum, will be misaligned). Otherwise pass
    ``wait_for_datum: true`` and ``datum: [lat_deg, lon_deg, yaw_rad]`` (robot_localization's
    double array). Evaluated here, not as substitutions, so the values are real floats.
    """
    lat = float(LaunchConfiguration("datum_lat").perform(context))
    lon = float(LaunchConfiguration("datum_lon").perform(context))
    yaw = float(LaunchConfiguration("datum_yaw").perform(context))

    parameters = [navsat_config]
    actions = []
    if lat == 0.0 and lon == 0.0:
        actions.append(LogInfo(msg=(
            "WARN: map origin (GPS datum) is not pinned: datum_lat/datum_lon are unset, so "
            "navsat_transform_node uses the first GPS fix as the origin (wait_for_datum: "
            "false) and the GUI map will be misaligned. Pass datum_lat:=<deg> "
            "datum_lon:=<deg> (the dock position, same as config/gui/mowgli_robot.yaml).")))
    else:
        parameters.append({
            "wait_for_datum": True,
            "datum": [lat, lon, yaw],
        })

    remappings = [
        # Its fix input is relative, so this is what points it at the
        # gate's output rather than the raw fix.
        ("gps/fix", LaunchConfiguration("gated_fix_topic")),
        ("imu", LaunchConfiguration("imu_data_topic")),
    ]
    if _mode(context) == "dual":
        # 2026-10-09: anchor on ekf_map, so world_frame (taken from this odometry's header)
        # is map and /odometry/gps comes out in the map frame for ekf_map + heading_aligner.
        remappings.append(("odometry/filtered", "/odometry/filtered_map"))
    # single: ekf_node publishes odometry/filtered under its own default name, which is
    # already the topic navsat subscribes to.
    actions.append(Node(
        package="robot_localization",
        executable="navsat_transform_node",
        respawn=True, respawn_delay=2.0,  # docs/crash_recovery.md
        name="navsat_transform_node",
        output="screen",
        parameters=parameters,
        remappings=remappings,
    ))
    return actions


LOCALIZATION_MODES = ("dual", "single")


def _mode(context) -> str:
    mode = LaunchConfiguration("localization_mode").perform(context).strip().lower()
    if mode not in LOCALIZATION_MODES:
        raise ValueError("localization_mode must be one of %s, got %r"
                         % ("|".join(LOCALIZATION_MODES), mode))
    return mode


def _ekf_nodes(context, configs):
    """The EKF(s) for localization_mode (2026-10-09).

    single: ekf_node = config/ekf.yaml (+ ekf_vio.yaml), GPS fused in odom; mower.launch.py
            publishes the static identity map -> odom. Byte-for-byte the pre-dual node.
    dual:   ekf_node (odom filter, no GPS) + ekf_map (map filter, GPS, map -> odom,
            /odometry/filtered_map) from config/ekf_dual.yaml (+ ekf_dual_vio.yaml).
    Node names must stay "ekf_node" / "ekf_map": the yaml files are keyed on them.
    """
    vio = LaunchConfiguration("vio").perform(context).strip().lower() in ("true", "1")
    common = {
        "map_frame": LaunchConfiguration("map_frame"),
        "odom_frame": LaunchConfiguration("odom_frame"),
        "base_link_frame": LaunchConfiguration("base_frame"),
        "odom0": LaunchConfiguration("odom_topic"),
        "imu0": LaunchConfiguration("imu_data_topic"),
    }

    def ekf(name, files, remappings=()):
        return Node(
            package="robot_localization",
            executable="ekf_node",
            respawn=True, respawn_delay=2.0,  # docs/crash_recovery.md
            name=name,
            output="screen",
            parameters=[*files, common],
            remappings=list(remappings),
        )

    if _mode(context) == "single":
        return [ekf("ekf_node", [configs["single"]] + ([configs["single_vio"]] if vio else []))]
    files = [configs["dual"]] + ([configs["dual_vio"]] if vio else [])
    return [
        ekf("ekf_node", files),
        ekf("ekf_map", files, remappings=[
            ("odometry/filtered", LaunchConfiguration("map_odom_topic"))]),
    ]


def generate_launch_description() -> LaunchDescription:
    share = FindPackageShare("mower_localization")
    ekf_config = PathJoinSubstitution([share, "config", "ekf.yaml"])
    navsat_config = PathJoinSubstitution([share, "config", "navsat.yaml"])
    ekf_vio_config = PathJoinSubstitution([share, "config", "ekf_vio.yaml"])
    ekf_configs = {
        "single": ekf_config,
        "single_vio": ekf_vio_config,
        "dual": PathJoinSubstitution([share, "config", "ekf_dual.yaml"]),
        "dual_vio": PathJoinSubstitution([share, "config", "ekf_dual_vio.yaml"]),
    }

    # The URDF is a stack asset under config/, not a colcon package, so
    # resolve it relative to this file instead of FindPackageShare.
    stack_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    default_urdf = os.path.join(
        stack_root, "config", "urdf", "mower.urdf.xacro")

    arguments = [
        # Toggles so launch/mower.launch.py can include this file for localization
        # only: it runs its own robot_state_publisher and leaves Nav2 to
        # mower_navigation/navigation.launch.py. Standalone use keeps both on.
        DeclareLaunchArgument("use_robot_state_publisher", default_value="true",
                              description="Start robot_state_publisher from this file."),
        # use_nav2_core: kept as a NO-OP for callers that still pass it (mower.launch.py).
        # The placeholder Nav2 nodes it used to gate were removed; Nav2 is now
        # mower_navigation/navigation.launch.py.
        DeclareLaunchArgument("use_nav2_core", default_value="true",
                              description="No-op (Nav2 moved to mower_navigation)."),
        # robot_description: xacro source for robot_state_publisher (entry 0
        # below). Defines the frame tree the drivers publish on — base_link,
        # imu_link, gps_link, bumper, ... — see the xacro header for the
        # per-frame contract.
        DeclareLaunchArgument("urdf_file", default_value=default_urdf,
                              description="xacro robot description."),
        # Frames. These match config/ekf.yaml; override only when the URDF for a specific
        # Tron chassis variant (m2-11..m2-16) says otherwise.
        DeclareLaunchArgument("map_frame", default_value="map",
                              description="GPS-anchored frame (ekf_map in dual mode)."),
        # 2026-10-09 REP-105 dual EKF (item 4). dual (default): ekf_node (odom, no GPS) +
        # ekf_map (map, RTK) owning map -> odom. single: the old one-filter config with the
        # static identity map -> odom that mower.launch.py publishes in that mode only.
        DeclareLaunchArgument("localization_mode", default_value="dual",
                              description="dual (ekf_node + ekf_map, map -> odom from GPS) | "
                                          "single (ekf.yaml, static map -> odom)."),
        DeclareLaunchArgument("map_odom_topic", default_value="/odometry/filtered_map",
                              description="ekf_map output (dual mode)."),
        DeclareLaunchArgument("gps_quality_covariance", default_value="true",
                              description="gps_gate: GPS covariance by RTK class (fixed "
                                          "tight, float loose, single dropped)."),
        DeclareLaunchArgument("localization_monitor", default_value="true",
                              description="Start localization_monitor (/localization/status)."),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("base_frame", default_value="base_link",
                              description="Rear drive axle centre, per the REP-105 chain."),
        # Sources. wit_imu_driver publishes /imu/data directly (its imu_topic parameter).
        DeclareLaunchArgument("odom_topic", default_value="/odom",
                              description="From mower_mcu_driver (SpeedData dead reckoning)."),
        # The EKF/navsat consume the heading-aligned IMU (heading_aligner, below); the raw
        # /imu/data yaw is gyro-integrated since power-on and arbitrary relative to ENU.
        DeclareLaunchArgument("raw_imu_topic", default_value="/imu/data",
                              description="From wit_imu_driver (JY61P, frame imu_link)."),
        DeclareLaunchArgument("imu_data_topic", default_value="/imu/data_aligned",
                              description="Heading-aligned IMU fused by ekf_node / navsat."),
        DeclareLaunchArgument("use_heading_aligner", default_value="true",
                              description="Start mower_localization/heading_aligner."),
        DeclareLaunchArgument("heading_offset_file",
                              default_value="/userdata/ros2/heading_offset.yaml",
                              description="heading_aligner persisted offset."),
        DeclareLaunchArgument("fix_topic", default_value="/fix",
                              description="Raw UM960 fix; gated before the filter sees it."),
        DeclareLaunchArgument("gated_fix_topic", default_value="/fix_gated"),
        # Gate thresholds. 100 m^2 is a 10 m sigma: loose enough to admit RTK float, tight
        # enough that a single-point 500 m^2 fix never reaches the filter. Set 0.0 to disable
        # the covariance check, or tighten used_fixes to RTK-fixed only.
        # Visual-inertial odometry (docs/vio.md). false (default) = the EKF is exactly
        # config/ekf.yaml and vio_gate is not started. true = vio_gate runs and ekf_node
        # also loads config/ekf_vio.yaml (odom2 = /odometry/vio_gated, vx/vy only).
        # The VIO producers (ov_msckf + vio_odom_bridge) are launch/vio.launch.py.
        DeclareLaunchArgument("vio", default_value="false",
                              description="Fuse gated VIO velocity into the EKF."),
        DeclareLaunchArgument("min_fix_status", default_value="0",
                              description="NavSatStatus.STATUS_FIX."),
        DeclareLaunchArgument("max_position_covariance", default_value="100.0",
                              description="m^2, largest diagonal. 0 disables the check."),
        # Map origin (GPS datum). Must equal datum_lat/datum_lon in
        # config/gui/mowgli_robot.yaml (both = dock position). 0.0/0.0 = unset: the first
        # fix becomes the origin (wait_for_datum: false) and a WARN is logged.
        DeclareLaunchArgument("datum_lat", default_value="0.0",
                              description="Map origin latitude, deg. 0.0 with datum_lon 0.0 = unset."),
        DeclareLaunchArgument("datum_lon", default_value="0.0",
                              description="Map origin longitude, deg. 0.0 with datum_lat 0.0 = unset."),
        DeclareLaunchArgument("datum_yaw", default_value="0.0",
                              description="Map origin heading, rad ENU (0 = east)."),
    ]

    return LaunchDescription(
        arguments
        + [
            # ------------------------------------------------------------------
            # 0. robot_state_publisher: parses the URDF/xacro above and owns all
            #    static links (base_link, imu_link, gps_link, bumpers). Drivers
            #    publish dynamic /odom->base_link and /imu.
            # ------------------------------------------------------------------
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                respawn=True, respawn_delay=2.0,  # docs/crash_recovery.md
                name="robot_state_publisher",
                condition=IfCondition(LaunchConfiguration("use_robot_state_publisher")),
                parameters=[{
                    "robot_description": Command(["xacro ", LaunchConfiguration("urdf_file")]),
                }],
            ),

            # ------------------------------------------------------------------
            # 1. GNSS quality gate. Start first: navsat_transform_node gets nothing
            #    on gps/fix (remapped to /fix_gated) until this is up.
            # ------------------------------------------------------------------
            Node(
                package="mower_localization",
                executable="gps_gate",
                respawn=True, respawn_delay=2.0,  # docs/crash_recovery.md
                name="gps_gate",
                output="screen",
                parameters=[{
                    "input_topic": LaunchConfiguration("fix_topic"),
                    "output_topic": LaunchConfiguration("gated_fix_topic"),
                    "min_fix_status": _int("min_fix_status"),
                    "max_position_covariance": _float("max_position_covariance"),
                    "quality_covariance": ParameterValue(
                        LaunchConfiguration("gps_quality_covariance"), value_type=bool),
                    # used_fixes stays at the package default
                    # (GPS/DGPS/RTK-fixed/RTK-float) unless this deployment must run
                    # RTK-fixed-only.
                }],
            ),

            # ------------------------------------------------------------------
            # 2. navsat_transform_node: /fix_gated -> /odometry/gps, which ekf_node
            #    fuses as odom1.
            #    Start order matters: this node derives its world/base frames from
            #    the odometry message header, so it waits for one
            #    /odometry/filtered before it produces anything — ekf_node below
            #    must be up and publishing even once. (Launch order does not
            #    guarantee start order; the dependency is on messages, not
            #    processes, so nothing special is needed here.)
            #    No frame parameters are passed: navsat_transform_node declares
            #    no world_frame/odom_frame/base_link_frame. It reads them off the
            #    /odometry/filtered header and child_frame_id.
            # ------------------------------------------------------------------
            OpaqueFunction(function=_navsat_transform, args=[navsat_config]),

            # ------------------------------------------------------------------
            # 2b. heading_aligner: /imu/data -> /imu/data_aligned with yaw in ENU
            #     (course-over-ground / dock pose / persisted offset). Publishes
            #     /heading_aligner/status (latched JSON) for the mission preflight.
            # ------------------------------------------------------------------
            Node(
                package="mower_localization",
                executable="heading_aligner",
                respawn=True, respawn_delay=2.0,  # docs/crash_recovery.md
                name="heading_aligner",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_heading_aligner")),
                parameters=[{
                    "imu_topic": LaunchConfiguration("raw_imu_topic"),
                    "output_topic": LaunchConfiguration("imu_data_topic"),
                    "offset_file": LaunchConfiguration("heading_offset_file"),
                }],
            ),

            # ------------------------------------------------------------------
            # 3. ekf_node: /odom + /imu (single: + /odometry/gps) -> odom -> base_link at
            #    20 Hz, publishing /odometry/filtered (its default topic name) for Nav2.
            #    Sole publisher of odom -> base_link.
            #    3b. dual only: ekf_map (+ /odometry/gps) -> map -> odom at 10 Hz and
            #    /odometry/filtered_map, which navsat_transform_node (2.) anchors on.
            #    Node names must stay "ekf_node"/"ekf_map": the yaml files are keyed on them.
            # ------------------------------------------------------------------
            # Built per localization_mode / vio by _ekf_nodes (single = byte-for-byte the
            # pre-dual node; dual adds ekf_map).
            OpaqueFunction(function=_ekf_nodes, args=[ekf_configs]),

            # 4. vio_gate (vio:=true only): /odometry/vio -> /odometry/vio_gated while
            #    RTK is not FIXED and VIO is healthy. mower_localization/vio_gate.py.
            Node(
                package="mower_localization",
                executable="vio_gate",
                respawn=True, respawn_delay=2.0,  # docs/crash_recovery.md
                name="vio_gate",
                output="screen",
                condition=IfCondition(LaunchConfiguration("vio")),
            ),

            # 5. localization_monitor (2026-10-09): /localization/status (ok|degraded|lost,
            #    RTK class, seconds since FIXED, drift estimate) for mower_mission's hold.
            Node(
                package="mower_localization",
                executable="localization_monitor",
                respawn=True, respawn_delay=2.0,  # docs/crash_recovery.md
                name="localization_monitor",
                output="screen",
                condition=IfCondition(LaunchConfiguration("localization_monitor")),
                parameters=[{"map_odom_topic": LaunchConfiguration("map_odom_topic")}],
            ),

            # Nav2 itself (controller/planner/behavior/bt_navigator/velocity_smoother
            # + lifecycle_manager_navigation) lives in
            # src/mower_navigation/launch/navigation.launch.py and
            # src/mower_navigation/config/nav2_params.yaml. This file is
            # localization only.
        ]
    )