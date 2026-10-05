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

    ros-humble-robot-localization    ekf_node, navsat_transform_node
    ros-humble-nav2-bringup          Nav2 params + lifecycle bringup
    ros-humble-nav2-controller       RegulatedPurePursuit, RotationShim
    ros-humble-nav2-planner          SmacPlanner2D
    ros-humble-nav2-costmap-2d       static + obstacle layers, keepout filter
    ros-humble-nav2-behaviors        spin, back up, wait
    ros-humble-nav2-smoother         costmap smoothing
    ros-humble-nav2-velocity-smoother velocity limits, collision_monitor chain
    ros-humble-nav2-map-server       static layer source
    ros-humble-nav2-lifecycle-manager
    ros-humble-robot-state-publisher URDF frames -> /tf (entry 0 below)
    ros-humble-xacro                 robot_description preprocessor
    ros-humble-tf2-ros, ros-humble-tf2-geometry-msgs, ros-humble-geometry2_msgs

NOT installed, deliberately: ``ros-humble-slam-toolbox``. This is a slam-less stack.

``docker/Dockerfile.humble`` already installs ``ros-humble-ros-base``,
``ros-humble-nav2-bringup`` and ``ros-humble-control-toolbox``, so the only genuinely new
line for this file is:

    apt-get install -y ros-humble-robot-localization

That belongs in the image build, not on the device: the host is Ubuntu 20.04/Noetic and
cannot apt-install Humble (docs/deployment.md).

``ros-humble-robot-state-publisher`` and ``ros-humble-xacro`` (entry 0 below)
are likewise image dependencies, not yet in ``docker/Dockerfile.humble`` —
``ros-humble-nav2-bringup`` may pull robot_state_publisher in transitively,
but install both explicitly to be sure.

======================================================================================
Frame tree (REP-105) and who owns each transform
======================================================================================

    map                                                   <- NOT PUBLISHED by this stack
      |        The map frame has no publisher yet. navsat_transform_node only
      |        broadcasts a transform when broadcast_utm_transform is true, and
      |        config/navsat.yaml sets it false — so there is no map -> odom
      |        anchor. Until one lands, everything is odom-framed dead
      |        reckoning corrected by GPS: continuous, but it starts wherever
      |        the machine happened to be switched on.
      |
    odom                                                  <- ekf_node (publish_tf: true)
      |        The ONLY transform this stack currently owns.
      |
    base_link                                             <- URDF: config/urdf/mower.urdf.xacro
      |        Centre of the REAR DRIVE AXLE, not the chassis centre
      |        (MowgliNext default chassis_center_x = 0.18 m).
      |        Published by the robot_state_publisher entry below
      |        (mower_mcu_driver's /odom carries it as child_frame_id).
      |
      +-- imu_link   WIT JY61P. wit_imu_driver broadcasts base_link ->
      |             imu_link itself (identity today); the URDF's
      |             imu_joint is identity on purpose so the two
      |             publishers agree. TODO(calib): set the real
      |             mounting offset in the xacro AND drop the driver
      |             broadcast (handoff_gap_analysis P0 #5).
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

Sole TF ownership (MowgliNext Invariant 2, kept here): the drivers publish no TF
except the IMU's static mount, ``robot_state_publisher`` owns the URDF's
fixed-joint frames, and ``ekf_node`` is the only publisher of
``odom -> base_link``.

======================================================================================
BLOCKER: navsat_transform_node needs an IMU orientation that we do not publish
======================================================================================

``navsat_transform_node`` builds its GPS-to-world transform from the IMU's orientation
quaternion (or, with ``use_odometry_yaw: true``, from the odometry message). But
``wit_imu_driver`` never populates ``Imu.orientation`` — ``fill_angle_from_tracked()`` only
logs the JY61P 0x53 angle frame at debug level and leaves the quaternion at all zeros. It also
publishes all-zero covariances.

Consequences, all of them quiet:

* navsat_transform_node reads a zero quaternion as orientation and produces a wrong heading
  for the whole GPS correction.
* ekf_node fusing ``imu0`` pose yaw (``config/ekf.yaml`` enables it) fuses zeros.
* Zero covariances tell robot_localization every IMU-derived quantity is a perfect
  measurement, so it over-trusts them.

So localization cannot be trusted until ``wit_imu_driver`` fills ``orientation`` from the
0x53 roll/pitch/yaw frames as a proper quaternion and reports realistic covariances. Fix that
driver first, or run this file knowing the GPS correction is garbage. Details in
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

    actions.append(Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform_node",
        output="screen",
        parameters=parameters,
        remappings=[
            # Its fix input is relative, so this is what points it at the
            # gate's output rather than the raw fix.
            ("gps/fix", LaunchConfiguration("gated_fix_topic")),
            ("imu", LaunchConfiguration("imu_data_topic")),
            # ekf_node publishes odometry/filtered under its own default name,
            # which is already the topic navsat subscribes to. Nothing to remap.
        ],
    ))
    return actions


def generate_launch_description() -> LaunchDescription:
    share = FindPackageShare("mower_localization")
    ekf_config = PathJoinSubstitution([share, "config", "ekf.yaml"])
    navsat_config = PathJoinSubstitution([share, "config", "navsat.yaml"])

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
                              description="Unpublished today; reserved for the GPS datum."),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("base_frame", default_value="base_link",
                              description="Rear drive axle centre, per the REP-105 chain."),
        # Sources. wit_imu_driver publishes /imu/data directly (its imu_topic parameter).
        DeclareLaunchArgument("odom_topic", default_value="/odom",
                              description="From mower_mcu_driver (SpeedData dead reckoning)."),
        DeclareLaunchArgument("imu_data_topic", default_value="/imu/data",
                              description="From wit_imu_driver (JY61P, frame imu_link)."),
        DeclareLaunchArgument("fix_topic", default_value="/fix",
                              description="Raw UM960 fix; gated before the filter sees it."),
        DeclareLaunchArgument("gated_fix_topic", default_value="/fix_gated"),
        # Gate thresholds. 100 m^2 is a 10 m sigma: loose enough to admit RTK float, tight
        # enough that a single-point 500 m^2 fix never reaches the filter. Set 0.0 to disable
        # the covariance check, or tighten used_fixes to RTK-fixed only.
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
                name="gps_gate",
                output="screen",
                parameters=[{
                    "input_topic": LaunchConfiguration("fix_topic"),
                    "output_topic": LaunchConfiguration("gated_fix_topic"),
                    "min_fix_status": _int("min_fix_status"),
                    "max_position_covariance": _float("max_position_covariance"),
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
            # 3. ekf_node: /odom + /imu (+ /odometry/gps) -> odom -> base_link at
            #    30 Hz, publishing /odometry/filtered (its default topic name) for the
            #    node above and Nav2.
            #    Sole publisher of odom -> base_link.
            #    Node name must stay "ekf_node": config/ekf.yaml is keyed on it.
            # ------------------------------------------------------------------
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_node",
                output="screen",
                parameters=[
                    ekf_config,
                    {
                        "map_frame": LaunchConfiguration("map_frame"),
                        "odom_frame": LaunchConfiguration("odom_frame"),
                        "base_link_frame": LaunchConfiguration("base_frame"),
                        "odom0": LaunchConfiguration("odom_topic"),
                        "imu0": LaunchConfiguration("imu_data_topic"),
                    },
                ],
            ),

            # Nav2 itself (controller/planner/behavior/bt_navigator/velocity_smoother
            # + lifecycle_manager_navigation) lives in
            # src/mower_navigation/launch/navigation.launch.py and
            # src/mower_navigation/config/nav2_params.yaml. This file is
            # localization only.
        ]
    )