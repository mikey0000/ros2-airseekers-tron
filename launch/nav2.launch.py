"""Nav2 bringup for the Airseekers Tron: slam-less localization + the Nav2 controller stack.

Layer on top of ``bringup.launch.py`` (the three serial drivers). Start those first — this
file consumes their topics:

    mower_mcu_driver -> /odom                     50 Hz  SpeedData dead reckoning
    wit_imu_driver   -> /imu                      100 Hz JY61P, frame imu_link
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
    ros-humble-tf2-ros, ros-humble-tf2-geometry-msgs, ros-humble-geometry2_msgs

NOT installed, deliberately: ``ros-humble-slam-toolbox``. This is a slam-less stack.

``docker/Dockerfile.humble`` already installs ``ros-humble-ros-base``,
``ros-humble-nav2-bringup`` and ``ros-humble-control-toolbox``, so the only genuinely new
line for this file is:

    apt-get install -y ros-humble-robot-localization

That belongs in the image build, not on the device: the host is Ubuntu 20.04/Noetic and
cannot apt-install Humble (docs/deployment.md).

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
    base_link                                             <- URDF (lands with mower_bringup)
      |        Centre of the REAR DRIVE AXLE, not the chassis centre
      |        (MowgliNext default chassis_center_x = 0.18 m).
      |
      +-- imu_link   WIT JY61P. wit_imu_driver broadcasts base_link -> imu_link
      |             itself; translation is identity for now. TODO(calib): set the
      |             real mounting offset.
      +-- gps        UM960 antenna. um960_gps_driver stamps /fix with frame_id
      |             "gps" and publishes NO transform, so this frame must exist in
      |             the URDF or navsat_transform_node logs "Unable to obtain
      |             base_link -> gps transform" and assumes the receiver sits at
      |             the robot origin — a silent lever-arm error.
      +-- blade_link cutter, if a costmap ever needs it.
      +-- (no lidar_link: the Tron has no 2D LiDAR. Obstacle input is Metoak
          stereo depth — planned metoak_stereo_driver, see
          docs/localization_control_plan.md.)

Sole TF ownership (MowgliNext Invariant 2, kept here): the drivers publish no TF except the
IMU's static mount, and ``ekf_node`` is the only publisher of ``odom -> base_link``.

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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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


def generate_launch_description() -> LaunchDescription:
    share = FindPackageShare("mower_localization")
    ekf_config = PathJoinSubstitution([share, "config", "ekf.yaml"])
    navsat_config = PathJoinSubstitution([share, "config", "navsat.yaml"])

    arguments = [
        # Frames. These match config/ekf.yaml; override only when the URDF for a specific
        # Tron chassis variant (m2-11..m2-16) says otherwise.
        DeclareLaunchArgument("map_frame", default_value="map",
                              description="Unpublished today; reserved for the GPS datum."),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("base_frame", default_value="base_link",
                              description="Rear drive axle centre, per the REP-105 chain."),
        # Sources. wit_imu_driver publishes /imu; the localization configs were written
        # against the Nav2-friendly /imu/data, so the remap happens here.
        DeclareLaunchArgument("odom_topic", default_value="/odom",
                              description="From mower_mcu_driver (SpeedData dead reckoning)."),
        DeclareLaunchArgument("imu_topic", default_value="/imu",
                              description="From wit_imu_driver (JY61P, frame imu_link)."),
        DeclareLaunchArgument("imu_data_topic", default_value="/imu/data",
                              description="Where the localization configs expect the IMU."),
        DeclareLaunchArgument("fix_topic", default_value="/fix",
                              description="Raw UM960 fix; gated before the filter sees it."),
        DeclareLaunchArgument("gated_fix_topic", default_value="/fix_gated"),
        # Gate thresholds. 100 m^2 is a 10 m sigma: loose enough to admit RTK float, tight
        # enough that a single-point 500 m^2 fix never reaches the filter. Set 0.0 to disable
        # the covariance check, or tighten used_fixes to RTK-fixed only.
        DeclareLaunchArgument("min_fix_status", default_value="1",
                              description="NavSatStatus.STATUS_FIX."),
        DeclareLaunchArgument("max_position_covariance", default_value="100.0",
                              description="m^2, largest diagonal. 0 disables the check."),
    ]

    return LaunchDescription(
        arguments
        + [
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
            Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform_node",
                output="screen",
                parameters=[navsat_config],
                remappings=[
                    # Its fix input is relative, so this is what points it at the
                    # gate's output rather than the raw fix.
                    ("gps/fix", LaunchConfiguration("gated_fix_topic")),
                    ("imu", LaunchConfiguration("imu_data_topic")),
                    # ekf_node publishes odometry/filtered under its own default name,
                    # which is already the topic navsat subscribes to. Nothing to remap.
                ],
            ),

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
                remappings=[
                    (LaunchConfiguration("imu_topic"),
                     LaunchConfiguration("imu_data_topic")),
                ],
            ),

            # ------------------------------------------------------------------
            # 4. Nav2 controller stack.
            #    Consumes /odometry/filtered (odom-frame) and /tf. It also needs the
            #    URDF and nav2_params.yaml, which land with mower_bringup — bring
            #    localization up alone first, then add this block once those exist.
            #    A wrong parameter key is SILENTLY IGNORED (see
            #    docs/mowglinext_baseline.md section 6), so check these key-by-key
            #    against nav2_params.yaml when it arrives.
            # ------------------------------------------------------------------
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[{
                    "controller_frequency": 20.0,
                    # Never leave odom_topic at its default "/odom": Nav2's default has
                    # no publisher in this stack, and the robot silently refuses to move.
                    "odom_topic": "/odometry/filtered",
                    "use_stamped_odom": True,
                    "path_topic": "/plan",
                    "transform_tolerance": 0.5,
                    "progress_checker_plugin": "progress_checker",
                    "goal_checker_plugins": ["general_goal_checker"],
                    # Transit: RegulatedPurePursuit behind a RotationShim, so a heading
                    # change costs a rotate-in-place instead of a wide loop.
                    "controller_plugins": ["FollowPath", "AlignToHeading"],
                }],
                remappings=[
                    ("odom", "/odometry/filtered"),
                    # Velocity chain: Nav2 -> cmd_vel_nav -> collision_monitor ->
                    # cmd_vel_monitored -> mower_control/cmd_vel_slew -> mower_mcu_driver.
                    # cmd_vel_slew lands with mower_control; until it is wired,
                    # bypassing the monitor leaves the collision_monitor timeout as
                    # the only backstop.
                    ("cmd_vel", "/cmd_vel_nav"),
                ],
            ),

            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[{
                    "expected_planner_frequency": 10.0,
                    "planner_plugins": ["SmacPlanner2D"],
                    "SmacPlanner2D": {
                        "plugin": "nav2_smac_planner::SmacPlanner2D",
                        "tolerance": 0.5,
                        "downsample_costmap": True,
                    },
                }],
            ),

            # ------------------------------------------------------------------
            # 5. Lifecycle manager, after localization. A controller_server that
            #    activates before /tf is complete reports "Timed out waiting for
            #    transform" and has to be cycled by hand.
            # ------------------------------------------------------------------
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[{
                    "autostart": True,
                    "node_names": ["controller_server", "planner_server"],
                }],
            ),
        ]
    )