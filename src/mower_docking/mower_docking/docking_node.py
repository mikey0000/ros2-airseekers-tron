# Copyright 2026 The mower_docking authors
# SPDX-License-Identifier: Apache-2.0
"""ROS 2 glue for the docking server (node name ``mower_docking``).

Actions:  ``/mower_docking/dock`` (mower_interfaces/Dock),
          ``/mower_docking/undock`` (mower_interfaces/Undock).
Publishes ``/cmd_vel_docking`` (twist_mux lane, priority 15, 0.5 s timeout)
at ``control_rate_hz`` while a goal drives the robot, and zeros for
``stop_burst_s`` on every exit.  All decisions live in :mod:`dock_logic`.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, UInt8

from mower_interfaces.action import Dock, Undock
from mower_interfaces.msg import MowerBaseDevStatus
from mower_interfaces.srv import ChargingControl

from mower_docking import dock_logic as dl
from mower_docking.sub_pump import SubscriptionPump, flat_parser, parse_odometry
from mower_docking.quiet_action import QuietActionClient

try:
    from mowgli_interfaces.msg import GnssStatus
except ImportError:  # pragma: no cover
    GnssStatus = None

MOTION_IDLE_STATES = (dl.DockState.NAV_TO_APPROACH,)


class DockingServer(Node):

    def __init__(self):
        super().__init__('mower_docking')
        dp = dl.DockParams()
        g = dl.ControllerGains()
        decl = self.declare_parameter
        # topics
        decl('cmd_vel_topic', '/cmd_vel_docking')
        decl('odom_topic', '/odom')
        decl('status_topic', '/mower_base/status')
        decl('dock_pose_topic', '/map_server_node/docking_pose')
        decl('dock_pose_measured_topic', '/map_server_node/docking_pose_measured')
        decl('progress_odom_topic', '/odometry/filtered')   # stall guard (fused pose)
        decl('dig_stall_topic', '/dig_stall')               # slip_detector latch
        decl('gps_status_topic', '/gps/status')
        decl('image_topic', '/rear_camera/image_raw')
        decl('camera_info_topic', '/rear_camera/camera_info')
        decl('charging_service', '/charging')
        decl('navigate_action', '/navigate_to_pose')
        decl('map_frame', 'map')
        decl('base_frame', 'base_link')
        # timing / freshness
        decl('control_rate_hz', 20.0)
        decl('stop_burst_s', 0.5)
        decl('odom_timeout_s', 0.5)
        decl('status_timeout_s', 1.0)
        decl('contact_debounce_samples', 3)
        # dock geometry / approach
        decl('dock_pose', [0.0, 0.0, 0.0])
        decl('approach_distance', dp.approach_distance)
        decl('skip_nav_to_approach', dp.skip_nav_to_approach)
        decl('nav_timeout_s', dp.nav_timeout_s)
        decl('nav_server_wait_s', 5.0)
        # vision
        decl('aruco_dictionary', 'DICT_4X4_50')
        decl('marker_size', 0.04)
        decl('marker_id', -1)
        decl('rear_camera_T_base', [-0.201, 0.0, 0.25, -1.5707963, 0.0, 1.5707963])
        decl('max_detection_rate_hz', 10.0)
        decl('search_timeout_s', dp.search_timeout_s)
        decl('max_lateral_error', dp.max_lateral_error)
        decl('max_yaw_error_deg', dp.max_yaw_error_deg)
        decl('marker_timeout_s', dp.marker_timeout_s)
        decl('max_lost_frames', dp.max_lost_frames)
        decl('docked_marker_offset', dp.docked_marker_offset)
        # docking motion
        decl('max_speed', g.max_speed)
        decl('final_speed', g.final_speed)
        decl('slow_distance', g.slow_distance)
        decl('slow_zone', g.slow_zone)
        decl('final_zone', g.final_zone)
        decl('k_lateral', g.k_lateral)
        decl('max_approach_angle', g.max_approach_angle)
        decl('kp_heading', g.kp_heading)
        decl('kd_heading', g.kd_heading)
        decl('max_angular', g.max_angular)
        decl('heading_hold_kp', dp.heading_hold_kp)
        decl('docking_timeout_s', dp.docking_timeout_s)
        decl('final_timeout_s', dp.final_timeout_s)
        decl('final_max_distance', dp.final_max_distance)
        decl('blind_extra_distance', dp.blind_extra_distance)
        decl('blind_timeout_margin_s', dp.blind_timeout_margin_s)
        decl('max_retries', dp.max_retries)
        decl('retry_forward_distance', dp.retry_forward_distance)
        decl('retry_speed', dp.retry_speed)
        decl('retry_timeout_s', dp.retry_timeout_s)
        decl('charging_timeout_s', dp.charging_timeout_s)
        # safety (incident 2026-10-06: blind reverse into a log)
        decl('allow_blind_docking', dp.allow_blind_docking)
        # bumper during docking: stop, wait for it (and bumper_controller's manoeuvre) to
        # clear, resume from the current pose
        decl('bumper_routing_topic', '/mower_base/bumper_routing_status')
        decl('bumper_clear_s', dp.bumper_clear_s)
        decl('bumper_wait_max_s', dp.bumper_wait_max_s)
        decl('blind_marker_max_age_s', dp.blind_marker_max_age_s)
        decl('dock_pose_measured_override', False)   # bench: treat the dock pose as measured
        decl('stall_min_cmd', dp.stall_min_cmd)
        decl('stall_progress_ratio', dp.stall_progress_ratio)
        decl('stall_timeout_s', dp.stall_timeout_s)
        decl('use_dig_stall', dp.use_dig_stall)
        decl('stall_turn_radius_m', dp.stall_turn_radius_m)
        decl('stall_contact_grace_s', dp.stall_contact_grace_s)
        # failed dock goal, then the base reports is_docking_done within this window while
        # the server is idle -> enable charging anyway (0 = off)
        decl('post_fail_contact_s', 10.0)
        # approach shortcuts / alignment / realign (2026-10-07 pivot-on-turf loop)
        decl('map_pose_topic', '/odometry/filtered_map')   # map-frame pose ('' = off)
        decl('approach_skip_radius_m', dp.approach_skip_radius_m)
        decl('nav_fail_arrived_radius_m', dp.nav_fail_arrived_radius_m)
        decl('relaxed_nav_retry', dp.relaxed_nav_retry)
        decl('approach_facing_dock', dp.approach_facing_dock)
        decl('align_angular', dp.align_angular)
        decl('align_creep', dp.align_creep)
        decl('align_max_travel', dp.align_max_travel)
        decl('align_timeout_s', dp.align_timeout_s)
        decl('align_tolerance_deg', dp.align_tolerance_deg)
        decl('docking_gate_scale', dp.docking_gate_scale)
        decl('final_max_lateral', dp.final_max_lateral)
        decl('final_max_heading_deg', dp.final_max_heading_deg)
        decl('max_realigns', dp.max_realigns)
        decl('realign_forward_distance', dp.realign_forward_distance)
        decl('turn_slow_err', g.turn_slow_err)
        decl('contact_settle_s', dp.contact_settle_s)
        decl('final_extra_creep_m', dp.final_extra_creep_m)
        decl('calib_max_marker_age_s', dp.calib_max_marker_age_s)
        decl('calib_min_offset', dp.calib_min_offset)
        decl('calib_max_offset', dp.calib_max_offset)
        # measured docked_marker_offset (written after each vision dock, overrides the yaml)
        decl('calibration_file', '/userdata/ros2/calibration/docking_calibration.yaml')
        decl('auto_calibrate_marker_offset', True)
        decl('min_turn_speed', g.min_turn_speed)
        decl('close_speed', g.close_speed)
        decl('lateral_slow', g.lateral_slow)
        decl('marker_median_n', dp.marker_median_n)
        # undock
        decl('undock_direction', 1.0)
        decl('undock_max_speed', 0.3)

        self._lock = threading.Lock()
        self._busy = False
        self._sensor_group = ReentrantCallbackGroup()
        self._image_group = MutuallyExclusiveCallbackGroup()
        self._action_group = ReentrantCallbackGroup()
        self._client_group = ReentrantCallbackGroup()

        # latest sensor state (monotonic clock)
        self._odom: Optional[dl.Pose2D] = None
        self._odom_t = -1e9
        self._status_t = -1e9
        self._status_count = 0
        self._stop = False
        self._lift = False
        self._bumper = False
        self._bumper_routing = False
        self._bumper_routing_t = -1e9
        self._contact = dl.ContactDebouncer(self.get_parameter('contact_debounce_samples').value)
        self._dock_pose_msg: Optional[dl.Pose2D] = None
        self._dock_measured = False
        self._dig_stall = False
        self._progress: Optional[dl.Pose2D] = None
        self._progress_t = -1e9
        self._map_pose: Optional[dl.Pose2D] = None
        self._raw_contact = False
        self._is_charging = False
        self._map_pose_t = -1e9
        self._rtk_fixed = False
        self._marker: Optional[dl.MarkerObs] = None
        self._cam_K = None
        self._cam_D = None
        self._last_detect_t = 0.0
        self._detector = None
        self._image_sub = None
        self._info_sub = None
        # async results
        self._nav_status: Optional[str] = None
        self._nav_goal_handle = None
        self._charging_result: Optional[bool] = None

        p = self.get_parameter
        self._cmd_pub = self.create_publisher(Twist, p('cmd_vel_topic').value, 10)
        self._marker_pub = self.create_publisher(PoseStamped, '~/marker_pose', 10)
        self._approach_pub = self.create_publisher(
            PoseStamped, '~/approach_pose',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST))
        # Owner rule: dock marker in view -> the mission ignores other camera (obstacle)
        # detections while docking. Latched; false outside an active dock goal.
        self._marker_in_view_pub = self.create_publisher(
            Bool, '/mower_docking/marker_in_view',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST))
        self._marker_in_view: Optional[bool] = None
        self._publish_marker_in_view(False)
        # Inputs bypass the 6-thread executor (see sub_pump.py); the handlers only store under
        # self._lock. Odometry and /mower_base/status are only read by a running goal, so their
        # subscriptions exist only while one runs (_resume_inputs / _release_inputs): even
        # untaken, DDS delivery of 100 Hz status + 50 Hz odometry costs ~5 % of a core, and
        # through the executor they cost ~45 % while idle.
        self._pump = SubscriptionPump(self, 'docking_inputs')
        sub = self._pump.subscribe
        self._goal_pump = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)
        sub(PoseStamped, p('dock_pose_topic').value, self._on_dock_pose, latched)
        sub(Bool, p('dock_pose_measured_topic').value, self._on_dock_measured, latched)
        sub(Bool, p('dig_stall_topic').value, self._on_dig_stall, latched)
        if GnssStatus is not None:
            sub(GnssStatus, p('gps_status_topic').value, self._on_gps, 10)
        else:
            self.get_logger().warn('mowgli_interfaces not importable: wait_for_rtk will time out')

        self._charging_cli = self.create_client(ChargingControl, p('charging_service').value,
                                                callback_group=self._client_group)
        # NavigateToPose client only while a goal needs it (_nav_client / _drop_nav_client):
        # its feedback subscription receives bt_navigator's ~100 Hz feedback of EVERY
        # navigate_to_pose goal (the mission's transits too), which kept this idle node busy.
        self._nav_cli = None

        self._dock_srv = ActionServer(
            self, Dock, '~/dock', execute_callback=self._execute_dock,
            goal_callback=self._goal_cb, cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=self._action_group)
        self._undock_srv = ActionServer(
            self, Undock, '~/undock', execute_callback=self._execute_undock,
            goal_callback=self._goal_cb, cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=self._action_group)
        self._load_calibration()
        self._pump.start()
        self.get_logger().info('docking server ready (~/dock, ~/undock)')

    def destroy_node(self):
        self._release_inputs()
        self._pump.stop()
        return super().destroy_node()

    # ------------------------------------------------------------------ subs
    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        pose = dl.Pose2D(msg.pose.pose.position.x, msg.pose.pose.position.y,
                         dl.yaw_from_quaternion(q.x, q.y, q.z, q.w))
        with self._lock:
            self._odom = pose
            self._odom_t = time.monotonic()

    def _on_status(self, msg: MowerBaseDevStatus) -> None:
        with self._lock:
            self._contact.update(bool(msg.is_docking_done))
            self._raw_contact = bool(msg.is_docking_done)
            self._is_charging = bool(msg.is_charging)
            self._status_count += 1
            self._stop = bool(msg.stop_triggered)
            self._lift = bool(msg.lift_triggered)
            self._bumper = bool(getattr(msg, 'bumper_triggered', False) or
                                getattr(msg, 'left_bumper_triggered', False) or
                                getattr(msg, 'right_bumper_triggered', False))
            self._status_t = time.monotonic()

    def _on_bumper_routing(self, msg) -> None:
        with self._lock:
            self._bumper_routing = int(msg.data) != 0
            self._bumper_routing_t = time.monotonic()

    def _on_dock_pose(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        pose = dl.Pose2D(msg.pose.position.x, msg.pose.position.y,
                         dl.yaw_from_quaternion(q.x, q.y, q.z, q.w))
        with self._lock:
            self._dock_pose_msg = pose
        self.get_logger().info('dock pose from %s: (%.3f, %.3f, %.1f deg)' % (
            msg.header.frame_id or '?', pose.x, pose.y, math.degrees(pose.yaw)))

    def _on_dock_measured(self, msg: Bool) -> None:
        with self._lock:
            changed = self._dock_measured != bool(msg.data)
            self._dock_measured = bool(msg.data)
        if changed:
            self.get_logger().info('dock pose measured: %s' % bool(msg.data))

    def _on_dig_stall(self, msg: Bool) -> None:
        with self._lock:
            self._dig_stall = bool(msg.data)

    def _on_progress_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        pose = dl.Pose2D(msg.pose.pose.position.x, msg.pose.pose.position.y,
                         dl.yaw_from_quaternion(q.x, q.y, q.z, q.w))
        with self._lock:
            self._progress = pose
            self._progress_t = time.monotonic()

    def _on_map_pose(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        pose = dl.Pose2D(msg.pose.pose.position.x, msg.pose.pose.position.y,
                         dl.yaw_from_quaternion(q.x, q.y, q.z, q.w))
        with self._lock:
            self._map_pose = pose
            self._map_pose_t = time.monotonic()

    def _on_gps(self, msg) -> None:
        with self._lock:
            self._rtk_fixed = int(msg.fix_type) == 3

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if len(msg.k) >= 9 and msg.k[0] > 0:
            with self._lock:
                self._cam_K = list(msg.k)
                self._cam_D = list(msg.d) if len(msg.d) else [0.0] * 5

    def _on_image(self, msg: Image) -> None:
        now = time.monotonic()
        rate = float(self.get_parameter('max_detection_rate_hz').value)
        if rate > 0 and now - self._last_detect_t < 1.0 / rate:
            return
        self._last_detect_t = now
        with self._lock:
            K, D, det = self._cam_K, self._cam_D, self._detector
        if K is None or det is None:
            return
        from mower_docking import aruco_detect
        try:
            gray = aruco_detect.to_gray(msg.encoding, msg.data, msg.height, msg.width, msg.step)
            if gray is None:
                self.get_logger().warn('unsupported image encoding %s' % msg.encoding,
                                       throttle_duration_sec=5.0)
                return
            res = det.detect(gray, K, D, int(self.get_parameter('marker_id').value))
        except Exception as exc:  # keep the server alive on bad frames
            self.get_logger().warn('aruco detection failed: %s' % exc, throttle_duration_sec=5.0)
            return
        if not res:
            return
        mid, rvec, tvec, _ = res[0]
        ext = list(self.get_parameter('rear_camera_T_base').value)
        obs = dl.marker_in_base(rvec, tvec, ext[0:3], ext[3:6], stamp=now, marker_id=mid)
        with self._lock:
            self._marker = obs
        ps = PoseStamped()
        ps.header.stamp = msg.header.stamp
        ps.header.frame_id = self.get_parameter('base_frame').value
        ps.pose.position.x, ps.pose.position.y = obs.x, obs.y
        qx, qy, qz, qw = dl.quaternion_from_yaw(obs.yaw)
        ps.pose.orientation.x, ps.pose.orientation.y = qx, qy
        ps.pose.orientation.z, ps.pose.orientation.w = qz, qw
        self._marker_pub.publish(ps)

    def _enable_vision(self, on: bool) -> bool:
        p = self.get_parameter
        if on and self._image_sub is None:
            try:
                from mower_docking import aruco_detect
                det = aruco_detect.ArucoDetector(p('aruco_dictionary').value,
                                                 float(p('marker_size').value))
            except Exception as exc:
                self.get_logger().error('vision unavailable (%s); dead-reckoning instead' % exc)
                return False
            with self._lock:
                self._detector = det
                self._marker = None
            self._info_sub = self.create_subscription(
                CameraInfo, p('camera_info_topic').value, self._on_camera_info,
                qos_profile_sensor_data, callback_group=self._sensor_group)
            self._image_sub = self.create_subscription(
                Image, p('image_topic').value, self._on_image, qos_profile_sensor_data,
                callback_group=self._image_group)
        elif not on and self._image_sub is not None:
            self.destroy_subscription(self._image_sub)
            self.destroy_subscription(self._info_sub)
            self._image_sub = self._info_sub = None
        return True

    # --------------------------------------------------------------- helpers
    def _goal_cb(self, _req) -> GoalResponse:
        with self._lock:
            if self._busy:
                self.get_logger().warn('rejecting goal: a dock/undock goal is already running')
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _snapshot(self) -> dl.Snapshot:
        now = time.monotonic()
        p = self.get_parameter
        with self._lock:
            fresh_progress = now - self._progress_t <= p('odom_timeout_s').value
            return dl.Snapshot(
                progress_pose=self._progress if fresh_progress else None,
                map_pose=self._map_pose if now - self._map_pose_t <= 1.0 else None,
                raw_contact=self._raw_contact and now - self._status_t <= 1.0,
                is_charging=self._is_charging and now - self._status_t <= 1.0,
                dig_stall=self._dig_stall,
                t=now,
                odom=self._odom if now - self._odom_t <= p('odom_timeout_s').value else None,
                marker=self._marker,
                contact=self._contact.value,
                status_fresh=now - self._status_t <= p('status_timeout_s').value,
                stop_triggered=self._stop, lift_triggered=self._lift,
                bumper=(self._bumper and now - self._status_t <= 1.0) or
                (self._bumper_routing and now - self._bumper_routing_t <= 1.0),
                nav_status=self._nav_status, charging_result=self._charging_result,
                rtk_fixed=self._rtk_fixed)

    def _publish_cmd(self, v: float, w: float) -> None:
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self._cmd_pub.publish(t)

    def _resume_inputs(self, timeout: float = 2.0) -> None:
        """Subscribe the goal-only inputs and wait for the state continuous updates give.

        Resets the contact debouncer and waits until it has seen ``n`` fresh status samples
        (its value then depends only on those, exactly as with continuous updates) and one
        fresh odometry sample; the timeout also covers DDS discovery of the new readers. On
        timeout the snapshot simply reports stale inputs, as before when they were missing.
        """
        p = self.get_parameter
        t0 = time.monotonic()
        with self._lock:
            self._contact.reset()
            self._status_count = 0
        pump = SubscriptionPump(self, 'docking_goal_inputs')
        pump.subscribe(Odometry, p('odom_topic').value, self._on_odom, 20,
                       parser=parse_odometry)
        pump.subscribe(MowerBaseDevStatus, p('status_topic').value, self._on_status, 20,
                       parser=flat_parser(MowerBaseDevStatus))
        if p('progress_odom_topic').value:
            pump.subscribe(Odometry, p('progress_odom_topic').value, self._on_progress_odom, 20,
                           parser=parse_odometry)
        if p('map_pose_topic').value:
            pump.subscribe(Odometry, p('map_pose_topic').value, self._on_map_pose, 5,
                           parser=parse_odometry)
        if p('bumper_routing_topic').value:
            pump.subscribe(UInt8, p('bumper_routing_topic').value, self._on_bumper_routing, 10,
                           parser=flat_parser(UInt8))
        pump.start()
        self._goal_pump = pump
        while time.monotonic() - t0 < timeout:
            with self._lock:
                map_ok = not p('map_pose_topic').value or self._map_pose_t >= t0
                if self._status_count >= self._contact.n and self._odom_t >= t0 and map_ok:
                    return
            time.sleep(0.005)
        self.get_logger().warn('fresh odom/status not received within %.1f s of goal start'
                               % timeout)

    def _release_inputs(self) -> None:
        pump, self._goal_pump = self._goal_pump, None
        if pump is not None:
            pump.close()

    def _stop_burst(self) -> None:
        rate = float(self.get_parameter('control_rate_hz').value)
        end = time.monotonic() + float(self.get_parameter('stop_burst_s').value)
        while time.monotonic() < end:
            self._publish_cmd(0.0, 0.0)
            time.sleep(1.0 / rate)
        self._publish_cmd(0.0, 0.0)

    def _call_charging(self, enable: bool) -> None:
        self._charging_result = None
        if not self._charging_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('%s not available' % self._charging_cli.srv_name)
            self._charging_result = False
            return
        req = ChargingControl.Request()
        req.enable_charging = bool(enable)
        fut = self._charging_cli.call_async(req)

        def done(f):
            ok = f.exception() is None
            self.get_logger().info('charging %s -> %s' % (
                'on' if enable else 'off', 'ok' if ok else f.exception()))
            self._charging_result = ok
        fut.add_done_callback(done)

    def _start_nav(self, pose: dl.Pose2D) -> None:
        self._nav_status = 'running'
        ps = PoseStamped()
        ps.header.frame_id = self.get_parameter('map_frame').value
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = pose.x, pose.y
        qx, qy, qz, qw = dl.quaternion_from_yaw(pose.yaw)
        ps.pose.orientation.x, ps.pose.orientation.y = qx, qy
        ps.pose.orientation.z, ps.pose.orientation.w = qz, qw
        self._approach_pub.publish(ps)
        if not self._nav_client().wait_for_server(
                timeout_sec=float(self.get_parameter('nav_server_wait_s').value)):
            self.get_logger().error('navigate_to_pose server not available')
            self._nav_status = 'failed'
            return
        goal = NavigateToPose.Goal()
        goal.pose = ps
        self.get_logger().info('navigating to approach pose (%.2f, %.2f, %.1f deg)' % (
            pose.x, pose.y, math.degrees(pose.yaw)))
        fut = self._nav_cli.send_goal_async(goal)

        def on_goal(f):
            gh = f.result()
            if gh is None or not gh.accepted:
                self._nav_status = 'failed'
                return
            self._nav_goal_handle = gh
            gh.get_result_async().add_done_callback(on_result)

        def on_result(f):
            res = f.result()
            ok = res is not None and res.status == GoalStatus.STATUS_SUCCEEDED
            self._nav_status = 'succeeded' if ok else 'failed'
            self._nav_goal_handle = None
        fut.add_done_callback(on_goal)

    def _nav_client(self):
        if self._nav_cli is None:
            self._nav_cli = QuietActionClient(
                self, NavigateToPose, self.get_parameter('navigate_action').value,
                callback_group=self._client_group)
        return self._nav_cli

    def _drop_nav_client(self) -> None:
        cli, self._nav_cli = self._nav_cli, None
        self._nav_goal_handle = None
        if cli is not None:
            try:
                cli.destroy()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn('nav client destroy failed: %s' % exc)

    def _cancel_nav(self) -> None:
        gh = self._nav_goal_handle
        if gh is not None:
            gh.cancel_goal_async()
        self._nav_goal_handle = None

    def _handle(self, requests) -> None:
        for r in requests:
            if r[0] == 'start_nav':
                self._start_nav(r[1])
            elif r[0] == 'cancel_nav':
                self._cancel_nav()
            elif r[0] == 'enable_charging':
                self._call_charging(True)
            elif r[0] == 'disable_charging':
                self._call_charging(False)

    def _dock_params(self) -> dl.DockParams:
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        g = dl.ControllerGains(
            max_speed=min(float(p('max_speed')), 0.15), final_speed=float(p('final_speed')),
            slow_distance=float(p('slow_distance')),
            slow_zone=float(p('slow_zone')), final_zone=float(p('final_zone')),
            k_lateral=float(p('k_lateral')), max_approach_angle=float(p('max_approach_angle')),
            kp_heading=float(p('kp_heading')), kd_heading=float(p('kd_heading')),
            max_angular=float(p('max_angular')),
            turn_slow_err=float(p('turn_slow_err')), min_turn_speed=float(p('min_turn_speed')),
            close_speed=min(float(p('close_speed')), 0.15),
            lateral_slow=float(p('lateral_slow')))
        return dl.DockParams(
            approach_distance=float(p('approach_distance')),
            skip_nav_to_approach=bool(p('skip_nav_to_approach')),
            nav_timeout_s=float(p('nav_timeout_s')),
            search_timeout_s=float(p('search_timeout_s')),
            max_lateral_error=float(p('max_lateral_error')),
            max_yaw_error_deg=float(p('max_yaw_error_deg')),
            marker_timeout_s=float(p('marker_timeout_s')),
            max_lost_frames=int(p('max_lost_frames')),
            docked_marker_offset=float(p('docked_marker_offset')),
            docking_timeout_s=float(p('docking_timeout_s')),
            final_timeout_s=float(p('final_timeout_s')),
            final_max_distance=float(p('final_max_distance')),
            blind_extra_distance=float(p('blind_extra_distance')),
            blind_timeout_margin_s=float(p('blind_timeout_margin_s')),
            max_retries=int(p('max_retries')),
            retry_forward_distance=float(p('retry_forward_distance')),
            retry_speed=float(p('retry_speed')),
            retry_timeout_s=float(p('retry_timeout_s')),
            heading_hold_kp=float(p('heading_hold_kp')),
            charging_timeout_s=float(p('charging_timeout_s')),
            allow_blind_docking=bool(p('allow_blind_docking')),
            bumper_clear_s=float(p('bumper_clear_s')),
            bumper_wait_max_s=float(p('bumper_wait_max_s')),
            blind_marker_max_age_s=float(p('blind_marker_max_age_s')),
            stall_min_cmd=float(p('stall_min_cmd')),
            stall_progress_ratio=float(p('stall_progress_ratio')),
            stall_timeout_s=float(p('stall_timeout_s')),
            use_dig_stall=bool(p('use_dig_stall')),
            stall_turn_radius_m=float(p('stall_turn_radius_m')),
            stall_contact_grace_s=float(p('stall_contact_grace_s')),
            approach_skip_radius_m=float(p('approach_skip_radius_m')),
            nav_fail_arrived_radius_m=float(p('nav_fail_arrived_radius_m')),
            relaxed_nav_retry=bool(p('relaxed_nav_retry')),
            approach_facing_dock=bool(p('approach_facing_dock')),
            align_angular=float(p('align_angular')),
            align_creep=float(p('align_creep')),
            align_max_travel=float(p('align_max_travel')),
            align_timeout_s=float(p('align_timeout_s')),
            align_tolerance_deg=float(p('align_tolerance_deg')),
            docking_gate_scale=float(p('docking_gate_scale')),
            final_max_lateral=float(p('final_max_lateral')),
            final_max_heading_deg=float(p('final_max_heading_deg')),
            max_realigns=int(p('max_realigns')),
            realign_forward_distance=float(p('realign_forward_distance')),
            contact_settle_s=float(p('contact_settle_s')),
            final_extra_creep_m=float(p('final_extra_creep_m')),
            marker_median_n=int(p('marker_median_n')),
            calib_max_marker_age_s=float(p('calib_max_marker_age_s')),
            calib_min_offset=float(p('calib_min_offset')),
            calib_max_offset=float(p('calib_max_offset')),
            gains=g)

    def _dock_pose(self) -> dl.Pose2D:
        with self._lock:
            if self._dock_pose_msg is not None:
                return self._dock_pose_msg
        v = list(self.get_parameter('dock_pose').value) + [0.0, 0.0, 0.0]
        self.get_logger().warn('no %s received; using param dock_pose %s' % (
            self.get_parameter('dock_pose_topic').value, v[:3]))
        return dl.Pose2D(float(v[0]), float(v[1]), float(v[2]))

    def _load_calibration(self) -> None:
        path = self.get_parameter('calibration_file').value
        if not path:
            return
        try:
            import yaml
            with open(path) as f:
                data = yaml.safe_load(f) or {}
            off = float(data['docked_marker_offset'])
        except FileNotFoundError:
            return
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('calibration %s unreadable: %s' % (path, exc))
            return
        from rclpy.parameter import Parameter
        self.set_parameters([Parameter('docked_marker_offset', value=off)])
        self.get_logger().info('docked_marker_offset %.3f m from %s' % (off, path))

    def _save_calibration(self, measured: float) -> None:
        if not bool(self.get_parameter('auto_calibrate_marker_offset').value):
            return
        old = float(self.get_parameter('docked_marker_offset').value)
        path = self.get_parameter('calibration_file').value
        import os
        if path and not os.path.exists(path):
            new = measured                    # first measurement replaces the yaml placeholder
        else:
            new = 0.5 * (old + measured)      # later docks blend: one noisy dock does not dominate
        from rclpy.parameter import Parameter
        self.set_parameters([Parameter('docked_marker_offset', value=new)])
        self.get_logger().info('docked_marker_offset %.3f -> %.3f m (measured %.3f)'
                               % (old, new, measured))
        if not path:
            return
        try:
            import os
            import yaml
            os.makedirs(os.path.dirname(path), exist_ok=True)
            tmp = path + '.tmp'
            with open(tmp, 'w') as f:
                yaml.safe_dump({'docked_marker_offset': round(new, 4),
                                'last_measured': round(measured, 4),
                                'updated': time.strftime('%Y-%m-%dT%H:%M:%S')}, f)
            os.replace(tmp, path)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('could not write %s: %s' % (path, exc))

    def _log_notes(self, out) -> None:
        for n in getattr(out, 'notes', None) or ():
            self.get_logger().info(n)

    # ------------------------------------------------------------- execution
    def _publish_marker_in_view(self, value: bool) -> None:
        value = bool(value)
        if value == self._marker_in_view:
            return
        self._marker_in_view = value
        self._marker_in_view_pub.publish(Bool(data=value))

    def _run(self, goal_handle, machine, make_feedback, result_type, publish_in):
        rate = float(self.get_parameter('control_rate_hz').value)
        period = 1.0 / rate
        self._nav_status = None
        self._charging_result = None
        self._resume_inputs()
        out = machine.start(self._snapshot())
        self._log_notes(out)
        self._handle(out.requests)
        last_state = None
        try:
            while rclpy.ok():
                tick = time.monotonic()
                if goal_handle.is_cancel_requested:
                    out = machine.cancel()
                    self._handle(out.requests)
                    self._publish_cmd(0.0, 0.0)
                    goal_handle.canceled()
                    self.get_logger().info('goal canceled in state %s' % last_state)
                    return result_type(success=False, message=out.message)
                out = machine.step(self._snapshot())
                self._publish_marker_in_view(out.marker_in_view)
                self._log_notes(out)
                self._handle(out.requests)
                if publish_in(out.state) or out.done:
                    self._publish_cmd(out.linear, out.angular)
                if out.state != last_state:
                    self.get_logger().info('state -> %s %s' % (out.state, out.message))
                    last_state = out.state
                if out.done:
                    break
                goal_handle.publish_feedback(make_feedback(out))
                time.sleep(max(0.0, period - (time.monotonic() - tick)))
            if out.success and getattr(out, 'calibrated_offset', None) is not None:
                self._save_calibration(float(out.calibrated_offset))
            if out.success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
                if result_type is Dock.Result:
                    self._start_post_fail_watch()
            self.get_logger().info('result: success=%s message=%s' % (out.success, out.message))
            return result_type(success=out.success, message=out.message)
        finally:
            if machine.state == dl.DockState.NAV_TO_APPROACH:
                self._cancel_nav()
            self._publish_marker_in_view(False)
            self._stop_burst()          # also gives the cancel request time to go out
            self._drop_nav_client()
            self._enable_vision(False)
            self._release_inputs()
            with self._lock:
                self._busy = False
                self._progress = None
                self._progress_t = -1e9
                self._map_pose = None
                self._map_pose_t = -1e9

    def _start_post_fail_watch(self) -> None:
        window = float(self.get_parameter('post_fail_contact_s').value)
        if window <= 0:
            return
        threading.Thread(target=self._post_fail_watch, args=(window,), daemon=True,
                         name='post_fail_contact').start()

    def _post_fail_watch(self, window: float) -> None:
        """A failed dock goal whose robot nevertheless ends on the contacts (2026-10-07:
        DOCK_STALLED, is_docking_done 8 s later): enable charging if the base reports
        is_docking_done within ``window`` s while no other goal runs."""
        time.sleep(0.5)    # let _run's finally release the goal inputs
        state = {'contact': False, 'charging': False}
        lock = threading.Lock()

        def on_status(msg):
            with lock:
                state['contact'] = state['contact'] or bool(msg.is_docking_done)
                state['charging'] = bool(msg.is_charging)
        pump = SubscriptionPump(self, 'docking_post_fail')
        pump.subscribe(MowerBaseDevStatus, self.get_parameter('status_topic').value, on_status,
                       20, parser=flat_parser(MowerBaseDevStatus))
        pump.start()
        try:
            watch = dl.PostFailWatch(time.monotonic(), window)
            while rclpy.ok():
                with self._lock:
                    busy = self._busy
                with lock:
                    contact, charging = state['contact'], state['charging']
                act = watch.step(time.monotonic(), contact, charging, busy)
                if act == 'enable':
                    self.get_logger().warn('is_docking_done after the failed dock goal: '
                                           'enabling charging')
                    self._call_charging(True)
                if act == 'charging':
                    self.get_logger().info('on the dock after the failed goal: already charging')
                if act is not None:
                    return
                time.sleep(0.1)
        finally:
            pump.close()

    def _execute_dock(self, goal_handle):
        req = goal_handle.request
        use_vision = bool(req.use_vision)
        if use_vision and not self._enable_vision(True):
            use_vision = False
        timeout = float(req.timeout_s) if req.timeout_s > 0 else 180.0
        with self._lock:
            measured = self._dock_measured
        measured = measured or bool(self.get_parameter('dock_pose_measured_override').value)
        machine = dl.DockStateMachine(self._dock_params(), self._dock_pose(), timeout, use_vision,
                                      dock_pose_measured=measured)
        self.get_logger().info('dock goal: use_vision=%s timeout=%.0fs approach=(%.2f, %.2f) '
                               'dock_pose_measured=%s allow_blind=%s' % (
                                   use_vision, timeout, machine.approach.x, machine.approach.y,
                                   measured, machine.p.allow_blind_docking))

        def fb(out):
            f = Dock.Feedback()
            # "STATE: detail" -- the mission relays the detail into its sub_state
            f.state = '%s: %s' % (out.state, out.detail) if out.detail else out.state
            f.distance_m = float(out.remaining) if math.isfinite(out.remaining) else -1.0
            f.retries = int(out.retries)
            return f
        return self._run(goal_handle, machine, fb, Dock.Result,
                         lambda st: st not in MOTION_IDLE_STATES)

    def _execute_undock(self, goal_handle):
        req = goal_handle.request
        up = dl.UndockParams(direction=1.0 if self.get_parameter('undock_direction').value >= 0
                             else -1.0,
                             heading_hold_kp=float(self.get_parameter('heading_hold_kp').value),
                             max_angular=float(self.get_parameter('max_angular').value),
                             max_speed=float(self.get_parameter('undock_max_speed').value),
                             charging_timeout_s=float(
                                 self.get_parameter('charging_timeout_s').value))
        dist = float(req.distance_m) if req.distance_m > 0 else 0.8
        speed = float(req.speed_mps) if req.speed_mps > 0 else 0.15
        machine = dl.UndockStateMachine(up, dist, speed, bool(req.wait_for_rtk),
                                        float(req.rtk_timeout_s))
        self.get_logger().info('undock goal: %.2f m at %.2f m/s, wait_for_rtk=%s' % (
            dist, machine.speed, req.wait_for_rtk))

        def fb(out):
            f = Undock.Feedback()
            f.state = out.state
            f.travelled_m = float(out.travelled)
            return f
        return self._run(goal_handle, machine, fb, Undock.Result, lambda st: True)


def main(args=None):
    try:  # crash records -> /userdata/ros2/crashes (docs/crash_recovery.md)
        from mower_control.crash_record import install as _install_crash_record
        _install_crash_record('mower_docking')
    except ImportError:
        pass
    rclpy.init(args=args)
    node = DockingServer()
    ex = MultiThreadedExecutor(num_threads=6)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._publish_cmd(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
