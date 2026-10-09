#!/usr/bin/env python3
"""MowgliNext GUI bridge for the Airseekers Tron ROS 2 stack.

Presents our hardware drivers (mower_mcu_driver, um960_gps_driver,
robot_localization) under the topic/service names and ``mowgli_interfaces``
types that the MowgliNext GUI backend expects, and serves a *stub* high-level
state machine (IDLE / RECORDING / MANUAL_MOWING / EMERGENCY) that is enough for
joystick driving and blade-on manual mowing.  See README.md.

All decision logic lives in :mod:`mower_gui_bridge.state_machine` (ROS-free,
unit-tested); this module is wiring only.
"""

import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.serialization import deserialize_message

from geometry_msgs.msg import Twist
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, CameraInfo, Imu, NavSatFix
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Empty, SetBool, Trigger

from mower_interfaces.msg import MowerBaseDevStatus, MowerSensorInfo
from mower_interfaces.srv import CutterControl
from mowgli_interfaces.msg import Emergency, GnssStatus, HighLevelStatus, Power, Status
from mowgli_interfaces.srv import EmergencyStop, HighLevelControl, MowerControl, StartInArea

from mower_gui_bridge import datum as dt
from mower_gui_bridge import gui_relay
from mower_gui_bridge import diagnostics as diag
from mower_gui_bridge import state_machine as sm
from mower_gui_bridge.sub_pump import (PeriodicRunner, SubscriptionPump, _header,
                                        flat_parser, odometry_frames, odometry_with_frames)

PARAM_DEFAULTS = {
    # inputs from our stack
    'mower_status_topic': '/mower_base/status',
    'sensor_info_topic': '/mower_sensor_info',
    'battery_topic': '/battery',
    'estop_topic': '/estop',
    'estop_request_topic': '/estop_request',
    'fix_topic': '/fix',
    'fix_status_topic': '/fix_status',
    'odom_topic': '/odom',
    'filtered_odom_topic': '/odometry/filtered',
    'cutter_control_service': '/cutter_control',
    'cutter_off_service': '/cutter_off',
    'clear_estop_service': '/clear_estop',
    # real mission node: GUI mow_enabled during MANUAL_MOWING goes here
    'manual_blade_service': '/behavior_tree_node/manual_blade',
    'emergency_twist_topic': '/cmd_vel_emergency',
    # outputs for the GUI
    'status_topic': '/hardware_bridge/status',
    'emergency_topic': '/hardware_bridge/emergency',
    'power_topic': '/hardware_bridge/power',
    'gps_fix_topic': '/gps/fix',
    # /gps/fix (this relay) is GUI-only (foxglove_bridge is its sole subscriber); the GUI
    # map marker needs ~2 Hz, the um960 source runs at 10 Hz. 0 = unthrottled.
    'gps_fix_rate_hz': 2.0,
    'gnss_status_topic': '/gps/status',
    'wheel_odom_topic': '/wheel_odom',
    # /wheel_odom is GUI-only (foxglove_bridge is its sole subscriber) and /odom runs at
    # ~50 Hz. Every relayed message costs foxglove_bridge a few ms, so the relay is
    # rate-limited. GUI consumers (2026-10-09): session odometer (sums position deltas, jump
    # guard 1 m: 2 Hz at 0.5 m/s = 0.25 m steps), updater readiness (3 s freshness), MQTT
    # retained copy, Diagnostics page numbers. 2 Hz is enough. 0 = unthrottled.
    'wheel_odom_rate_hz': 2.0,
    'filtered_map_topic': '/odometry/filtered_map',
    # GUI-only low-rate copies (gui_relay.py): foxglove_bridge costs a few ms per delivered
    # message per client and cannot rate-limit, while the originals keep the rates the
    # motion path needs. The GUI provider subscribes to these instead. '' = off.
    'gui_pose_topic': '/gui/pose',
    'gui_pose_rate_hz': 5.0,
    'gui_status_topic': '/gui/status',
    'gui_status_rate_hz': 2.0,
    'gui_emergency_topic': '/gui/emergency',
    'gui_emergency_rate_hz': 1.0,          # plus immediately on any change
    # /gps/status keeps 10 Hz for heading_aligner/map_server/docking/mission; the GUI reads
    # this copy (fix-type changes go out at once).
    'gui_gnss_status_topic': '/gui/gnss_status',
    'gui_gnss_status_rate_hz': 2.0,
    'detections_topic': '/ai/det/detections',
    'gui_detections_topic': '/gui/detections',
    'gui_detections_rate_hz': 2.0,         # per camera (frame_id); relayed only while subscribed
    # obstacle_guard's Bool (one per detection frame, ~13 Hz): the GUI badge gets changes
    # (sampled at 5 Hz) plus a keep-alive at this rate, only while subscribed. '' = off.
    'obstacle_close_topic': '/vision/obstacle_close',
    'gui_obstacle_close_topic': '/gui/obstacle_close',
    'gui_obstacle_close_rate_hz': 1.0,
    # /diagnostics (~9 Hz from 6 publishers, this node's included) merged per status name into
    # one DiagnosticArray at this rate, only while subscribed. '' = off.
    'diagnostics_source_topic': '/diagnostics',
    'gui_diagnostics_topic': '/gui/diagnostics',
    'gui_diagnostics_rate_hz': 1.0,
    'high_level_status_topic': '/behavior_tree_node/high_level_status',
    'coverage_resume_topic': '/behavior_tree_node/coverage_resume_available',
    'mower_control_service': '/hardware_bridge/mower_control',
    'emergency_stop_service': '/hardware_bridge/emergency_stop',
    'reboot_board_service': '/hardware_bridge/reboot_board',
    'high_level_control_service': '/behavior_tree_node/high_level_control',
    'start_in_area_service': '/behavior_tree_node/start_in_area',
    'clear_coverage_resume_service': '/behavior_tree_node/clear_coverage_resume',
    'set_datum_service': '/navsat_to_absolute_pose/set_datum',
    # set_datum: robot_localization navsat_transform_node's SetDatum service ('' = do not call)
    'rl_set_datum_service': '/datum',
    # set_datum persistence: GUI settings file and the launch defaults file ('' = skip)
    'robot_yaml_path': '',
    'datum_env_path': '/userdata/ros2/datum.env',
    # datum the stack was launched with (mower.launch.py datum_lat/datum_lon; 0/0 = unset)
    'datum_lat': 0.0,
    'datum_lon': 0.0,
    'datum_fix_max_age_s': 2.0,
    # on start, rewrite datum_lat/lon in robot_yaml_path when it disagrees with the launch datum
    'sync_robot_yaml_datum': True,
    # frames
    'map_frame': 'map',
    'base_frame': 'base_footprint',
    # rates / timeouts
    'status_rate_hz': 5.0,
    'emergency_rate_hz': 5.0,      # clamped to >= 2 Hz (BT treats >2 s silence as emergency)
    'power_rate_hz': 2.0,
    'high_level_rate_hz': 1.0,
    'mcu_timeout_s': 1.0,          # /mower_base/status older than this -> MCU not alive
    'emergency_on_mcu_timeout': True,
    'fix_status_timeout_s': 3.0,
    'service_timeout_s': 1.5,
    # stub mission layer; set false once a real behavior_tree_node exists
    'serve_high_level': True,
    # hardware diagnostics (diagnostics.py) as diagnostic_msgs/DiagnosticArray ('' = off)
    'diagnostics_topic': '/diagnostics',
    'diagnostics_rate_hz': 1.0,
    'diagnostics_timeout_s': 3.0,      # an input older than this is reported stale
    'diagnostics_motor_status_alerts': True,    # WARN on negative (fault) MotorStatus codes
    'imu_topic': '/imu/data',          # '' = no IMU entry
    # '' by default: wit_imu_driver only publishes /imu/temperature_c (at 100 Hz) while
    # someone subscribes, so a 1 Hz temperature reading would cost a 100 Hz stream.
    'imu_temperature_topic': '',
    'imu_bias_status_topic': '/bias_status',
    'heading_status_topic': '/heading_aligner/status',   # '' = no Heading entry
    # 2026-10-09: mower_control tilt_monitor's slope band (ok/caution/limit/critical)
    'tilt_status_topic': '/tilt/status',   # '' = no Tilt entry
    'lights_state_topic': '/light_controller/state',   # '' = no lights entry
    # 'id|image_topic|freshness_topic|on_demand' (see diagnostics.parse_camera_spec). Only
    # the small freshness topic is subscribed, and never for on-demand cameras (a
    # subscriber would make them capture).
    'diagnostics_cameras': [
        'left|/left_oa_camera/image_raw|/left_oa_camera/camera_info|0',
        'right|/right_oa_camera/image_raw|/right_oa_camera/camera_info|0',
        'rear|/rear_camera/image_raw||1',
        'front_left|/vio/left/image_raw||1',
        'front_right|/vio/right/image_raw||1',
    ],
}


def _stamp_is_zero(t):
    return t.sec == 0 and t.nanosec == 0



class GuiBridgeNode(Node):

    def __init__(self, **kwargs):
        super().__init__('gui_bridge', **kwargs)
        for name, default in PARAM_DEFAULTS.items():
            self.declare_parameter(name, default)
        p = {name: self.get_parameter(name).value for name in PARAM_DEFAULTS}
        self._p = p
        self._serve_hl = bool(p['serve_high_level'])

        self._lock = threading.RLock()
        self._sm = sm.HighLevelStateMachine()
        self._external_hl_state = sm.STATE_IDLE  # used when serve_high_level is false

        # latest inputs
        self._base = None
        self._base_time = None          # monotonic receipt time of /mower_base/status
        self._sensor = None
        self._battery = None
        self._estop = False
        self._fix_status = None
        self._fix_status_time = None
        self._fix_status_raw = None
        self._last_fix = None           # (status, lat, lon, alt, monotonic receipt time)
        self._gnss_quality = 0.0
        self._gnss_fix_type = None
        self._cutter_requested = False
        self._blade_stamp = None
        self._emergency_active = False
        self._emergency_fields = (False, False, False, '')
        self._sensor_time = None
        self._battery_time = None
        self._filtered_time = None
        self._imu_time = None
        self._imu_temperature = None
        self._heading_status = None
        self._tilt_status = None
        self._imu_bias = None
        self._lights = None
        self._lights_time = None
        self._camera_times = {}

        self._srv_group = ReentrantCallbackGroup()
        self._cli_group = ReentrantCallbackGroup()

        # ---- publishers -------------------------------------------------
        self._status_pub = self.create_publisher(Status, p['status_topic'], 10)
        self._emergency_pub = self.create_publisher(Emergency, p['emergency_topic'], 10)
        self._power_pub = self.create_publisher(Power, p['power_topic'], 10)
        self._gps_fix_pub = self.create_publisher(NavSatFix, p['gps_fix_topic'], 10)
        self._gnss_pub = self.create_publisher(GnssStatus, p['gnss_status_topic'], 10)
        self._wheel_odom_pub = self.create_publisher(Odometry, p['wheel_odom_topic'], 10)
        rate = float(p['wheel_odom_rate_hz'])
        # 5 % slack so a 10 Hz cap on a jittery 50 Hz source still yields ~10 Hz.
        self._wheel_odom_period = 0.95 / rate if rate > 0.0 else 0.0
        self._wheel_odom_last = -1e9
        # 2026-10-09 dual EKF: ekf_map publishes /odometry/filtered_map itself and the launch
        # points filtered_odom_topic at it; republishing onto our own input would loop.
        self._relay_map_odom = str(p['filtered_odom_topic']) != str(p['filtered_map_topic'])
        self._map_odom_pub = self.create_publisher(Odometry, p['filtered_map_topic'], 10) \
            if self._relay_map_odom else None

        def gui_pub(msg_type, key):
            return self.create_publisher(msg_type, str(p[key]), 10) if str(p[key]) else None
        self._gui_pose_pub = gui_pub(Odometry, 'gui_pose_topic')
        self._gui_pose_thr = gui_relay.Throttle(gui_relay.period_for(p['gui_pose_rate_hz']))
        self._gui_status_pub = gui_pub(Status, 'gui_status_topic')
        self._gui_status_thr = gui_relay.Throttle(gui_relay.period_for(p['gui_status_rate_hz']))
        self._gui_emergency_pub = gui_pub(Emergency, 'gui_emergency_topic')
        self._gui_emergency_thr = gui_relay.Throttle(
            gui_relay.period_for(p['gui_emergency_rate_hz']))
        self._gui_emergency_last = None
        self._gui_gnss_pub = gui_pub(GnssStatus, 'gui_gnss_status_topic')
        self._gui_gnss_thr = gui_relay.Throttle(
            gui_relay.period_for(p['gui_gnss_status_rate_hz']))
        self._gui_gnss_last_fix_type = None
        self._gps_fix_thr = gui_relay.Throttle(gui_relay.period_for(p['gps_fix_rate_hz']))
        self._gui_det_pub = None
        if str(p['gui_detections_topic']) and str(p['detections_topic']):
            from vision_msgs.msg import Detection2DArray
            self._Detection2DArray = Detection2DArray
            self._gui_det_pub = self.create_publisher(
                Detection2DArray, str(p['gui_detections_topic']), 10)
        self._gui_det_thr = gui_relay.Throttle(gui_relay.period_for(p['gui_detections_rate_hz']))
        self._gui_det_sub = None
        self._gui_obstacle_pub = gui_pub(Bool, 'gui_obstacle_close_topic') \
            if str(p['obstacle_close_topic']) else None
        self._gui_obstacle_gate = gui_relay.ChangeOrKeepalive(
            gui_relay.period_for(p['gui_obstacle_close_rate_hz']))
        self._gui_obstacle_sub = None
        self._gui_diag_pub = gui_pub(DiagnosticArray, 'gui_diagnostics_topic') \
            if str(p['diagnostics_source_topic']) else None
        self._gui_diag_merge = gui_relay.LatestByName()
        self._gui_diag_sub = None
        self._estop_req_pub = self.create_publisher(Bool, p['estop_request_topic'], 10)
        self._twist_pub = self.create_publisher(Twist, p['emergency_twist_topic'], 10)
        if self._serve_hl:
            self._hl_pub = self.create_publisher(
                HighLevelStatus, p['high_level_status_topic'], 10)
            latched = QoSProfile(depth=1,
                                 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=QoSReliabilityPolicy.RELIABLE)
            self._resume_pub = self.create_publisher(Bool, p['coverage_resume_topic'], latched)
            self._resume_pub.publish(Bool(data=False))
        else:
            self._hl_pub = None

        # ---- subscriptions ----------------------------------------------
        # Inputs bypass the executor (see sub_pump.py): through the MultiThreadedExecutor the
        # ~390 msg/s of inputs cost ~75 % of a core. Event inputs are handled as they arrive
        # (the handlers only cache under self._lock or relay); the two hot flat messages are
        # parsed straight from CDR, the relays forward the serialized bytes, and
        # /mower_sensor_info (97 Hz, only read by the 5 Hz status) is sampled when consumed.
        self._map_frame = str(p['map_frame'])
        self._base_frame = str(p['base_frame'])
        self._pump = SubscriptionPump(self, 'gui_bridge_inputs')
        sub = self._pump.subscribe
        if not self._serve_hl:
            sub(HighLevelStatus, p['high_level_status_topic'], self._on_external_hl, 10)
        sub(MowerBaseDevStatus, p['mower_status_topic'], self._on_base, 10,
            parser=flat_parser(MowerBaseDevStatus))
        sub(Bool, p['estop_topic'], self._on_estop, 10, parser=flat_parser(Bool))
        self._sensor_sub = sub(MowerSensorInfo, p['sensor_info_topic'], self._on_sensor, 1,
                               sampled=True)
        sub(BatteryState, p['battery_topic'], self._on_battery, 10)
        sub(NavSatFix, p['fix_topic'], self._on_fix, 10)
        sub(String, p['fix_status_topic'], self._on_fix_status, 10)
        sub(Odometry, p['odom_topic'], self._on_odom_raw, 10, raw=True)
        sub(Odometry, p['filtered_odom_topic'], self._on_filtered_raw, 10, raw=True)
        self._diag_pub = None
        self._diag_sampled = []
        self._cameras = []
        if str(p['diagnostics_topic']):
            self._setup_diagnostics_inputs(sub)
        if self._gui_det_pub is not None:
            # sampled: no wake per detection; drained by the 4 Hz relay task, and only while
            # /gui/detections has a subscriber (otherwise flushed).
            self._gui_det_sub = sub(self._Detection2DArray, str(p['detections_topic']),
                                    self._on_detections_raw, qos_profile_sensor_data,
                                    raw=True, sampled=True, deliver_all=True)
        if self._gui_obstacle_pub is not None:
            # sampled newest-only at 5 Hz: <= 5 takes/s instead of a pump wake per frame
            self._gui_obstacle_sub = sub(Bool, str(p['obstacle_close_topic']),
                                         self._on_obstacle_close, 1,
                                         parser=flat_parser(Bool), sampled=True)
        if self._gui_diag_pub is not None:
            self._gui_diag_sub = sub(DiagnosticArray, str(p['diagnostics_source_topic']),
                                     self._on_diagnostics_in, 50, sampled=True,
                                     deliver_all=True)

        # ---- clients -----------------------------------------------------
        self._cutter_cli = self.create_client(
            CutterControl, p['cutter_control_service'], callback_group=self._cli_group)
        self._cutter_off_cli = self.create_client(
            Trigger, p['cutter_off_service'], callback_group=self._cli_group)
        self._clear_estop_cli = self.create_client(
            Empty, p['clear_estop_service'], callback_group=self._cli_group)
        self._manual_blade_cli = self.create_client(
            SetBool, p['manual_blade_service'], callback_group=self._cli_group)
        self._rl_datum_cli = None
        self._GeoPose = None
        if str(p['rl_set_datum_service']):
            try:
                from geographic_msgs.msg import GeoPose
                from robot_localization.srv import SetDatum
                self._GeoPose = GeoPose
                self._rl_datum_cli = self.create_client(
                    SetDatum, str(p['rl_set_datum_service']), callback_group=self._cli_group)
            except ImportError as exc:
                self.get_logger().warn('robot_localization SetDatum unavailable (%s); '
                                       'set_datum only persists the datum' % exc)
        self._datum = self._initial_datum()

        # ---- services ----------------------------------------------------
        g = self._srv_group
        self.create_service(MowerControl, p['mower_control_service'],
                            self._srv_mower_control, callback_group=g)
        self.create_service(EmergencyStop, p['emergency_stop_service'],
                            self._srv_emergency_stop, callback_group=g)
        self.create_service(Trigger, p['reboot_board_service'],
                            self._srv_reboot_board, callback_group=g)
        self.create_service(Trigger, p['set_datum_service'],
                            self._srv_set_datum, callback_group=g)
        if self._serve_hl:
            self.create_service(HighLevelControl, p['high_level_control_service'],
                                self._srv_high_level_control, callback_group=g)
            self.create_service(StartInArea, p['start_in_area_service'],
                                self._srv_start_in_area, callback_group=g)
            self.create_service(Trigger, p['clear_coverage_resume_service'],
                                self._srv_clear_coverage_resume, callback_group=g)

        # ---- timers --------------------------------------------------------
        # Periodic publishers on one plain thread (sub_pump.PeriodicRunner) instead of rclpy
        # timers: each timer firing woke the MultiThreadedExecutor (~13 wakes/s idle).
        tasks = [(1.0 / max(0.1, float(p['status_rate_hz'])), self._publish_status),
                 (1.0 / max(2.0, float(p['emergency_rate_hz'])), self._emergency_tick),
                 (1.0 / max(0.1, float(p['power_rate_hz'])), self._publish_power)]
        if self._serve_hl:
            tasks.append((1.0 / max(0.1, float(p['high_level_rate_hz'])),
                          self._publish_high_level))
            self._publish_high_level()
        if self._diag_pub is not None:
            tasks.append((1.0 / max(0.1, float(p['diagnostics_rate_hz'])),
                          self._publish_diagnostics))
        if self._gui_det_sub is not None:
            tasks.append((0.25, self._relay_detections))
        if self._gui_obstacle_sub is not None:
            tasks.append((0.2, self._relay_obstacle_close))
        if self._gui_diag_sub is not None:
            tasks.append((1.0 / max(0.1, float(p['gui_diagnostics_rate_hz'])),
                          self._relay_diagnostics))
        self._periodic = PeriodicRunner(self, tasks, 'gui_bridge_periodic')
        self._pump.start()              # inputs first, then the periodic evaluation
        self._periodic.start()
        self.get_logger().info('gui_bridge up (serve_high_level=%s)' % self._serve_hl)

    def destroy_node(self):
        self._periodic.stop()
        self._pump.stop()
        return super().destroy_node()

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------
    def _now(self):
        return self.get_clock().now().to_msg()

    def _mcu_alive(self):
        t = self._base_time
        return t is not None and (time.monotonic() - t) <= float(self._p['mcu_timeout_s'])

    def _hl_state(self):
        return self._sm.state if self._serve_hl else self._external_hl_state

    def _call(self, client, request, name, on_done=None):
        """call_async without blocking; returns the future or None."""
        if not client.service_is_ready():
            self.get_logger().warn('%s not available' % name)
            return None
        fut = client.call_async(request)
        if on_done is not None:
            fut.add_done_callback(on_done)
        return fut

    def _call_wait(self, client, request, name):
        """Call and wait (only from reentrant-group service callbacks)."""
        fut = self._call(client, request, name)
        if fut is None:
            return None
        done = threading.Event()
        fut.add_done_callback(lambda _f: done.set())
        if not done.wait(float(self._p['service_timeout_s'])):
            self.get_logger().warn('%s timed out' % name)
            return None
        try:
            return fut.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('%s failed: %s' % (name, exc))
            return None

    def _cutter_request(self, enable, direction=False):
        req = CutterControl.Request()
        req.cutter.enable = bool(enable)
        req.cutter.direction = bool(direction)
        req.cutter.speed = 0          # 0 -> driver default speed
        req.cutter.position = 0
        req.height.enable = False
        return req

    def _cutter_off(self, why):
        self._cutter_requested = False
        self.get_logger().info('cutter off (%s)' % why)
        self._call(self._cutter_off_cli, Trigger.Request(), self._p['cutter_off_service'])

    def _cutter_on_async(self):
        with self._lock:
            allowed, reason = sm.cutter_request_allowed(
                True, self._hl_state(), self._emergency_active)
        if not allowed:
            self.get_logger().warn('cutter on %s' % reason)
            return

        def done(fut):
            try:
                ok = bool(fut.result().result)
            except Exception:  # noqa: BLE001
                ok = False
            with self._lock:
                # Only keep the request if we are still in a blade state.
                self._cutter_requested = ok and self._hl_state() in sm.BLADE_STATES
            if not ok:
                self.get_logger().warn('cutter_control refused/failed while entering '
                                       'MANUAL_MOWING (blade stays off)')

        if self._call(self._cutter_cli, self._cutter_request(True),
                      self._p['cutter_control_service'], done) is None:
            self.get_logger().warn('MANUAL_MOWING without blade: cutter service unavailable')

    def _execute(self, actions):
        for action in actions:
            if action == sm.ACTION_CUTTER_OFF:
                self._cutter_off('state machine')
            elif action == sm.ACTION_CUTTER_ON:
                self._cutter_on_async()
            elif action == sm.ACTION_ZERO_TWIST:
                self._twist_pub.publish(Twist())
            elif action == sm.ACTION_CLEAR_ESTOP:
                self._call(self._clear_estop_cli, Empty.Request(),
                           self._p['clear_estop_service'])

    def _apply(self, result):
        """Log, publish on change, then run the side effects."""
        if result.log:
            self.get_logger().info(result.log)
        if result.changed:
            self._publish_high_level()
        self._execute(result.actions)

    # ------------------------------------------------------------------
    # subscriptions
    # ------------------------------------------------------------------
    def _on_base(self, msg):
        with self._lock:
            self._base = msg
            self._base_time = time.monotonic()
            if msg.is_cutting:
                self._blade_stamp = self._now()
            docked = self._sm.set_docked(bool(msg.is_docking_done)) if self._serve_hl else None
        if docked is not None and docked.changed:
            self._publish_high_level()
        self._evaluate_emergency()

    def _on_sensor(self, msg):
        with self._lock:
            self._sensor = msg
            self._sensor_time = time.monotonic()

    def _on_battery(self, msg):
        with self._lock:
            self._battery = msg
            self._battery_time = time.monotonic()

    def _on_estop(self, msg):
        changed = False
        with self._lock:
            changed = bool(msg.data) != self._estop
            self._estop = bool(msg.data)
        if changed:
            self._evaluate_emergency()
            self._publish_emergency()

    def _on_external_hl(self, msg):
        with self._lock:
            self._external_hl_state = int(msg.state)
            if self._external_hl_state not in sm.BLADE_STATES:
                self._cutter_requested = False

    def _on_fix_status(self, msg):
        with self._lock:
            self._fix_status = sm.parse_fix_status(msg.data)
            self._fix_status_raw = msg.data
            self._fix_status_time = time.monotonic()

    def _on_fix(self, msg):
        if self._gps_fix_thr.due(time.monotonic()):
            self._gps_fix_pub.publish(msg)
        with self._lock:
            self._last_fix = (int(msg.status.status), float(msg.latitude),
                              float(msg.longitude), float(msg.altitude), time.monotonic())
            parsed = None
            if self._fix_status_time is not None and \
                    time.monotonic() - self._fix_status_time <= float(
                        self._p['fix_status_timeout_s']):
                parsed = self._fix_status
        values = sm.derive_gnss_status(int(msg.status.status), list(msg.position_covariance),
                                       int(msg.position_covariance_type), parsed)
        out = GnssStatus()
        out.header = msg.header
        out.backend = 'um960_gps_driver'
        out.receiver_vendor = 'Unicore'
        out.receiver_model = 'UM960'
        for key, value in values.items():
            setattr(out, key, value)
        with self._lock:
            self._gnss_quality = float(values['quality_percent'])
            self._gnss_fix_type = int(values['fix_type'])
        self._gnss_pub.publish(out)
        if self._gui_gnss_pub is not None:
            fix_type = int(values['fix_type'])
            changed = fix_type != self._gui_gnss_last_fix_type
            self._gui_gnss_last_fix_type = fix_type
            if self._gui_gnss_thr.due(time.monotonic(), changed=changed):
                self._gui_gnss_pub.publish(out)

    def _wheel_odom_due(self):
        period = self._wheel_odom_period
        if period <= 0.0:
            return True
        now = time.monotonic()
        if now - self._wheel_odom_last < period:
            return False
        self._wheel_odom_last = now
        return True

    def _on_odom(self, msg):
        if self._wheel_odom_due():
            self._wheel_odom_pub.publish(msg)

    def _on_odom_raw(self, data):
        if self._wheel_odom_due():
            self._wheel_odom_pub.publish(data)

    def _on_filtered(self, msg):
        msg.header.frame_id = self._p['map_frame']
        msg.child_frame_id = self._p['base_frame']
        if self._map_odom_pub is not None:
            self._map_odom_pub.publish(msg)

    def _on_filtered_raw(self, data):
        self._filtered_time = time.monotonic()
        # Same output as _on_filtered, done on the serialized bytes (no deserialize /
        # serialize round trip): relay untouched when the frames already match, otherwise
        # rewrite the two frame strings (ekf publishes odom/base_link).
        if odometry_frames(data) == (self._map_frame, self._base_frame):
            out = data
        else:
            out = odometry_with_frames(data, self._map_frame, self._base_frame)
        if out is None:
            self._on_filtered(deserialize_message(data, Odometry))
            return
        if self._map_odom_pub is not None:
            self._map_odom_pub.publish(out)
        if self._gui_pose_pub is not None and self._gui_pose_thr.due(time.monotonic()):
            self._gui_pose_pub.publish(out)

    def _on_detections_raw(self, data):
        try:
            key = _header(data)[0].frame_id
        except (ValueError, IndexError, Exception):  # noqa: BLE001
            key = None
        if self._gui_det_thr.due(time.monotonic(), key):
            self._gui_det_pub.publish(data)

    def _relay_detections(self):
        if self._gui_det_pub.get_subscription_count() > 0:
            self._pump.poll((self._gui_det_sub,))
        else:
            self._pump.flush(self._gui_det_sub)

    def _on_obstacle_close(self, msg):
        value = bool(msg.data)
        if self._gui_obstacle_gate.due(value, time.monotonic()):
            self._gui_obstacle_pub.publish(Bool(data=value))

    def _relay_obstacle_close(self):
        if self._gui_obstacle_pub.get_subscription_count() > 0:
            self._pump.poll((self._gui_obstacle_sub,))
        else:
            self._pump.flush(self._gui_obstacle_sub)

    def _on_diagnostics_in(self, msg):
        for st in msg.status:
            self._gui_diag_merge.add(st.name, st)

    def _relay_diagnostics(self):
        if self._gui_diag_pub.get_subscription_count() == 0:
            self._pump.flush(self._gui_diag_sub)
            self._gui_diag_merge.take()
            return
        self._pump.poll((self._gui_diag_sub,))
        statuses = self._gui_diag_merge.take()
        if statuses:
            out = DiagnosticArray(status=statuses)
            out.header.stamp = self._now()
            self._gui_diag_pub.publish(out)

    # ------------------------------------------------------------------
    # emergency
    # ------------------------------------------------------------------
    def _evaluate_emergency(self):
        with self._lock:
            base = self._base
            stale = bool(self._p['emergency_on_mcu_timeout']) and not self._mcu_alive()
            fields = sm.emergency_summary(
                self._estop,
                bool(base.stop_triggered) if base is not None else False,
                bool(base.lift_triggered) if base is not None else False,
                stale)
            self._emergency_fields = fields
            was = self._emergency_active
            self._emergency_active = fields[0]
            result = self._sm.set_emergency(fields[0]) if self._serve_hl else None
        if result is not None:
            self._apply(result)
        elif fields[0] and not was:
            # Real mission node owns the state, but still force the blade off.
            self._cutter_off('emergency: %s' % fields[3])

    def _emergency_tick(self):
        self._evaluate_emergency()
        self._publish_emergency()

    def _publish_emergency(self):
        with self._lock:
            active, latched, lift, reason = self._emergency_fields
        msg = Emergency()
        msg.stamp = self._now()
        msg.active_emergency = active
        msg.latched_emergency = latched
        msg.lift_warning = lift
        msg.lift_duration_sec = 0.0
        msg.reason = reason
        self._emergency_pub.publish(msg)
        if self._gui_emergency_pub is not None:
            fields = (active, latched, lift, reason)
            changed = fields != self._gui_emergency_last
            self._gui_emergency_last = fields
            if self._gui_emergency_thr.due(time.monotonic(), changed=changed):
                self._gui_emergency_pub.publish(msg)

    # ------------------------------------------------------------------
    # periodic publishers
    # ------------------------------------------------------------------
    def _publish_status(self):
        self._pump.poll((self._sensor_sub,))     # newest /mower_sensor_info (sampled input)
        with self._lock:
            base, sensor = self._base, self._sensor
            alive = self._mcu_alive()
            cutting = bool(base.is_cutting) if base is not None and alive else False
            msg = Status()
            msg.stamp = self._now()
            msg.mower_status = Status.MOWER_STATUS_OK if alive else \
                Status.MOWER_STATUS_INITIALIZING
            msg.is_charging = bool(base.is_charging) if base is not None else False
            msg.rain_detected = bool(base.rain_triggered) if base is not None else False
            msg.mow_enabled = self._cutter_requested or cutting
            msg.esc_power = cutting
            msg.mower_esc_status = 1 if cutting else 0
            if sensor is not None:
                motor = getattr(sensor, 'cutter_motor', None)
                rpm = getattr(motor, 'speed_rpm', None)
                if rpm is None:
                    rpm = getattr(motor, 'speed', 0)
                msg.mower_motor_rpm = float(rpm or 0)
                msg.mower_motor_temperature = float(getattr(motor, 'temperature', 0) or 0)
                msg.mower_esc_current = float(getattr(motor, 'current', 0) or 0) * 0.01
                msg.firmware_version = str(getattr(sensor, 'cutter_board_version', '') or '')
            if self._blade_stamp is not None:
                msg.blade_status_stamp = self._blade_stamp
            msg.firmware_compatible = True
        self._status_pub.publish(msg)
        if self._gui_status_pub is not None and self._gui_status_thr.due(time.monotonic()):
            self._gui_status_pub.publish(msg)

    def _publish_power(self):
        with self._lock:
            batt, base = self._battery, self._base
        msg = Power()
        msg.stamp = self._now()
        charging = bool(base.is_charging) if base is not None else False
        if batt is not None:
            charging = charging or \
                batt.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING
            msg.v_battery = float(batt.voltage)
            msg.v_charge = float(batt.voltage) if charging else 0.0
            msg.charge_current = abs(float(batt.current)) if charging else 0.0
        msg.charger_enabled = charging
        if batt is None:
            msg.charger_status = 'no battery data'
        elif charging:
            msg.charger_status = 'charging'
        elif base is not None and base.is_docking_done:
            msg.charger_status = 'docked, not charging'
        else:
            msg.charger_status = 'not charging'
        self._power_pub.publish(msg)

    def _publish_high_level(self):
        if self._hl_pub is None:
            return
        with self._lock:
            msg = HighLevelStatus()
            msg.state, msg.state_name = self._sm.snapshot()
            msg.sub_state_name = ''
            msg.current_area = -1
            msg.coverage_percent = 0.0
            # Upstream behavior_tree_node publishes gps quality as a 0..1 fraction.
            msg.gps_quality_percent = self._gnss_quality / 100.0
            batt, base = self._battery, self._base
            msg.battery_percent = sm.battery_percent(batt.percentage) if batt else 0.0
            msg.is_charging = bool(base.is_charging) if base is not None else False
            msg.emergency = self._emergency_active
        self._hl_pub.publish(msg)

    # ------------------------------------------------------------------
    # hardware diagnostics (/diagnostics, see diagnostics.py)
    # ------------------------------------------------------------------
    def _setup_diagnostics_inputs(self, sub):
        """Low-rate, sampled inputs used only by the 1 Hz diagnostics publisher."""
        p = self._p
        self._diag_pub = self.create_publisher(DiagnosticArray, p['diagnostics_topic'], 10)
        sensor_qos = qos_profile_sensor_data          # best effort matches any publisher

        def sampled(msg_type, topic, callback, **kw):
            if str(topic):
                self._diag_sampled.append(sub(msg_type, topic, callback, sensor_qos,
                                              sampled=True, with_receipt=True, **kw))

        sampled(Imu, p['imu_topic'], self._on_imu_receipt, raw=True)
        sampled(Float32, p['imu_temperature_topic'], self._on_imu_temperature)
        sampled(String, p['lights_state_topic'], self._on_lights)
        if str(p['imu_bias_status_topic']):
            latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=QoSReliabilityPolicy.RELIABLE)
            sub(String, p['imu_bias_status_topic'], self._on_imu_bias, latched)
        if str(p['heading_status_topic']):
            latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=QoSReliabilityPolicy.RELIABLE)
            sub(String, p['heading_status_topic'], self._on_heading_status, latched)
        if str(p['tilt_status_topic']):
            latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=QoSReliabilityPolicy.RELIABLE)
            sub(String, p['tilt_status_topic'], self._on_tilt_status, latched)
        for spec in p['diagnostics_cameras'] or []:
            cam = diag.parse_camera_spec(spec)
            if cam is None:
                self.get_logger().warn('ignoring malformed diagnostics_cameras entry %r' % spec)
                continue
            self._cameras.append(cam)
            if cam['freshness_topic'] and not cam['on_demand']:
                sampled(CameraInfo, cam['freshness_topic'],
                        lambda _d, receipt, _id=cam['id']: self._on_camera_frame(_id, receipt),
                        raw=True)

    def _on_imu_receipt(self, _data, receipt):
        self._imu_time = receipt

    def _on_imu_temperature(self, msg, _receipt):
        self._imu_temperature = float(msg.data)

    def _on_imu_bias(self, msg):
        self._imu_bias = msg.data

    def _on_heading_status(self, msg):
        self._heading_status = msg.data

    def _on_tilt_status(self, msg):
        self._tilt_status = msg.data

    def _on_lights(self, msg, receipt):
        self._lights, self._lights_time = msg.data, receipt

    def _on_camera_frame(self, cam_id, receipt):
        self._camera_times[cam_id] = receipt

    @staticmethod
    def _motor_dict(motor):
        if motor is None:
            return None
        status = getattr(motor, 'status', None)
        return {'speed_rpm': int(getattr(motor, 'speed_rpm', 0)),
                'current': int(getattr(motor, 'current', 0)),
                'voltage': int(getattr(motor, 'voltage', 0)),
                'temperature': int(getattr(motor, 'temperature', 0)),
                'status': int(getattr(status, 'status', 0) if status is not None else 0)}

    def _diagnostics_snapshot(self):
        now = time.monotonic()

        def age(t):
            return None if t is None else max(0.0, now - t)

        self._pump.poll(self._diag_sampled)
        with self._lock:
            sensor, batt, base = self._sensor, self._battery, self._base
            snap = {'base_age': age(self._base_time), 'sensor_age': age(self._sensor_time),
                    'battery_age': age(self._battery_time),
                    'fix_status': self._fix_status_raw,
                    'fix_status_age': age(self._fix_status_time),
                    'filtered_age': age(self._filtered_time),
                    'datum': self._datum, 'fix_type': self._gnss_fix_type}
            last_fix = self._last_fix
        if sensor is not None:
            snap['sensor'] = {k: getattr(sensor, k, None) for k in (
                'cutter_board_version', 'chassis_board_version', 'rtk_board_version',
                'mower_package_version', 'bumper_triggered', 'lift_triggered',
                'stop_triggered', 'rain_triggered', 'is_charging', 'is_cutting',
                'is_docking_done', 'battery_error', 'battery_temperature')}
            for key in ('cutter_motor', 'left_motor', 'right_motor'):
                snap['sensor'][key] = self._motor_dict(getattr(sensor, key, None))
        if batt is not None:
            charging = (base is not None and bool(base.is_charging)) or \
                batt.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING
            snap['battery'] = {'voltage': float(batt.voltage), 'current': float(batt.current),
                               'percentage': float(batt.percentage),
                               'temperature': float(batt.temperature), 'charging': charging}
        if last_fix is not None:
            snap['fix'] = {'status': last_fix[0], 'lat': last_fix[1], 'lon': last_fix[2],
                           'alt': last_fix[3]}
            snap['fix_age'] = age(last_fix[4])
        if str(self._p['imu_topic']):
            snap['imu_age'] = age(self._imu_time)
            snap['imu_temperature'] = self._imu_temperature
            snap['imu_bias'] = self._imu_bias
        if str(self._p['heading_status_topic']):
            snap['heading'] = self._heading_status
        if str(self._p['tilt_status_topic']):
            snap['tilt'] = self._tilt_status
        if str(self._p['lights_state_topic']):
            snap['lights'], snap['lights_age'] = self._lights, age(self._lights_time)
        cams = []
        for cam in self._cameras:
            topic = cam['topic']
            cams.append({'spec': cam, 'age': age(self._camera_times.get(cam['id'])),
                         'publishers': self.count_publishers(topic),
                         'viewers': self.count_subscribers(topic)})
        snap['cameras'] = cams
        return snap

    def _publish_diagnostics(self):
        statuses = diag.build_statuses(
            self._diagnostics_snapshot(), float(self._p['diagnostics_timeout_s']),
            bool(self._p['diagnostics_motor_status_alerts']))
        out = DiagnosticArray()
        out.header.stamp = self._now()
        for st in statuses:
            ds = DiagnosticStatus()
            ds.level = bytes([st.level])
            ds.name, ds.message, ds.hardware_id = st.name, st.message, st.hardware_id
            ds.values = [KeyValue(key=k, value=v) for k, v in st.values]
            out.status.append(ds)
        self._diag_pub.publish(out)

    # ------------------------------------------------------------------
    # services
    # ------------------------------------------------------------------
    def _srv_mower_control(self, req, resp):
        enable = int(req.mow_enabled) != 0
        with self._lock:
            allowed, reason = sm.cutter_request_allowed(
                enable, self._hl_state(), self._emergency_active)
        if not allowed:
            self.get_logger().warn('mower_control %s' % reason)
            resp.success = False
            return resp
        with self._lock:
            route = sm.mower_control_route(self._serve_hl, self._hl_state())
        if route == sm.ROUTE_MISSION:
            return self._mower_control_via_mission(enable, resp)
        if not enable:
            self._cutter_requested = False
        res = self._call_wait(self._cutter_cli,
                              self._cutter_request(enable, int(req.mow_direction) != 0),
                              self._p['cutter_control_service'])
        resp.success = bool(res is not None and res.result)
        if resp.success:
            with self._lock:
                self._cutter_requested = enable
        elif not enable:
            # cutter_control refused/unavailable: fall back to the unconditional off.
            self._cutter_off('mower_control fallback')
        self.get_logger().info('mower_control enable=%s -> %s' % (enable, resp.success))
        return resp

    def _mower_control_via_mission(self, enable, resp):
        """MANUAL_MOWING under the real mission node: the mission owns the blade."""
        req = SetBool.Request()
        req.data = enable
        res = self._call_wait(self._manual_blade_cli, req, self._p['manual_blade_service'])
        resp.success = bool(res is not None and res.success)
        with self._lock:
            # The mission owns the request; Status.mow_enabled follows is_cutting.
            self._cutter_requested = False
        if not resp.success and not enable:
            self._cutter_off('mower_control fallback (mission manual_blade unavailable)')
        self.get_logger().info('mower_control enable=%s -> mission manual_blade: %s (%s)' % (
            enable, resp.success, res.message if res is not None else 'unavailable'))
        return resp

    def _srv_emergency_stop(self, req, resp):
        if int(req.emergency) != 0:
            self.get_logger().warn('emergency stop requested from GUI')
            self._estop_req_pub.publish(Bool(data=True))
            self._cutter_off('GUI emergency stop')
            resp.success = True
        else:
            res = self._call_wait(self._clear_estop_cli, Empty.Request(),
                                  self._p['clear_estop_service'])
            resp.success = res is not None
            self.get_logger().info('emergency reset requested -> %s' % resp.success)
        return resp

    def _srv_reboot_board(self, req, resp):
        resp.success = True
        resp.message = 'not supported on Tron'
        return resp

    def _initial_datum(self):
        """The datum in force: the launch args, else datum.env, else mowgli_robot.yaml.
        Also re-syncs mowgli_robot.yaml to the launch datum (a deploy sync can reset it)."""
        p = self._p
        launched = (float(p['datum_lat']), float(p['datum_lon']))
        yaml_path = str(p['robot_yaml_path'])
        if dt.datum_is_set(*launched):
            if yaml_path and bool(p['sync_robot_yaml_datum']):
                text = dt.read_text(yaml_path)
                current = dt.read_yaml_datum(text) if text is not None else None
                if text is not None and (current is None or dt.distance_m(
                        current[0], current[1], launched[0], launched[1]) > 0.01):
                    new, ok = dt.splice_datum_yaml(text, launched[0], launched[1])
                    if ok:
                        try:
                            dt.write_text_atomic(yaml_path, new)
                            self.get_logger().warn(
                                '%s datum %s -> launch datum %.9f,%.9f' % (
                                    yaml_path, current, launched[0], launched[1]))
                        except OSError as exc:
                            self.get_logger().warn('cannot update %s: %s' % (yaml_path, exc))
            return launched
        for path, parse in ((str(p['datum_env_path']), dt.parse_datum_env),
                            (yaml_path, dt.read_yaml_datum)):
            text = dt.read_text(path) if path else None
            found = parse(text) if text is not None else None
            if found is not None and dt.datum_is_set(*found):
                return found
        return None

    def _srv_set_datum(self, req, resp):
        p = self._p
        with self._lock:
            fix = self._last_fix
        if fix is None:
            ok, reason = dt.fix_usable(None, None, None, None, 0.0)
        else:
            status, lat, lon, alt, t = fix
            ok, reason = dt.fix_usable(status, lat, lon, time.monotonic() - t,
                                       float(p['datum_fix_max_age_s']))
        if not ok:
            resp.success = False
            resp.message = 'set_datum refused: %s' % reason
            self.get_logger().warn(resp.message)
            return resp

        previous = self._datum
        reply = dt.format_reply(lat, lon, previous)
        problems = []

        yaml_path = str(p['robot_yaml_path'])
        if yaml_path:
            text = dt.read_text(yaml_path)
            if text is None:
                problems.append('cannot read %s' % yaml_path)
            else:
                new, spliced = dt.splice_datum_yaml(text, lat, lon)
                if not spliced:
                    problems.append('no datum keys in %s' % yaml_path)
                else:
                    try:
                        dt.write_text_atomic(yaml_path, new)
                    except OSError as exc:
                        problems.append('cannot write %s: %s' % (yaml_path, exc))
        env_path = str(p['datum_env_path'])
        if env_path:
            try:
                dt.write_text_atomic(env_path, dt.format_datum_env(
                    lat, lon, time.strftime('%Y-%m-%dT%H:%M:%S%z')))
            except OSError as exc:
                problems.append('cannot write %s: %s' % (env_path, exc))

        self._datum = (lat, lon)
        rl = self._call_rl_set_datum(lat, lon, alt)
        if problems:
            # keep the reply's single comma (GUI contract): only lat,lon may contain one
            reply += ' NOT SAVED: ' + '; '.join(problems).replace(',', ';')
        elif not rl:
            reply += ' (saved; restart the stack to re-anchor localization)'
        resp.success = True
        resp.message = reply
        self.get_logger().info('set_datum -> %s' % reply)
        return resp

    def _call_rl_set_datum(self, lat, lon, alt):
        """Fire-and-forget robot_localization SetDatum (yaw 0 = ENU east). Returns
        whether the call was sent."""
        cli = self._rl_datum_cli
        if cli is None or not cli.service_is_ready():
            return False
        req = cli.srv_type.Request()
        geo = self._GeoPose()
        geo.position.latitude = float(lat)
        geo.position.longitude = float(lon)
        geo.position.altitude = float(alt) if alt == alt else 0.0
        geo.orientation.w = 1.0
        req.geo_pose = geo
        name = str(self._p['rl_set_datum_service'])

        def done(fut):
            try:
                fut.result()
                self.get_logger().info('%s accepted %.9f,%.9f' % (name, lat, lon))
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn('%s failed: %s' % (name, exc))

        self._call(cli, req, name, done)
        return True

    def _srv_high_level_control(self, req, resp):
        with self._lock:
            result = self._sm.command(int(req.command))
        if not result.success:
            self.get_logger().warn('high_level_control: %s' % result.log)
            result.log = ''
        self._apply(result)
        resp.success = bool(result.success)
        return resp

    def _srv_start_in_area(self, req, resp):
        self.get_logger().warn('start_in_area(%d) refused: no mission layer yet' % req.area)
        resp.success = False
        return resp

    def _srv_clear_coverage_resume(self, req, resp):
        resp.success = True
        resp.message = 'no coverage resume state in the stub'
        return resp


def main(args=None):
    try:  # crash records -> /userdata/ros2/crashes (docs/crash_recovery.md)
        from mower_control.crash_record import install as _install_crash_record
        _install_crash_record('gui_bridge')
    except ImportError:
        pass
    rclpy.init(args=args)
    node = GuiBridgeNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
