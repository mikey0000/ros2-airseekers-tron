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
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, NavSatFix
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty, Trigger

from mower_interfaces.msg import MowerBaseDevStatus, MowerSensorInfo
from mower_interfaces.srv import CutterControl
from mowgli_interfaces.msg import Emergency, GnssStatus, HighLevelStatus, Power, Status
from mowgli_interfaces.srv import EmergencyStop, HighLevelControl, MowerControl, StartInArea

from mower_gui_bridge import state_machine as sm

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
    'emergency_twist_topic': '/cmd_vel_emergency',
    # outputs for the GUI
    'status_topic': '/hardware_bridge/status',
    'emergency_topic': '/hardware_bridge/emergency',
    'power_topic': '/hardware_bridge/power',
    'gps_fix_topic': '/gps/fix',
    'gnss_status_topic': '/gps/status',
    'wheel_odom_topic': '/wheel_odom',
    'filtered_map_topic': '/odometry/filtered_map',
    'high_level_status_topic': '/behavior_tree_node/high_level_status',
    'coverage_resume_topic': '/behavior_tree_node/coverage_resume_available',
    'mower_control_service': '/hardware_bridge/mower_control',
    'emergency_stop_service': '/hardware_bridge/emergency_stop',
    'reboot_board_service': '/hardware_bridge/reboot_board',
    'high_level_control_service': '/behavior_tree_node/high_level_control',
    'start_in_area_service': '/behavior_tree_node/start_in_area',
    'clear_coverage_resume_service': '/behavior_tree_node/clear_coverage_resume',
    'set_datum_service': '/navsat_to_absolute_pose/set_datum',
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
        self._gnss_quality = 0.0
        self._cutter_requested = False
        self._blade_stamp = None
        self._emergency_active = False
        self._emergency_fields = (False, False, False, '')

        self._srv_group = ReentrantCallbackGroup()
        self._cli_group = ReentrantCallbackGroup()

        # ---- publishers -------------------------------------------------
        self._status_pub = self.create_publisher(Status, p['status_topic'], 10)
        self._emergency_pub = self.create_publisher(Emergency, p['emergency_topic'], 10)
        self._power_pub = self.create_publisher(Power, p['power_topic'], 10)
        self._gps_fix_pub = self.create_publisher(NavSatFix, p['gps_fix_topic'], 10)
        self._gnss_pub = self.create_publisher(GnssStatus, p['gnss_status_topic'], 10)
        self._wheel_odom_pub = self.create_publisher(Odometry, p['wheel_odom_topic'], 10)
        self._map_odom_pub = self.create_publisher(Odometry, p['filtered_map_topic'], 10)
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
            self.create_subscription(HighLevelStatus, p['high_level_status_topic'],
                                     self._on_external_hl, 10)

        # ---- subscriptions ----------------------------------------------
        self.create_subscription(MowerBaseDevStatus, p['mower_status_topic'], self._on_base, 10)
        self.create_subscription(MowerSensorInfo, p['sensor_info_topic'], self._on_sensor, 10)
        self.create_subscription(BatteryState, p['battery_topic'], self._on_battery, 10)
        self.create_subscription(Bool, p['estop_topic'], self._on_estop, 10)
        self.create_subscription(NavSatFix, p['fix_topic'], self._on_fix, 10)
        self.create_subscription(String, p['fix_status_topic'], self._on_fix_status, 10)
        self.create_subscription(Odometry, p['odom_topic'], self._on_odom, 10)
        self.create_subscription(Odometry, p['filtered_odom_topic'], self._on_filtered, 10)

        # ---- clients -----------------------------------------------------
        self._cutter_cli = self.create_client(
            CutterControl, p['cutter_control_service'], callback_group=self._cli_group)
        self._cutter_off_cli = self.create_client(
            Trigger, p['cutter_off_service'], callback_group=self._cli_group)
        self._clear_estop_cli = self.create_client(
            Empty, p['clear_estop_service'], callback_group=self._cli_group)

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
        self.create_timer(1.0 / max(0.1, float(p['status_rate_hz'])), self._publish_status)
        self.create_timer(1.0 / max(2.0, float(p['emergency_rate_hz'])), self._emergency_tick)
        self.create_timer(1.0 / max(0.1, float(p['power_rate_hz'])), self._publish_power)
        if self._serve_hl:
            self.create_timer(1.0 / max(0.1, float(p['high_level_rate_hz'])),
                              self._publish_high_level)
            self._publish_high_level()

        self.get_logger().info('gui_bridge up (serve_high_level=%s)' % self._serve_hl)

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

    def _on_battery(self, msg):
        with self._lock:
            self._battery = msg

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

    def _on_fix_status(self, msg):
        with self._lock:
            self._fix_status = sm.parse_fix_status(msg.data)
            self._fix_status_time = time.monotonic()

    def _on_fix(self, msg):
        self._gps_fix_pub.publish(msg)
        with self._lock:
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
        self._gnss_pub.publish(out)

    def _on_odom(self, msg):
        self._wheel_odom_pub.publish(msg)

    def _on_filtered(self, msg):
        msg.header.frame_id = self._p['map_frame']
        msg.child_frame_id = self._p['base_frame']
        self._map_odom_pub.publish(msg)

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

    # ------------------------------------------------------------------
    # periodic publishers
    # ------------------------------------------------------------------
    def _publish_status(self):
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

    def _srv_set_datum(self, req, resp):
        resp.success = False
        resp.message = 'datum is fixed in src/mower_localization/config/navsat.yaml'
        return resp

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
