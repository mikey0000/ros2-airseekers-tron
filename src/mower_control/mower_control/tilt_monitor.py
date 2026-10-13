#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Slope / tilt monitor (ROS 2 Jazzy): IMU attitude -> ok / caution / limit / critical.

2026-10-09. Band logic, filtering, hysteresis and the mounting correction live in
``tilt_logic.py`` (pure, unit tested); this node only feeds it and acts on CRITICAL itself.
The mission (mower_mission ``tilt_*`` params) owns the caution speed cap and the limit
back-out; doing the critical stop here as well keeps it independent of the mission FSM
(a stuck tick, a missing action server) and of twist_mux priorities below 100.

Subscribed
----------
``/imu/data`` (``imu_topic``)  sensor_msgs/Imu  wit_imu_driver, RELIABLE, 100 Hz. Taken
                               as a sampled input at ``rate_hz`` (newest message per tick).
                               Deliberately the driver topic, not /imu/data_aligned: the
                               aligner only rotates yaw, and a safety guard should depend on
                               as few nodes as possible.

Published
---------
``/tilt/status`` std_msgs/String JSON, latched (TRANSIENT_LOCAL depth 1): published on every
                 band change and at ``status_rate_hz`` (contract: ``TiltBands.status``).
``/cmd_vel_emergency`` geometry_msgs/Twist: zeros at ``rate_hz`` from entering CRITICAL
                 (``critical_stop``) for as long as the band stays critical (or goes unknown
                 while critical: IMU lost on a slope), then ``critical_hold_s`` more, and at
                 least ``zero_burst_s`` in all. A stream, not a burst: twist_mux drops the
                 emergency lane 0.2 s after its last message, so the old 0.5 s burst let the
                 navigation lane drive again ~0.7 s later unless the mission's EMERGENCY
                 had closed the motion gate by then (2026-10-13 review). The stop must not
                 depend on the mission FSM.
``/estop_request`` std_msgs/Bool ``true`` once on entering CRITICAL when
                 ``critical_estop_request`` (default false): mcu_node latches motion + blade
                 off until cleared (GUI / ``/clear_estop``).

Called
------
``/cutter_off`` std_srvs/Trigger on entering CRITICAL (``critical_stop``), fire and forget.
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_srvs.srv import Trigger

from mower_control import tilt_logic
from mower_control.sub_pump import PeriodicRunner, SubscriptionPump, parse_imu


class TiltMonitorNode(Node):

    def __init__(self):
        super().__init__('tilt_monitor')
        d = tilt_logic.DEFAULTS
        dp = self.declare_parameter
        dp('imu_topic', '/imu/data')
        dp('status_topic', '/tilt/status')
        dp('rate_hz', 25.0)
        dp('status_rate_hz', 2.0)
        dp('roll_thresholds_deg', list(d['roll_thresholds_deg']))
        dp('pitch_thresholds_deg', list(d['pitch_thresholds_deg']))
        dp('hysteresis_deg', d['hysteresis_deg'])
        dp('debounce_s', list(d['debounce_s']))
        dp('clear_s', d['clear_s'])
        dp('filter_tau_s', d['filter_tau_s'])
        dp('stale_s', d['stale_s'])
        dp('mount_roll_offset_deg', d['mount_roll_offset_deg'])
        dp('mount_pitch_offset_deg', d['mount_pitch_offset_deg'])
        dp('critical_stop', True)
        dp('emergency_twist_topic', '/cmd_vel_emergency')
        dp('cutter_off_service', '/cutter_off')
        dp('zero_burst_s', 0.5)
        dp('critical_hold_s', 1.0)
        dp('critical_estop_request', False)
        dp('estop_request_topic', '/estop_request')
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self._bands = tilt_logic.TiltBands(**{k: gp(k) for k in tilt_logic.DEFAULTS})
        self._critical_stop = bool(gp('critical_stop'))
        self._zero_burst_s = float(gp('zero_burst_s'))
        self._critical_hold_s = float(gp('critical_hold_s'))
        self._estop_request = bool(gp('critical_estop_request'))
        self._status_period = 1.0 / max(0.1, float(gp('status_rate_hz')))

        self._sample = None          # (receipt monotonic, roll, pitch) of the newest IMU msg
        self._band = None            # last published band
        self._last_status = -1e9
        self._burst_until = 0.0
        self._inverted_warned = False
        self._stale_warned = False

        imu_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             history=QoSHistoryPolicy.KEEP_LAST)
        self._pump = SubscriptionPump(self, 'tilt_inputs')
        self._pump.subscribe(Imu, str(gp('imu_topic')), self._on_imu, imu_qos,
                             parser=parse_imu, sampled=True, with_receipt=True)
        self._pump.start()
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             history=QoSHistoryPolicy.KEEP_LAST,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._status_pub = self.create_publisher(String, str(gp('status_topic')), latched)
        self._twist_pub = self.create_publisher(Twist, str(gp('emergency_twist_topic')), 10)
        from std_msgs.msg import Bool
        self._estop_pub = (self.create_publisher(Bool, str(gp('estop_request_topic')), 10)
                           if self._estop_request else None)
        self._cutter_off = self.create_client(Trigger, str(gp('cutter_off_service')))
        self._runner = PeriodicRunner(
            self, [(1.0 / max(1.0, float(gp('rate_hz'))), self._tick)], 'tilt_monitor')
        self._runner.start()
        b = self._bands
        self.get_logger().info(
            'tilt_monitor: roll %s / pitch %s deg (caution/limit/critical), hysteresis %.1f, '
            'debounce %s s, mount offset roll %.1f pitch %.1f deg, critical_stop=%s'
            % (b.roll_thr, b.pitch_thr, b.hyst, b.debounce, gp('mount_roll_offset_deg'),
               gp('mount_pitch_offset_deg'), self._critical_stop))

    def shutdown(self):
        self._runner.stop()
        self._pump.stop()

    def _on_imu(self, msg, receipt):
        q = msg.orientation
        if q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0:
            return                   # orientation not filled: no attitude to judge
        roll, pitch = tilt_logic.quat_to_roll_pitch(q.x, q.y, q.z, q.w)
        self._sample = (receipt, roll, pitch)

    def _tick(self):
        self._pump.poll()
        now = time.monotonic()
        s, self._sample = self._sample, None
        if s is not None:
            self._bands.update(s[0], s[1], s[2])
        self.step(now)

    def step(self, now):
        """Band change handling + status publishing (split from _tick for tests)."""
        b = self._bands
        band = b.band(now)
        if band == tilt_logic.UNKNOWN:
            if not self._stale_warned and self._band not in (None, tilt_logic.UNKNOWN):
                self.get_logger().warn('tilt: no IMU attitude for > %.1f s: band unknown'
                                       % b.stale_s)
            self._stale_warned = True
        else:
            self._stale_warned = False
        if b.inverted() and not self._inverted_warned:
            self._inverted_warned = True
            self.get_logger().error(
                'tilt: IMU reads |roll| %.0f deg (upside down). If the board is mounted '
                'inverted set mount_roll_offset_deg: 180; until then this is CRITICAL' % abs(b.roll))
        changed = band != self._band
        prev = self._band
        if changed:
            self._on_band(self._band, band, now)
            self._band = band
        if self._critical_stop and (band == 'critical' or (
                band == tilt_logic.UNKNOWN and getattr(self, '_stop_latched', False))):
            # Hold the emergency lane for as long as it is critical (+ hold-off after).
            self._stop_latched = True
            self._burst_until = max(self._burst_until, now + self._critical_hold_s)
        elif getattr(self, '_stop_latched', False) and changed:
            self._stop_latched = False
            self.get_logger().warn('tilt: left critical (%s -> %s); emergency stop released '
                                   'in %.1f s' % (prev, band, max(0.0, self._burst_until - now)))
        if now < self._burst_until:
            self._twist_pub.publish(Twist())
        if changed or now - self._last_status >= self._status_period:
            self._last_status = now
            self._status_pub.publish(String(data=json.dumps(b.status(now), sort_keys=True)))

    def _on_band(self, old, new, now=None):
        b = self._bands
        text = 'tilt band %s -> %s (roll %s, pitch %s deg%s)' % (
            old, new, _deg(b.roll), _deg(b.pitch), ', axis ' + b.axis if b.axis else '')
        if new == 'critical':
            self.get_logger().error(text)
        elif new in ('limit', 'caution'):
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)
        if new == 'critical' and self._critical_stop:
            now = time.monotonic() if now is None else now
            self._burst_until = max(self._burst_until, now + self._zero_burst_s)
            self._twist_pub.publish(Twist())
            if getattr(self, '_estop_pub', None) is not None:
                from std_msgs.msg import Bool
                self._estop_pub.publish(Bool(data=True))
            if self._cutter_off.service_is_ready():
                self._cutter_off.call_async(Trigger.Request())
            else:
                self.get_logger().error('tilt critical: %s not available (MCU interlock / '
                                        'mission blade-off remain)' % self._cutter_off.srv_name)


def _deg(v):
    return '-' if v is None or not math.isfinite(v) else '%.1f' % v


def main(args=None):
    try:  # crash records -> /userdata/ros2/crashes (docs/crash_recovery.md)
        from mower_control.crash_record import install as _install_crash_record
        _install_crash_record('tilt_monitor')
    except ImportError:
        pass
    rclpy.init(args=args)
    node = TiltMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
