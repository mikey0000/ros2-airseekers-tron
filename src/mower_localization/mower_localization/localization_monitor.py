#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""localization_monitor: publishes /localization/status (latched JSON) for the mission layer.

2026-10-09 (localization item 6). See :mod:`mower_localization.localization_status_logic`
for the drift model and the ok / degraded / lost thresholds. mower_mission holds (blade off,
stop) while the state is not ``ok`` during a mow and resumes once it is ``ok`` again.

Subscribed (all sampled by one 5 Hz thread through sub_pump, no executor wakes)
----------
``/fix_status``              std_msgs/String       um960 solution tokens (RTK class)
``/odometry/filtered``       nav_msgs/Odometry     ekf_odom, continuous: distance travelled
``/odometry/filtered_map``   nav_msgs/Odometry     ekf_map: freshness + pose covariance
                                                   (ok needs xy sigma <= loc_sigma_ok_m)
``/odometry/vio_gated``      nav_msgs/Odometry     vio_gate output: VIO is being fused
``/heading_aligner/status``  std_msgs/String JSON  heading alignment (reported only)
``/tilt/status``             std_msgs/String JSON  roll / pitch: the drift model's tilt terms
``/mcu/measured_speed``      geometry_msgs/TwistStamped  wheel yaw rate: slip against the gyro
``/odometry/gps``            nav_msgs/Odometry     RTK position in the map frame: checked
                                                   against dead reckoning (coast mode)

Published
---------
``/localization/status``     std_msgs/String JSON, latched, 1 Hz + on state change:
    {state: ok|degraded|lost, reason, rtk: fixed|float|dgps|single|none|stale,
     since_fixed_s, dist_since_fixed_m, est_drift_m, drift_budget_m, lost_drift_m,
     ekf_xy_sigma_m, sigma_ok_m, vio_active, tilt_mean_deg, pivots, coast_budget_m,
     coast_left_m, rtk_dr_m, rtk_dr_gate_m, dr_lost, heading_aligned, heading_source,
     heading_sigma_deg, ...}

Parameters: ``config/localization_monitor.yaml`` (drift model, coast budget, slip, float gate).
"""

import json
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from mower_localization import localization_status_logic as lsl
from mower_common.clock import node_clock
from mower_common.sub_pump import PeriodicRunner, SubscriptionPump, parse_odometry
from mower_common.angles import yaw_from_quaternion


def _yaw(q):
    return yaw_from_quaternion(q.x, q.y, q.z, q.w)


class LocalizationMonitor(Node):

    def __init__(self):
        super().__init__('localization_monitor')
        # node clock: the pump's receipt times are sim time under use_sim_time
        self._clk_now = node_clock(self).now
        d = self.declare_parameter
        d('status_topic', '/localization/status')
        d('fix_status_topic', '/fix_status')
        d('odom_topic', '/odometry/filtered')
        d('map_odom_topic', '/odometry/filtered_map')
        d('vio_topic', '/odometry/vio_gated')
        d('heading_status_topic', '/heading_aligner/status')
        d('tilt_topic', '/tilt/status')
        d('wheel_topic', '/mcu/measured_speed')
        d('gps_odom_topic', '/odometry/gps')
        d('input_rate', 5.0)
        d('status_rate', 1.0)
        defaults = lsl.StatusParams()
        for name, value in vars(defaults).items():
            d(name, value)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.est = lsl.LocalizationStatus(lsl.StatusParams(**{
            name: type(value)(g(name)) for name, value in vars(defaults).items()}))
        self._lock = threading.Lock()
        self._last_state = None
        self._last_pub = 0.0
        self._status_period = 1.0 / max(0.1, float(g('status_rate')))

        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        one = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                         history=QoSHistoryPolicy.KEEP_LAST)
        self._pub = self.create_publisher(String, g('status_topic'), latched)
        self._pump = SubscriptionPump(self, 'localization_monitor_inputs')
        s = dict(sampled=True, with_receipt=True)
        self._pump.subscribe(String, g('fix_status_topic'), self._on_fix_status, one, **s)
        self._pump.subscribe(Odometry, g('odom_topic'), self._on_odom, one,
                             parser=parse_odometry, **s)
        self._pump.subscribe(Odometry, g('map_odom_topic'), self._on_map_odom, one,
                             parser=parse_odometry, **s)
        self._pump.subscribe(Odometry, g('vio_topic'), self._on_vio, one,
                             parser=parse_odometry, **s)
        self._pump.subscribe(String, g('heading_status_topic'), self._on_heading, latched, **s)
        self._pump.subscribe(String, g('tilt_topic'), self._on_tilt, latched, **s)
        self._pump.subscribe(TwistStamped, g('wheel_topic'), self._on_wheel, one, **s)
        self._pump.subscribe(Odometry, g('gps_odom_topic'), self._on_gps_odom, one,
                             parser=parse_odometry, **s)
        self._periodic = PeriodicRunner(self, [
            (1.0 / max(1.0, float(g('input_rate'))), self._tick),
        ], 'localization_monitor_periodic')
        self._pump.start()
        self._periodic.start()
        p = self.est.p
        self.get_logger().info(
            'localization_monitor up: budget %.2f m, lost %.2f m, drift %.0f cm + %.0f%%/m flat '
            '.. %.0f cm + %.0f%%/m at %.0f deg, coast %.1f m flat / %.1f m slope / %.0f s, ok '
            'needs ekf sigma <= %.2f m -> %s' % (
                p.drift_budget_m, p.lost_drift_m, 100 * p.drift_base_flat_m,
                100 * p.drift_rate_flat, 100 * p.drift_base_slope_m, 100 * p.drift_rate_slope,
                p.drift_tilt_slope_deg, p.coast_max_flat_m, p.coast_max_slope_m, p.coast_max_s,
                p.loc_sigma_ok_m, g('status_topic')))

    # sampled callbacks (run inside _tick under the lock)
    def _on_fix_status(self, msg, t):
        self.est.on_fix_status(t, msg.data)

    def _on_odom(self, msg, t):
        pos, tw = msg.pose.pose.position, msg.twist.twist
        self.est.on_odom_pose(t, pos.x, pos.y, _yaw(msg.pose.pose.orientation),
                              tw.linear.x, tw.angular.z)

    def _on_map_odom(self, msg, t):
        pos, cov = msg.pose.pose.position, msg.pose.covariance
        self.est.on_map_odom(t, cov[0], cov[7], (pos.x, pos.y, _yaw(msg.pose.pose.orientation)))

    def _on_gps_odom(self, msg, t):
        pos = msg.pose.pose.position
        self.est.on_gps_odom(t, pos.x, pos.y)

    def _on_wheel(self, msg, t):
        self.est.on_wheel_twist(t, msg.twist.angular.z)

    def _on_tilt(self, msg, t):
        try:
            st = json.loads(msg.data)
            self.est.on_tilt(t, float(st['roll_deg']), float(st['pitch_deg']))
        except (ValueError, KeyError, TypeError):
            pass

    def _on_vio(self, _msg, t):
        self.est.on_vio(t)

    def _on_heading(self, msg, _t):
        try:
            self.est.on_heading_status(json.loads(msg.data))
        except ValueError:
            self.est.on_heading_status({})

    def _tick(self):
        now = self._clk_now()
        with self._lock:
            self._pump.poll()
            st = self.est.status(now)
        changed = st['state'] != self._last_state
        if not changed and now - self._last_pub < self._status_period:
            return
        if changed:
            msg = 'localization %s -> %s%s' % (self._last_state, st['state'],
                                               (': ' + st['reason']) if st['reason'] else '')
            # rclpy (Jazzy) raises if one call site changes severity, so keep these apart.
            if st['state'] == lsl.STATE_OK:
                self.get_logger().info(msg)
            else:
                self.get_logger().warn(msg)
        self._last_state = st['state']
        self._last_pub = now
        self._pub.publish(String(data=json.dumps(st, sort_keys=True)))

    def shutdown(self):
        self._periodic.stop()
        self._pump.stop()


def main(args=None):
    try:  # crash records -> /userdata/ros2/crashes (docs/crash_recovery.md)
        from mower_control.crash_record import install as _install_crash_record
        _install_crash_record('localization_monitor')
    except ImportError:
        pass
    rclpy.init(args=args)
    node = LocalizationMonitor()
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
