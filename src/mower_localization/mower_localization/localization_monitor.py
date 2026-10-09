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
``/odometry/vio_gated``      nav_msgs/Odometry     vio_gate output: VIO is being fused
``/heading_aligner/status``  std_msgs/String JSON  heading alignment (reported only)

Published
---------
``/localization/status``     std_msgs/String JSON, latched, 1 Hz + on state change:
    {state: ok|degraded|lost, reason, rtk: fixed|float|dgps|single|none|stale,
     since_fixed_s, dist_since_fixed_m, est_drift_m, drift_budget_m, lost_drift_m,
     ekf_xy_sigma_m, vio_active, heading_aligned, heading_source, heading_sigma_deg, ...}
"""

import json
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from mower_localization import localization_status_logic as lsl
from mower_localization.sub_pump import PeriodicRunner, SubscriptionPump, parse_odometry


class LocalizationMonitor(Node):

    def __init__(self):
        super().__init__('localization_monitor')
        d = self.declare_parameter
        d('status_topic', '/localization/status')
        d('fix_status_topic', '/fix_status')
        d('odom_topic', '/odometry/filtered')
        d('map_odom_topic', '/odometry/filtered_map')
        d('vio_topic', '/odometry/vio_gated')
        d('heading_status_topic', '/heading_aligner/status')
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
        self._periodic = PeriodicRunner(self, [
            (1.0 / max(1.0, float(g('input_rate'))), self._tick),
        ], 'localization_monitor_periodic')
        self._pump.start()
        self._periodic.start()
        p = self.est.p
        self.get_logger().info(
            'localization_monitor up: budget %.2f m, lost %.2f m, drift %.1f%% wheel / %.1f%% '
            'VIO -> %s' % (p.drift_budget_m, p.lost_drift_m, 100 * p.drift_rate_wheel,
                           100 * p.drift_rate_vio, g('status_topic')))

    # sampled callbacks (run inside _tick under the lock)
    def _on_fix_status(self, msg, t):
        self.est.on_fix_status(t, msg.data)

    def _on_odom(self, msg, t):
        pos = msg.pose.pose.position
        self.est.on_odom_pose(t, pos.x, pos.y)

    def _on_map_odom(self, msg, t):
        cov = msg.pose.covariance
        self.est.on_map_odom(t, cov[0], cov[7])

    def _on_vio(self, _msg, t):
        self.est.on_vio(t)

    def _on_heading(self, msg, _t):
        try:
            self.est.on_heading_status(json.loads(msg.data))
        except ValueError:
            self.est.on_heading_status({})

    def _tick(self):
        now = time.monotonic()
        with self._lock:
            self._pump.poll()
            st = self.est.status(now)
        changed = st['state'] != self._last_state
        if not changed and now - self._last_pub < self._status_period:
            return
        if changed:
            log = self.get_logger().info if st['state'] == lsl.STATE_OK else \
                self.get_logger().warn
            log('localization %s -> %s%s' % (self._last_state, st['state'],
                                             (': ' + st['reason']) if st['reason'] else ''))
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
