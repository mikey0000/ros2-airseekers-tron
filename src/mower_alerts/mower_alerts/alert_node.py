# SPDX-License-Identifier: GPL-3.0-or-later
"""``alert_node``: theft, lift and incident alerts (rclpy glue around rules.py).

Inputs (all existing topics, nothing is added to the mission or the drivers):

* ``/behavior_tree_node/high_level_status`` (mower_mission): state, state_name, sub state,
  battery, emergency;
* ``/mower_base/status`` (mcu_node, ~100 Hz, sampled): lift, stop button, dock contact,
  charging;
* ``/hardware_bridge/emergency`` (gui_bridge, sampled): emergency reason text;
* ``/fix`` (um960_node, sampled): position for the parked-spot geofence;
* ``/gps/status`` (gui_bridge GnssStatus, sampled): RTK fix type;
* ``/odometry/filtered_map`` (sampled) + latched ``/keepout_mask`` (mower_map): mapped area;
* ``/imu/data`` (wit_node, sampled): tilt.

Output: ``/mower_alerts/events`` (std_msgs/String JSON, see README), read by the MowgliNext
GUI backend, which pushes them over the operator's notification channel and its MQTT broker.
Everything is evaluated on one ``tick_hz`` timer; the sampled inputs cost one take per tick.
"""

import json
import math
import os
import threading
import time

import rclpy
from mower_interfaces.msg import MowerBaseDevStatus
from mowgli_interfaces.msg import Emergency, GnssStatus, HighLevelStatus
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String

from mower_alerts import rules
from mower_alerts.sub_pump import PeriodicRunner, SubscriptionPump

TOPIC_DEFAULTS = {
    'high_level_status_topic': '/behavior_tree_node/high_level_status',
    'base_status_topic': '/mower_base/status',
    'emergency_topic': '/hardware_bridge/emergency',
    'fix_topic': '/fix',
    'gnss_status_topic': '/gps/status',
    'pose_topic': '/odometry/filtered_map',
    'mask_topic': '/keepout_mask',
    'imu_topic': '/imu/data',
    'events_topic': '/mower_alerts/events',
}
NODE_DEFAULTS = {
    'tick_hz': 2.0,
    'heartbeat_s': 10.0,
    'fix_stale_s': 5.0,
    'hl_stale_s': 10.0,
    'tilt_enabled': True,
    'anchor_path': '~/.ros/mower_alerts/parked_anchor.json',
}


def _expand(path):
    return os.path.abspath(os.path.expanduser(os.path.expandvars(path))) if path else ''


def load_anchor(path):
    try:
        with open(path) as f:
            d = json.load(f)
        return rules.Fix(float(d['lat']), float(d['lon']), float(d['accuracy_m']))
    except (OSError, ValueError, KeyError, TypeError):
        return None


def save_anchor(path, anchor):
    """Atomic write; None removes the file."""
    if not path:
        return
    if anchor is None:
        try:
            os.remove(path)
        except OSError:
            pass
        return
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump({'lat': anchor.lat, 'lon': anchor.lon, 'accuracy_m': anchor.accuracy_m}, f)
    os.replace(tmp, path)


def alert_payload(alert, wall_time):
    return {'type': 'alert', 'id': '%s-%d' % (alert.incident, int(wall_time * 1000)),
            'incident': alert.incident, 'kind': alert.kind, 'message': alert.message,
            'priority': int(alert.priority), 'text': alert.text,
            'params': {k: str(v) for k, v in alert.params.items()}, 'stamp': wall_time}


def heartbeat_payload(active, wall_time):
    return {'type': 'heartbeat', 'active': list(active), 'stamp': wall_time}


def mask_from_grid(msg):
    info = msg.info
    return rules.MapMask(info.origin.position.x, info.origin.position.y, info.resolution,
                         info.width, info.height, msg.data)


class AlertNode(Node):

    def __init__(self, **kwargs):
        super().__init__('alert_node', **kwargs)
        defaults = rules.Params()
        names = list(defaults.__dataclass_fields__)
        for name in names:
            self.declare_parameter(name, getattr(defaults, name))
        for name, default in {**TOPIC_DEFAULTS, **NODE_DEFAULTS}.items():
            self.declare_parameter(name, default)
        gp = self.get_parameter
        self._p = {n: gp(n).value for n in list(TOPIC_DEFAULTS) + list(NODE_DEFAULTS)}
        params = rules.Params(**{n: gp(n).value for n in names})
        self.engine = rules.AlertEngine(params)
        self._anchor_path = _expand(self._p['anchor_path'])
        self.engine.restore_anchor(load_anchor(self._anchor_path))

        self._lock = threading.Lock()
        self._hl = None             # (msg, receipt)
        self._base = None
        self._emergency = None
        self._fix = None
        self._gnss = None
        self._pose = None
        self._imu = None
        self._last_heartbeat = 0.0

        self._pub = self.create_publisher(String, self._p['events_topic'], 50)
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        p = self._p
        self._pump = SubscriptionPump(self, 'alert_inputs')
        sub = self._pump.subscribe
        sub(HighLevelStatus, p['high_level_status_topic'], self._on_hl, 10, lock=self._lock)
        sub(OccupancyGrid, p['mask_topic'], self._on_mask, latched)
        sampled = dict(sampled=True, with_receipt=True, lock=self._lock)
        sub(MowerBaseDevStatus, p['base_status_topic'], self._store('_base'), 1, **sampled)
        sub(Emergency, p['emergency_topic'], self._store('_emergency'), 1, **sampled)
        sub(NavSatFix, p['fix_topic'], self._store('_fix'), 1, **sampled)
        sub(GnssStatus, p['gnss_status_topic'], self._store('_gnss'), 1, **sampled)
        sub(Odometry, p['pose_topic'], self._store('_pose'), 1, **sampled)
        if bool(p['tilt_enabled']):
            sub(Imu, p['imu_topic'], self._store('_imu'), qos_profile_sensor_data, **sampled)
        self._pump.start()
        self._periodic = PeriodicRunner(
            self, [(1.0 / max(0.2, float(p['tick_hz'])), self._tick)], 'alert_tick')
        self._periodic.start()
        self.get_logger().info(
            'alerts %s: geofence %.0f m, map margin %.1f m, theft armed after %.0f s parked, '
            'anchor %s' % ('on' if params.enabled else 'OFF', params.geofence_radius_m,
                           params.map_exit_margin_m, params.theft_arm_s,
                           'restored' if self.engine.anchor is not None else 'none'))

    # ---- inputs -------------------------------------------------------------
    def _store(self, attr):
        def cb(msg, receipt):
            setattr(self, attr, (msg, receipt))
        return cb

    def _on_hl(self, msg):
        self._hl = (msg, time.monotonic())

    def _on_mask(self, msg):
        try:
            mask = mask_from_grid(msg)
        except Exception as ex:  # noqa: BLE001
            self.get_logger().warn('keepout mask unusable: %s' % ex)
            return
        with self._lock:
            self.engine.set_mask(mask)

    def _fresh(self, entry, now, max_age):
        if entry is None or now - entry[1] > max_age:
            return None
        return entry[0]

    def inputs(self, now):
        """Snapshot of the cached inputs (call with self._lock held)."""
        stale = float(self.engine.p.stale_s)
        i = rules.Inputs()
        hl = self._fresh(self._hl, now, float(self._p['hl_stale_s']))
        if hl is not None:
            i.state = int(hl.state)
            i.state_name = str(hl.state_name)
            i.sub_state_name = str(hl.sub_state_name)
            b = float(hl.battery_percent)
            i.battery_percent = b if math.isfinite(b) else None
            i.hl_emergency = bool(hl.emergency)
            i.hl_charging = bool(hl.is_charging)
        base = self._fresh(self._base, now, stale)
        if base is not None:
            i.lift = bool(base.lift_triggered)
            i.stop_button = bool(base.stop_triggered)
            i.docked = bool(base.is_docking_done)
            i.charging = bool(base.is_charging)
        em = self._fresh(self._emergency, now, stale)
        if em is not None:
            i.emergency_reason = str(em.reason)
        gnss = self._fresh(self._gnss, now, float(self._p['fix_stale_s']))
        if gnss is not None:
            i.rtk_fix_type = int(gnss.fix_type)
        fix = self._fresh(self._fix, now, float(self._p['fix_stale_s']))
        if fix is not None and fix.status.status >= 0 \
                and math.isfinite(fix.latitude) and math.isfinite(fix.longitude):
            acc = rules.accuracy_from_covariance(fix.position_covariance,
                                                 fix.position_covariance_type)
            if not math.isfinite(acc) and gnss is not None \
                    and float(gnss.horizontal_accuracy_m) > 0.0:
                acc = float(gnss.horizontal_accuracy_m)
            i.fix = rules.Fix(float(fix.latitude), float(fix.longitude), acc, i.rtk_fix_type)
        pose = self._fresh(self._pose, now, stale)
        if pose is not None:
            i.pose = (float(pose.pose.pose.position.x), float(pose.pose.pose.position.y))
        imu = self._fresh(self._imu, now, stale)
        if imu is not None:
            q = imu.orientation
            i.tilt_deg = rules.tilt_from_quaternion(q.x, q.y, q.z, q.w)
        return i

    # ---- tick ---------------------------------------------------------------
    def _tick(self):
        self._pump.poll()
        now = time.monotonic()
        with self._lock:
            alerts = self.engine.step(self.inputs(now), now)
            anchor_changed = self.engine.anchor_changed
            self.engine.anchor_changed = False
            anchor = self.engine.anchor
            active = self.engine.active_incidents()
        wall = time.time()
        for a in alerts:
            self.get_logger().warn('ALERT %s: %s' % (a.incident, a.text))
            self._pub.publish(String(data=json.dumps(alert_payload(a, wall))))
        if anchor_changed:
            try:
                save_anchor(self._anchor_path, anchor)
            except OSError as ex:
                self.get_logger().warn('cannot persist parked anchor: %s' % ex,
                                       throttle_duration_sec=300.0)
        if alerts or now - self._last_heartbeat >= float(self._p['heartbeat_s']):
            self._last_heartbeat = now
            self._pub.publish(String(data=json.dumps(heartbeat_payload(active, wall))))

    def stop(self):
        self._periodic.stop()
        self._pump.stop()


def main(args=None):
    rclpy.init(args=args)
    node = AlertNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
