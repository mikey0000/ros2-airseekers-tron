#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""heading_aligner: puts the JY61P's gyro-integrated yaw into ENU for the EKF / Nav2.

See :mod:`mower_localization.heading_logic` for the estimator (COG / dock / file sources).

Subscribed
----------
``/imu/data``                       sensor_msgs/Imu        100 Hz, event (pump thread)
``/cmd_vel``                        geometry_msgs/Twist    commanded motion (post-mux/slew)
``/odometry/gps``                   nav_msgs/Odometry      navsat output, map/ENU (= odom)
``/gps/status``                     mowgli GnssStatus      fix_type (2 float, 3 fixed)
``/mower_base/status``              MowerBaseDevStatus     is_charging / is_docking_done
``/map_server_node/docking_pose``   PoseStamped (latched)  dock pose; fallback ``dock_pose_file``
``/map_server_node/docking_pose_measured``  Bool (latched)  dock pose recorded by set_docking_point;
                                    sets ``dock_yaw_trusted`` (the param = true forces it on)

Published
---------
``/imu/data_aligned``        sensor_msgs/Imu  = /imu/data with orientation rotated by the
                             offset about world Z and orientation_covariance[8] (yaw) set
                             from the alignment quality (huge while unaligned). Bytes are
                             patched in place (no message objects at 100 Hz).
``/heading_aligner/status``  std_msgs/String JSON, latched, 1 Hz + on change:
                             {aligned, source: none|file|dock|cog, offset_deg, quality, ...}

Persisted: ``offset_file`` (default /userdata/ros2/heading_offset.yaml) on every accepted COG
update; loaded on boot as the low-quality ``file`` source.

CPU: inputs go through sub_pump (no executor wakes); only /imu/data is event-driven, the rest
are sampled by one PeriodicRunner thread.
"""

import json
import math
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String

from mower_localization import heading_logic as hl
from mower_localization.sub_pump import (PeriodicRunner, SubscriptionPump, flat_parser,
                                         parse_odometry)

try:
    from mowgli_interfaces.msg import GnssStatus
except ImportError:  # pragma: no cover
    GnssStatus = None
try:
    from mower_interfaces.msg import MowerBaseDevStatus
except ImportError:  # pragma: no cover
    MowerBaseDevStatus = None

DEG_PARAMS = ('max_imu_yaw_change', 'outlier', 'reseed_agree', 'dock_heading_offset',
              'imu_jump', 'sigma_cog', 'sigma_dock', 'sigma_file', 'sigma_file_verified',
              'verify_max_yaw')


def _read_dock_measured(path):
    """True when mower_map's dock_pose.yaml carries ``dock_pose_measured: true``."""
    try:
        with open(path) as fh:
            for line in fh:
                key, _, value = line.partition(':')
                if key.strip() == 'dock_pose_measured':
                    return value.split('#')[0].strip().lower() == 'true'
    except OSError:
        pass
    return False


def _read_dock_file(path):
    """(x, y, yaw) from mower_map's dock_pose.yaml, or None."""
    vals = {}
    try:
        with open(path) as fh:
            for line in fh:
                key, _, value = line.partition(':')
                if key.strip() in ('dock_pose_x', 'dock_pose_y', 'dock_pose_yaw'):
                    vals[key.strip()] = float(value)
    except (OSError, ValueError):
        return None
    if len(vals) != 3:
        return None
    return vals['dock_pose_x'], vals['dock_pose_y'], vals['dock_pose_yaw']


def _parse_twist(data):
    if len(data) < 52 or data[0] != 0 or data[1] != 1:
        return None
    v = struct.unpack_from('<6d', data, 4)
    return v[0], v[5]


class HeadingAligner(Node):

    def __init__(self):
        super().__init__('heading_aligner')
        d = self.declare_parameter
        d('imu_topic', '/imu/data')
        d('output_topic', '/imu/data_aligned')
        d('status_topic', '/heading_aligner/status')
        d('cmd_vel_topic', '/cmd_vel')
        d('gps_odom_topic', '/odometry/gps')
        d('gnss_status_topic', '/gps/status')
        d('base_status_topic', '/mower_base/status')
        d('dock_pose_topic', '/map_server_node/docking_pose')
        d('dock_pose_file', '/ros2_ws/maps/dock_pose.yaml')
        d('dock_pose_measured_topic', '/map_server_node/docking_pose_measured')
        d('offset_file', '/userdata/ros2/heading_offset.yaml')
        d('use_offset_file', True)
        d('use_dock_seed', True)
        d('input_rate', 20.0)        # Hz sampling of cmd / gps
        d('status_rate', 1.0)
        d('persist_period', 30.0)    # s; re-save offset + IMU yaw + position while aligned
        defaults = hl.AlignParams()
        for name, value in vars(defaults).items():
            d(name, float(math.degrees(value)) if name in DEG_PARAMS else value)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        kw = {}
        for name, value in vars(defaults).items():
            v = g(name)
            kw[name] = math.radians(v) if name in DEG_PARAMS else type(value)(v)
        self.est = hl.HeadingEstimator(hl.AlignParams(**kw))
        # dock_yaw_trusted: param true = forced override; otherwise follows mower_map's
        # dock_pose_measured flag (topic, file fallback at startup).
        self._dock_trust_forced = bool(kw.get('dock_yaw_trusted', False))
        self._dock_measured = False
        self._lock = threading.Lock()
        self._offset_file = g('offset_file')
        self._use_dock = bool(g('use_dock_seed'))
        self._last_status = None
        self._persist_period = float(g('persist_period'))
        self._persisted_at = time.monotonic()
        self._persisted_xy = None     # position in the last saved record (re-save after 0.3 m)
        self._persisted_yaw = None    # IMU yaw in the last saved record (re-save after 5 deg)
        self._tail = None             # cached orientation offset (frame_id length fixed)
        self._tail_key = None

        if g('use_offset_file'):
            rec = hl.load_record(self._offset_file)
            off = None if rec is None else rec['offset']
            if off is not None:
                self.est.load_persisted(off, time.monotonic(), record=rec)
                self.get_logger().info('persisted heading offset %.1f deg from %s (low quality '
                                       'until dock/COG)' % (math.degrees(off), self._offset_file))

        rel = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                         history=QoSHistoryPolicy.KEEP_LAST)
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._imu_pub = self.create_publisher(Imu, g('output_topic'), rel)
        self._status_pub = self.create_publisher(String, g('status_topic'), latched)

        self._pump = SubscriptionPump(self, 'heading_aligner_inputs')
        self._pump.subscribe(Imu, g('imu_topic'), self._on_imu, rel, raw=True)
        sampled = dict(sampled=True)
        one = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                         history=QoSHistoryPolicy.KEEP_LAST)
        self._cmd_sub = self._pump.subscribe(Twist, g('cmd_vel_topic'), self._on_cmd, one,
                                             raw=True, **sampled)
        self._gps_sub = self._pump.subscribe(
            Odometry, g('gps_odom_topic'), self._on_gps,
            QoSProfile(depth=20, reliability=QoSReliabilityPolicy.RELIABLE,
                       history=QoSHistoryPolicy.KEEP_LAST),
            parser=parse_odometry, sampled=True, deliver_all=True)
        slow = []
        if GnssStatus is not None:
            slow.append(self._pump.subscribe(GnssStatus, g('gnss_status_topic'), self._on_gnss,
                                             one, **sampled))
        else:
            self.get_logger().warn('mowgli_interfaces missing: no fix_type, no COG')
        if MowerBaseDevStatus is not None:
            slow.append(self._pump.subscribe(MowerBaseDevStatus, g('base_status_topic'),
                                             self._on_base, one,
                                             parser=flat_parser(MowerBaseDevStatus), **sampled))
        slow.append(self._pump.subscribe(PoseStamped, g('dock_pose_topic'), self._on_dock_pose,
                                         latched, **sampled))
        slow.append(self._pump.subscribe(Bool, g('dock_pose_measured_topic'),
                                         self._on_dock_measured, latched, **sampled))
        self._slow_subs = slow
        if g('dock_pose_file'):
            self._apply_dock_measured(_read_dock_measured(g('dock_pose_file')), 'file')
        self._dock_logged = None
        dock = _read_dock_file(g('dock_pose_file')) if g('dock_pose_file') else None
        if dock is not None:
            self._set_dock(dock, 'file %s' % g('dock_pose_file'))

        self._periodic = PeriodicRunner(self, [
            (1.0 / max(1.0, float(g('input_rate'))), self._fast_tick),
            (0.5, self._slow_tick),
            (1.0 / max(0.1, float(g('status_rate'))), self._status_tick),
        ], 'heading_aligner_periodic')
        self._pump.start()
        self._periodic.start()
        self.get_logger().info('heading_aligner up: %s -> %s' % (g('imu_topic'),
                                                                   g('output_topic')))

    # ------------------------------------------------------------------ IMU (pump thread)
    def _on_imu(self, data):
        now = time.monotonic()
        key = bytes(data[12:16])
        if key != self._tail_key:
            self._tail = hl.imu_tail_offset(data)
            self._tail_key = key
        pos = self._tail
        if pos is None:
            return
        q, gz = hl.imu_fields(data, pos)
        with self._lock:
            est = self.est
            if q == (0.0, 0.0, 0.0, 0.0):
                out = data           # no orientation at all: pass through
            else:
                est.on_imu(now, hl.yaw_from_quaternion(*q), gz)
                off = est.offset
                if off is not None:
                    q = hl.rotate_yaw(q, off)
                s = est.yaw_sigma(now)
                out = hl.imu_rewrite(data, pos, q, s * s)
        self._imu_pub.publish(out)

    # ------------------------------------------------------------------ sampled inputs
    def _on_cmd(self, data):
        v = _parse_twist(data)
        if v is not None:
            self.est.on_cmd(time.monotonic(), v[0], v[1])

    def _on_gps(self, msg):
        p = msg.pose.pose.position
        res = self.est.on_gps(time.monotonic(), p.x, p.y, msg.pose.covariance[0])
        if res is not None:
            self._report(res)

    def _on_gnss(self, msg):
        self.est.on_fix_type(time.monotonic(), int(msg.fix_type))

    def _on_base(self, msg):
        docked = bool(msg.is_charging) or bool(msg.is_docking_done)
        if self._use_dock:
            before = self.est.source
            self.est.on_docked(time.monotonic(), docked)
            if self.est.source != before:
                self.get_logger().info(self.est.last_event)
        else:
            self.est.docked = docked

    def _on_dock_pose(self, msg):
        q = msg.pose.orientation
        self._set_dock((msg.pose.position.x, msg.pose.position.y,
                        hl.yaw_from_quaternion(q.x, q.y, q.z, q.w)), 'topic')

    def _on_dock_measured(self, msg):
        self._apply_dock_measured(bool(msg.data), 'topic')

    def _apply_dock_measured(self, measured, origin):
        changed = measured != self._dock_measured
        self._dock_measured = measured
        self.est.p.dock_yaw_trusted = self._dock_trust_forced or measured
        if changed or origin == 'file':
            self.get_logger().info('dock pose measured=%s (%s) -> dock_yaw_trusted=%s%s' % (
                measured, origin, self.est.p.dock_yaw_trusted,
                ' (forced by param)' if self._dock_trust_forced else ''))

    def _set_dock(self, pose, origin):
        x, y, yaw = pose
        self.est.on_dock_pose(x, y, yaw)
        if pose == self._dock_logged:
            return
        self._dock_logged = pose
        self.get_logger().info('dock pose from %s: (%.2f, %.2f) yaw %.1f deg (docked robot '
                               'heading)' % (origin, x, y, math.degrees(yaw)))
        if yaw == 0.0:
            self.get_logger().warn(
                'dock pose yaw is exactly 0.0: if it was never measured the dock seed is '
                'wrong (COG will replace it after the first straight drive; the undock COG is '
                'logged as the measured dock heading)')

    def _report(self, res):
        log = self.get_logger()
        if res.get('accepted'):
            log.info('%s (COG %.1f deg over %.2f m, lateral %.3f m)' % (
                self.est.last_event, math.degrees(res['cog']), res['distance'], res['lateral']))
        else:
            log.info('COG window not used: %s' % res.get('reason', '?'))

    # ------------------------------------------------------------------ ticks
    def _fast_tick(self):
        with self._lock:
            self._pump.poll([self._cmd_sub, self._gps_sub])

    def _slow_tick(self):
        with self._lock:
            self._pump.poll(self._slow_subs)

    def _status_tick(self):
        now = time.monotonic()
        with self._lock:
            st = self.est.status(now)
            dirty, self.est.dirty = self.est.dirty, False
            rec = self.est.persist_record()
            src, n = self.est.source, self.est.n_cog
        moved = rec is not None and rec[2] is not None and (
            self._persisted_xy is None
            or math.hypot(rec[2] - self._persisted_xy[0], rec[3] - self._persisted_xy[1]) > 0.3)
        turned = rec is not None and (
            self._persisted_yaw is None
            or abs(hl.wrap(rec[1] - self._persisted_yaw)) > math.radians(5.0))
        if rec is not None and (dirty or moved or turned
                                or now - self._persisted_at >= self._persist_period):
            # Save on change, after 0.3 m of travel and periodically: the continuity check on
            # the next boot compares against the LAST resting position, so a restart right
            # after a drive must not see a record from before it.
            self._persisted_at = now
            self._persisted_xy = (rec[2], rec[3]) if rec[2] is not None else self._persisted_xy
            self._persisted_yaw = rec[1]
            try:
                hl.save_offset(self._offset_file, rec[0], src, n, imu_yaw=rec[1], x=rec[2],
                               y=rec[3])
            except OSError as exc:
                self.get_logger().warn('cannot persist heading offset: %s' % exc)
        key = (st['aligned'], st['source'], st['offset_deg'], st['quality'], st['event'])
        text = hl.status_json(st)
        self._status_pub.publish(String(data=text))
        if key != self._last_status:
            self._last_status = key
            self.get_logger().info('heading status: %s' % json.dumps(
                {k: st[k] for k in ('aligned', 'source', 'offset_deg', 'quality', 'event')}))

    def shutdown(self):
        self._periodic.stop()
        self._pump.stop()
        # final save so a container restart can verify continuity from the exact
        # last IMU yaw / position (not one up to persist_period old)
        with self._lock:
            rec = self.est.persist_record()
            src, n = self.est.source, self.est.n_cog
        if rec is not None:
            try:
                hl.save_offset(self._offset_file, rec[0], src, n, imu_yaw=rec[1], x=rec[2],
                               y=rec[3])
            except OSError:
                pass


def main(args=None):
    try:  # crash records -> /userdata/ros2/crashes (docs/crash_recovery.md)
        from mower_control.crash_record import install as _install_crash_record
        _install_crash_record('heading_aligner')
    except ImportError:
        pass
    rclpy.init(args=args)
    node = HeadingAligner()
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
