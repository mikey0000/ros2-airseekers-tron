#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Replay a field bag through robot_localization ``ekf_map`` variants (2026-11-13 RTK dropout).

The field bags do not record the filter inputs (/odom, /imu/data_aligned, /odometry/gps), so
they are rebuilt from what is recorded:

* ``/odom``: /odometry/filtered twist (vx, yaw rate; vy 0) with mcu_node's twist covariance
  (vx ``--vx-var``, vy per variant: 1e6 = unobserved, mcu_node's default before 2026-11-13);
* ``/imu/data_aligned``: the /odometry/filtered yaw (heading_aligner's ENU yaw) and yaw rate;
* ``/odometry/gps``: /fix in the map frame, classified by /fix_status and given gps_gate's
  covariance (``rtk_quality.fused_variance``: fixed 3 cm floor, float x4 / 0.4 m floor, single
  dropped). The ENU -> map offset and the antenna lever arm are fitted to the recorded
  /odometry/filtered_map over the RTK-fixed fixes.

Each variant runs as its own ``ekf_node`` with ``ekf_dual.yaml``'s ``ekf_map`` parameters
plus the variant's overrides, on sim time; this script publishes /clock and the inputs at
``--speed`` x real time and records every variant's output.

Run in the dev image with the workspace sourced (robot_localization is installed there)::

    python3 scripts/replay/ekf_replay.py BAG_DIR OUT_DIR --t0 EPOCH --t1 EPOCH \
        --variant base: --variant vy:vy_var=0.01,fuse_vy=1

Output: ``OUT_DIR/<variant>.csv`` (t, x, y, yaw), ``OUT_DIR/inputs.csv`` (t, kind, x, y,
var / class) and ``OUT_DIR/fit.json``. Positions are map metres only.
"""

import argparse
import csv
import glob
import json
import math
import os
import subprocess
import sys
import time

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
STACK = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(STACK, 'src', 'mower_localization'))

from mower_localization import rtk_quality   # noqa: E402

EKF_YAML = os.path.join(STACK, 'src', 'mower_localization', 'config', 'ekf_dual.yaml')
TOPICS = ('/odometry/filtered', '/fix', '/fix_status', '/odometry/filtered_map')
EARTH_R = 6378137.0
_TIMES = {}


# ---------------------------------------------------------------------------- bag reading
def read_bag(bag, t0, t1):
    """[(t, topic, msg)] of TOPICS between t0 and t1 (bag receive time, s), time order."""
    import sqlite3
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    dbs = sorted(glob.glob(os.path.join(bag, '*.db3')),
                 key=lambda p: int(p.rsplit('_', 1)[1].split('.')[0]))
    out = []
    for d in dbs:
        c = sqlite3.connect('file:%s?mode=ro' % d, uri=True)
        tps = {r[0]: (r[1], r[2]) for r in c.execute('select id, name, type from topics')}
        ids = [k for k, v in tps.items() if v[0] in TOPICS]
        if not ids:
            continue
        q = 'select topic_id, timestamp, data from messages where topic_id in (%s) and ' \
            'timestamp between ? and ? order by timestamp' % ','.join('?' * len(ids))
        for tid, ts, data in c.execute(q, ids + [int(t0 * 1e9), int(t1 * 1e9)]):
            name, typ = tps[tid]
            out.append((ts / 1e9, name, deserialize_message(data, get_message(typ))))
    out.sort(key=lambda r: r[0])
    return out


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def interp(series, t):
    """Linear interpolation in [(t, x, y, yaw)] (sorted); None outside."""
    import bisect
    ts = _TIMES.get(id(series))
    if ts is None or len(ts) != len(series):
        ts = _TIMES[id(series)] = [s[0] for s in series]
    k = bisect.bisect_left(ts, t)
    if k <= 0 or k >= len(series):
        return None
    a, b = series[k - 1], series[k]
    if b[0] - a[0] > 0.5:
        return None
    f = (t - a[0]) / (b[0] - a[0])
    dyaw = math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3]))
    return (a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), a[3] + f * dyaw)


# ---------------------------------------------------------------------------- inputs
def build_inputs(msgs):
    """(odom [(t, vx, wz)], imu [(t, yaw, wz)], fixes [(t, e, n, class, rx_var)],
    field_map [(t, x, y, yaw)])."""
    odom, imu, fixes, field = [], [], [], []
    cls, lat0, lon0 = None, None, None
    for t, name, m in msgs:
        if name == '/fix_status':
            c = rtk_quality.classify_fix_status(m.data)
            if c is not None:
                cls = c
        elif name == '/odometry/filtered':
            vx, wz = m.twist.twist.linear.x, m.twist.twist.angular.z
            odom.append((t, vx, wz))
            imu.append((t, yaw_of(m.pose.pose.orientation), wz))
        elif name == '/odometry/filtered_map':
            p = m.pose.pose
            field.append((t, p.position.x, p.position.y, yaw_of(p.orientation)))
        elif name == '/fix':
            if not (math.isfinite(m.latitude) and math.isfinite(m.longitude)):
                continue
            if lat0 is None:
                lat0, lon0 = m.latitude, m.longitude
            e = math.radians(m.longitude - lon0) * EARTH_R * math.cos(math.radians(lat0))
            n = math.radians(m.latitude - lat0) * EARTH_R
            cov = m.position_covariance
            rx = max(cov[0], cov[4]) if m.position_covariance_type else None
            fixes.append((t, e, n, cls, rx))
    return odom, imu, fixes, field


def fit_enu_to_map(fixes, field):
    """Least squares map = ENU + (tx, ty) - R(yaw) (lx, ly) over the RTK-fixed fixes."""
    import numpy as np
    rows, rhs = [], []
    for t, e, n, c, _ in fixes:
        if c != rtk_quality.RTK_FIXED:
            continue
        p = interp(field, t)
        if p is None:
            continue
        cy, sy = math.cos(p[2]), math.sin(p[2])
        rows.append([1.0, 0.0, -cy, sy])
        rhs.append(p[0] - e)
        rows.append([0.0, 1.0, -sy, -cy])
        rhs.append(p[1] - n)
    a, b = np.array(rows), np.array(rhs)
    sol, *_ = np.linalg.lstsq(a, b, rcond=None)
    res = a @ sol - b
    return {'tx': sol[0], 'ty': sol[1], 'lever_x': sol[2], 'lever_y': sol[3],
            'n': len(rhs) // 2, 'resid_rms_m': float(np.sqrt(np.mean(res ** 2)))}


def gps_stream(fixes, field, fit):
    """[(t, x, y, var, class)] of the fixes gps_gate passes, in the map frame."""
    out = []
    for t, e, n, c, rx in fixes:
        var = rtk_quality.fused_variance(c, rx)
        p = interp(field, t)
        if var is None or p is None:
            out.append((t, None, None, None, c))
            continue
        cy, sy = math.cos(p[2]), math.sin(p[2])
        x = e + fit['tx'] - (cy * fit['lever_x'] - sy * fit['lever_y'])
        y = n + fit['ty'] - (sy * fit['lever_x'] + cy * fit['lever_y'])
        out.append((t, x, y, var, c))
    return out


# ---------------------------------------------------------------------------- variants
def parse_variant(text):
    """'name:key=val,key=val' -> (name, {vy_var, fuse_vy})."""
    name, _, rest = text.partition(':')
    opts = {'vy_var': 1e6, 'fuse_vy': 0}
    for kv in filter(None, rest.split(',')):
        k, _, v = kv.partition('=')
        opts[k.strip()] = float(v)
    return name, opts


def write_params(out_dir, name, opts):
    with open(EKF_YAML) as fh:
        base = yaml.safe_load(fh)['ekf_map']['ros__parameters']
    p = dict(base)
    p.update({'use_sim_time': True, 'publish_tf': False, 'print_diagnostics': False,
              'odom0': '/rp/%s/odom' % name, 'imu0': '/rp/imu', 'odom1': '/rp/gps'})
    cfg = list(p['odom0_config'])
    cfg[7] = bool(opts.get('fuse_vy'))
    p['odom0_config'] = cfg
    path = os.path.join(out_dir, 'params_%s.yaml' % name)
    with open(path, 'w') as fh:
        yaml.safe_dump({'/**': {'ros__parameters': p}}, fh)
    return path


# ---------------------------------------------------------------------------- playback
def play(odom, imu, gps, variants, speed, vx_var, out_dir):
    import rclpy
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Imu
    rclpy.init()
    node = rclpy.create_node('ekf_replay_player')
    pub_clock = node.create_publisher(Clock, '/clock', 10)
    pub_imu = node.create_publisher(Imu, '/rp/imu', 50)
    pub_gps = node.create_publisher(Odometry, '/rp/gps', 50)
    pub_odom = {n: node.create_publisher(Odometry, '/rp/%s/odom' % n, 50) for n, _ in variants}
    rec = {n: [] for n, _ in variants}

    def on_out(name):
        def cb(m):
            p = m.pose.pose
            rec[name].append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                              p.position.x, p.position.y, yaw_of(p.orientation)))
        return cb
    for n, _ in variants:
        node.create_subscription(Odometry, '/rp_%s/odometry/filtered' % n, on_out(n), 100)

    def stamp(msg, t):
        msg.header.stamp.sec = int(t)
        msg.header.stamp.nanosec = int(round((t - int(t)) * 1e9)) % 1000000000

    events = [(t, 'odom', r) for t, *r in odom] + [(t, 'imu', r) for t, *r in imu] + \
        [(t, 'gps', r) for t, *r in gps if r[0] is not None]
    events.sort(key=lambda e: e[0])
    deadline = time.time() + 60.0
    while time.time() < deadline:
        if all(node.count_subscribers('/rp/%s/odom' % n) for n, _ in variants) and \
                node.count_subscribers('/rp/gps') >= len(variants):
            break
        rclpy.spin_once(node, timeout_sec=0.1)
    wall0, sim0 = time.time(), events[0][0]
    for t, kind, r in events:
        while (t - sim0) / speed > time.time() - wall0:
            rclpy.spin_once(node, timeout_sec=0.002)
        c = Clock()
        c.clock.sec, c.clock.nanosec = int(t), int(round((t - int(t)) * 1e9)) % 1000000000
        pub_clock.publish(c)
        if kind == 'odom':
            for n, opts in variants:
                m = Odometry()
                stamp(m, t)
                m.header.frame_id, m.child_frame_id = 'odom', 'base_link'
                m.twist.twist.linear.x, m.twist.twist.angular.z = r[0], r[1]
                cov = [0.0] * 36
                cov[0], cov[7], cov[14], cov[21], cov[28], cov[35] = \
                    vx_var, opts['vy_var'], 1e6, 1e6, 1e6, 0.05
                m.twist.covariance = cov
                pub_odom[n].publish(m)
        elif kind == 'imu':
            m = Imu()
            stamp(m, t)
            m.header.frame_id = 'base_link'
            m.orientation.z, m.orientation.w = math.sin(r[0] / 2.0), math.cos(r[0] / 2.0)
            m.orientation_covariance = [1e-3, 0, 0, 0, 1e-3, 0, 0, 0, 1e-3]
            m.angular_velocity.z = r[1]
            m.angular_velocity_covariance = [1e-4, 0, 0, 0, 1e-4, 0, 0, 0, 1e-4]
            m.linear_acceleration_covariance = [-1.0] + [0.0] * 8
            pub_imu.publish(m)
        else:
            x, y, var, _c = r
            m = Odometry()
            stamp(m, t)
            m.header.frame_id, m.child_frame_id = 'map', 'base_link'
            m.pose.pose.position.x, m.pose.pose.position.y = x, y
            m.pose.pose.orientation.w = 1.0
            cov = [0.0] * 36
            cov[0], cov[7], cov[14], cov[21], cov[28], cov[35] = var, var, 1e6, 1e6, 1e6, 1e6
            m.pose.covariance = cov
            pub_gps.publish(m)
    end = time.time() + 2.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()
    for n, rows in rec.items():
        with open(os.path.join(out_dir, '%s.csv' % n), 'w', newline='') as fh:
            w = csv.writer(fh)
            w.writerow(['t', 'x', 'y', 'yaw'])
            w.writerows(rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bag')
    ap.add_argument('out')
    ap.add_argument('--t0', type=float, required=True)
    ap.add_argument('--t1', type=float, required=True)
    ap.add_argument('--variant', action='append', required=True,
                    help='name:vy_var=V,fuse_vy=0|1 (repeat)')
    ap.add_argument('--speed', type=float, default=4.0)
    ap.add_argument('--vx-var', type=float, default=0.02)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    msgs = read_bag(a.bag, a.t0, a.t1)
    odom, imu, fixes, field = build_inputs(msgs)
    fit = fit_enu_to_map(fixes, field)
    gps = gps_stream(fixes, field, fit)
    with open(os.path.join(a.out, 'fit.json'), 'w') as fh:
        json.dump(fit, fh, indent=1)
    with open(os.path.join(a.out, 'inputs.csv'), 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['t', 'kind', 'x', 'y', 'v1', 'v2'])
        w.writerows([(t, 'field', x, y, yaw, '') for t, x, y, yaw in field])
        w.writerows([(t, 'gps', x, y, var, c) for t, x, y, var, c in gps])
        w.writerows([(t, 'odom', '', '', vx, wz) for t, vx, wz in odom])
    variants = [parse_variant(v) for v in a.variant]
    procs = []
    try:
        for n, opts in variants:
            pf = write_params(a.out, n, opts)
            procs.append(subprocess.Popen(
                ['ros2', 'run', 'robot_localization', 'ekf_node', '--ros-args',
                 '--params-file', pf, '-r', '__node:=ekf_%s' % n, '-r', '__ns:=/rp_%s' % n],
                stdout=subprocess.DEVNULL, stderr=open(os.path.join(a.out, 'ekf_%s.log' % n),
                                                       'w')))
        time.sleep(3.0)
        play(odom, imu, gps, variants, a.speed, a.vx_var, a.out)
    finally:
        for p in procs:
            p.terminate()
        for p in procs:
            p.wait(timeout=10)
    print(json.dumps(fit))


if __name__ == '__main__':
    main()
