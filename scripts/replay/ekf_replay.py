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

GPS dropouts (2026-11-13 dead-reckoning drift): a variant with ``wh_len=L`` withholds its GPS
for L s in every ``wh_every`` s, the first window starting ``wh_phase`` s after the first
odometry sample. While it is withheld, the variant's IMU yaw is what heading_aligner would publish
without RTK: no COG updates, so the offset (``/heading_aligner/status`` ``offset_deg``) stays
at its value at the window start. (The tracked ``offset_rate`` is not applied either: it is
only integrated while RTK positions show motion, heading_logic ``rate_motion_window``.)
Several variants with staggered phases give overlapping windows::

    --variant base0:wh_len=90,wh_every=120,wh_phase=30 --variant base1:...,wh_phase=50

Live inputs (VIO recordings, scripts/vio/record_vio_bag.sh, 2026-11-14): ``--live-inputs``
takes ekf_map's real inputs instead of rebuilding them: /odom (mcu_node wheel twist, 50 Hz)
and /imu/data_aligned (heading_aligner, 100 Hz). The ring bags do not record those two.

VIO (2026-11-14): ``--vio FILE`` is the /odometry/vio stream (``scripts/vio/eval_vio_bag.py
bridge``: t, vx, vy, vz, var_vx, var_vy, wz in base_link). A variant with ``vio=1`` adds
``ekf_dual_vio.yaml``'s ekf_map overlay (odom2: vx, vy) and gets the stream through
``vio_gate``'s logic, run per variant with that variant's withheld windows as RTK dropouts
(``scripts/vio/vio_offline.run_gate``); ``vio_hz=N`` thins the gated stream to N Hz (default:
every message, as live). Gate verdict changes go to ``OUT_DIR/gate_<variant>.csv``, the time
shares inside the withheld windows to ``OUT_DIR/gate.json``. Name VIO variants with a prefix
other than ``b`` / ``v`` (drift_analyse.py reads ``o`` as wheel + VIO)::

    --live-inputs --vio OUT/vio.csv --variant v0:vy_var=0.05,fuse_vy=1,wh_len=100,... \
        --variant o0:vy_var=0.05,fuse_vy=1,vio=1,wh_len=100,...

Output: ``OUT_DIR/<variant>.csv`` (t, x, y, yaw), ``OUT_DIR/inputs.csv`` (t, kind, x, y,
var / class; kinds field, gps, odom, mcu (/mcu/measured_speed vx, wz), tilt (roll, pitch
deg), hdg (offset rad; offset rate rad/s, last COG time, last COG measured offset rad)),
``OUT_DIR/windows.csv`` (variant, start, end) and ``OUT_DIR/fit.json``. ``--inputs-only``
writes fit.json and inputs.csv without running a filter. Positions are map metres only.
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
EKF_VIO_YAML = os.path.join(STACK, 'src', 'mower_localization', 'config', 'ekf_dual_vio.yaml')
TOPICS = ('/odometry/filtered', '/fix', '/fix_status', '/odometry/filtered_map',
          '/heading_aligner/status', '/tilt/status', '/mcu/measured_speed')
LIVE_TOPICS = ('/odom', '/imu/data_aligned')
EARTH_R = 6378137.0
_TIMES = {}


# ---------------------------------------------------------------------------- bag reading
def read_bag(bag, t0, t1, topics=TOPICS):
    """[(t, topic, msg)] of ``topics`` between t0 and t1 (bag receive time, s), time order."""
    import sqlite3
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    dbs = sorted(glob.glob(os.path.join(bag, '*.db3')),
                 key=lambda p: int(p.rsplit('_', 1)[1].split('.')[0]))
    out = []
    for d in dbs:
        # immutable: WAL-mode splits (bag_recorder's resilient preset) open on a read-only mount
        c = sqlite3.connect('file:%s?mode=ro&immutable=1' % d, uri=True)
        tps = {r[0]: (r[1], r[2]) for r in c.execute('select id, name, type from topics')}
        ids = [k for k, v in tps.items() if v[0] in topics]
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
def build_inputs(msgs, aux=None, live=False):
    """(odom [(t, vx, wz)], imu [(t, yaw, wz)], fixes [(t, e, n, class, rx_var)],
    field_map [(t, x, y, yaw)]). ``aux`` (a dict), when given, gets ``tilt`` [(t, roll_deg,
    pitch_deg)], ``mcu`` [(t, wheel vx, wheel yaw rate)] (/mcu/measured_speed), ``hdg``
    [(t, offset_rad, offset_rate_rad_s, last_cog_t, last COG measured offset rad)],
    ``status`` [(t, /fix_status text)] and ``wheel`` [(t, vx)] (/odom when recorded, else
    /mcu/measured_speed: vio_gate's wheel check). ``live``: odom from /odom and imu from
    /imu/data_aligned instead of both from /odometry/filtered."""
    odom, imu, fixes, field = [], [], [], []
    tilt, hdg, mcu, status, wodom = [], [], [], [], []
    cls, lat0, lon0 = None, None, None
    for t, name, m in msgs:
        if name == '/fix_status':
            status.append((t, m.data))
            c = rtk_quality.classify_fix_status(m.data)
            if c is not None:
                cls = c
        elif name == '/odometry/filtered':
            if not live:
                vx, wz = m.twist.twist.linear.x, m.twist.twist.angular.z
                odom.append((t, vx, wz))
                imu.append((t, yaw_of(m.pose.pose.orientation), wz))
        elif name == '/odom':
            vx = m.twist.twist.linear.x
            wodom.append((t, vx))
            if live:
                odom.append((t, vx, m.twist.twist.angular.z))
        elif name == '/imu/data_aligned':
            if live:
                imu.append((t, yaw_of(m.orientation), m.angular_velocity.z))
        elif name == '/odometry/filtered_map':
            p = m.pose.pose
            field.append((t, p.position.x, p.position.y, yaw_of(p.orientation)))
        elif name == '/mcu/measured_speed':
            mcu.append((t, m.twist.linear.x, m.twist.angular.z))
        elif name == '/tilt/status':
            d = json.loads(m.data)
            if d.get('roll_deg') is not None and not d.get('stale'):
                tilt.append((t, d['roll_deg'], d.get('pitch_deg') or 0.0))
        elif name == '/heading_aligner/status':
            d = json.loads(m.data)
            if d.get('offset_deg') is not None:
                age = d.get('last_cog_age_s')
                cog = d.get('last_cog_offset_deg')
                hdg.append((t, math.radians(d['offset_deg']),
                            math.radians(d.get('offset_rate_deg_min') or 0.0) / 60.0,
                            t - age if age is not None else -1.0,
                            math.radians(cog) if cog is not None else float('nan')))
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
    if aux is not None:
        aux['tilt'], aux['hdg'], aux['mcu'], aux['status'] = tilt, hdg, mcu, status
        aux['wheel'] = wodom or [(t, vx) for t, vx, _wz in mcu]
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
    """'name:key=val,key=val' -> (name, {vy_var, fuse_vy, wh_len, wh_every, wh_phase, vio,
    vio_hz})."""
    name, _, rest = text.partition(':')
    opts = {'vy_var': 1e6, 'fuse_vy': 0, 'wh_len': 0.0, 'wh_every': 0.0, 'wh_phase': 0.0,
            'vio': 0, 'vio_hz': 0.0}
    for kv in filter(None, rest.split(',')):
        k, _, v = kv.partition('=')
        opts[k.strip()] = float(v)
    return name, opts


def withheld_windows(opts, t0, t1):
    """[(start, end)] of the variant's GPS dropouts between t0 and t1 (bag time, s)."""
    if opts['wh_len'] <= 0.0:
        return []
    every = opts['wh_every'] if opts['wh_every'] > 0.0 else 1e18
    out, s = [], t0 + opts['wh_phase']
    while s < t1:
        out.append((s, min(s + opts['wh_len'], t1)))
        s += every
    return out


def in_windows(windows, t):
    """Whether t lies in one of the sorted, disjoint [(start, end)]."""
    import bisect
    k = bisect.bisect_right(windows, (t, math.inf)) - 1
    return k >= 0 and t < windows[k][1]


def thin(rows, hz):
    """Rows (t first) at most ``hz`` per second (0: all)."""
    if hz <= 0.0:
        return list(rows)
    out, last = [], -math.inf
    for r in rows:
        if r[0] - last >= 1.0 / hz - 1e-6:
            out.append(r)
            last = r[0]
    return out


def gated_vio(vio, aux, windows, opts, out_dir, name):
    """The variant's VIO stream after vio_gate (its windows are RTK dropouts), thinned to
    ``vio_hz``. Writes the gate's verdict changes to gate_<name>.csv; returns (stream, gate
    summary inside the windows)."""
    sys.path.insert(0, os.path.join(STACK, 'scripts', 'vio'))
    import vio_offline
    verdicts = vio_offline.run_gate(vio, aux['status'], aux['wheel'], windows)
    with open(os.path.join(out_dir, 'gate_%s.csv' % name), 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['t', 'state'])
        last = None
        for t, r in verdicts:
            c = vio_offline.reason_class(r)
            if c != last:
                w.writerow([t, c])
                last = c
    inside = [v for v in verdicts if in_windows(windows, v[0])]
    passed = [row for row, (_t, r) in zip(vio, verdicts) if r is None]
    return thin(passed, opts['vio_hz']), vio_offline.gate_summary(inside)


def offset_at(hdg, t):
    """The heading_aligner status in force at t: (offset, rate, last_cog_t), or None."""
    import bisect
    ts = _TIMES.get(id(hdg))
    if ts is None or len(ts) != len(hdg):
        ts = _TIMES[id(hdg)] = [h[0] for h in hdg]
    k = bisect.bisect_right(ts, t) - 1
    if k < 0:
        return None
    if k + 1 < len(hdg) and hdg[k + 1][0] - hdg[k][0] < 2.0:
        # the offset moves between 1 Hz statuses (rate term): interpolate, unwrapped
        a, b = hdg[k], hdg[k + 1]
        f = (t - a[0]) / (b[0] - a[0])
        d = math.atan2(math.sin(b[1] - a[1]), math.cos(b[1] - a[1]))
        return (a[1] + f * d, a[2], a[3])
    return hdg[k][1:]


def dead_reckoned_yaw(imu, hdg, windows):
    """imu [(t, yaw, wz)] with the yaw of each withheld window replaced by heading_aligner's
    no-RTK yaw: the recorded yaw minus the recorded offset change since the window start."""
    if not windows or not hdg:
        return imu
    out, w = [], 0
    for t, yaw, wz in imu:
        while w < len(windows) and t >= windows[w][1]:
            w += 1
        if w >= len(windows) or t < windows[w][0]:
            out.append((t, yaw, wz))
            continue
        s = windows[w][0]
        h0, h = offset_at(hdg, s), offset_at(hdg, t)
        if h0 is None or h is None:
            out.append((t, yaw, wz))
            continue
        d = math.atan2(math.sin(h0[0] - h[0]), math.cos(h0[0] - h[0]))
        out.append((t, yaw + d, wz))
    return out


def write_params(out_dir, name, opts):
    with open(EKF_YAML) as fh:
        base = yaml.safe_load(fh)['ekf_map']['ros__parameters']
    p = dict(base)
    p.update({'use_sim_time': True, 'publish_tf': False, 'print_diagnostics': False,
              'odom0': '/rp/%s/odom' % name, 'imu0': '/rp/%s/imu' % name,
              'odom1': '/rp/%s/gps' % name})
    cfg = list(p['odom0_config'])
    cfg[7] = bool(opts.get('fuse_vy'))
    p['odom0_config'] = cfg
    if opts.get('vio'):
        with open(EKF_VIO_YAML) as fh:
            p.update(yaml.safe_load(fh)['ekf_map']['ros__parameters'])
        p['odom2'] = '/rp/%s/vio' % name
    path = os.path.join(out_dir, 'params_%s.yaml' % name)
    with open(path, 'w') as fh:
        yaml.safe_dump({'/**': {'ros__parameters': p}}, fh)
    return path


# ---------------------------------------------------------------------------- playback
def play(odom, imu, gps, variants, speed, vx_var, out_dir, vio=None):
    """Publish /clock and every variant's inputs at ``speed`` x real time; record outputs.

    ``imu`` and ``gps`` are {variant: stream}: a variant's GPS skips its withheld windows and
    its IMU yaw is dead-reckoned in them. ``vio`` {variant: gated stream} for the vio=1
    variants. The outputs are taken by an executor thread, so the publishing loop never has
    to spin."""
    vio = vio or {}
    import threading
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Imu
    rclpy.init()
    node = rclpy.create_node('ekf_replay_player')
    pub_clock = node.create_publisher(Clock, '/clock', 10)
    pub = {n: {k: node.create_publisher(Imu if k == 'imu' else Odometry, '/rp/%s/%s' % (n, k), 50)
               for k in ('odom', 'imu', 'gps') + (('vio',) if n in vio else ())}
           for n, _ in variants}
    rec = {n: [] for n, _ in variants}

    def on_out(name):
        def cb(m):
            p = m.pose.pose
            rec[name].append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                              p.position.x, p.position.y, yaw_of(p.orientation)))
        return cb
    for n, _ in variants:
        node.create_subscription(Odometry, '/rp_%s/odometry/filtered' % n, on_out(n), 1000)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()

    def stamp(msg, t):
        msg.header.stamp.sec = int(t)
        msg.header.stamp.nanosec = int(round((t - int(t)) * 1e9)) % 1000000000

    events = [(t, 'odom', None, r) for t, *r in odom]
    for n, _ in variants:
        events += [(t, 'imu', n, r) for t, *r in imu[n]]
        events += [(t, 'gps', n, r) for t, *r in gps[n] if r[0] is not None]
        events += [(t, 'vio', n, r) for t, *r in vio.get(n, ())]
    events.sort(key=lambda e: e[0])
    deadline = time.time() + 60.0
    while time.time() < deadline:
        if all(node.count_subscribers('/rp/%s/odom' % n) and
               node.count_subscribers('/rp/%s/gps' % n) and
               (n not in vio or node.count_subscribers('/rp/%s/vio' % n))
               for n, _ in variants):
            break
        time.sleep(0.1)
    wall0, sim0 = time.time(), events[0][0]
    for t, kind, vn, r in events:
        ahead = (t - sim0) / speed - (time.time() - wall0)
        if ahead > 0.0:
            time.sleep(ahead)
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
                pub[n]['odom'].publish(m)
        elif kind == 'imu':
            m = Imu()
            stamp(m, t)
            m.header.frame_id = 'base_link'
            m.orientation.z, m.orientation.w = math.sin(r[0] / 2.0), math.cos(r[0] / 2.0)
            m.orientation_covariance = [1e-3, 0, 0, 0, 1e-3, 0, 0, 0, 1e-3]
            m.angular_velocity.z = r[1]
            m.angular_velocity_covariance = [1e-4, 0, 0, 0, 1e-4, 0, 0, 0, 1e-4]
            m.linear_acceleration_covariance = [-1.0] + [0.0] * 8
            pub[vn]['imu'].publish(m)
        elif kind == 'vio':
            # /odometry/vio_gated: body twist, vx / vy / vz variances (vio_odom_bridge)
            m = Odometry()
            stamp(m, t)
            m.header.frame_id, m.child_frame_id = 'odom', 'base_link'
            m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z = r[:3]
            cov = [0.0] * 36
            cov[0], cov[7], cov[14], cov[21], cov[28], cov[35] = \
                r[3], r[4], 1e6, 1e6, 1e6, 1e6
            m.twist.covariance = cov
            m.pose.covariance = [1e6 if i % 7 == 0 else 0.0 for i in range(36)]
            m.pose.pose.orientation.w = 1.0
            pub[vn]['vio'].publish(m)
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
            pub[vn]['gps'].publish(m)
    time.sleep(2.0)
    ex.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    for n, rows in rec.items():
        rows.sort()
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
    ap.add_argument('--variant', action='append', default=[],
                    help='name:vy_var=V,fuse_vy=0|1,wh_len=S,wh_every=S,wh_phase=S (repeat)')
    ap.add_argument('--speed', type=float, default=4.0)
    ap.add_argument('--vx-var', type=float, default=0.02)
    ap.add_argument('--inputs-only', action='store_true',
                    help='write fit.json and inputs.csv, run no filter')
    ap.add_argument('--live-inputs', action='store_true',
                    help='odom from /odom, imu from /imu/data_aligned (VIO recordings)')
    ap.add_argument('--vio', default=None,
                    help='/odometry/vio CSV (eval_vio_bag.py bridge) for vio=1 variants')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    msgs = read_bag(a.bag, a.t0, a.t1, TOPICS + LIVE_TOPICS)
    aux = {}
    odom, imu, fixes, field = build_inputs(msgs, aux, live=a.live_inputs)
    del msgs
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
        w.writerows([(t, 'tilt', r, p, '', '') for t, r, p in aux['tilt']])
        w.writerows([(t, 'mcu', '', '', vx, wz) for t, vx, wz in aux['mcu']])
        w.writerows([(t, 'hdg', '', '', o, '%.9g %.3f %.6f' % (rt, ct, co))
                     for t, o, rt, ct, co in aux['hdg']])
    if a.inputs_only:
        print(json.dumps(fit))
        return
    variants = [parse_variant(v) for v in a.variant]
    t_first = min(r[0] for r in odom)
    t_last = max(r[0] for r in odom)
    imu_v, gps_v, vio_v, gate = {}, {}, {}, {}
    vio_rows = None
    if a.vio:
        with open(a.vio, newline='') as fh:
            r = csv.reader(fh)
            next(r)
            vio_rows = [tuple(float(x) for x in row[:7]) for row in r]
    with open(os.path.join(a.out, 'windows.csv'), 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['variant', 'start', 'end'])
        for n, opts in variants:
            wins = withheld_windows(opts, t_first, t_last)
            w.writerows([(n, s, e) for s, e in wins])
            imu_v[n] = dead_reckoned_yaw(imu, aux['hdg'], wins)
            gps_v[n] = [g for g in gps if not in_windows(wins, g[0])]
            if opts['vio']:
                if vio_rows is None:
                    sys.exit('variant %s has vio=1 but no --vio stream' % n)
                stream, gate[n] = gated_vio(vio_rows, aux, wins, opts, a.out, n)
                # (t, vx, vy, vz, var_vx, var_vy)
                vio_v[n] = [row[:6] for row in stream]
    if gate:
        with open(os.path.join(a.out, 'gate.json'), 'w') as fh:
            json.dump(gate, fh, indent=1)
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
        play(odom, imu_v, gps_v, variants, a.speed, a.vx_var, a.out, vio_v)
    finally:
        for p in procs:
            p.terminate()
        for p in procs:
            p.wait(timeout=10)
    print(json.dumps(fit))


if __name__ == '__main__':
    main()
