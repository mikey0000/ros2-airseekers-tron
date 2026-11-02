#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Offline replay of a recorded mow through both aligner COG methods (window / keyframes).

    heading_replay.py <bagdir> [--csv out.csv]

Reads the sqlite splits of a bag_recorder ring directory (``*.db3``; ``.db3.zstd`` is
decompressed next to the original when a zstd tool is available). Needs the message packages:
run it in the dev image with the workspace on PYTHONPATH and ``install/setup.bash`` sourced.

Inputs taken from the bag:
  /fix                     RTK position (lat/lon -> local tangent plane, rotated onto the UTM
                           grid like the map frame)
  /fix_status              fix quality (RTK_FIXED 3, RTK_FLOAT 2)
  /cmd_vel                 commanded motion (the straight gate)
  /odometry/filtered_map   EKF map yaw + yaw rate
  /heading_aligner/status  the live aligner (offset, source, imu_yaw_deg, docked)

The rings do not record /imu/data, so the raw IMU yaw is reconstructed as
``EKF map yaw - live aligner offset`` (offset interpolated between the 1 Hz status samples);
the status ``imu_yaw_deg`` checks that reconstruction. The EKF map yaw therefore carries the
live (window) aligner's offset, so it is only an independent reference for the keyframe
method. The report also scores both against the RTK track heading over >= ``--truth-chord``
m straights (cog - imu yaw), which depends on neither aligner.
"""

import argparse
import bisect
import glob
import json
import math
import os
import re
import shutil
import sqlite3
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src', 'mower_localization'))
sys.path.insert(0, os.path.join(HERE, '..', 'src', 'mower_common'))

from mower_localization import heading_logic as hl  # noqa: E402

TOPICS = ('/fix', '/fix_status', '/cmd_vel', '/odometry/filtered_map', '/heading_aligner/status')
SPLIT_RE = re.compile(r'_(\d+)\.db3$')
R_EARTH = 6378137.0


def bag_files(d):
    for z in glob.glob(os.path.join(d, '*.db3.zstd')):
        out = z[:-5]
        if not os.path.isfile(out) and shutil.which('zstd'):
            r = subprocess.run(['zstd', '-dkq', '-o', out, z])
            if r.returncode:
                print('skipping bad chunk %s' % z)
    fs = glob.glob(os.path.join(d, '*.db3'))
    return sorted(fs, key=lambda f: int(SPLIT_RE.search(f).group(1)) if SPLIT_RE.search(f) else -1)


def read(paths):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    out = {t: [] for t in TOPICS}
    for p in paths:
        try:
            db = sqlite3.connect('file:%s?mode=ro' % p, uri=True)
            ids = {i: (n, get_message(ty)) for i, n, ty in
                   db.execute('SELECT id, name, type FROM topics') if n in TOPICS}
            q = ('SELECT topic_id, timestamp, data FROM messages WHERE topic_id IN (%s) '
                 'ORDER BY timestamp' % ','.join('?' * len(ids)))
            for tid, ts, data in db.execute(q, list(ids)):
                name, cls = ids[tid]
                out[name].append((ts * 1e-9, deserialize_message(bytes(data), cls)))
            db.close()
        except sqlite3.DatabaseError as exc:
            print('skipping %s: %s' % (p, exc))
    return out


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def pct(vals, q):
    if not vals:
        return float('nan')
    s = sorted(vals)
    return s[min(len(s) - 1, int(round(q / 100.0 * (len(s) - 1))))]


def deg(a):
    return math.degrees(a)


class Interp:
    def __init__(self, ts, vs):
        self.ts, self.vs = ts, vs

    def __call__(self, t):
        i = bisect.bisect_left(self.ts, t)
        if i <= 0:
            return self.vs[0]
        if i >= len(self.ts):
            return self.vs[-1]
        t0, t1 = self.ts[i - 1], self.ts[i]
        a, b = self.vs[i - 1], self.vs[i]
        return hl.wrap(a + hl.wrap(b - a) * (t - t0) / max(t1 - t0, 1e-9))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bagdir')
    ap.add_argument('--truth-chord', type=float, default=1.5, help='m of straight RTK track')
    ap.add_argument('--truth-max-turn', type=float, default=0.15, help='rad/s |cmd w|')
    ap.add_argument('--csv', help='write the offset traces here')
    a = ap.parse_args()
    data = read(bag_files(a.bagdir))
    st = []
    for t, m in data['/heading_aligner/status']:
        try:
            j = json.loads(m.data)
        except ValueError:
            continue
        if j.get('offset_deg') is not None:
            st.append((t, j))
    if not st or not data['/fix'] or not data['/odometry/filtered_map']:
        raise SystemExit('bag lacks /fix, /odometry/filtered_map or /heading_aligner/status')
    live = Interp([t for t, _ in st], [math.radians(j['offset_deg']) for _, j in st])
    t_undock = next((t for t, j in st if not j.get('docked')), st[0][0])
    t_start = data['/fix'][0][0]

    lat0 = data['/fix'][0][1].latitude
    lon0 = data['/fix'][0][1].longitude
    conv = math.radians(hl.utm_grid_convergence_deg(lat0, lon0))
    c, s = math.cos(conv), math.sin(conv)

    def to_grid(lat, lon):
        e = math.radians(lon - lon0) * R_EARTH * math.cos(math.radians(lat0))
        n = math.radians(lat - lat0) * R_EARTH
        return c * e - s * n, s * e + c * n      # rotate ENU by +convergence (grid yaw)

    # merged event stream
    ev = []
    for t, m in data['/odometry/filtered_map']:
        ev.append((t, 0, (yaw_of(m.pose.pose.orientation), m.twist.twist.angular.z)))
    for t, m in data['/fix_status']:
        q = 3 if 'RTK_FIXED' in m.data else 2 if 'RTK_FLOAT' in m.data else 1
        ev.append((t, 1, q))
    for t, m in data['/cmd_vel']:
        ev.append((t, 2, (m.linear.x, m.angular.z)))
    for t, m in data['/fix']:
        ev.append((t, 3, to_grid(m.latitude, m.longitude)))
    ev.sort(key=lambda e: (e[0], e[1]))

    # reconstruction check: EKF yaw - live offset vs the status imu_yaw_deg
    odo_t = [t for t, _ in data['/odometry/filtered_map']]
    odo_y = [yaw_of(m.pose.pose.orientation) for _, m in data['/odometry/filtered_map']]
    odo = Interp(odo_t, odo_y)
    rec_err = [abs(deg(hl.wrap(odo(t) - live(t) - math.radians(j['imu_yaw_deg']))))
               for t, j in st if j.get('imu_yaw_deg') is not None and odo_t[0] < t < odo_t[-1]]

    methods = ('window', 'keyframes')
    est = {m: hl.HeadingEstimator(hl.AlignParams(cog_method=m, imu_timeout=0.3))
           for m in methods}
    trace = {m: [] for m in methods}          # (t, offset) after every accepted update
    sample = {m: [] for m in methods}          # (t, offset) at every IMU sample
    meas = {m: [] for m in methods}            # (t, measured offset, accepted) per measurement
    fix_q, cmd, imu_yaw = 1, (0.0, 0.0), None
    track = []                                 # (t, x, y, imu_yaw) on straight fixed motion
    truth = []                                 # (t, track offset) from >= truth_chord chords
    for t, kind, v in ev:
        if kind == 0:
            imu_yaw = hl.wrap(v[0] - live(t))
            for m in methods:
                est[m].on_imu(t, imu_yaw, v[1])
                if est[m].offset is not None:
                    sample[m].append((t, est[m].offset))
        elif kind == 1:
            fix_q = v
            for m in methods:
                est[m].on_fix_type(t, v)
        elif kind == 2:
            cmd = v
            for m in methods:
                est[m].on_cmd(t, *v)
        else:
            for m in methods:
                r = est[m].on_gps(t, v[0], v[1], 0.01)
                if r is not None and 'measured' in r:
                    meas[m].append((t, r['measured'], r.get('accepted', False)))
                if r is not None and r.get('accepted'):
                    trace[m].append((t, r['offset']))
            straight = (fix_q == 3 and abs(cmd[0]) > 0.1 and abs(cmd[1]) < a.truth_max_turn
                        and imu_yaw is not None)
            if not straight or (track and abs(hl.wrap(imu_yaw - track[0][3])) > math.radians(3)):
                track = []
            if straight:
                track.append((t, v[0], v[1], imu_yaw))
                dx, dy = v[0] - track[0][1], v[1] - track[0][2]
                if math.hypot(dx, dy) >= a.truth_chord:
                    lat = max(abs((p[1] - track[0][1]) * dy - (p[2] - track[0][2]) * dx)
                              / math.hypot(dx, dy) for p in track)
                    if lat < 0.05:
                        cm = hl.CircularMean()
                        for p in track:
                            cm.add(p[3])
                        cog = math.atan2(dy, dx) + (math.pi if cmd[0] < 0 else 0.0)
                        truth.append((t, hl.wrap(cog - cm.mean())))
                    track = []

    print('bag %s: %.0f s, undock at +%.0f s, %d RTK fixes, grid convergence %.2f deg'
          % (a.bagdir, ev[-1][0] - t_start, t_undock - t_start, len(data['/fix']), deg(conv)))
    print('IMU yaw reconstruction (EKF yaw - live offset vs status imu_yaw): p50 %.2f p95 %.2f deg'
          % (pct(rec_err, 50), pct(rec_err, 95)))
    print('live aligner: %d COG updates; %d truth chords (>= %.1f m straight, RTK fixed)\n'
          % (st[-1][1].get('cog_updates', 0), len(truth), a.truth_chord))
    hdr = ('method', 'updates', 'cadence p50 s', 'first upd +s', 'conv 2deg +s', 'final deg',
           'map p50', 'map p95', 'chord p50', 'chord p95')
    print('%-10s %8s %13s %12s %12s %9s %8s %8s %9s %9s' % hdr)
    interp = {}
    for m in methods:
        tr = trace[m]
        if not tr:
            print('%-10s no accepted updates' % m)
            continue
        smp = sample[m]
        interp[m] = Interp([t for t, _ in smp], [o for _, o in smp])
        gaps = [b[0] - a_[0] for a_, b in zip(tr, tr[1:])]
        final = smp[-1][1]
        conv_t = next((t for t, o in smp if t >= t_undock and abs(hl.wrap(o - final)) <
                       math.radians(2.0)), None)
        # vs EKF map yaw on straights == vs the live offset there (yaw = imu + offset)
        e_map = [abs(deg(hl.wrap(interp[m](t) - live(t)))) for t, _ in truth]
        e_ch = [abs(deg(hl.wrap(interp[m](t) - o))) for t, o in truth]
        print('%-10s %8d %13.2f %12.0f %12s %9.2f %8.2f %8.2f %9.2f %9.2f' % (
            m, len(tr), pct(gaps, 50), tr[0][0] - t_undock,
            '-' if conv_t is None else '%.0f' % (conv_t - t_undock), deg(final),
            pct(e_map, 50), pct(e_map, 95), pct(e_ch, 50), pct(e_ch, 95)))
    e_live = [abs(deg(hl.wrap(live(t) - o))) for t, o in truth]
    print('%-10s %8s %13s %12s %12s %9.2f %8s %8s %9.2f %9.2f' % (
        'live', '-', '-', '-', '-', deg(live(ev[-1][0])), '0', '0', pct(e_live, 50),
        pct(e_live, 95)))
    if len(interp) == 2:
        ts = [t for t, _ in sample['keyframes'] if t >= max(trace['window'][0][0],
                                                            trace['keyframes'][0][0])]
        d = [abs(deg(hl.wrap(interp['window'](t) - interp['keyframes'](t)))) for t in ts]
        print('\nwindow vs keyframes offset (after both seeded): p50 %.2f p95 %.2f max %.2f deg'
              % (pct(d, 50), pct(d, 95), max(d) if d else float('nan')))
    if a.csv:
        with open(a.csv, 'w') as fh:
            fh.write('t_rel,method,kind,deg,accepted\n')
            for m in methods:
                for t, o in trace[m]:
                    fh.write('%.2f,%s,offset,%.3f,1\n' % (t - t_undock, m, deg(o)))
                for t, o, acc in meas[m]:
                    fh.write('%.2f,%s,measured,%.3f,%d\n' % (t - t_undock, m, deg(o), acc))
            for t, o in truth:
                fh.write('%.2f,truth,measured,%.3f,1\n' % (t - t_undock, deg(o)))
        print('traces -> %s' % a.csv)


if __name__ == '__main__':
    main()
