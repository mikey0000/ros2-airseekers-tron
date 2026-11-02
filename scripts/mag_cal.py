#!/usr/bin/env python3
"""Magnetometer calibration + heading evaluation from a recorded mow (2026-11-03 experiment).

Question: can the stereo module's LIS2MDL seed the heading after a reboot off the dock
(heading_aligner ``heading_mag_seed``)? This fits hard / soft iron and compares the
magnetometer heading with the aligner's COG-based map heading over a whole mow.

Topics (bag_recorder records them during a mow since 2026-11-03):
  /imu/mag              sensor_msgs/MagneticField, base_link, RAW (stereo_imu)
  /imu/mag_gravity      geometry_msgs/Vector3Stamped, base_link accel used for the tilt
  /odometry/filtered_map  EKF map pose: the reference yaw (ENU, UTM grid)
  /heading_aligner/status JSON: reference only while source cog / rtk, quality good / fair
  /hardware_bridge/status mowgli Status: blade on = mower_motor_rpm > 300 (or mow_enabled)
  /fix                  lat / lon for the grid convergence (optional)

Run in the dev image (rclpy + numpy + yaml), as scripts/wheel_cal.py:

    docker run --rm --network none -v "$PWD":/work:ro -v /path/to/bag:/bag:ro \\
      -v /tmp/out:/out mower:jazzy-dev-amd64 bash -c 'source /opt/ros/jazzy/setup.bash && \\
      source /work/install/setup.bash && python3 /work/scripts/mag_cal.py /bag \\
      --declination-deg <site value> --out /out/mag_calibration.yaml'

``--declination-deg`` must be the value the stereo_imu node will run with
(``mag_declination_deg``): the residual bias is stored as ``yaw_offset_deg`` relative to it.
``--vertical-ut`` (site vertical field, WMM "Z", down positive, uT) pins the z hard iron when
the samples are planar (a mow only turns about z).

Output: the fit (blade off and blade on separately: the difference is the blade motor's
field), heading error stats (bias, std, p95 with the bias removed, p95 raw) overall, blade
off / on, by distance from the dock, and with the runtime quality gate; a calibration yaml
(blade-off fit, the seed happens before the blade starts) and a verdict:
  seed        p95 < 10 deg (blade off, quality-gated): heading_mag_seed can be turned on
  watchdog    p95 < 25 deg: good enough to flag a grossly wrong heading, not to seed
  not usable  otherwise
"""
import argparse
import bisect
import json
import math
import os
import shutil
import sqlite3
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'src', 'mower_cameras'))
from mower_cameras import mag_logic as ml  # noqa: E402

TOPICS = {'/imu/mag': 'mag', '/imu/mag_gravity': 'grav', '/odometry/filtered_map': 'map',
          '/heading_aligner/status': 'hdg', '/hardware_bridge/status': 'hw', '/fix': 'fix'}
REF_SOURCES = ('cog', 'rtk')
REF_QUALITY = ('good', 'fair')
DOCK_BINS = (0.0, 2.0, 5.0, 10.0, 20.0, float('inf'))
BLADE_RPM = 300.0


def read_bag(paths):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    out = {k: [] for k in TOPICS.values()}
    tmp = tempfile.mkdtemp(prefix='mag_cal_')
    try:
        for p in paths:
            g = os.path.join(tmp, os.path.basename(p))
            for sfx in ('', '-wal'):
                if os.path.exists(p + sfx):
                    shutil.copy(p + sfx, g + sfx)
            db = sqlite3.connect(g)
            try:
                tp = {i: (n, t) for i, n, t in db.execute('SELECT id, name, type FROM topics')
                      if n in TOPICS}
                if not tp:
                    continue
                cls = {i: get_message(t) for i, (_n, t) in tp.items()}
                q = ('SELECT topic_id, timestamp, data FROM messages WHERE topic_id IN (%s) '
                     'ORDER BY timestamp' % ','.join('?' * len(tp)))
                for tid, ts, data in db.execute(q, list(tp)):
                    key = TOPICS[tp[tid][0]]
                    m = deserialize_message(bytes(data), cls[tid])
                    t = ts * 1e-9
                    if key == 'mag':
                        f = m.magnetic_field
                        out[key].append((t, f.x * 1e6, f.y * 1e6, f.z * 1e6))
                    elif key == 'grav':
                        out[key].append((t, m.vector.x, m.vector.y, m.vector.z))
                    elif key == 'map':
                        o = m.pose.pose.orientation
                        yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y ** 2 + o.z ** 2))
                        out[key].append((t, m.pose.pose.position.x, m.pose.pose.position.y, yaw))
                    elif key == 'hdg':
                        try:
                            st = json.loads(m.data)
                        except ValueError:
                            continue
                        out[key].append((t, st.get('source'), st.get('quality'),
                                         bool(st.get('docked'))))
                    elif key == 'hw':
                        out[key].append((t, float(m.mower_motor_rpm), bool(m.mow_enabled),
                                         bool(m.is_charging)))
                    elif key == 'fix':
                        out[key].append((t, m.latitude, m.longitude))
            finally:
                db.close()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    for v in out.values():
        v.sort(key=lambda r: r[0])
    return out


def _latest(rows, ts, t, max_age):
    i = bisect.bisect_right(ts, t) - 1
    if i < 0 or t - rows[i][0] > max_age:
        return None
    return rows[i]


def build_samples(d):
    """Join every /imu/mag sample with its gravity, reference yaw, blade state, dock distance."""
    mag = d['mag']
    if not mag:
        raise SystemExit('no /imu/mag in the bag (stereo_imu mag_enable, bag_recorder topics)')
    grav = d['grav']
    gts = [r[0] for r in grav]
    mp = d['map']
    mts = [r[0] for r in mp]
    hdg, hts = d['hdg'], [r[0] for r in d['hdg']]
    hw, wts = d['hw'], [r[0] for r in d['hw']]
    use_rpm = any(r[1] > 0 for r in hw)
    # dock = mean map position while charging, else the first map pose
    dock_xy = None
    if hw and mp:
        pts = []
        for t, _rpm, _en, chg in hw:
            if chg:
                r = _latest(mp, mts, t, 0.5)
                if r is not None:
                    pts.append(r[1:3])
        if pts:
            dock_xy = tuple(np.mean(pts, axis=0))
    if dock_xy is None and mp:
        dock_xy = mp[0][1:3]
    rows = []
    for t, x, y, z in mag:
        g = _latest(grav, gts, t, 0.2)
        r = _latest(mp, mts, t, 0.3)
        h = _latest(hdg, hts, t, 2.5)
        w = _latest(hw, wts, t, 1.0)
        blade = None
        if w is not None:
            blade = (w[1] > BLADE_RPM) if use_rpm else w[2]
        ref = None
        if r is not None and h is not None and h[1] in REF_SOURCES and h[2] in REF_QUALITY:
            ref = r[3]
        dist = None if (r is None or dock_xy is None) else math.hypot(r[1] - dock_xy[0],
                                                                      r[2] - dock_xy[1])
        rows.append({'t': t, 'm': (x, y, z), 'g': None if g is None else g[1:4], 'ref': ref,
                     'blade': blade, 'dist': dist,
                     'docked': bool(h[3]) if h is not None else (w[3] if w else False)})
    return rows


def fit(rows, vertical_ut, rpy):
    """Fit on the samples with a gravity vector when there are enough, else on all (level)."""
    with_g = [r for r in rows if r['g'] is not None]
    if len(with_g) >= 200:
        return ml.calibrate(np.array([r['m'] for r in with_g]),
                            np.array([r['g'] for r in with_g]), vertical_ut=vertical_ut,
                            mag_to_base_rpy_deg=rpy)
    return ml.calibrate(np.array([r['m'] for r in rows]), None, vertical_ut=vertical_ut,
                        mag_to_base_rpy_deg=rpy)


def evaluate(rows, cal, decl, conv):
    out = []
    for r in rows:
        if r['ref'] is None:
            continue
        h = ml.heading(np.array(r['m']), None if r['g'] is None else np.array(r['g']), cal,
                       decl, conv)
        out.append({'err': math.degrees(ml.wrap(h['yaw'] - r['ref'])),
                    'yaw': math.degrees(h['yaw']), 'ref': math.degrees(r['ref']),
                    'q': h['quality'], 'blade': r['blade'], 'dist': r['dist']})
    return out


def _stats(ev, pred=lambda e: True):
    return ml.error_stats([e['err'] for e in ev if pred(e)])


def fmt(s):
    if not s['n']:
        return 'n 0'
    return ('n %6d  bias %+7.2f  std %6.2f  p95 %6.2f  p95_raw %6.2f  max %6.1f'
            % (s['n'], s['bias'], s['std'], s['p95'], s['p95_raw'], s['max']))


def main(argv=None):
    sys.path.insert(0, HERE)
    from wheel_bag_metrics import bag_files
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bag', help='bag split, pinned mow dir or session dir')
    ap.add_argument('--declination-deg', type=float, default=0.0)
    ap.add_argument('--vertical-ut', type=float, default=None,
                    help='site vertical field (WMM Z, down positive, uT)')
    ap.add_argument('--mag-to-base-rpy-deg', default='-90.917,0,90',
                    help='the stereo_imu mag_to_base_rpy_deg the bag was recorded with')
    ap.add_argument('--min-quality', type=float, default=0.7,
                    help='heading_aligner mag_min_quality for the gated stats')
    ap.add_argument('--out', default='mag_calibration.yaml')
    a = ap.parse_args(argv)
    paths = bag_files(a.bag)
    if not paths:
        raise SystemExit('no bag splits under %s' % a.bag)
    d = read_bag(paths)
    rows = build_samples(d)
    rpy = [float(v) for v in a.mag_to_base_rpy_deg.split(',')]
    undocked = [r for r in rows if not r['docked']]
    off = [r for r in undocked if r['blade'] is False]
    on = [r for r in undocked if r['blade'] is True]
    print('%d mag samples (%d undocked: %d blade off, %d blade on, %d unknown), %d with a '
          'COG/RTK reference' % (len(rows), len(undocked), len(off), len(on),
                                 len(undocked) - len(off) - len(on),
                                 sum(r['ref'] is not None for r in rows)))
    conv = 0.0
    if d['fix']:
        fx = [f for f in d['fix'] if math.isfinite(f[1]) and (f[1] or f[2])]
        if fx:
            conv = ml.utm_grid_convergence(fx[0][1], fx[0][2])
    decl = math.radians(a.declination_deg)
    print('declination %+.2f deg, grid convergence %+.2f deg' % (a.declination_deg,
                                                                 math.degrees(conv)))
    cals = {}
    for name, sub in (('blade off', off), ('blade on', on)):
        if len(sub) < 200:
            print('%-9s: too few samples for a fit (%d)' % (name, len(sub)))
            continue
        try:
            c = fit(sub, a.vertical_ut, rpy)
        except (ValueError, np.linalg.LinAlgError) as exc:
            print('%-9s: fit failed: %s' % (name, exc))
            continue
        cals[name] = c
        print('%-9s: fit %s (spread %.2f, z %s)  hard iron [%s] uT  |B| %.1f uT  H %.1f uT  '
              'dip %+.1f deg' % (name, c.fit, c.extra.get('spread_ratio', 0.0),
                                 'observed' if c.z_observed else 'NOT observed',
                                 ', '.join('%+.2f' % v for v in c.hard_iron), c.field_ut,
                                 c.horizontal_ut, c.inclination_deg))
    if 'blade off' in cals and 'blade on' in cals:
        dh = cals['blade on'].hard_iron - cals['blade off'].hard_iron
        h0 = cals['blade off'].horizontal_ut
        print('blade motor field: [%s] uT (horizontal %.2f uT = up to %.1f deg of heading)' % (
            ', '.join('%+.2f' % v for v in dh), math.hypot(dh[0], dh[1]),
            math.degrees(math.atan2(math.hypot(dh[0], dh[1]), h0)) if h0 > 0 else float('nan')))
    cal = cals.get('blade off') or cals.get('blade on')
    if cal is None:
        raise SystemExit('no calibration: record a mow with /imu/mag (blade off part included)')
    ev = evaluate(undocked, cal, decl, conv)
    if not ev:
        raise SystemExit('no reference heading (aligner never cog/rtk good/fair)')
    if ml.mirrored([e['yaw'] for e in ev], [e['ref'] for e in ev]):
        print('WARNING: the mag heading turns AGAINST the map heading: an axis sign in '
              'mag_to_base_rpy_deg is wrong (fix it, re-record)')
    bias = _stats(ev)['bias']
    cal.yaw_offset_deg = -bias                       # residual, relative to --declination-deg
    ev = evaluate(undocked, cal, decl, conv)   # with the bias removed
    print('\nheading error (mag - COG map heading, deg; yaw_offset_deg %+.2f applied):'
          % cal.yaw_offset_deg)
    print('  all            ' + fmt(_stats(ev)))
    print('  blade off      ' + fmt(_stats(ev, lambda e: e['blade'] is False)))
    print('  blade on       ' + fmt(_stats(ev, lambda e: e['blade'] is True)))
    if 'blade on' in cals:
        ev_on = evaluate([r for r in on], cals['blade on'], decl, conv)
        print('  blade on (own fit) ' + fmt(_stats(ev_on)))
    gated = [e for e in ev if e['q'] >= a.min_quality]
    print('  q >= %.2f      ' % a.min_quality + fmt(_stats(gated)) +
          '  (%.0f %% of samples)' % (100.0 * len(gated) / len(ev)))
    print('  by distance from the dock:')
    for lo, hi in zip(DOCK_BINS[:-1], DOCK_BINS[1:]):
        s = _stats(ev, lambda e, lo=lo, hi=hi: e['dist'] is not None and lo <= e['dist'] < hi)
        print('    %4.0f-%-4s m   %s' % (lo, '' if hi == float('inf') else '%.0f' % hi, fmt(s)))
    seed = _stats(gated, lambda e: e['blade'] is False)
    if not seed['n']:
        seed = _stats(gated)
    v = ml.verdict(seed['p95'])
    hdr = ('written by scripts/mag_cal.py from %s\nfit on the blade-off samples; '
           'yaw_offset_deg is relative to mag_declination_deg %+.2f\nverdict %s (p95 %s deg, '
           'blade off, q >= %.2f)' % (os.path.basename(os.path.normpath(a.bag)),
                                      a.declination_deg, v, seed['p95'], a.min_quality))
    cal.extra['declination_deg'] = a.declination_deg
    cal.extra['verdict'] = v
    cal.extra['p95_deg'] = seed['p95']
    if 'blade on' in cals:
        cal.extra['hard_iron_blade_on_ut'] = [round(float(x), 4)
                                              for x in cals['blade on'].hard_iron]
    with open(a.out, 'w') as fh:
        fh.write(cal.to_yaml(hdr))
    print('\nverdict: %s (p95 %s deg blade off with q >= %.2f; seed < %.0f, watchdog < %.0f)'
          % (v.upper(), seed['p95'], a.min_quality, ml.SEED_P95_DEG, ml.WATCHDOG_P95_DEG))
    print('calibration -> %s (copy to /userdata/ros2/mag_calibration.yaml on the robot)' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
