#!/usr/bin/env python3
"""Wheel / heading calibration from a recorded drive with RTK fixed (A2, 2026-10-22).

Method and field numbers: docs/analysis/2026-10-22_wheel_scale_and_motor_current.md.
Field procedure (manual mode, blade OFF, then the same with the blade ON): drive 4+ straight
lines of 5+ m each way over the same strip, at a steady ~0.2 m/s, then 2 slow pivots each way.

Needs these topics in the bag (bag_recorder records all of them during a mow):
  /fix /cmd_vel /mcu/measured_speed /mower_sensor_info /odometry/filtered
  /odometry/filtered_map  (optional: /imu/data for pivots, /battery)

Run in the dev image (rclpy + numpy); `.zstd` splits as in wheel_bag_metrics.py:

    docker run --rm --network none -v "$PWD":/work:ro -v /path/to/bag:/bag:ro \\
      mower:jazzy-dev-amd64 bash -c 'source /opt/ros/jazzy/setup.bash && \\
      source /work/install/setup.bash && python3 /work/scripts/wheel_cal.py /bag'

It prints, with 95 % confidence intervals over the straight segments (one segment = one sample):
  * speed scale        ground (RTK) speed / commanded           -> mcu_node linear_scale
  * odom scale         ground speed / MCU measured speed         -> wheel odometry scale
  * yaw bias           commanded yaw rate needed to drive straight (vehicle frame, paired
                       opposite-direction lanes; slope effects cancel)  -> yaw trim
  * wheel rpm ratio    MotorInfo right/left rpm on straights (coarse: integer rpm at 1 Hz)
  * heading lead       fused (EKF) yaw minus the RTK course over ground -> heading_aligner
  * lane-gap bias      steady right-of-travel offset from adjacent opposite lanes
  * pivots             measured / commanded yaw rate, per direction
"""
import argparse
import math
import os
import shutil
import sqlite3
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from wheel_bag_metrics import bag_files  # noqa: E402

TRACK_MCU = 0.40      # m, MCU v/w -> wheel split
WHEEL_D = 0.22        # m tyre diameter (MotorInfo rpm -> m/s)
RPM2MPS = math.pi * WHEEL_D / 60.0


# ---------------------------------------------------------------- bag reading
def read_bag(paths):
    """{name: np.array} rows (t, ...) per topic. Splits are copied to a temp dir first: the
    pinned splits are WAL databases that a read-only open cannot always attach."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    def yaw(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    out = {k: [] for k in ('fix', 'cmd', 'meas', 'msi', 'odom', 'map', 'imu', 'bat')}
    tmp = tempfile.mkdtemp(prefix='wheel_cal_')
    try:
        for p in paths:
            g = os.path.join(tmp, os.path.basename(p))
            for sfx in ('', '-wal'):
                if os.path.exists(p + sfx):
                    shutil.copy(p + sfx, g + sfx)
            try:
                db = sqlite3.connect(g)
                tp = {i: (n, t) for i, n, t in db.execute('SELECT id, name, type FROM topics')}
                rows = db.execute('SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp')
                for tid, ts, data in rows:
                    n, typ = tp[tid]
                    key = {'/fix': 'fix', '/cmd_vel': 'cmd', '/mcu/measured_speed': 'meas',
                           '/mower_sensor_info': 'msi', '/odometry/filtered': 'odom',
                           '/odometry/filtered_map': 'map', '/imu/data': 'imu',
                           '/battery': 'bat'}.get(n)
                    if key is None:
                        continue
                    m = deserialize_message(bytes(data), get_message(typ))
                    t = ts * 1e-9
                    if key == 'fix':
                        out[key].append((t, m.latitude, m.longitude, m.status.status))
                    elif key == 'cmd':
                        out[key].append((t, m.linear.x, m.angular.z))
                    elif key == 'meas':
                        out[key].append((t, m.twist.linear.x, m.twist.angular.z))
                    elif key == 'msi':
                        out[key].append((t, float(m.is_cutting), m.left_motor.speed_rpm,
                                         m.right_motor.speed_rpm, m.left_motor.current,
                                         m.right_motor.current))
                    elif key in ('odom', 'map'):
                        p_ = m.pose.pose
                        out[key].append((t, p_.position.x, p_.position.y, yaw(p_.orientation),
                                         m.twist.twist.linear.x, m.twist.twist.angular.z))
                    elif key == 'imu':
                        out[key].append((t, m.angular_velocity.z))
                    elif key == 'bat':
                        out[key].append((t, m.voltage, m.current))
                db.close()
            except sqlite3.DatabaseError as exc:
                print('skipping %s: %s' % (p, exc))
            for sfx in ('', '-wal', '-shm'):
                if os.path.exists(g + sfx):
                    os.unlink(g + sfx)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    return {k: np.array(v, dtype=float) for k, v in out.items() if v}


# ---------------------------------------------------------------- analysis (pure numpy)
def enu(fix):
    lat0, lon0 = fix[0, 1], fix[0, 2]
    r = 6378137.0
    x = np.radians(fix[:, 2] - lon0) * r * math.cos(math.radians(lat0))
    y = np.radians(fix[:, 1] - lat0) * r
    return x, y


def rdp(x, y, idx, eps):
    """Ramer-Douglas-Peucker split of a track into near-straight pieces (index pairs)."""
    if len(idx) < 3:
        return [(idx[0], idx[-1])]
    a = np.array([x[idx[0]], y[idx[0]]])
    b = np.array([x[idx[-1]], y[idx[-1]]])
    P = np.c_[x[idx], y[idx]]
    L = np.hypot(*(b - a))
    if L > 1e-3:
        dist = np.abs((b - a)[0] * (P - a)[:, 1] - (b - a)[1] * (P - a)[:, 0]) / L
    else:
        dist = np.hypot(*(P - a).T)
    m = int(np.argmax(dist))
    if dist[m] <= eps:
        return [(idx[0], idx[-1])]
    return rdp(x, y, idx[:m + 1], eps) + rdp(x, y, idx[m:], eps)


def ci(x):
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    if len(x) < 2:
        return (float(x.mean()) if len(x) else float('nan'), float('nan'), len(x))
    return float(x.mean()), float(1.96 * x.std(ddof=1) / math.sqrt(len(x))), len(x)


def straights(d, min_len=3.0, eps=0.05, max_resid=0.03):
    """Per straight segment: dict of the A2 quantities (see module doc)."""
    f = d['fix']
    f = f[f[:, 3] >= 0]
    fx, fy = enu(f)
    ft = f[:, 0]
    cv = d['cmd']
    dt = np.gradient(ft)
    spd = np.hypot(np.gradient(fx), np.gradient(fy)) / dt
    vcmd = np.interp(ft, cv[:, 0], cv[:, 1])
    ok = (vcmd > 0.08) & (spd > 0.05) & (dt < 0.35)
    pieces = []
    i, n = 0, len(ft)
    while i < n:
        if not ok[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and ok[j + 1] and ft[j + 1] - ft[j] < 0.35:
            j += 1
        if j - i > 20:
            pieces += rdp(fx, fy, np.arange(i, j + 1), eps)
        i = j + 1

    def sel(a, col, t0, t1):
        if a is None:
            return np.array([])
        return a[(a[:, 0] >= t0) & (a[:, 0] <= t1), col]
    segs = []
    for i, j in pieces:
        k = np.where((ft >= ft[i] + 0.5) & (ft <= ft[j] - 0.2))[0]
        if len(k) < 15:
            continue
        P = np.c_[fx[k], fy[k]]
        L = float(np.hypot(*(P[-1] - P[0])))
        if L < min_len:
            continue
        u = (P[-1] - P[0]) / L
        nrm = np.array([-u[1], u[0]])
        s = (P - P[0]) @ u
        e = (P - P[0]) @ nrm
        c = np.polyfit(s, e, 2)
        if np.std(e - np.polyval(c, s)) > max_resid:
            continue
        t0, t1 = ft[k[0]], ft[k[-1]]
        T = t1 - t0
        v_rtk = (s[-1] - s[0]) / T
        kappa = 2 * c[0]
        seg = dict(t0=t0, L=L, v_rtk=v_rtk, kappa=kappa, w_rtk=kappa * v_rtk,
                   cog=math.atan2(u[1], u[0]), mid=P.mean(0),
                   vcmd=sel(cv, 1, t0, t1).mean(), wcmd=sel(cv, 2, t0, t1).mean())
        mm = d.get('meas')
        seg['mcu_v'] = sel(mm, 1, t0, t1).mean() if mm is not None else float('nan')
        ms = d.get('msi')
        if ms is not None and len(sel(ms, 2, t0, t1)):
            seg['rpmL'] = -sel(ms, 2, t0, t1).mean()
            seg['rpmR'] = -sel(ms, 3, t0, t1).mean()
            seg['cut'] = sel(ms, 1, t0, t1).mean()
        mp = d.get('map')
        if mp is not None and len(sel(mp, 3, t0, t1)) > 3:
            q = np.c_[sel(mp, 1, t0, t1), sel(mp, 2, t0, t1)]
            cq = math.atan2(*(q[-1] - q[0])[::-1])
            yw = sel(mp, 3, t0, t1)
            seg['crab_deg'] = math.degrees(math.atan2(np.mean(np.sin(yw - cq)),
                                                      np.mean(np.cos(yw - cq))))
        segs.append(seg)
    return segs


def pairs(segs, max_dt=300.0, max_gap=0.3, ang_tol_deg=6.0):
    """Adjacent opposite-direction lanes driven back to back: (a, b, gap, side)."""
    out = []
    for a in range(len(segs)):
        for b in range(a + 1, len(segs)):
            A, B = segs[a], segs[b]
            if abs(math.cos(A['cog'] - B['cog']) + 1) > 1 - math.cos(math.radians(ang_tol_deg)):
                continue
            if B['t0'] - A['t0'] > max_dt:
                continue
            u = np.array([math.cos(A['cog']), math.sin(A['cog'])])
            nrm = np.array([-u[1], u[0]])
            dm = B['mid'] - A['mid']
            lat = float(dm @ nrm)
            if not 0.05 < abs(lat) < max_gap or abs(dm @ u) > 0.25 * (A['L'] + B['L']):
                continue
            out.append((a, b, abs(lat), 1 if lat > 0 else -1))
    return out


def lane_gap_bias(pr):
    """gap = s + 2 d side (side +1: the lanes lie on each other's LEFT); d > 0 = right of travel."""
    if len(pr) < 4 or len({p[3] for p in pr}) < 2:
        return None
    g = np.array([p[2] for p in pr])
    X = np.c_[np.ones(len(pr)), 2.0 * np.array([p[3] for p in pr])]
    sol = np.linalg.lstsq(X, g, rcond=None)[0]
    r = g - X @ sol
    se = np.sqrt(np.diag(np.linalg.inv(X.T @ X)) * r.var(ddof=2))
    return sol[0], 1.96 * se[0], sol[1], 1.96 * se[1], len(pr)


def pivots(d, min_s=1.5):
    cv = d['cmd']
    gyro = d['imu'] if 'imu' in d else d['odom'][:, [0, 5]]
    pv = (np.abs(cv[:, 2]) > 0.15) & (np.abs(cv[:, 1]) < 0.03)
    res = []
    i = 0
    while i < len(cv):
        if not pv[i]:
            i += 1
            continue
        j = i
        while (j + 1 < len(cv) and pv[j + 1] and np.sign(cv[j + 1, 2]) == np.sign(cv[i, 2])
               and cv[j + 1, 0] - cv[j, 0] < 0.3):
            j += 1
        if cv[j, 0] - cv[i, 0] > min_s:
            a, b = cv[i, 0] + 0.7, cv[j, 0] - 0.1
            k = (gyro[:, 0] >= a) & (gyro[:, 0] <= b)
            kc = (cv[:, 0] >= a) & (cv[:, 0] <= b)
            if k.any():
                w = cv[kc, 2].mean()
                res.append((w, gyro[k, 1].mean() / w))
        i = j + 1
    return res


def report(d, min_len=3.0):
    segs = straights(d, min_len=min_len)
    print('%d straight segments >= %.1f m, %.1f m in total' % (len(segs), min_len,
                                                               sum(s['L'] for s in segs)))
    if len(segs) < 2:
        print('not enough straights: drive longer, straighter lines (RTK fixed)')
        return {}
    col = lambda k: np.array([s.get(k, np.nan) for s in segs])  # noqa: E731
    v = col('v_rtk')
    rec = {}

    def line(name, x, fmt='%.4f'):
        m, h, n = ci(x)
        print(('  %-34s ' + fmt + ' +- ' + fmt + '  (n=%d)') % (name, m, h, n))
        return m, h
    print('Straights (95 % CI over segments):')
    rec['speed_scale'] = line('ground / commanded speed', v / col('vcmd'))
    rec['odom_scale'] = line('ground / MCU measured speed', v / col('mcu_v'))
    line('RTK curvature [1/m] (closed loop ~0)', col('kappa'))
    line('commanded yaw rate [rad/s]', col('wcmd'))
    line('right / left wheel rpm', col('rpmR') / col('rpmL'))
    if np.isfinite(col('crab_deg')).any():
        rec['heading_lead_deg'] = line('fused yaw - course [deg]', col('crab_deg'), '%.2f')
    pr = pairs(segs)
    if pr:
        a = np.array([p[0] for p in pr])
        b = np.array([p[1] for p in pr])
        print('Paired opposite lanes (vehicle-frame part; slope / map effects cancel), n=%d:' % len(pr))
        w = col('wcmd') - col('w_rtk')
        rec['yaw_bias'] = line('yaw-rate command to go straight', (w[a] + w[b]) / 2)
        line('  as curvature [1/m]', ((w / v)[a] + (w / v)[b]) / 2)
        if np.isfinite(col('crab_deg')).any():
            line('fused yaw - course [deg]', (col('crab_deg')[a] + col('crab_deg')[b]) / 2, '%.2f')
        gb = lane_gap_bias(pr)
        if gb:
            print('  lane gaps: swath %.3f +- %.3f m, right-of-travel offset %.4f +- %.4f m (n=%d)' % gb)
            rec['right_offset_m'] = (gb[2], gb[3])
    pv = pivots(d)
    if pv:
        print('Pivots (measured / commanded yaw rate):')
        for sgn, nm in ((1, 'CCW'), (-1, 'CW')):
            r = [x[1] for x in pv if np.sign(x[0]) == sgn]
            if r:
                line('%s' % nm, r, '%.3f')
    print('Suggested constants (apply only if the CI excludes "no change"):')
    m, h = rec['speed_scale']
    print('  mcu_node linear_scale            %.3f  (now 1.0; revert 1.0)' % (1.0 / m if m else 1.0))
    if 'yaw_bias' in rec:
        m, h = rec['yaw_bias']
        print('  yaw trim at %.2f m/s            %+.4f rad/s +- %.4f (%s)' % (
            float(np.nanmean(v)), m, h, 'significant' if abs(m) > h else 'not significant'))
        print('  equivalent wheel split           right/left commanded %.4f' % (
            1.0 + m * TRACK_MCU / float(np.nanmean(v))))
    if 'heading_lead_deg' in rec:
        m, h = rec['heading_lead_deg']
        print('  heading lead %.2f +- %.2f deg   (|lead| > 1 deg: check heading_aligner rate_beta)' % (m, h))
    return rec


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bag', help='bag split, pinned mow/incident dir or session dir')
    ap.add_argument('--min-len', type=float, default=3.0, help='m, shortest straight used')
    a = ap.parse_args(argv)
    paths = bag_files(a.bag)
    if not paths and os.path.isdir(a.bag):
        paths = sorted(os.path.join(r, n) for r, _d, ns in os.walk(a.bag) for n in ns
                       if n.endswith('.db3'))
        print('using the %d plain .db3 splits only' % len(paths))
    if not paths:
        return 1
    d = read_bag(paths)
    if 'fix' not in d or 'cmd' not in d:
        print('need /fix and /cmd_vel in the bag')
        return 1
    report(d, a.min_len)
    return 0


if __name__ == '__main__':
    sys.exit(main())
