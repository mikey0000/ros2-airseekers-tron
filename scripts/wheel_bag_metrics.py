#!/usr/bin/env python3
"""Wheel-smoothness metrics from a black-box rosbag2 (docs/wheel_smoothness_field_tests.md).

Reads the sqlite3 splits that ``bag_recorder`` pins under /userdata/ros2/mows/<ts>/ and
/userdata/ros2/incidents/<ts>_<reason>/ (bare ``*.db3.zstd`` files, no metadata.yaml) or a
complete rosbag2 session directory, and prints, per run:

* turn regime changes classified from ``/cmd_vel`` (pivot / slow pivot / arc / tight arc /
  counter-rotation), flip bursts, and sustained slow pivots
* one-wheel ticks on ``/cmd_vel`` (inner wheel in (-0.06, 0)) and ramp ticks in (0, 0.06)
* lag ``/cmd_vel_nav`` -> ``/cmd_vel`` -> ``/mcu/commanded_speed`` (cross-correlation of linear.x)
* stop latency: first zero on an emergency / bumper / nav lane -> zero on ``/cmd_vel`` ->
  zero on ``/mcu/commanded_speed`` -> measured zero, plus the distance rolled meanwhile
* pivot authority: commanded ``/cmd_vel`` yaw rate vs EKF / MCU yaw rate per pivot episode
* accel / decel actually commanded on ``/cmd_vel`` and seen on ``/mcu/measured_speed``

The splits are read straight from sqlite (the rosbag2 reader insists on following the split
chain recorded inside each file, which the pin's renaming breaks). Deserialising needs rclpy +
the message packages, i.e. run it in the dev image; the dev image has no zstd, so decompress
on the host first (``zstd -dk`` keeps the originals):

    zstd -dkq /path/to/mows/<ts>/*.zstd
    docker run --rm -v "$PWD":/work -v /path/to/mows/<ts>:/bag mower:jazzy-dev-amd64 bash -c \\
      'source /opt/ros/jazzy/setup.bash && python3 /work/scripts/wheel_bag_metrics.py /bag'

If the ``zstandard`` module or the ``zstd`` binary is available where the script runs it
decompresses the splits itself. Every split under the directory is read, in split order.
``--csv out.csv`` also dumps the resampled 20 Hz command/feedback series.
"""
import argparse
import bisect
import math
import os
import re
import shutil
import sqlite3
import subprocess
import sys

TRACK_HALF_M = 0.20          # MCU wheel split b 0.40 / 2 (2026-10-20; was 0.24); inner = |v| - |w| * 0.20
MIN_INNER_MPS = 0.06         # cmd_vel_slew min_inner_wheel_mps
TOPICS = {
    '/cmd_vel_nav': 'twist', '/cmd_vel_docking': 'twist', '/cmd_vel_teleop': 'twist',
    '/cmd_vel_bumper': 'twist', '/cmd_vel_emergency': 'twist', '/cmd_vel': 'twist',
    '/mcu/commanded_speed': 'twist', '/mcu/measured_speed': 'twist',
    '/odometry/filtered': 'odom', '/cmd_vel_slew/shape_status': 'string',
    '/cmd_vel_slew/gate_status': 'string', '/behavior_tree_node/high_level_status': 'skip',
}


# ---------------------------------------------------------------- bag reading
SPLIT_RE = re.compile(r'_(\d+)\.db3$')


def _decompress(zst):
    """`x.db3.zstd` -> `x.db3` next to it; returns the plain path or None if no tool is available."""
    out = zst[:-len('.zstd')]
    if os.path.isfile(out):
        return out
    try:
        import zstandard
        with open(zst, 'rb') as fi, open(out + '.part', 'wb') as fo:
            zstandard.ZstdDecompressor().copy_stream(fi, fo)
        os.replace(out + '.part', out)
        return out
    except ImportError:
        pass
    if shutil.which('zstd'):
        subprocess.run(['zstd', '-dkq', '-o', out, zst], check=True)
        return out
    return None


def bag_files(path):
    """Every sqlite3 split under `path` (a split, a pinned mow/incident dir, or a session dir),
    in split order. `.zstd` splits are decompressed next to the original when possible."""
    if os.path.isfile(path):
        files = [path]
    else:
        files = []
        for root, _dirs, names in os.walk(path):
            files += [os.path.join(root, n) for n in names if n.endswith('.db3') or n.endswith('.db3.zstd')]
    plain, missing = [], []
    for f in files:
        if f.endswith('.zstd'):
            out = f[:-len('.zstd')]
            if out in files or out in plain:
                continue
            got = _decompress(f)
            (plain.append(got) if got else missing.append(f))
        else:
            plain.append(f)
    if missing:
        print('no zstd decompressor here; on the host run:  zstd -dkq %s/*.zstd' % os.path.dirname(missing[0]))
        return []

    def key(f):
        m = SPLIT_RE.search(f)
        return (os.path.dirname(f), int(m.group(1)) if m else -1, f)
    return sorted(set(plain), key=key)


def read_series(paths):
    """{topic: [(t_s, v, w)]} for twist-like topics, [(t_s, text)] for strings."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    series = {t: [] for t in TOPICS}
    wanted = [t for t in TOPICS if TOPICS[t] != 'skip']
    for p in paths:
        db = sqlite3.connect('file:%s?mode=ro' % p, uri=True)
        try:
            topics = {name: (tid, typ) for tid, name, typ in
                      db.execute('SELECT id, name, type FROM topics') if name in wanted}
            if not topics:
                continue
            cls = {}
            for name, (tid, typ) in topics.items():
                try:
                    cls[tid] = (name, get_message(typ))
                except (ImportError, AttributeError, ValueError) as exc:
                    print('skipping %s: cannot import %s (%s)' % (name, typ, exc))
            if not cls:
                continue
            q = ('SELECT topic_id, timestamp, data FROM messages WHERE topic_id IN (%s) ORDER BY timestamp'
                 % ','.join('?' * len(cls)))
            for tid, t_ns, data in db.execute(q, list(cls)):
                topic, mcls = cls[tid]
                msg = deserialize_message(bytes(data), mcls)
                t = t_ns * 1e-9
                kind = TOPICS[topic]
                if kind == 'string':
                    series[topic].append((t, msg.data))
                elif kind == 'odom':
                    series[topic].append((t, msg.twist.twist.linear.x, msg.twist.twist.angular.z))
                else:
                    tw = msg.twist if hasattr(msg, 'twist') else msg     # TwistStamped or Twist
                    series[topic].append((t, tw.linear.x, tw.angular.z))
        finally:
            db.close()
    for s in series.values():
        s.sort(key=lambda r: r[0])
    return series


# ---------------------------------------------------------------- helpers
def resample(rows, t0, t1, hz=20.0, hold_s=0.6):
    """Zero-order hold onto a fixed grid; a gap longer than hold_s reads as zero (lane silent)."""
    n = int((t1 - t0) * hz) + 1
    ts = [r[0] for r in rows]
    out = []
    for i in range(n):
        t = t0 + i / hz
        k = bisect.bisect_right(ts, t + 1e-6) - 1
        if k < 0 or t - ts[k] > hold_s:
            out.append((t, 0.0, 0.0))
        else:
            out.append((t, rows[k][1], rows[k][2]))
    return out


def xcorr_lag(a, b, hz=20.0, max_lag_s=1.0):
    """Lag (s) at which b best matches a (b delayed by lag), on the linear.x column."""
    x = [r[1] for r in a]
    y = [r[1] for r in b]
    n = min(len(x), len(y))
    if n < 50:
        return None
    x, y = x[:n], y[:n]
    mx, my = sum(x) / n, sum(y) / n
    x = [v - mx for v in x]
    y = [v - my for v in y]
    sx = math.sqrt(sum(v * v for v in x)) or 1.0
    sy = math.sqrt(sum(v * v for v in y)) or 1.0
    best, best_lag = -2.0, 0
    for lag in range(0, int(max_lag_s * hz) + 1):
        c = sum(x[i] * y[i + lag] for i in range(n - lag)) / (sx * sy)
        if c > best:
            best, best_lag = c, lag
    return best_lag / hz, best


def episodes(rows, pred, min_len_s=0.3):
    """[(t_start, t_end, rows)] where pred(row) holds continuously."""
    out, cur = [], []
    for r in rows:
        if pred(r):
            cur.append(r)
        elif cur:
            if cur[-1][0] - cur[0][0] >= min_len_s:
                out.append((cur[0][0], cur[-1][0], cur))
            cur = []
    if cur and cur[-1][0] - cur[0][0] >= min_len_s:
        out.append((cur[0][0], cur[-1][0], cur))
    return out


def mean_at(rows, t0, t1, col):
    ts = [r[0] for r in rows]
    i, j = bisect.bisect_left(ts, t0), bisect.bisect_right(ts, t1)
    vals = [rows[k][col] for k in range(i, j)]
    return (sum(vals) / len(vals)) if vals else None


def first_after(rows, t, pred):
    ts = [r[0] for r in rows]
    for k in range(bisect.bisect_left(ts, t), len(rows)):
        if pred(rows[k]):
            return rows[k]
    return None


def inner_wheel(v, w):
    return abs(v) - abs(w) * TRACK_HALF_M


def fmt(x, unit='', nd=3):
    return '-' if x is None else ('%.*f%s' % (nd, x, unit))


# ---------------------------------------------------------------- metrics
def report(series, csv_path=None):
    cmd = series['/cmd_vel']
    if not cmd:
        print('no /cmd_vel in bag')
        return
    t0, t1 = cmd[0][0], cmd[-1][0]
    print('== topics (%.0f s)' % (t1 - t0))
    for t, rows in series.items():
        if rows:
            dur = rows[-1][0] - rows[0][0]
            print('  %-40s %6d msgs  %5.1f Hz' % (t, len(rows), len(rows) / dur if dur > 0 else 0))

    # output regimes, classified from /cmd_vel itself. The shaper status string is not a
    # regime: a legal pivot at or above the pivot floor (0.3 rad/s since 2026-10-20) reads "pass" and a snapped one
    # "pivot", so counting status changes doubles every pivot (run 2, 2026-10-14).
    def regime(v, w):
        if abs(v) < 0.005 and abs(w) < 0.005:
            return 'stop'
        if abs(v) < 0.01:
            return 'pivot' if abs(w) >= 0.2 else 'slowpivot'
        if abs(w) < 0.02:
            return 'straight'
        inner = inner_wheel(v, w)
        return 'arc' if inner >= MIN_INNER_MPS - 1e-9 else ('counter' if inner < 0.0 else 'tightarc')
    kinds = [(t, regime(v, w)) for t, v, w in cmd]
    changes = [(kinds[i][0], kinds[i - 1][1], kinds[i][1]) for i in range(1, len(kinds))
               if kinds[i][1] != kinds[i - 1][1]]
    real = [c for c in changes if 'stop' not in c[1:] and 'straight' not in c[1:]]
    bursts = sum(1 for i in range(1, len(real)) if real[i][0] - real[i - 1][0] < 1.0)
    counts = {}
    for _, k in kinds:
        counts[k] = counts.get(k, 0) + 1
    print('== /cmd_vel regimes (ticks): %s' % ', '.join('%s %d' % kv for kv in sorted(counts.items())))
    print('== turn regime changes (pivot/slowpivot/arc/tightarc/counter, stops and straights '
          'ignored): %d, %d within 1 s of the previous (flip bursts)' % (len(real), bursts))
    for t, a, b in real[:12]:
        print('  t=%7.1f  %s -> %s' % (t - t0, a, b))
    if len(real) > 12:
        print('  ... %d more' % (len(real) - 12))
    # sustained slow pivots: v = 0 with 0 < |w| < 0.2 for longer than the 0.25 s ramp
    runs, start, prev = [], None, None
    for t, k in kinds:
        if k == 'slowpivot':
            start = t if start is None else start
            prev = t
        elif start is not None:
            runs.append(prev - start)
            start = None
    if start is not None:
        runs.append(prev - start)
    long_runs = [r for r in runs if r > 0.3]
    print('== slow pivots (v = 0, 0 < |w| < 0.2): %d runs, %d longer than 0.3 s, %.1f s total, longest %.1f s'
          % (len(runs), len(long_runs), sum(runs), max(runs) if runs else 0.0))

    # one-wheel ticks
    moving = [r for r in cmd if abs(r[1]) > 1e-6 or abs(r[2]) > 1e-6]
    turning = [r for r in moving if abs(r[2]) > 1e-6]
    bad = [r for r in turning if -MIN_INNER_MPS < inner_wheel(r[1], r[2]) < 0.0]
    ramp = [r for r in turning if 0.0 < inner_wheel(r[1], r[2]) < MIN_INNER_MPS]
    print('== /cmd_vel one-wheel check: %d moving ticks, %d turning; inner in (-0.06,0): %d; '
          'inner in (0,0.06): %d' % (len(moving), len(turning), len(bad), len(ramp)))
    for r in bad[:5]:
        print('  t=%7.1f v=%.3f w=%.3f inner=%.3f' % (r[0] - t0, r[1], r[2], inner_wheel(r[1], r[2])))

    # lag
    nav = series['/cmd_vel_nav']
    mcu_c = series['/mcu/commanded_speed']
    # short hold: a silent lane reads as zero almost at once, so a watchdog stop on
    # /cmd_vel is not mistaken for the lane leading the command
    g_nav, g_cmd, g_mcu = (resample(nav, t0, t1, hold_s=0.15), resample(cmd, t0, t1, hold_s=0.15),
                           resample(mcu_c, t0, t1, hold_s=0.15))
    print('== lag (cross-correlation of linear.x, 20 Hz grid)')
    for name, a, b in (('/cmd_vel_nav -> /cmd_vel', g_nav, g_cmd),
                       ('/cmd_vel -> /mcu/commanded_speed', g_cmd, g_mcu),
                       ('/cmd_vel_nav -> /mcu/commanded_speed', g_nav, g_mcu)):
        r = xcorr_lag(a, b)
        print('  %-40s %s' % (name, 'n/a' if r is None else '%.0f ms (corr %.2f)' % (r[0] * 1e3, r[1])))

    # stops
    meas = series['/mcu/measured_speed']
    print('== stops (lane zero -> /cmd_vel zero -> MCU commanded zero -> measured < 0.01 m/s)')
    for lane in ('/cmd_vel_emergency', '/cmd_vel_bumper', '/cmd_vel_nav', '/cmd_vel_docking'):
        rows = series[lane]
        shown = 0
        events = []                      # (tz, speed before, label)
        cmd_ts = [r[0] for r in cmd]
        for i in range(len(rows)):
            prev = rows[i - 1] if i else None
            cur = rows[i]
            if abs(cur[1]) > 1e-6 or abs(cur[2]) > 1e-6:
                continue
            if prev is not None and abs(prev[1]) > 0.02:
                events.append((cur[0], prev[1], 'lane zero'))
            elif prev is None or cur[0] - prev[0] > 1.0:
                # a lane that only ever sends zeros (emergency, bumper): count it when the
                # robot was moving at that moment
                k = bisect.bisect_right(cmd_ts, cur[0]) - 1
                if k >= 0 and abs(cmd[k][1]) > 0.02:
                    events.append((cur[0], cmd[k][1], 'lane zero'))
        for i in range(1, len(rows)):            # lane went silent while asking for motion
            prev, cur = rows[i - 1], rows[i]
            if abs(prev[1]) > 0.02 and cur[0] - prev[0] > 0.6:
                events.append((prev[0] + 0.5, prev[1], 'silence+0.5s'))
        if rows and abs(rows[-1][1]) > 0.02 and t1 - rows[-1][0] > 0.6:
            events.append((rows[-1][0] + 0.5, rows[-1][1], 'silence+0.5s'))
        for tz, v_before, label in sorted(events):
            if True:
                c = first_after(cmd, tz, lambda r: abs(r[1]) < 1e-6 and abs(r[2]) < 1e-6)
                m = first_after(mcu_c, tz, lambda r: abs(r[1]) < 1e-6 and abs(r[2]) < 1e-6)
                s = first_after(meas, tz, lambda r: abs(r[1]) < 0.01)
                dist = None
                if s is not None and meas:
                    ts = [r[0] for r in meas]
                    i0, i1 = bisect.bisect_left(ts, tz), bisect.bisect_left(ts, s[0])
                    dist = sum(abs(meas[k][1]) * (meas[k + 1][0] - meas[k][0])
                               for k in range(i0, min(i1, len(meas) - 1)))
                print('  %-20s t=%7.1f %-12s from %.2f m/s: cmd_vel %s  mcu %s  measured %s  rolled %s'
                      % (lane, tz - t0, label, v_before,
                         fmt(None if c is None else (c[0] - tz) * 1e3, ' ms', 0),
                         fmt(None if m is None else (m[0] - tz) * 1e3, ' ms', 0),
                         fmt(None if s is None else (s[0] - tz) * 1e3, ' ms', 0),
                         fmt(dist, ' m', 2)))
                shown += 1
                if shown >= 6:
                    break
        if shown == 0 and rows:
            print('  %-20s no moving->zero transitions' % lane)

    # pivot authority
    ekf = series['/odometry/filtered']
    print('== pivots on /cmd_vel (|v| < 0.01, |w| >= 0.2): commanded vs EKF vs MCU yaw rate')
    piv = episodes(cmd, lambda r: abs(r[1]) < 0.01 and abs(r[2]) >= 0.2)
    for a, b, rows in piv[:10]:
        wc = sum(r[2] for r in rows) / len(rows)
        we = mean_at(ekf, a + 0.3, b, 2)
        wm = mean_at(meas, a + 0.3, b, 2)
        print('  t=%7.1f %5.1f s  cmd %+.3f  ekf %s  mcu %s  ratio %s'
              % (a - t0, b - a, wc, fmt(we, ' rad/s'), fmt(wm, ' rad/s'),
                 fmt(None if we is None or abs(wc) < 1e-6 else we / wc, '', 2)))
    if not piv:
        print('  none')
    arcs = episodes(cmd, lambda r: abs(r[1]) >= 0.01 and abs(r[2]) >= 0.1)
    print('== arcs on /cmd_vel: %d episodes, mean radius %s' % (
        len(arcs), fmt(None if not arcs else sum(abs(r[1]) / abs(r[2]) for _, _, rows in arcs
                                                  for r in rows) / sum(len(rows) for _, _, rows in arcs), ' m', 2)))

    # accel / decel
    def accel_stats(rows, col):
        acc, dec = 0.0, 0.0
        for i in range(1, len(rows)):
            dt = rows[i][0] - rows[i - 1][0]
            if dt < 0.02 or dt > 0.5:     # near-duplicate stamps (hard-stop zeros) are not accel
                continue
            a = (abs(rows[i][col]) - abs(rows[i - 1][col])) / dt
            acc, dec = max(acc, a), min(dec, a)
        return acc, dec
    for name, rows in (('/cmd_vel', cmd), ('/mcu/measured_speed', meas)):
        if rows:
            la, ld = accel_stats(rows, 1)
            aa, ad = accel_stats(rows, 2)
            print('== %s peak linear accel %+.2f / decel %+.2f m/s^2, angular %+.2f / %+.2f rad/s^2'
                  % (name, la, ld, aa, ad))

    if csv_path:
        g_meas = resample(meas, t0, t1, hold_s=0.3)
        g_ekf = resample(ekf, t0, t1, hold_s=0.3)
        with open(csv_path, 'w') as f:
            f.write('t,nav_v,nav_w,cmd_v,cmd_w,mcu_cmd_v,mcu_cmd_w,meas_v,meas_w,ekf_v,ekf_w\n')
            for i in range(len(g_cmd)):
                f.write('%.3f,%s\n' % (g_cmd[i][0] - t0, ','.join(
                    '%.4f,%.4f' % (g[i][1], g[i][2]) for g in (g_nav, g_cmd, g_mcu, g_meas, g_ekf))))
        print('csv: %s' % csv_path)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag', help='a .db3 split, a pinned mow/incident directory, or a session directory')
    ap.add_argument('--csv', help='also write the resampled 20 Hz series here')
    args = ap.parse_args(argv)
    paths = bag_files(args.bag)
    if not paths:
        print('no .db3 splits under %s' % args.bag)
        return 1
    print('reading %d split(s) under %s' % (len(paths), args.bag))
    try:
        series = read_series(paths)
    except ImportError as exc:
        print('needs rclpy + the message packages (run inside mower:jazzy-dev-amd64, see --help): %s' % exc)
        return 1
    report(series, args.csv)
    return 0


if __name__ == '__main__':
    sys.exit(main())
