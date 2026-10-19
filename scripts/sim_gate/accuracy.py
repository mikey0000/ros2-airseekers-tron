#!/usr/bin/env python3
"""Leg-end and pivot accuracy (the error budget, docs/analysis/2026-10-20_error_budget.md).

usage: accuracy.py sim   <run_dir>/data.json [--json out.json]    # sim gate run (ground truth)
       accuracy.py field <series.json> <mission.log> [--json out.json]
         (field: run6 dump.py series with /odometry/filtered_map; the EKF is the reference)

Every in-place pivot the mission logs ("pivot vertex at", "route corner at", "leg start at",
"final approach ... turn N deg") is paired with its /spin goal and the following /follow_path
goal. With a = pose when the spin is sent (the leg has ended) and b = pose when the next leg
is sent (the pivot has ended):
  (a0 = pose when the spin is sent, a = first rest after it, b = where the rotation in the
  commanded direction stopped; 2026-10-20: measured at rest, see measure())
  coast after leg          |a - a0| (still moving when the leg was declared done)
  leg-end along / lateral  a - vertex in the frame of the heading at a (+along = overshoot)
  pivot drift              |b - a| (base_link should not move in a pivot); excursion = max
  pivot heading error      yaw(b) - yaw(a0) - commanded angle (deg, + = turned too far);
                           'heading at next leg' = the same at the next leg's goal (old metric)
  leg-start offset         b - vertex in the frame of the target heading yaw(a0) + commanded
Legs are classed by the planned distance from the previous logged vertex: swath (>= 1.5 m),
connector (< 1.5 m); route corners are transit. Sim: pose = /sim/ground_truth (the truth)
and, for comparison, /odometry/filtered_map (what the stack believed).
"""
import bisect
import json
import math
import re
import sys

EV = re.compile(r'(pivot vertex|route corner|leg start) at \((-?[\d.]+), (-?[\d.]+)\)')


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def events(lines):
    """lines: [(t, msg)] of the mission node -> pivot events with ts (spin) and tf (next leg)."""
    out, cur = [], None
    for t, l in lines:
        if 'ntrip' in l.lower():
            continue
        m = EV.search(l)
        if m:
            a = re.search(r'pivot (-?\d+) deg', l)
            cur = {'kind': m.group(1), 'v': (float(m.group(2)), float(m.group(3))),
                   'cmd': float(a.group(1)) if a else None}
        elif 'final approach' in l:
            a = re.search(r'turn (-?\d+) deg', l)
            cur = {'kind': 'final approach', 'v': None, 'cmd': float(a.group(1)) if a else None}
        elif 'sending /spin goal' in l and cur and 'ts' not in cur:
            cur['ts'] = t
        elif 'sending /follow_path goal' in l and cur and 'ts' in cur:
            cur['tf'] = t
            out.append(cur)
            cur = None
    return out


def _rates(poses):
    """[(t, speed m/s, yaw rate rad/s)] by finite differences over ~0.1 s."""
    out = []
    k = 0
    for i, p in enumerate(poses):
        while k < i and p[0] - poses[k][0] > 0.1:
            k += 1
        j = max(0, min(k, i - 1))
        q = poses[j]
        dt = p[0] - q[0]
        if dt <= 0:
            out.append((p[0], 0.0, 0.0))
            continue
        out.append((p[0], math.hypot(p[1] - q[1], p[2] - q[2]) / dt, wrap(p[3] - q[3]) / dt))
    return out


def measure(ev, poses):
    """poses: sorted [t, x, y, yaw, ...]. Measured AT REST (2026-10-20 pivot fix): the leg
    end where the robot has stopped after the spin goal (it may still be coasting when the
    spin is sent), the pivot end where the rotation in the commanded direction has stopped
    (after the next leg's goal: the robot is still turning when the Spin reports success)."""
    ts = [p[0] for p in poses]
    rt = _rates(poses)

    def idx(t):
        return min(bisect.bisect(ts, t), len(poses) - 1)
    rows, prev = [], None
    for e in ev:
        i_s, i_f = idx(e['ts']), idx(e['tf'])
        gap = max([ts[k + 1] - ts[k] for k in range(max(i_s - 1, 0), min(i_f + 1, len(ts) - 1))]
                  or [0])
        sg = math.copysign(1.0, e['cmd']) if e['cmd'] else 1.0
        # leg end at rest
        i_a = i_s
        while i_a < i_f and rt[i_a][1] > 0.01:
            i_a += 1
        # pivot end: the peak rotation in the commanded direction (unwrapped from a0) before
        # the next leg drives off (cmd_vel_slew holds linear until the rotation has stopped)
        lim = idx(e['tf'] + 3.0)
        i_m = i_f
        while i_m < lim and rt[i_m][1] <= 0.03:
            i_m += 1
        cum, best, i_b = 0.0, -1e9, i_a
        for k in range(i_s + 1, i_m + 1):
            cum += sg * wrap(poses[k][3] - poses[k - 1][3])
            if k >= i_a and cum > best:
                best, i_b = cum, k
        a, b, a0 = poses[i_a], poses[i_b], poses[i_s]
        r = {'kind': e['kind'], 'cmd': e['cmd'], 'gap': gap,
             'coast': math.hypot(a[1] - a0[1], a[2] - a0[2]),
             'drift': math.hypot(b[1] - a[1], b[2] - a[2]),
             'exc': max([math.hypot(p[1] - a[1], p[2] - a[2]) for p in poses[i_a:i_b + 1]]
                        or [0])}
        if e['cmd'] is not None:
            # + = turned too far (in the commanded direction)
            r['herr'] = math.degrees(best) - abs(e['cmd']) if best > -1e8 else 0.0
            r['herr_tf'] = math.degrees(sg * wrap(poses[i_f][3] - a0[3]
                                                   - math.radians(e['cmd'])))
        if e['v']:
            v = e['v']
            if e['kind'] != 'leg start':
                dx, dy = a[1] - v[0], a[2] - v[1]
                r['along'] = dx * math.cos(a[3]) + dy * math.sin(a[3])
                r['lat'] = -dx * math.sin(a[3]) + dy * math.cos(a[3])
                r['legL'] = math.hypot(v[0] - prev[0], v[1] - prev[1]) if prev else None
            if e['cmd'] is not None:
                th = a0[3] + math.radians(e['cmd'])
                dx, dy = b[1] - v[0], b[2] - v[1]
                r['s_along'] = dx * math.cos(th) + dy * math.sin(th)
                r['s_lat'] = -dx * math.sin(th) + dy * math.cos(th)
            prev = v
        rows.append(r)
    return [r for r in rows if r['gap'] <= 1.0]


def cls(r):
    if r['kind'] == 'route corner':
        return 'transit'
    if r.get('legL') is None:
        return 'start'
    return 'swath' if r['legL'] >= 1.5 else 'connector'


def stats(v):
    v = sorted(abs(x) for x in v)
    n = len(v)
    if not n:
        return None
    q = lambda p: v[min(n - 1, int(math.ceil(p * n)) - 1)]
    return {'n': n, 'median': q(.5), 'p90': q(.9), 'p99': q(.99), 'max': v[-1]}


def table(rows, title):
    print('\n' + title)
    print('%-22s %-10s %3s %7s %7s %7s %7s' % ('metric', 'class', 'n', 'median', 'p90', 'p99',
                                               'max'))
    res = {}
    for key, name in [('along', 'leg-end along m'), ('lat', 'leg-end lateral m'),
                      ('coast', 'coast after leg m'),
                      ('drift', 'pivot drift m'), ('exc', 'pivot excursion m'),
                      ('herr', 'pivot heading deg'), ('herr_tf', 'heading at next leg deg'), ('s_along', 'leg-start along m'),
                      ('s_lat', 'leg-start lateral m')]:
        for c in ['swath', 'connector', 'transit', 'start', 'ALL']:
            s = stats([r[key] for r in rows if key in r and (c == 'ALL' or cls(r) == c)])
            if s:
                res['%s/%s' % (key, c)] = s
                print('%-22s %-10s %3d %7.3f %7.3f %7.3f %7.3f' % (
                    name, c, s['n'], s['median'], s['p90'], s['p99'], s['max']))
    sa = sorted(r['along'] for r in rows if 'along' in r)
    sh = sorted(r['herr'] for r in rows if 'herr' in r)
    if sa and sh:
        print('signed: leg-end along median %+.3f m (+ = past the vertex); heading error median '
              '%+.1f deg (+ = too far), %d/%d pivots turned too far' % (
                  sa[len(sa) // 2], sh[len(sh) // 2], sum(h > 0 for h in sh), len(sh)))
    return res


def main(argv):
    out = argv[argv.index('--json') + 1] if '--json' in argv else None
    res = {}
    if argv[1] == 'sim':
        D = json.load(open(argv[2]))
        lines = [(r[0], r[3]) for r in D['rosout'] if 'behavior_tree_node' in r[1]]
        ev = events(lines)
        print('pivot events', len(ev))
        res['truth'] = table(measure(ev, D['gt']), 'ground truth (/sim/ground_truth)')
        if D.get('odom_map'):
            res['ekf'] = table(measure(ev, D['odom_map']), 'stack belief (/odometry/filtered_map)')
    else:
        s = json.load(open(argv[2]))
        lines = []
        for l in open(argv[3]):
            m = re.search(r'\[(\d{9,}\.\d+)\]', l)
            if m:
                lines.append((float(m.group(1)), l))
        od = sorted({r[0]: r for r in s['/odometry/filtered_map']}.values())
        ev = events(lines)
        print('pivot events', len(ev))
        res['ekf'] = table(measure(ev, od), 'field (/odometry/filtered_map)')
    if out:
        json.dump(res, open(out, 'w'), indent=1)


if __name__ == '__main__':
    main(sys.argv)
