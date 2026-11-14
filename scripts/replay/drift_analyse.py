#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Dead-reckoning drift from ``ekf_replay.py`` GPS-dropout runs (2026-11-13).

Each OUT_DIR is one ``ekf_replay.py`` run whose variants withheld the GPS in windows
(``wh_len`` / ``wh_every`` / ``wh_phase``). Variant names start with ``b`` (ekf_map without wheel
vy), ``v`` (wheel vy fused) or ``o`` (wheel vy fused + gated VIO, 2026-11-14,
scripts/vio/eval_vio_bag.sh); the rest of the name is the phase. With both ``v`` and ``o``
runs a side-by-side budget table compares wheel only against wheel + VIO. Truth is the RTK-FIXED
``/fix`` in the map frame (lever arm applied by ekf_replay), heading truth the COG measurement
heading_aligner took at its next update.

    uv run -q --no-project --with numpy --with matplotlib python3 \\
        scripts/replay/drift_analyse.py OUT_DIR... --img DIR --json FILE

Prints the tables of docs/analysis/2026-11-13_dead_reckoning_drift.md. Map metres only.
"""

import argparse
import bisect
import csv
import json
import math
import os
import sys

import numpy as np

MILESTONES_M = (1, 2, 5, 10, 20, 30)
MILESTONES_S = (10, 20, 30, 60, 90)
THRESHOLDS_M = (0.10, 0.20, 0.30)
PIVOT_VX = 0.05          # m/s: |wheel vx| below this while turning is a pivot in place
PIVOT_MIN_DEG = 45.0     # a pivot event turns at least this much
SLOPE_DEG = 8.0          # mean tilt (hypot(roll, pitch)) at or above this: slope
FLAT_DEG = 4.0           # mean tilt below this: flat
PRE_FIXED_S = 10.0       # the window start needs RTK FIXED for this long before it
SLIP_WINDOW_S = 2.0      # s: wheel-odometry turn vs fused (gyro) turn compared over this window
SLIP_RAD = 0.6           # a mismatch this large (34 deg) within the window is a slip event
# variant name prefix -> label (report keys use the label)
PREFIXES = (('v', 'vy fused (branch)'), ('b', 'vy not fused'), ('o', 'wheel + VIO'))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def pct(xs, p):
    return float(np.percentile(xs, p)) if len(xs) else float('nan')


class Series:
    """Sorted (t, *values) rows with linear interpolation (gaps over max_gap -> None)."""

    def __init__(self, rows, max_gap=0.5, angle_col=None):
        rows.sort(key=lambda r: r[0])
        self.rows, self.t = rows, [r[0] for r in rows]
        self.max_gap, self.angle_col = max_gap, angle_col

    def at(self, t):
        k = bisect.bisect_left(self.t, t)
        if k <= 0 or k >= len(self.rows):
            return None
        a, b = self.rows[k - 1], self.rows[k]
        if b[0] - a[0] > self.max_gap:
            return None
        f = (t - a[0]) / (b[0] - a[0])
        out = []
        for i in range(1, len(a)):
            if i == self.angle_col:
                out.append(a[i] + f * wrap(b[i] - a[i]))
            else:
                out.append(a[i] + f * (b[i] - a[i]))
        return out


def load_run(d):
    field, gps, odom, tilt, hdg, mcu = [], [], [], [], [], []
    with open(os.path.join(d, 'inputs.csv')) as fh:
        r = csv.reader(fh)
        next(r)
        for t, k, x, y, v1, v2 in r:
            t = float(t)
            if k == 'field':
                field.append((t, float(x), float(y), float(v1)))
            elif k == 'gps':
                gps.append((t, float(x) if x else None, float(y) if y else None, v2))
            elif k == 'odom':
                odom.append((t, float(v1), float(v2)))
            elif k == 'mcu':
                mcu.append((t, float(v1), float(v2)))
            elif k == 'tilt':
                tilt.append((t, float(x), float(y)))
            elif k == 'hdg':
                rate, cog_t, cog_off = v2.split()
                hdg.append((t, float(v1), float(rate), float(cog_t), float(cog_off)))
    wins = {}
    with open(os.path.join(d, 'windows.csv')) as fh:
        r = csv.reader(fh)
        next(r)
        for n, s, e in r:
            wins.setdefault(n, []).append((float(s), float(e)))
    est = {}
    for n in wins:
        rows = []
        with open(os.path.join(d, n + '.csv')) as fh:
            r = csv.reader(fh)
            next(r)
            rows = [tuple(float(v) for v in row) for row in r]
        est[n] = Series(rows, 0.5, angle_col=3)
    gps.sort(key=lambda g: g[0])
    odom.sort(key=lambda o: o[0])
    tilt.sort(key=lambda o: o[0])
    hdg.sort(key=lambda o: o[0])
    mcu.sort(key=lambda o: o[0])
    return {'mcu': Series(mcu, 0.5), 'name': os.path.basename(os.path.normpath(d)),
            'field': Series(field, 0.5, 3),
            'gps': gps, 'gps_t': [g[0] for g in gps], 'odom': odom,
            'odom_t': [o[0] for o in odom], 'tilt': tilt, 'tilt_t': [o[0] for o in tilt],
            'hdg': hdg, 'wins': wins, 'est': est}


def cog_measurements(run):
    """[(t, measured offset rad)]: heading_aligner's raw COG measurements (status
    ``last_cog_offset_deg`` at each new ``last_cog_age_s`` epoch), before its low-pass."""
    out, last = [], None
    for t, _off, _rate, cog_t, cog_off in run['hdg']:
        if cog_t < 0 or cog_off != cog_off or (last is not None and abs(cog_t - last) < 0.5):
            continue
        out.append((cog_t, cog_off))
        last = cog_t
    return out


_SAMPLES = {}


def window_samples(run, n, s, e):
    """Per RTK-fixed truth sample in the window: dict rows, or None if the start is unusable."""
    key = (run['name'], n, s)
    if key not in _SAMPLES:
        _SAMPLES[key] = _window_samples(run, n, s, e)
    return _SAMPLES[key]


def _window_samples(run, n, s, e):
    gps, gt = run['gps'], run['gps_t']
    k0 = bisect.bisect_left(gt, s - PRE_FIXED_S)
    k1 = bisect.bisect_left(gt, s)
    pre = gps[k0:k1]
    if len(pre) < 0.8 * PRE_FIXED_S * 10 or any(g[3] != 'fixed' for g in pre):
        return None
    est, field = run['est'][n], run['field']
    odom, ot = run['odom'], run['odom_t']
    i0, i1 = bisect.bisect_left(ot, s), bisect.bisect_left(ot, e)
    # cumulative motion features at each odom sample
    feats, d, dfwd, drev, turn, piv, pivots, cur_piv = [], 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.0
    slip, mcu, mis, ring = 0.0, run['mcu'], 0.0, []
    prev = None
    for o in odom[i0:i1]:
        if prev is not None:
            dt = min(o[0] - prev[0], 0.2)
            vx, wz = prev[1], prev[2]
            step = abs(vx) * dt
            d += step
            if vx >= 0:
                dfwd += step
            else:
                drev += step
            turn += abs(wz) * dt
            # slip: the wheels' integrated turn and the fused (gyro-led) turn disagree over
            # SLIP_WINDOW_S (stuck pivots, a wheel spinning in place); integrals, so the
            # 5 Hz MCU sampling lag in a normal pivot cancels
            w = mcu.at(prev[0])
            if w is not None:
                mis += (w[1] - wz) * dt
            ring.append((o[0], mis))
            while ring and ring[0][0] < o[0] - SLIP_WINDOW_S:
                ring.pop(0)
            slip = max(slip, abs(mis - ring[0][1]))
            if abs(vx) < PIVOT_VX and abs(wz) > 0.05:
                cur_piv += wz * dt
            elif abs(cur_piv) > 0.0:
                if abs(math.degrees(cur_piv)) >= PIVOT_MIN_DEG:
                    pivots += 1
                    piv += abs(cur_piv)
                cur_piv = 0.0
        feats.append((o[0], d, dfwd, drev, turn, pivots, piv, slip))
        prev = o
    if not feats:
        return None
    ft = [f[0] for f in feats]
    tilt, tt = run['tilt'], run['tilt_t']
    rows = []
    roll_sum, tilt_sum, roll_n, j = 0.0, 0.0, 0, bisect.bisect_left(tt, s)
    for g in gps[k1:bisect.bisect_left(gt, e)]:
        t = g[0]
        while j < len(tilt) and tilt[j][0] <= t:
            roll_sum += abs(tilt[j][1])
            tilt_sum += math.hypot(tilt[j][1], tilt[j][2])
            roll_n += 1
            j += 1
        if g[3] != 'fixed' or g[1] is None:
            continue
        p, tr = est.at(t), field.at(t)
        if p is None or tr is None:
            continue
        kf = bisect.bisect_right(ft, t) - 1
        if kf < 0:
            continue
        f = feats[kf]
        ex, ey = p[0] - g[1], p[1] - g[2]
        c, si = math.cos(tr[2]), math.sin(tr[2])
        rows.append({'t': t - s, 'd': f[1], 'rev': f[3], 'turn': f[4], 'pivots': f[5],
                     'pivot_rad': f[6], 'slip': f[7],
                     'roll': tilt_sum / roll_n if roll_n else float('nan'),
                     'side_roll': roll_sum / roll_n if roll_n else float('nan'),
                     'err': math.hypot(ex, ey), 'along': ex * c + ey * si,
                     'cross': -ex * si + ey * c, 'yaw_err': wrap(p[2] - tr[2]),
                     'net_turn': abs(wrap(tr[2] - field.at(s + 0.05)[2]))
                     if field.at(s + 0.05) else float('nan')})
    return rows


def _ok(r):
    return r['slip'] < SLIP_RAD


CLASSES = {
    'all incl. slip': lambda r: True,
    'slip events': lambda r: not _ok(r),
    'all': _ok,
    'straight fwd': lambda r: _ok(r) and r['rev'] < 0.05 * max(r['d'], 1e-3)
    and r['pivots'] == 0 and r['net_turn'] < math.radians(20),
    'reverse legs': lambda r: _ok(r) and r['rev'] > 0.5 * r['d'],
    'no pivot': lambda r: _ok(r) and r['pivots'] == 0,
    '>=1 pivot': lambda r: _ok(r) and r['pivots'] >= 1,
    'flat (tilt <4 deg)': lambda r: _ok(r) and r['roll'] < FLAT_DEG,
    'slope (tilt >=8 deg)': lambda r: _ok(r) and r['roll'] >= SLOPE_DEG,
    'side slope (|roll| >=8 deg)': lambda r: _ok(r) and r['side_roll'] >= SLOPE_DEG,
    'flat, no pivot': lambda r: _ok(r) and r['roll'] < FLAT_DEG and r['pivots'] == 0,
    'flat, >=1 pivot': lambda r: _ok(r) and r['roll'] < FLAT_DEG and r['pivots'] >= 1,
    'slope, no pivot': lambda r: _ok(r) and r['roll'] >= SLOPE_DEG and r['pivots'] == 0,
    'slope, >=1 pivot': lambda r: _ok(r) and r['roll'] >= SLOPE_DEG and r['pivots'] >= 1,
}


def at_distance(rows, dist):
    for r in rows:
        if r['d'] >= dist:
            return r
    return None


def at_time(rows, ts):
    for r in rows:
        if r['t'] >= ts:
            return r
    return None


def collect(runs, prefix):
    """[(run, rows)] for every usable window of the variants starting with prefix."""
    out = []
    for run in runs:
        for n, wins in run['wins'].items():
            if not n.startswith(prefix):
                continue
            for s, e in wins:
                rows = window_samples(run, n, s, e)
                if rows:
                    out.append((run['name'], rows))
    return out


def dist_table(windows, classes=CLASSES):
    res = {}
    for cname, pred in classes.items():
        res[cname] = {}
        for m in MILESTONES_M:
            vals = []
            for _run, rows in windows:
                r = at_distance(rows, m)
                if r is not None and pred(r):
                    vals.append(r)
            e = [v['err'] for v in vals]
            res[cname][m] = {'n': len(e), 'p50': pct(e, 50), 'p90': pct(e, 90),
                             'max': max(e) if e else float('nan'),
                             'cross_p90': pct([abs(v['cross']) for v in vals], 90),
                             'along_p90': pct([abs(v['along']) for v in vals], 90),
                             'along_p50_signed': pct([v['along'] for v in vals], 50),
                             'cross_p50_signed': pct([v['cross'] for v in vals], 50)}
    return res


def budget(windows, pred, thresholds=THRESHOLDS_M, step=0.25, dmax=30.0, min_n=15):
    """Distance at which the p90 error over the windows (in class at that distance) first
    reaches each threshold; '>' + last distance with enough windows if it never does."""
    grid = np.arange(step, dmax + 1e-9, step)
    p90 = []
    for dd in grid:
        e = []
        for _run, rows in windows:
            r = at_distance(rows, dd)
            if r is not None and pred(r):
                e.append(r['err'])
        p90.append((dd, pct(e, 90) if len(e) >= min_n else float('nan'), len(e)))
    out = {}
    for th in thresholds:
        hit = next((dd for dd, v, _n in p90 if v == v and v >= th), None)
        last = max((dd for dd, v, _n in p90 if v == v), default=0.0)
        out[th] = hit if hit is not None else '>%.1f' % last
    return out, p90


def time_table(windows):
    res = {}
    for ts in MILESTONES_S:
        vals = [at_time(rows, ts) for _r, rows in windows]
        vals = [v for v in vals if v is not None and v['d'] > 0.5]
        res[ts] = {'n': len(vals), 'err_p50': pct([v['err'] for v in vals], 50),
                   'err_p90': pct([v['err'] for v in vals], 90),
                   'yaw_p50_deg': pct([abs(math.degrees(v['yaw_err'])) for v in vals], 50),
                   'yaw_p90_deg': pct([abs(math.degrees(v['yaw_err'])) for v in vals], 90)}
    return res


def stationary(windows, t_check=60.0):
    """Position drift of windows in which the robot stayed put (< 0.1 m wheel distance)."""
    out = []
    for _run, rows in windows:
        r = at_time(rows, t_check)
        if r is not None and r['d'] < 0.1:
            r0 = rows[0]
            out.append(r['err'] - r0['err'])
    return out


def heading_from_cog(runs, prefix):
    """Heading error of the dead-reckoned yaw against heading_aligner's later COG updates:
    the recorded offset after the first COG update past t minus the offset frozen at the
    window start, per (t since start, pivots so far, pivot angle so far)."""
    out = []
    for run in runs:
        hdg = run['hdg']
        ht = [h[0] for h in hdg]
        cogs = cog_measurements(run)
        ct = [c[0] for c in cogs]
        for n, wins in run['wins'].items():
            if not n.startswith(prefix):
                continue
            for s, e in wins:
                k = bisect.bisect_right(ht, s) - 1
                if k < 0:
                    continue
                off0 = hdg[k][1]
                rows = window_samples(run, n, s, e)
                if not rows:
                    continue
                for j in range(bisect.bisect_left(ct, s + 5.0), bisect.bisect_left(ct, e)):
                    tc, off = cogs[j]
                    r = at_time(rows, tc - s)
                    if r is None:
                        continue
                    out.append({'t': tc - s, 'd': r['d'], 'pivots': r['pivots'],
                                'pivot_deg': math.degrees(r['pivot_rad']),
                                'err_deg': math.degrees(wrap(off0 - off))})
    return out


def pivot_events(runs, prefix):
    """Per pivot inside a window: change of the position error across it (truth samples
    before and after), and of the yaw error at the next COG update after it vs before it."""
    out = []
    for run in runs:
        for n, wins in run['wins'].items():
            if not n.startswith(prefix):
                continue
            for s, e in wins:
                rows = window_samples(run, n, s, e)
                if not rows:
                    continue
                for a, b in zip(rows, rows[1:]):
                    if b['pivots'] > a['pivots'] and b['t'] - a['t'] < 30.0:
                        out.append({'derr': b['err'] - a['err'],
                                    'dvec': math.hypot(b['along'] - a['along'],
                                                       b['cross'] - a['cross']),
                                    'deg': math.degrees(b['pivot_rad'] - a['pivot_rad']),
                                    'roll': b['roll']})
    return out


def whole_bag_heading(runs):
    """Offset change between consecutive heading_aligner COG updates, per minute, split by
    whether a pivot happened between them (the whole bag, no withholding; intervals of
    60-300 s, so the ~1.5 deg COG measurement noise is a small share)."""
    out = []
    for run in runs:
        cogs = cog_measurements(run)
        odom, ot = run['odom'], run['odom_t']
        for (t0, o0), (t1, o1) in zip(cogs, cogs[1:]):
            if t1 - t0 < 60.0 or t1 - t0 > 300.0:
                continue
            seg = odom[bisect.bisect_left(ot, t0):bisect.bisect_left(ot, t1)]
            pv = sum(abs(o[2]) * 0.05 for o in seg if abs(o[1]) < PIVOT_VX)
            out.append({'dt': t1 - t0, 'd_deg': math.degrees(wrap(o1 - o0)),
                        'pivot_deg': math.degrees(pv), 'run': run['name']})
    return out


def float_check(runs):
    """Real RTK float stretches: position jump at fixed->float and float->fixed, and the
    float positions against a wheel + recorded-yaw dead reckoning from the last fix."""
    out = []
    for run in runs:
        gps, field = run['gps'], run['field']
        odom, ot = run['odom'], run['odom_t']
        k = 0
        while k < len(gps):
            if gps[k][3] != 'float':
                k += 1
                continue
            j = k
            while j < len(gps) and gps[j][3] == 'float':
                j += 1
            seg = [g for g in gps[k:j] if g[1] is not None]
            prev = gps[k - 1] if k > 0 else None
            nxt = gps[j] if j < len(gps) else None
            ent = {'run': run['name'], 't0': gps[k][0] - gps[0][0],
                   'dur_s': gps[j - 1][0] - gps[k][0],
                   'n': len(seg), 'prev': prev[3] if prev else None,
                   'next': nxt[3] if nxt else None}
            if prev is not None and prev[3] == 'fixed' and seg and prev[1] is not None:
                ent['jump_in_m'] = math.hypot(seg[0][1] - prev[1], seg[0][2] - prev[2])
                # dead-reckon from the last fixed position with wheel vx + recorded yaw
                x, y, tp = prev[1], prev[2], prev[0]
                offs = []
                i = bisect.bisect_left(ot, tp)
                for g in seg:
                    while i < len(odom) and odom[i][0] <= g[0]:
                        f = field.at(odom[i][0])
                        if f is not None:
                            dt = min(odom[i][0] - tp, 0.2)
                            x += odom[i][1] * math.cos(f[2]) * dt
                            y += odom[i][1] * math.sin(f[2]) * dt
                        tp = odom[i][0]
                        i += 1
                    offs.append((g[0] - seg[0][0], math.hypot(g[1] - x, g[2] - y)))
                ent['float_vs_dr_p50'] = pct([o[1] for o in offs], 50)
                ent['float_vs_dr_max'] = max(o[1] for o in offs)
            if nxt is not None and nxt[3] == 'fixed' and seg and nxt[1] is not None:
                ent['jump_out_m'] = math.hypot(nxt[1] - seg[-1][1], nxt[2] - seg[-1][2])
            out.append(ent)
            k = j
    return out


def wheel_scale(runs, chunk_s=5.0):
    """RTK-fixed distance over wheel distance on straight 5 s chunks (|wz| < 0.15 rad/s,
    |vx| > 0.1 m/s throughout): [(run, ratio, forward?, mean tilt deg, mean pitch along the
    direction of travel deg)]."""
    out = []
    for run in runs:
        odom, gps, gt = run['odom'], run['gps'], run['gps_t']
        tilt, tt = run['tilt'], run['tilt_t']
        k = 0
        while k < len(odom):
            t0 = odom[k][0]
            j = k
            dist, sign, ok = 0.0, 0, True
            while j + 1 < len(odom) and odom[j + 1][0] < t0 + chunk_s:
                vx, wz = odom[j][1], odom[j][2]
                if abs(vx) < 0.1 or abs(wz) > 0.15 or (sign and (vx > 0) != (sign > 0)):
                    ok = False
                    break
                sign = 1 if vx > 0 else -1
                dist += abs(vx) * min(odom[j + 1][0] - odom[j][0], 0.2)
                j += 1
            if ok and odom[j][0] - t0 > chunk_s - 0.5 and dist > 0.3:
                a = gps[min(bisect.bisect_left(gt, t0), len(gps) - 1)]
                b = gps[min(bisect.bisect_left(gt, odom[j][0]), len(gps) - 1)]
                if a[3] == 'fixed' and b[3] == 'fixed' and a[1] is not None and b[1] is not None:
                    tl = tilt[bisect.bisect_left(tt, t0):bisect.bisect_left(tt, odom[j][0])]
                    rr = [math.hypot(x[1], x[2]) for x in tl]
                    pp = sum(x[2] for x in tl) / len(tl) if tl else 0.0
                    out.append((run['name'], math.hypot(b[1] - a[1], b[2] - a[2]) / dist,
                                sign > 0, sum(rr) / len(rr) if rr else float('nan'),
                                pp * sign))
                k = j + 1
            else:
                k = max(j, k + 1)
    return out


def est_drift_model(d):
    """localization_status_logic today: 3 cm + 3 % of the distance (wheel)."""
    return 0.03 + 0.03 * d


def proposed_drift_model(r):
    """The model this analysis proposes (p90 fit, ekf_map with wheel vy): 5 cm + 2 %/m on
    flat ground (tilt <= 4 deg), rising linearly to 10 cm + 10 %/m at 11 deg, + 1 cm per
    pivot."""
    tilt = r['roll'] if r['roll'] == r['roll'] else 0.0
    f = min(1.0, max(0.0, (tilt - FLAT_DEG) / 7.0))
    return 0.05 + 0.05 * f + (0.02 + 0.08 * f) * r['d'] + 0.01 * r['pivots']


def model_check(windows, model):
    res = {}
    for m in MILESTONES_M:
        vals = [at_distance(rows, m) for _r, rows in windows]
        vals = [v for v in vals if v is not None]
        if not vals:
            continue
        over = sum(1 for v in vals if v['err'] > model(v))
        res[m] = {'n': len(vals), 'model_m': model(vals[0]) if callable(model) else None,
                  'under_estimated_pct': 100.0 * over / len(vals)}
    return res


def fmt(v, nd=2):
    return '-' if v != v else ('%.*f' % (nd, v))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--img', default=None)
    ap.add_argument('--json', default=None)
    ap.add_argument('--min-n', type=int, default=15,
                    help='windows needed at a distance step for the budget table')
    a = ap.parse_args()
    runs = [load_run(d) for d in a.runs]
    report = {}
    by_prefix = {p: collect(runs, p) for p, _ in PREFIXES}
    wb, wv = by_prefix['b'], by_prefix['v']
    print('usable windows: ' + ', '.join('%s %d' % (lab, len(by_prefix[p]))
                                         for p, lab in PREFIXES))
    for rn in sorted({r for r, _ in wv}):
        ws = [rows for r, rows in wv if r == rn]
        dd = [rows[-1]['d'] for rows in ws]
        rolls = [rows[-1]['roll'] for rows in ws if rows[-1]['roll'] == rows[-1]['roll']]
        print('  %s: %d windows, distance per window p50 %.1f m, max %.1f m, mean tilt '
              'p50 %.1f p90 %.1f deg'
              % (rn, len(ws), pct(dd, 50), max(dd), pct(rolls, 50), pct(rolls, 90)))
    for prefix, label in PREFIXES:
        ws = by_prefix[prefix]
        if not ws:
            continue
        print('\n## Position error by distance, %s' % label)
        t = dist_table(ws)
        report['dist_' + label] = t
        print('| class | ' + ' | '.join('%d m' % m for m in MILESTONES_M) + ' |')
        print('|---|' + '---|' * len(MILESTONES_M))
        for c, row in t.items():
            print('| %s | ' % c + ' | '.join(
                '%s / %s / %s (%d)' % (fmt(row[m]['p50']), fmt(row[m]['p90']),
                                       fmt(row[m]['max']), row[m]['n'])
                for m in MILESTONES_M) + ' |')
        print('\ncross / along p90, signed p50 (all):', ', '.join(
            '%d m: %s / %s (%s / %s)' % (m, fmt(t['all'][m]['cross_p90']),
                                         fmt(t['all'][m]['along_p90']),
                                         fmt(t['all'][m]['cross_p50_signed']),
                                         fmt(t['all'][m]['along_p50_signed']))
            for m in MILESTONES_M))
        print('\n## Budget (distance to p90 error), %s' % label)
        report['budget_' + label] = {}
        for c, pred in CLASSES.items():
            b, _ = budget(ws, pred, min_n=a.min_n)
            report['budget_' + label][c] = b
            print('| %s | ' % c + ' | '.join(
                (('%.2f m' % b[th]) if not isinstance(b[th], str) else b[th] + ' m')
                for th in THRESHOLDS_M) + ' |')
        print('\n## Time', label)
        tt = time_table(ws)
        report['time_' + label] = tt
        for ts, v in tt.items():
            print('  %3d s: n %d err p50 %s p90 %s, |yaw err| p50 %.2f p90 %.2f deg' % (
                ts, v['n'], fmt(v['err_p50']), fmt(v['err_p90']), v['yaw_p50_deg'],
                v['yaw_p90_deg']))
        st = stationary(ws)
        report['stationary_' + label] = st
        print('  stationary 60 s: n %d, drift p50 %s p90 %s max %s m' % (
            len(st), fmt(pct(st, 50), 3), fmt(pct(st, 90), 3),
            fmt(max(st) if st else float('nan'), 3)))
        ok = [(rn, [r for r in rows if r['slip'] < SLIP_RAD]) for rn, rows in ws]
        for name, model in (('est_drift (3 cm + 3 %/m)', lambda r: est_drift_model(r['d'])),
                            ('proposed (5 cm + 2 %/m flat .. 10 cm + 10 %/m at 11 deg, '
                             '+ 1 cm/pivot)',
                             proposed_drift_model)):
            for lab2, sub in (('all', ok),
                              ('flat', [(rn, [r for r in rows if r['roll'] < FLAT_DEG])
                                        for rn, rows in ok]),
                              ('slope', [(rn, [r for r in rows if r['roll'] >= SLOPE_DEG])
                                         for rn, rows in ok])):
                mc = model_check(sub, model)
                report['model_%s_%s_%s' % (label, name, lab2)] = mc
                print('  %s below the actual error, %s (no slip): %s' % (
                    name, lab2, ', '.join('%d m %.0f %% (n %d)' % (
                        m, v['under_estimated_pct'], v['n']) for m, v in mc.items())))
    if by_prefix['v'] and by_prefix['o']:
        print('\n## Budget: distance to p90 0.10 / 0.20 / 0.30 m, wheel only (v) against '
              'wheel + VIO (o), same windows')
        print('| class | wheel only | wheel + VIO |')
        print('|---|---|---|')
        for c in CLASSES:
            cells = []
            for lab in ('vy fused (branch)', 'wheel + VIO'):
                b = report['budget_' + lab][c]
                cells.append(' / '.join(('%.2f' % b[th]) if not isinstance(b[th], str)
                                        else b[th] for th in THRESHOLDS_M) + ' m')
            print('| %s | %s |' % (c, ' | '.join(cells)))
    hc = heading_from_cog(runs, 'v')
    report['heading_cog'] = hc
    print('\n## Heading error vs later COG updates (dead-reckoned yaw), n %d' % len(hc))
    for lo, hi in ((5, 30), (30, 60), (60, 100)):
        sel = [h for h in hc if lo <= h['t'] < hi]
        for lab, sub in (('no pivot', [h for h in sel if h['pivots'] == 0]),
                         ('pivot', [h for h in sel if h['pivots'] > 0])):
            e = [abs(h['err_deg']) for h in sub]
            print('  %3d-%3d s %-8s n %4d |err| p50 %.2f p90 %.2f max %.2f deg' % (
                lo, hi, lab, len(e), pct(e, 50), pct(e, 90), max(e) if e else float('nan')))
    wh = whole_bag_heading(runs)
    report['heading_bag'] = wh
    rates = [abs(w['d_deg']) / w['dt'] * 60 for w in wh if w['pivot_deg'] < 20]
    rates_p = [abs(w['d_deg']) / w['dt'] * 60 for w in wh if w['pivot_deg'] >= 45]
    per_piv = [abs(w['d_deg']) / (w['pivot_deg'] / 90.0) for w in wh if w['pivot_deg'] >= 45]
    print('\n## Whole-bag COG offset steps: %d intervals' % len(wh))
    print('  no pivot: |d offset| per min p50 %.2f p90 %.2f deg/min (n %d)' % (
        pct(rates, 50), pct(rates, 90), len(rates)))
    print('  with pivot: |d offset| per min p50 %.2f p90 %.2f, per 90 deg pivot p50 %.2f p90 '
          '%.2f deg (n %d)' % (pct(rates_p, 50), pct(rates_p, 90), pct(per_piv, 50),
                               pct(per_piv, 90), len(per_piv)))
    for lab, pre in (('vy', 'v'), ('no vy', 'b'), ('vio', 'o')):
        if not by_prefix[pre]:
            continue
        pe = pivot_events(runs, pre)
        report['pivots_' + lab] = pe
        print('  pivots in windows (%s): n %d, |error change| across a pivot p50 %.3f p90 %.3f '
              'max %.3f m; error vector change p50 %.3f p90 %.3f m; slope (tilt >=8) n %d '
              'p90 %.3f' % (
                  lab, len(pe), pct([abs(p['derr']) for p in pe], 50),
                  pct([abs(p['derr']) for p in pe], 90),
                  max([abs(p['derr']) for p in pe]) if pe else float('nan'),
                  pct([p['dvec'] for p in pe], 50), pct([p['dvec'] for p in pe], 90),
                  len([p for p in pe if p['roll'] >= SLOPE_DEG]),
                  pct([p['dvec'] for p in pe if p['roll'] >= SLOPE_DEG], 90)))
    ws_ = wheel_scale(runs)
    report['wheel_scale'] = ws_
    print('\n## Wheel scale (RTK distance / wheel distance, straight 5 s chunks)')
    for lab, sel in (('forward flat', lambda w: w[2] and w[3] < FLAT_DEG),
                     ('forward 4-8', lambda w: w[2] and FLAT_DEG <= w[3] < SLOPE_DEG),
                     ('forward slope', lambda w: w[2] and w[3] >= SLOPE_DEG),
                     ('reverse flat', lambda w: not w[2] and w[3] < FLAT_DEG),
                     ('reverse slope', lambda w: not w[2] and w[3] >= SLOPE_DEG),
                     ('pitch +>=5', lambda w: w[4] >= 5.0),
                     ('pitch -<=-5', lambda w: w[4] <= -5.0)):
        for rn in sorted({w[0] for w in ws_}):
            r = [w[1] for w in ws_ if w[0] == rn and sel(w)]
            if r:
                print('  %-14s %-6s n %4d ratio p10 %.3f p50 %.3f p90 %.3f' % (
                    lab, rn, len(r), pct(r, 10), pct(r, 50), pct(r, 90)))
    fc = float_check(runs)
    report['float'] = fc
    print('\n## RTK float stretches: %d' % len(fc))
    for f in fc:
        if f['dur_s'] < 0.5 and f['n'] < 3:
            continue
        nan = float('nan')
        print('  %s +%.0f s: %.1f s (%d fixes) %s->float->%s, jump in %s, out %s, float vs DR '
              'p50 %s max %s m'
              % (f['run'], f['t0'], f['dur_s'], f['n'], f['prev'], f['next'],
                 fmt(f.get('jump_in_m', nan)), fmt(f.get('jump_out_m', nan)),
                 fmt(f.get('float_vs_dr_p50', nan)), fmt(f.get('float_vs_dr_max', nan))))
    if a.json:
        with open(a.json, 'w') as fh:
            json.dump(report, fh, indent=1, default=str)
    if a.img:
        plots(a.img, wv, wb, hc)


def plots(img, wv, wb, hc):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    os.makedirs(img, exist_ok=True)
    grid = np.arange(0.5, 30.01, 0.5)

    def curve(ws, pred, p, min_n=30):
        out = []
        for dd in grid:
            e = [r['err'] for _x, rows in ws for r in [at_distance(rows, dd)]
                 if r is not None and pred(r)]
            out.append(pct(e, p) if len(e) >= min_n else float('nan'))
        return np.array(out)
    # 1: p90 vs distance by terrain / pivots, with the current and proposed drift models
    fig, ax = plt.subplots(figsize=(8, 5))
    for cname, col in (('all', 'k'), ('flat, no pivot', 'tab:green'),
                       ('flat, >=1 pivot', 'tab:olive'), ('slope (tilt >=8 deg)', 'tab:red')):
        pred = CLASSES[cname]
        ax.plot(grid, curve(wv, pred, 90), color=col, label='%s p90' % cname)
        if cname == 'all':
            ax.plot(grid, curve(wv, pred, 50), color=col, ls=':', label='all p50')
    ax.plot(grid, est_drift_model(grid), 'k--', lw=1, label='est_drift today: 3 cm + 3 %')
    for tilt, col in ((0.0, 'tab:green'), (11.0, 'tab:red')):
        ax.plot(grid, [proposed_drift_model({'roll': tilt, 'd': g, 'pivots': 0}) for g in grid],
                color=col, ls='--', lw=1, label='proposed model, tilt %g deg' % tilt)
    for th in THRESHOLDS_M:
        ax.axhline(th, color='0.8', lw=0.8)
    ax.set_xlabel('distance driven without RTK (m, wheel odometry)')
    ax.set_ylabel('position error vs RTK fixed (m)')
    ax.set_ylim(0, 1.2)
    ax.set_xlim(0, 25)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    ax.set_title('Dead-reckoning error (ekf_map, wheel vy fused), simulated GPS dropouts\n'
                 '(curves end where fewer than 30 windows remain; slip events excluded)',
                 fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(img, 'dr_error_vs_distance.png'), dpi=110)
    plt.close(fig)
    # 2: vy fused vs not
    fig, ax = plt.subplots(figsize=(8, 4.5))
    for ws, lab, col in ((wv, 'vy fused (0.05)', 'tab:blue'), (wb, 'vy not fused', 'tab:red')):
        ax.plot(grid, curve(ws, CLASSES['all'], 90), color=col, label=lab + ' p90')
        ax.plot(grid, curve(ws, CLASSES['all'], 50), color=col, ls=':', label=lab + ' p50')
    ax.set_xlabel('distance driven without RTK (m)')
    ax.set_ylabel('position error (m)')
    ax.set_ylim(0, 1.2)
    ax.set_xlim(0, 25)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    ax.set_title('ekf_map wheel vy fusion and coasting error (all windows, no slip)')
    fig.tight_layout()
    fig.savefig(os.path.join(img, 'dr_vy_fusion.png'), dpi=110)
    plt.close(fig)
    # 3: heading error vs time, two references
    fig, ax = plt.subplots(figsize=(8, 4.5))
    edges = np.arange(0, 101, 10)
    mids = (edges[:-1] + edges[1:]) / 2
    rec = [abs(math.degrees(r['yaw_err'])) for _x, rows in wv for r in rows]
    rec_t = [r['t'] for _x, rows in wv for r in rows]
    rec, rec_t = np.array(rec), np.array(rec_t)
    cog = np.array([abs(h['err_deg']) for h in hc])
    cog_t = np.array([h['t'] for h in hc])
    for vals, ts, lab, col in ((rec, rec_t, 'vs recorded fused yaw', 'tab:blue'),
                               (cog, cog_t, 'vs next COG measurement', 'tab:orange')):
        for p, ls in ((50, ':'), (90, '-')):
            ys = [pct(vals[(ts >= a) & (ts < b)], p) if ((ts >= a) & (ts < b)).sum() >= 30
                  else float('nan') for a, b in zip(edges[:-1], edges[1:])]
            ax.plot(mids, ys, color=col, ls=ls, marker='o', ms=3, label='%s p%d' % (lab, p))
    ax.set_xlabel('time since RTK lost (s)')
    ax.set_ylabel('|yaw error| (deg)')
    ax.set_ylim(0, 5)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    ax.set_title('Heading of the frozen-offset yaw (COG reference carries ~1-2 deg noise)',
                 fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(img, 'dr_heading_error.png'), dpi=110)
    plt.close(fig)


if __name__ == '__main__':
    sys.exit(main())
