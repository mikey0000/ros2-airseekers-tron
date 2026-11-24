#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""How far ekf_map follows an offset RTK float after a dropout (2026-11-25 coast mode).

Reads one ``ekf_replay.py`` run whose variants withheld the GPS (``wh_len``) and then fed
``fl_len`` s of float offset by (``fl_dx``, ``fl_dy``) at sigma floor ``fl_floor``. Truth is
the RTK-FIXED ``gps`` rows of inputs.csv (map frame). For every window the error vector at the
float start is the dead-reckoning error; the pull at time T into the float is the error
(estimate - truth) along the offset as a share of it: 0 = on the truth, 1 = on the float::

    python3 scripts/replay/float_pull.py OUT_DIR

Prints, per variant: the pull p50 / p90 at 0.6, 2, 5 and 10 s into the float, and the error
p90 against truth at the float start, at its end and 2 s after FIXED returned. Map metres.
"""

import bisect
import csv
import json
import math
import os
import sys

TIMES = (0.6, 2.0, 5.0, 10.0)


def pct(xs, p):
    xs = sorted(xs)
    if not xs:
        return float('nan')
    k = (len(xs) - 1) * p / 100.0
    lo = int(k)
    hi = min(lo + 1, len(xs) - 1)
    return xs[lo] + (xs[hi] - xs[lo]) * (k - lo)


def load_xy(path):
    with open(path) as fh:
        rows = [(float(r['t']), float(r['x']), float(r['y'])) for r in csv.DictReader(fh)]
    rows.sort()
    return rows


def at(rows, ts, t, gap=0.5):
    k = bisect.bisect_left(ts, t)
    if k <= 0 or k >= len(rows) or rows[k][0] - rows[k - 1][0] > gap:
        return None
    a, b = rows[k - 1], rows[k]
    f = (t - a[0]) / (b[0] - a[0])
    return a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2])


def main(out):
    with open(os.path.join(out, 'variants.json')) as fh:
        variants = json.load(fh)
    truth = []
    with open(os.path.join(out, 'inputs.csv')) as fh:
        for r in csv.DictReader(fh):
            if r['kind'] == 'gps' and r['v2'] == 'fixed' and r['x']:
                truth.append((float(r['t']), float(r['x']), float(r['y'])))
    truth.sort()
    tt = [r[0] for r in truth]
    wins = {}
    with open(os.path.join(out, 'windows.csv')) as fh:
        for r in csv.DictReader(fh):
            wins.setdefault(r['variant'], []).append((float(r['start']), float(r['end'])))
    print('variant  floor  n   pull p50/p90 at %s s   err p90 start / float end / +2 s fixed'
          % ' / '.join('%g' % t for t in TIMES))
    for name, o in variants.items():
        est = load_xy(os.path.join(out, '%s.csv' % name))
        et = [r[0] for r in est]
        off = math.hypot(o['fl_dx'], o['fl_dy'])
        if off <= 0.0:
            continue
        u = (o['fl_dx'] / off, o['fl_dy'] / off)
        pulls = {t: [] for t in TIMES}
        e0s, e1s, e2s = [], [], []

        def err(t):
            p, q = at(est, et, t), at(truth, tt, t, gap=1.0)
            return None if p is None or q is None else (p[0] - q[0], p[1] - q[1])
        for s, e in wins.get(name, []):
            e0 = err(e)
            if e0 is None:
                continue
            e0s.append(math.hypot(*e0))
            for t in TIMES:
                et_ = err(e + t) if t <= o['fl_len'] else None
                if et_ is not None:
                    pulls[t].append((et_[0] * u[0] + et_[1] * u[1]) / off)
            e1 = err(e + o['fl_len'] - 0.2)
            e2 = err(e + o['fl_len'] + 2.0)
            if e1 is not None:
                e1s.append(math.hypot(*e1))
            if e2 is not None:
                e2s.append(math.hypot(*e2))
        cells = ['%.2f/%.2f' % (pct(pulls[t], 50), pct(pulls[t], 90)) if pulls[t] else '-'
                 for t in TIMES]
        print('%-8s %.2f %3d   %s   %.3f / %.3f / %.3f' % (
            name, o['fl_floor'], len(e0s), '  '.join(cells), pct(e0s, 90), pct(e1s, 90),
            pct(e2s, 90)))


if __name__ == '__main__':
    main(sys.argv[1])
