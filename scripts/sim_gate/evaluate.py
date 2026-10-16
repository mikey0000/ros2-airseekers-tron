#!/usr/bin/env python3
"""Judge one sim gate run (docs/simulation.md "Sim gate").

usage: evaluate.py <run_dir> <world.yaml>
<run_dir> holds data.json (extract.py) and start_epoch. Prints the PASS/FAIL table, writes
<run_dir>/result.json, exits 0 on PASS, 1 on FAIL.

Everything is judged from ground truth (/sim/ground_truth) and the recorded topics, not from
the stack's own opinion: the boundary is recomputed here with mower_map.areas from the
world's areas file and dock pose, not taken from /map_server_node/outside_m.

Criteria:
  a  max ground-truth distance outside (areas u drawn path polygons u dock corridor), per
     phase; PASS: no sample > 0.10 m.
  b  hard-boundary trips (mission log "HARD BOUNDARY"); PASS: 0.
  c  mission reached MOWING_COMPLETE, then docked and charging (sim contacts); PASS: both.
  d  STUCK_NEEDS_HELP entries, sub-paths NOT mowed, costmap-fallback /navigate_to_pose
     transits; PASS: all 0 (FollowPath / Spin aborts are reported, not judged).
  e  deviation at sharp corners (>= 79 deg, geometry.route_corners): max ground-truth
     distance from the drawn path within 0.8 m of the vertex while that path was driven, for
     coverage sub-paths and transit / dock routes; PASS: every corner <= 0.15 m.
  f  one-wheel ticks on /cmd_vel while moving (a turn whose inner wheel is in (-0.06, 0):
     one wheel turns, the other is held / creeps back under the MCU floor); PASS: 0.
     Inner wheel in [0, 0.06) (stalled by the MCU floor) is reported as "floor".
  g  time pivoting (ground truth |v| < 0.02 m/s, |w| >= 0.1 rad/s) vs moving time; reported.
  h  MOWING cross-track (ground truth to the sub-path actually sent: ~/mow_plan, conditioned
     when path_conditioning is on), median / p90 / p99 / max per segment type: line, arc,
     corner (raw plans: within 0.3 m of a drawn vertex >= 20 deg) and pivoting in place;
     PASS: p99 <= 0.10 m on line, arc and corner (design 2026-10 section 4). Field run 5
     MOWING (raw plan, all types): 0.026 / 0.074 / 0.204 / 0.262 m.
Also listed: how often each runtime corner rule of design section 3.3 fired (log / shaper),
the turn shaper rewrite rate (every moving nav-lane command, /cmd_vel_nav, MOWING and
TRANSIT, and the raw controller output /cmd_vel_nav_raw, run through
mower_control.cmd_vel_slew.shape_for_both_wheels with its defaults: the share it would
change) and the controller loop rate (/cmd_vel_nav_raw messages per second while the
controller publishes, median / p10).
"""
import bisect
import json
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(os.path.dirname(os.path.dirname(HERE)), 'src')
sys.path.insert(0, os.path.join(SRC, 'mower_map'))
sys.path.insert(0, os.path.join(SRC, 'mower_mission'))
sys.path.insert(0, os.path.join(SRC, 'mower_control'))
from mower_map import areas as A          # noqa: E402
from mower_mission import geometry as G   # noqa: E402

OUT_LIMIT = 0.10
CORNER_RAD = 1.38          # = mission.yaml route_corner_turn_rad
CORNER_LIMIT = 0.15
CORNER_NEAR = 0.8
HALF_TRACK = 0.24
FLOOR = 0.06


def load_world(path):
    import yaml
    with open(path) as fh:
        doc = yaml.safe_load(fh)
    if 'world' in doc and isinstance(doc['world'], dict):
        doc = doc['world']
    base = os.path.dirname(os.path.abspath(path))
    polys = []
    if doc.get('areas_file'):
        f = doc['areas_file']
        areas, _ = A.load_areas_file(f if os.path.isabs(f) else os.path.join(base, f))
        sel = doc.get('areas_select')
        if sel:
            areas = [a for a in areas if a.name in sel]
    else:
        areas = [A.Area(name='lawn', polygon=[tuple(p) for p in doc['lawn']])]
        for a in doc.get('areas') or []:
            areas.append(A.Area(name=a.get('name', ''), polygon=[tuple(p) for p in a['polygon']],
                                is_navigation=bool(a.get('is_navigation'))))
    polys = [list(a.polygon) for a in areas if len(a.polygon) >= 3]
    d = doc.get('dock') or {}
    dock = A.DockPose(float(d.get('x', 0.0)), float(d.get('y', 0.0)), float(d.get('yaw', 0.0)))
    corr = A.dock_corridor(dock, areas, 0.8, 0.35, 5.0)
    if corr is not None:
        polys.append(list(corr.polygon()))
    return polys


def outside(x, y, polys):
    best = math.inf
    for p in polys:
        if A.point_in_polygon(x, y, p):
            return 0.0
        best = min(best, A.closest_edge_point(x, y, p)[2])
    return best


def seg_dist(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dy * dy
    u = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2))
    return math.hypot(a[0] + u * dx - p[0], a[1] + u * dy - p[1])


def poly_dist(p, pts):
    if len(pts) == 1:
        return math.hypot(p[0] - pts[0][0], p[1] - pts[0][1])
    return min(seg_dist(p, pts[i], pts[i + 1]) for i in range(len(pts) - 1))


def window(pts, cum, i, half):
    lo = i
    while lo > 0 and cum[i] - cum[lo] < half:
        lo -= 1
    hi = i
    while hi < len(pts) - 1 and cum[hi] - cum[i] < half:
        hi += 1
    return pts[lo:hi + 1]


def main(run, world):
    D = json.load(open(os.path.join(run, 'data.json')))
    t0 = float(open(os.path.join(run, 'start_epoch')).read())
    polys = load_world(world)
    hl = D['hl']
    hT = [r[0] for r in hl]

    def hl_at(t):
        i = bisect.bisect_right(hT, t) - 1
        return hl[i] if i >= 0 else None

    def state_at(t):
        r = hl_at(t)
        return r[1] if r else '?'

    gt = [r for r in D['gt'] if r[0] >= t0 - 1.0]
    res = {'run': os.path.basename(os.path.normpath(run)), 'criteria': {}}
    if not gt:
        print('no ground truth after START')
        res['verdict'] = 'FAIL'
        json.dump(res, open(os.path.join(run, 'result.json'), 'w'), indent=1)
        return 1
    dur = gt[-1][0] - t0
    # phases
    segs = []
    for r in gt:
        st = state_at(r[0])
        if not segs or segs[-1][0] != st:
            segs.append([st, r[0], r[0]])
        segs[-1][2] = r[0]
    phases = ' -> '.join('%s(%.0fs)' % (s[0], s[2] - s[1]) for s in segs)
    print('== %s: %.0f s after START' % (res['run'], dur))
    print('   phases: %s' % phases)
    res['phases'] = phases
    res['duration_s'] = round(dur, 1)

    # ---- (a) boundary from ground truth
    per = {}
    worst = []
    for r in gt[::2]:                          # 25 Hz
        st = state_at(r[0])
        d = outside(r[1], r[2], polys)
        e = per.setdefault(st, {'n': 0, 'max': 0.0, 'gt10': 0, 'at': None})
        e['n'] += 1
        if d > e['max']:
            e['max'], e['at'] = d, [round(r[0] - t0, 1), round(r[1], 2), round(r[2], 2)]
        if d > OUT_LIMIT:
            e['gt10'] += 1
            worst.append((r[0] - t0, st, d, r[1], r[2]))
    n_out = sum(e['gt10'] for e in per.values())
    a_max = max((e['max'] for e in per.values()), default=0.0)
    res['criteria']['a'] = {'pass': n_out == 0, 'samples_over': n_out, 'max_m': round(a_max, 3),
                            'per_phase': {k: {'max_m': round(v['max'], 3), 'over': v['gt10'],
                                              'at': v['at']} for k, v in per.items()}}
    eps = []
    for w in worst:
        if eps and w[0] - eps[-1][1] < 2.0:
            eps[-1][1] = w[0]
            eps[-1][2] = max(eps[-1][2], w[2])
        else:
            eps.append([w[0], w[0], w[2], w[1], w[3], w[4]])
    res['criteria']['a']['episodes'] = [[round(e[0], 1), round(e[1], 1), e[3], round(e[2], 3),
                                         round(e[4], 2), round(e[5], 2)] for e in eps]

    logs = [(t - t0, n, m) for t, n, _lvl, m in D['rosout'] if t >= t0]
    bt = [(t, m) for t, n, m in logs if n == 'behavior_tree_node']

    def grep(pat):
        return [(t, m) for t, m in bt if re.search(pat, m)]

    # ---- (b) hard boundary
    hb = grep(r'HARD BOUNDARY')
    res['criteria']['b'] = {'pass': not hb, 'trips': len(hb),
                            'lines': ['%.1f %s' % (t, m[:160]) for t, m in hb]
                            + ['%.1f %s' % (t, m[:160]) for t, m in grep(
                                r'boundary: back inside|retrace ended|retrace aborted|'
                                r'coverage corner retry')]}

    # ---- (c) complete + docked
    states = [r[1] for r in hl if r[0] >= t0]
    complete = 'MOWING_COMPLETE' in states
    st = [r for r in D['status'] if r[0] >= t0]
    docked = bool(st and st[-1][1])
    charging = bool(st and st[-1][2])
    final = states[-1] if states else '?'
    res['criteria']['c'] = {'pass': complete and docked and charging, 'mowing_complete': complete,
                            'docked': docked, 'charging': charging, 'final_state': final}

    # ---- (d) stuck / aborts / fallback
    stuck = sum(1 for a, b in zip([''] + states, states) if b == 'STUCK_NEEDS_HELP' and a != b)
    notmowed = grep(r'sub-path \d+ NOT mowed')
    fallback = grep(r'transit via the costmap planner|navigate_to_pose transit')
    aborts = grep(r'(follow_path|navigate_to_pose|spin) finished: (ABORTED|TIMEOUT)')
    loopg = grep(r'leg loop guard|route progress watch|turn leg end')
    navgoals = set()
    for _t, sl in D['nav_status']:
        for gid, _code in sl:
            navgoals.add(gid)
    res['criteria']['d'] = {'pass': stuck == 0 and not notmowed and not fallback,
                            'stuck': stuck, 'not_mowed': len(notmowed),
                            'fallback_nav': len(fallback), 'aborts': len(aborts),
                            'nav_goals_any': len(navgoals), 'watch_lines': len(loopg),
                            'lines': ['%.1f %s' % (t, m[:160]) for t, m in
                                      (notmowed + fallback + aborts + loopg)[:40]]}

    # ---- (e) corners
    corners = []
    gtT = [r[0] for r in gt]
    plan_msg = D['mow_plan'][-1][1] if D['mow_plan'] else None
    subpaths = json.loads(plan_msg)['subpaths'] if plan_msg else []
    for k, sp in enumerate(subpaths):
        pts = [(p[0], p[1]) for p in sp]
        cum = G.cumulative_lengths(pts)
        for i, turn in G.route_corners(pts, CORNER_RAD):
            win = window(pts, cum, i, 1.6)
            dev, n = 0.0, 0
            for r in gt:
                h = hl_at(r[0])
                if h is None or h[1] != 'MOWING' or h[4] != k or h[5] < 0:
                    continue
                if abs(cum[min(h[5], len(cum) - 1)] - cum[i]) > 1.5:
                    continue
                if math.hypot(r[1] - pts[i][0], r[2] - pts[i][1]) > CORNER_NEAR:
                    continue
                n += 1
                dev = max(dev, poly_dist((r[1], r[2]), win))
            if n:
                corners.append(['cov', k, i, round(pts[i][0], 2), round(pts[i][1], 2),
                                round(math.degrees(turn)), round(dev, 3), n])
    plans = [p for p in D['plan'] if p[0] >= t0]
    croutes = [(t, json.loads(m)['poses']) for t, m in D.get('croute', []) if t >= t0]
    if croutes:
        # path conditioning: judge the route as it was conditioned and driven, not the raw
        # plan_route output (a fillet cuts the drawn corner on purpose)
        plans = croutes
    for j, (tp, pts) in enumerate(plans):
        stp = state_at(tp + 0.5)
        if stp not in ('TRANSIT', 'RETURNING_HOME') or len(pts) < 3:
            continue
        tend = plans[j + 1][0] if j + 1 < len(plans) else gt[-1][0]
        for s in segs:
            if s[1] <= tp + 0.5 <= s[2] + 0.5:
                tend = min(tend, s[2])
                break
        dockg = [t + t0 for t, m in bt if 'sending /mower_docking/dock goal' in m
                 or 'dock goal: use_vision' in m]
        dockg = [t for t in dockg if t > tp]
        if dockg:
            tend = min(tend, dockg[0])
        lo, hi = bisect.bisect_left(gtT, tp), bisect.bisect_right(gtT, tend)
        cum = G.cumulative_lengths(pts)
        for i, turn in G.route_corners(pts, CORNER_RAD):
            win = window(pts, cum, i, 1.6)
            near = [r for r in gt[lo:hi] if math.hypot(r[1] - pts[i][0], r[2] - pts[i][1])
                    <= CORNER_NEAR]
            if not near:
                continue
            dev = max(poly_dist((r[1], r[2]), win) for r in near)
            corners.append(['route:%s' % stp, j, i, round(pts[i][0], 2), round(pts[i][1], 2),
                            round(math.degrees(turn)), round(dev, 3), len(near)])
    bad = [c for c in corners if c[6] > CORNER_LIMIT]
    res['criteria']['e'] = {'pass': not bad, 'corners': len(corners), 'over': len(bad),
                            'max_m': round(max((c[6] for c in corners), default=0.0), 3),
                            'list': corners}

    # ---- (f) one-wheel ticks on /cmd_vel
    ow = floor = 0
    owph, owlane = {}, {}
    lanes = {k: [r[0] for r in D[k]] for k in ('cmd_dock', 'cmd_emerg', 'cmd_nav')}

    def lane_at(t):
        for k, ts in lanes.items():
            j = bisect.bisect_left(ts, t - 0.1)
            if j < len(ts) and ts[j] <= t + 0.05:
                return {'cmd_dock': 'docking', 'cmd_emerg': 'emergency', 'cmd_nav': 'nav'}[k]
        return 'other'
    for t, v, w in D['cmd']:
        if t < t0 or abs(v) < 0.01 or abs(w) < 0.02:
            continue
        inner = abs(v) - abs(w) * HALF_TRACK
        if -FLOOR < inner < 0.0:
            ow += 1
            s = state_at(t)
            owph[s] = owph.get(s, 0) + 1
            ln = lane_at(t)
            owlane[ln] = owlane.get(ln, 0) + 1
        elif 0.0 <= inner < FLOOR:
            floor += 1
    res['criteria']['f'] = {'pass': ow == 0, 'one_wheel': ow, 'floor_inner': floor,
                            'per_phase': owph, 'per_lane': owlane}

    # ---- (g) pivot time
    piv = mov = 0.0
    pivm = movm = 0.0
    for a, b in zip(gt, gt[1:]):
        dt = min(0.1, b[0] - a[0])
        v, w = abs(a[4]), abs(a[5])
        moving = v >= 0.02 or w >= 0.1
        if not moving:
            continue
        p = v < 0.02 and w >= 0.1
        mov += dt
        piv += dt if p else 0.0
        if state_at(a[0]) == 'MOWING':
            movm += dt
            pivm += dt if p else 0.0
    res['criteria']['g'] = {'pass': True, 'pivot_s': round(piv, 1), 'moving_s': round(mov, 1),
                            'share': round(piv / mov, 3) if mov else 0.0,
                            'mowing_pivot_s': round(pivm, 1), 'mowing_moving_s': round(movm, 1)}

    # ---- (h) cross-track by segment type
    types_all = json.loads(plan_msg).get('types') if plan_msg else None
    prep = []
    for k, sp in enumerate(subpaths):
        pts = [(p[0], p[1]) for p in sp]
        cum = G.cumulative_lengths(pts)
        ty = (types_all[k] if types_all and k < len(types_all) else '') or ''
        if len(ty) != len(pts):
            # raw plan: corner neighbourhoods by the drawn vertices, arcs by local radius
            ty = []
            vx = [i for i, _t in G.route_corners(pts, math.radians(20))]
            for i in range(len(pts)):
                if any(abs(cum[i] - cum[v]) <= 0.3 for v in vx):
                    ty.append('c')
                    continue
                lo = max(0, i - 3)
                hi = min(len(pts) - 1, i + 3)
                if hi - lo >= 2 and cum[hi] - cum[lo] > 1e-6:
                    h1 = math.atan2(pts[i][1] - pts[lo][1], pts[i][0] - pts[lo][0]) \
                        if i > lo else None
                    h2 = math.atan2(pts[hi][1] - pts[i][1], pts[hi][0] - pts[i][0]) \
                        if hi > i else None
                    if h1 is not None and h2 is not None and abs(
                            G.wrap_angle(h2 - h1)) / (0.5 * (cum[hi] - cum[lo])) >= 0.5:
                        ty.append('a')
                        continue
                ty.append('l')
            ty = ''.join(ty)
        prep.append((pts, cum, ty))
    names = {'l': 'line', 'a': 'arc', 'p': 'pivot_vertex', 'c': 'corner', 'P': 'pivoting'}
    xt = {}
    for r in gt[::2]:
        h = hl_at(r[0])
        if h is None or h[1] != 'MOWING' or h[4] is None or h[4] < 0 or h[4] >= len(prep) \
                or h[5] < 0 or str(h[2]).startswith('boundary'):
            continue
        pts, cum, ty = prep[h[4]]
        i = min(h[5], len(pts) - 1)
        lo = i
        while lo > 0 and cum[i] - cum[lo] < 1.5:
            lo -= 1
        hi = i
        while hi < len(pts) - 1 and cum[hi] - cum[i] < 1.5:
            hi += 1
        win = pts[lo:hi + 1]
        best, bj = math.inf, lo
        for j in range(len(win) - 1):
            d = seg_dist((r[1], r[2]), win[j], win[j + 1])
            if d < best:
                best, bj = d, lo + j
        if len(win) == 1:
            best = math.hypot(r[1] - win[0][0], r[2] - win[0][1])
        if abs(r[4]) < 0.02 and abs(r[5]) >= 0.1:
            key = 'P'
        else:
            key = ty[bj] if bj < len(ty) else 'l'
        name = names.get(key, key)
        if types_all and not (types_all[h[4]] if h[4] < len(types_all) else ''):
            name = 'turn-leg ' + name           # sent as planned (not conditioned)
        xt.setdefault(name, []).append(best)

    def stats(v):
        v = sorted(v)
        q = lambda f: v[min(len(v) - 1, int(f * (len(v) - 1) + 0.5))]   # noqa: E731
        return {'n': len(v), 'median': round(q(0.5), 3), 'p90': round(q(0.9), 3),
                'p99': round(q(0.99), 3), 'max': round(v[-1], 3)}
    allv = [d for vals in xt.values() for d in vals]
    hs = {k: stats(v) for k, v in xt.items() if v}
    if allv:
        hs['all'] = stats(allv)
    judged = [k for k in ('line', 'arc', 'corner', 'turn-leg line', 'turn-leg arc')
              if k in hs and hs[k]['n'] >= 25]
    res['criteria']['h'] = {'pass': bool(hs) and all(hs[k]['p99'] <= 0.10 for k in judged),
                            'stats': hs}

    # ---- runtime corner rules of design 3.3 still firing
    shp = [m for t, m in D['shape'] if t >= t0]
    slew = [m for t, n, m in logs if n == 'cmd_vel_slew']
    res['rules_fired'] = {
        'route corner pivot (angle split)': len(grep(r'route corner at')),
        'route corner reached rule': len(grep(r'route leg .*corner .* reached')),
        'route stub trim / start join': len(grep(r'stub before the corner|first leg joins')),
        'route cusp about-turn': len(grep(r'route cusp at')),
        'route progress watch': len(grep(r'route progress watch|no progress along route')),
        'turn-leg cap (fix 10)': len(grep(r'turn leg end .*leg done')),
        'leg loop guard': len(grep(r'leg loop guard')),
        'costmap fallback nav': len(fallback),
        'follow premature continue': len(grep(r'goal checker fired early')),
        'coverage pivot vertex reached': len(grep(r'pivot vertex .* reached')),
        'swath-end reverse turns replaced': sum(int(m.group(1)) for _t, msg in grep(
            r'swath-end reverse turn') for m in [re.search(r'(\d+) swath-end', msg)] if m),
        'swath-end reverse turns kept': sum(int(m.group(1)) for _t, msg in grep(
            r'swath-end reverse turn') for m in [re.search(r'(\d+) kept as planned', msg)] if m),
        'coverage pivots (vertex / leg start)': len(grep(r'pivot .* deg in place')),
        'shaper pivot-instead': sum('instead' in m for m in shp),
        'shaper widened arc': sum(bool(re.search(r'arc r=', m)) for m in shp),
        'shaper pivot hold': sum('hold' in m for m in shp),
        'pivot assist creep': sum('pivot assist ON' in m for m in slew),
        'stuck guard': len(grep(r'stuck: |STUCK')),
    }

    # ---- shaper rewrite rate and controller loop rate (reported)
    try:
        from mower_control.cmd_vel_slew import shape_for_both_wheels
    except Exception as e:                    # noqa: BLE001 - reporting only
        shape_for_both_wheels = None
        print('   (shaper rewrite rate not computed: %s)' % e)

    def rewrite(rows, phases=None):
        n = k = 0
        for t, v, w in rows:
            if t < t0 or (abs(v) < 0.01 and abs(w) < 0.02):
                continue
            if phases is not None and state_at(t) not in phases:
                continue
            n += 1
            k += shape_for_both_wheels(v, w)[2] is not None
        return round(100.0 * k / n, 1) if n else None
    if shape_for_both_wheels is not None:
        res['shaper_rewrite_pct'] = {
            'nav_mowing': rewrite(D['cmd_nav'], ('MOWING',)),
            'nav_transit': rewrite(D['cmd_nav'], ('TRANSIT', 'RETURNING_HOME')),
            'raw_all': rewrite(D.get('cmd_raw', []))}
    raw_t = [r[0] for r in D.get('cmd_raw', []) if r[0] >= t0]
    bins = {}
    for t in raw_t:
        bins[int(t)] = bins.get(int(t), 0) + 1
    full = sorted(c for s, c in bins.items() if s - 1 in bins and s + 1 in bins)
    if full:
        res['loop_hz'] = {'median': full[len(full) // 2],
                          'p10': full[int(0.1 * (len(full) - 1))], 'seconds': len(full)}

    ok = all(c['pass'] for c in res['criteria'].values())
    res['verdict'] = 'PASS' if ok else 'FAIL'
    C = res['criteria']
    rows = [
        ('a', 'max outside (gt), samples > 0.10 m', '%.3f m, %d' % (C['a']['max_m'],
                                                                  C['a']['samples_over'])),
        ('b', 'hard-boundary trips', '%d' % C['b']['trips']),
        ('c', 'MOWING_COMPLETE / docked / charging', '%s / %s / %s (final %s)' % (
            C['c']['mowing_complete'], C['c']['docked'], C['c']['charging'],
            C['c']['final_state'])),
        ('d', 'STUCK / NOT mowed / fallback nav (aborts)', '%d / %d / %d (%d)' % (
            C['d']['stuck'], C['d']['not_mowed'], C['d']['fallback_nav'], C['d']['aborts'])),
        ('e', 'sharp corners > 0.15 m (max dev)', '%d of %d (%.3f m)' % (
            C['e']['over'], C['e']['corners'], C['e']['max_m'])),
        ('f', 'one-wheel ticks (floor-stalled inner)', '%d (%d) %s' % (
            C['f']['one_wheel'], C['f']['floor_inner'],
            ' '.join('%s:%d' % kv for kv in sorted(C['f']['per_lane'].items())))),
        ('g', 'pivot time / moving time', '%.0f / %.0f s (%.0f %%)' % (
            C['g']['pivot_s'], C['g']['moving_s'], 100 * C['g']['share'])),
        ('h', 'MOWING cross-track p99 line / arc / corner', ' / '.join(
            '%.3f' % C['h']['stats'][k]['p99'] if k in C['h']['stats'] else '-'
            for k in ('line', 'arc', 'corner'))),
    ]
    print('   %-2s %-44s %-40s %s' % ('', 'criterion', 'value', 'result'))
    for k, name, val in rows:
        print('   %-2s %-44s %-40s %s' % (k, name, val, 'PASS' if C[k]['pass'] else 'FAIL'))
    for k, v in C['h']['stats'].items():
        print('   cross-track %-13s n %5d  median %.3f  p90 %.3f  p99 %.3f  max %.3f' % (
            k, v['n'], v['median'], v['p90'], v['p99'], v['max']))
    print('   rules fired: ' + ', '.join('%s %d' % kv for kv in res['rules_fired'].items()))
    if 'shaper_rewrite_pct' in res:
        print('   shaper rewrite %%: nav MOWING %s, nav TRANSIT %s, raw controller %s' % (
            res['shaper_rewrite_pct']['nav_mowing'], res['shaper_rewrite_pct']['nav_transit'],
            res['shaper_rewrite_pct']['raw_all']))
    if 'loop_hz' in res:
        print('   controller loop Hz: median %d, p10 %d (%d s)' % (
            res['loop_hz']['median'], res['loop_hz']['p10'], res['loop_hz']['seconds']))
    print('   boundary per phase: ' + ', '.join('%s %.3f' % (k, v['max_m'])
                                                for k, v in C['a']['per_phase'].items()))
    for e in C['a']['episodes']:
        print('   outside > 0.10 m: t %.1f..%.1f %s max %.3f at (%.2f, %.2f)' % tuple(e))
    for line in C['b']['lines']:
        print('   ' + line)
    for c in corners:
        if c[6] > CORNER_LIMIT or c[0] == 'cov':
            print('   corner %-22s #%d idx %d (%.2f, %.2f) %+d deg: max dev %.3f m (%d samples)%s'
                  % (c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7],
                     '  FAIL' if c[6] > CORNER_LIMIT else ''))
    for line in C['d']['lines'][:15]:
        print('   ' + line)
    print('   VERDICT: %s' % res['verdict'])
    try:
        with open(os.path.join(run, 'result.json'), 'w') as fh:
            json.dump(res, fh, indent=1)
    except OSError as e:
        print('   (result.json not written: %s)' % e)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main(sys.argv[1], sys.argv[2]))
