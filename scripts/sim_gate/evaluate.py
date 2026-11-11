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
     phase; PASS: no sample > 0.10 m. Since 2026-10-17 (design section 7) measured for the
     whole robot BODY (padded Nav2 footprint at the ground-truth pose, mower_map.robot_body,
     the nav2_params.yaml footprint), not only base_link: per mowing area by its edge_mode
     (run's maps/area_settings.yaml; hard = outline, soft = nose / tail); on a drawn path
     only and with base_link inside the dock corridor only the point counts. The point
     measure is always reported (a_point); SIM_GATE_MEASURE=point judges it instead (revert).
     The body is judged in MOWING only (coverage); TRANSIT / RETURNING_HOME / docking are
     judged on the point, the body there is reported as info (a_body_info).
  i  body overhang, plan-relative (round 3): MOWING samples minus the overhang of the planned
     pose they track (lap 0 on driven arcs, row ends, pivot sweeps are planned out) <= 0.10;
     TRANSIT absolute <= 0.40; the absolute side / nose / sweep maxima are reported.
  b  hard-boundary trips (mission log "BOUNDARY OUT", formerly "HARD BOUNDARY"), boundary-line
     crossings included ("BOUNDARY OUT: body|blade crosses boundary line <id>", also counted as
     line_trips); PASS: 0.
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
sys.path.insert(0, os.path.join(SRC, 'mower_common'))
sys.path.insert(0, os.path.join(SRC, 'mower_map'))
sys.path.insert(0, os.path.join(SRC, 'mower_mission'))
sys.path.insert(0, os.path.join(SRC, 'mower_control'))
from mower_map import areas as A          # noqa: E402
from mower_map import robot_body as RB    # noqa: E402
from mower_mission import geometry as G   # noqa: E402
from mower_common import geometry as RG   # noqa: E402

OUT_LIMIT = 0.10
CORNER_RAD = 1.38          # = mission.yaml route_corner_turn_rad
CORNER_LIMIT = 0.15
CORNER_NEAR = 0.8
HALF_TRACK = RG.TRACK_SPLIT_M / 2.0   # MCU wheel split b 0.40 / 2 (was 0.24)
FLOOR = 0.06


LOOP_HZ_MIN = 18      # controller loop median (Hz, sim time) below this: INVALID
CTRL_BASE_HZ = 20.0   # nav2_params controller_server controller_frequency (sim Hz)
START_MARK = 'high_level_control(1) -> True'
RATE_RACE = 'controller_frequency not applied (sim rate race)'
_MISSED = re.compile(r'Control loop missed its desired rate of ([0-9.]+) Hz')


def run_rate(run):
    """sim_gate.sh -x RATE from the run's run.txt (1.0 when absent)."""
    try:
        with open(os.path.join(run, 'run.txt')) as fh:
            for line in fh:
                if line.startswith('rate='):
                    return float(line.split('=', 1)[1])
    except (OSError, ValueError):
        pass
    return 1.0


def rate_race(lines, rate, base_hz=CTRL_BASE_HZ):
    """True when, faster than real time, controller_server ran at an unscaled
    controller_frequency after START: its missed-rate warnings name a desired rate other
    than base x rate (mower_sim sets base x rate after every activation; a set lost in the
    dock pause left 20 Hz wall = 7 Hz sim at 3x, docs/simulation.md)."""
    if abs(rate - 1.0) < 1e-6:
        return False
    started = not any(START_MARK in ln for ln in lines)
    want = base_hz * rate
    for ln in lines:
        if not started:
            started = START_MARK in ln
            continue
        m = _MISSED.search(ln)
        if m and abs(float(m.group(1)) - want) > 0.01 * want:
            return True
    return False

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
    load_world.areas = areas
    d = doc.get('dock') or {}
    dock = A.DockPose(float(d.get('x', 0.0)), float(d.get('y', 0.0)), float(d.get('yaw', 0.0)))
    corr = A.dock_corridor(dock, areas, 0.8, 0.35, 5.0)
    if corr is not None:
        polys.append(list(corr.polygon()))
    return polys


def world_features(path):
    """(has terrain regions, has RTK injection) of a world YAML (j / k apply only then)."""
    import yaml
    with open(path) as fh:
        doc = yaml.safe_load(fh) or {}
    if isinstance(doc.get('world'), dict):
        doc = doc['world']
    terr = bool((doc.get('terrain') or {}).get('regions'))
    rtk = doc.get('rtk') or {}
    return terr, bool(rtk.get('events') or rtk.get('multipath'))


SLIP_WIN_S, SLIP_MIN_M, SLIP_RATIO = 3.0, 0.3, 3.0


def slip_events(wodom, gt, t0=0.0, win=SLIP_WIN_S, min_m=SLIP_MIN_M, ratio=SLIP_RATIO):
    """Wheel-odometry path vs ground-truth displacement over ``win`` s windows (1 s hop):
    windows with wheel >= min_m and wheel > ratio x real, merged into events
    [t_from, t_to, wheel m, real m] (relative to t0)."""
    if len(wodom) < 2 or not gt:
        return []
    gT = [r[0] for r in gt]

    def gt_at(t):
        i = max(0, min(len(gt) - 1, bisect.bisect_left(gT, t)))
        return gt[i]
    cum = [0.0]
    for a, b in zip(wodom, wodom[1:]):
        cum.append(cum[-1] + math.hypot(b[1] - a[1], b[2] - a[2]))
    wT = [r[0] for r in wodom]
    ev = []
    t = wT[0]
    while t + win <= wT[-1]:
        i, j = bisect.bisect_left(wT, t), bisect.bisect_left(wT, t + win)
        j = min(j, len(wT) - 1)
        wheel = cum[j] - cum[i]
        a, b = gt_at(wT[i]), gt_at(wT[j])
        real = math.hypot(b[1] - a[1], b[2] - a[2])
        if wheel >= min_m and wheel > ratio * real:
            if ev and wT[i] - t0 <= ev[-1][1] + 1e-6:
                e = ev[-1]
                e[1] = wT[j] - t0
                e[2] = max(e[2], wheel)
                e[3] = max(e[3], real)
            else:
                ev.append([wT[i] - t0, wT[j] - t0, wheel, real])
        t += 1.0
    return [[round(v, 2) for v in e] for e in ev]


def rtk_windows(simx, t0=0.0):
    """[t_from, t_to, mode] runs of a non-fixed sim RTK mode (/sim/status 'rtk')."""
    out = []
    for r in simx:
        mode = r[4] if len(r) > 4 else None
        if mode in (None, 'fixed'):
            if out and out[-1][1] is None:
                out[-1][1] = r[0] - t0
            continue
        if out and out[-1][1] is None and out[-1][2] == mode:
            continue
        if out and out[-1][1] is None:
            out[-1][1] = r[0] - t0
        out.append([r[0] - t0, None, mode])
    if out and out[-1][1] is None:
        out[-1][1] = (simx[-1][0] - t0) if simx else out[-1][0]
    return out


def rtk_handling(windows, loc, bt, cutting, outside_over, hold_s=3.0, resume_tail_s=120.0):
    """Per RTK-loss window: did /localization/status leave ok, did the mission hold within
    ``hold_s`` of that (blade off), resume after the fix came back, and stay inside.

    ``loc`` [t, state, ...], ``bt`` [(t, msg)], ``cutting`` [(t, bool)], ``outside_over``
    [t] of boundary samples > 0.10 m; all relative to START."""
    res = []
    for ta, tb, mode in windows:
        nxt = [w[0] for w in windows if w[0] > tb]
        horizon = min([tb + resume_tail_s] + nxt)
        td = next((r[0] for r in loc if ta <= r[0] <= tb + 10.0 and r[1] not in ('ok', None)),
                  None)
        th = next((t for t, m in bt if t >= ta and t <= horizon
                   and re.search(r'^localization \w+ \(.*\): stopped, blade off', m)), None)
        tr = next((t for t, m in bt if th is not None and t > th and t <= horizon
                   and m.startswith('localization ok again')), None)
        blade = None
        if th is not None:
            # from 0.3 s after the hold: /sim/status (10 Hz) still reports the cutter the
            # sample after the blade-off command (branch-scen4 2026-11-04: one true sample
            # 30 ms after the hold, off from 0.1 s on, judged "blade not off")
            after = [c for t, c in cutting if th + 0.3 <= t <= th + hold_s]
            blade = not any(after) if after else None
        exc = [t for t in outside_over if ta <= t <= (tr if tr is not None else horizon) + 10.0]
        e = {'from': round(ta, 1), 'to': round(tb, 1), 'kind': mode,
             'degraded_at': None if td is None else round(td, 1),
             'hold_at': None if th is None else round(th, 1),
             'resume_at': None if tr is None else round(tr, 1),
             'blade_off': blade, 'excursions': len(exc)}
        if td is None:
            e['held'] = True
            e['ok'] = not exc and th is None
            e['note'] = 'status stayed ok (drift within budget): no hold needed'
        else:
            e['held'] = th is not None and th - td <= hold_s
            e['resumed'] = tr is not None and tr >= tb
            e['ok'] = bool(e['held'] and blade is not False and e['resumed'] and not exc)
        res.append(e)
    return res


def load_kills(run):
    out = []
    try:
        with open(os.path.join(run, 'kills.jsonl')) as fh:
            for line in fh:
                line = line.strip()
                if line:
                    try:
                        out.append(json.loads(line))
                    except ValueError:
                        out.append({'error': 'bad line %r' % line[:80]})
    except OSError:
        pass
    return out


def load_body():
    """The measured body (nav2 footprint = front 0.60 / rear 0.10 / half width 0.235), no
    Nav2 padding."""
    nav2 = os.path.join(SRC, 'mower_navigation', 'config', 'nav2_params.yaml')
    return RB.load_body(padding=RB.MEASURE_PADDING, nav2_params_file=nav2, step=0.04)


def _nearest_dir(px, py, polys):
    """Unit vector from the nearest region edge point to (px, py) (outward when outside)."""
    best, bv = math.inf, (0.0, 0.0)
    for p in polys:
        n = len(p)
        for i in range(n):
            ax, ay = p[i - 1][0], p[i - 1][1]
            bx, by = p[i][0], p[i][1]
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy
            t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
            qx, qy = ax + t * dx, ay + t * dy
            d = math.hypot(px - qx, py - qy)
            if d < best:
                best, bv = d, ((px - qx) / max(d, 1e-9), (py - qy) / max(d, 1e-9))
    return bv


def body_split(body, x, y, yaw, polys, pivoting):
    """Body overhang beyond the line at one pose, split (owner 2026-10-21): 'sweep' while
    pivoting in place, else by the overhang direction in the body frame: 'nose' (front /
    tail, the outward normal mostly along the heading: row ends, lap corners) or 'side'
    (mostly lateral: edge laps). Returns (kind, metres outside, worst body point) or None."""
    c0 = RB.clearance(x, y, polys)
    if c0 >= body.radius:
        return None
    worst, wp = 0.0, None
    for px, py in body.world(x, y, yaw):
        c = RB.clearance(px, py, polys)
        if -c > worst:
            worst, wp = -c, (px, py)
    if wp is None:
        return None
    if pivoting:
        return 'sweep', worst, wp
    nx, ny = _nearest_dir(wp[0], wp[1], polys)
    along = abs(nx * math.cos(yaw) + ny * math.sin(yaw))
    lat = abs(-nx * math.sin(yaw) + ny * math.cos(yaw))
    return ('nose' if along >= lat else 'side'), worst, wp


def edge_modes(run):
    """Area name -> edge_mode from the run's maps/area_settings.yaml (default soft)."""
    try:
        import yaml
        with open(os.path.join(run, 'maps', 'area_settings.yaml')) as fh:
            doc = yaml.safe_load(fh) or {}
        dflt = (doc.get('defaults') or {}).get('edge_mode', 'soft')
        return {n: (v or {}).get('edge_mode', dflt) for n, v in (doc.get('areas') or {}).items()}, dflt
    except (OSError, ValueError, AttributeError):
        return {}, 'soft'


def measure_body(body, soft, areas, modes, dflt, x, y):
    """As map_server _measure_body: hard / soft body of the (nearest) mowing area, None on
    a drawn path only."""
    mowing = [a for a in areas if not a.is_navigation and len(a.polygon) >= 3]
    inside = [a for a in mowing if A.point_in_polygon(x, y, a.polygon)]
    if not inside:
        if any(a.is_navigation and A.point_in_polygon(x, y, a.polygon) for a in areas
               if len(a.polygon) >= 3) or not mowing:
            return None
        inside = [min(mowing, key=lambda a: A.closest_edge_point(x, y, a.polygon)[2])]
    return body if any(modes.get(a.name, dflt) == 'hard' for a in inside) else soft


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


def edge_joins(D, t_lo=-math.inf, t_hi=math.inf):
    """{sub: [(local, side, inset, D)]} from the mission's "edge start join" log lines
    (logged in [t_lo, t_hi): the sub indices of one mow_plan, a cross / repeat run each)."""
    joins = {}
    for r in D.get('rosout', []):
        if not t_lo <= r[0] < t_hi:
            continue
        msg = r[3] if len(r) > 3 else ''
        mj = re.search(r'edge start join: sub (\d+) local (\d+) side ([+-]\d+) inset '
                       r'([\d.]+) over ([\d.]+) m', str(msg))
        if mj:
            joins.setdefault(int(mj.group(1)), []).append(
                (int(mj.group(2)), int(mj.group(3)), float(mj.group(4)), float(mj.group(5))))
    return joins


def joined(pts, jl):
    """The sub-path as sent: edge-start tangent joins (mission_fsm._join_leg) applied."""
    out = list(pts)
    for loc, sgn, off, dj in jl:
        acc = 0.0
        for i in range(loc, len(pts)):
            if i > loc:
                acc += math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
            if acc >= dj:
                break
            a_, b_ = pts[max(0, i - 1)], pts[min(len(pts) - 1, i + 1)]
            hh = math.atan2(b_[1] - a_[1], b_[0] - a_[0])
            w = off * 0.5 * (1.0 + math.cos(math.pi * acc / dj))
            out[i] = (pts[i][0] - w * math.sin(hh) * sgn, pts[i][1] + w * math.cos(hh) * sgn)
    return out


def lawn_polygon(world):
    """Outer of the world's lawn area (lawn_area, else the first mowing area) after
    load_world, or None."""
    import yaml
    with open(world) as fh:
        doc = yaml.safe_load(fh)
    doc = doc.get('world', doc) if isinstance(doc.get('world'), dict) else doc
    areas = [a for a in getattr(load_world, 'areas', []) if not a.is_navigation]
    name = doc.get('lawn_area')
    lawn = next((a for a in areas if a.name == name), None) if name else None
    lawn = lawn or (areas[0] if areas else None)
    return [tuple(q) for q in lawn.polygon] if lawn is not None else None


def coverage_report(gt, status, progress, hl_state, lawn, t0):
    """Reported, not judged (A3 2026-11-05): the cut share of the lawn from ground truth
    (blade disc at the deck centre while /sim/status cutting), the mission's raster value
    (last mowed_area_percent) and plan percent, and after a STOP -> START resume the length
    the blade centre drove in MOWING over ground cut before the STOP (re-mown)."""
    from mower_mission import mowed_raster as MR
    if not lawn or not gt:
        return None
    ras = MR.MowedRaster.for_area(lawn)
    cT = [r[0] for r in status]
    r_blade = 0.5 * RG.BLADE_DISC_M

    def cutting(t):
        i = bisect.bisect_right(cT, t) - 1
        return i >= 0 and bool(status[i][4])
    # the STOP of a resume: the first IDLE after MOWING that a later MOWING follows
    stop_t = resume_t = None
    seen_mow = False
    for r in gt:
        st = hl_state(r[0])
        if st == 'MOWING':
            if stop_t is not None and resume_t is None:
                resume_t = r[0]
            seen_mow = True
        elif st.startswith('IDLE') and seen_mow and stop_t is None:
            stop_t = r[0]
    before = None
    last = None
    recut = total = 0.0
    for r in gt:
        if r[0] < t0:
            continue
        if stop_t is not None and before is None and r[0] >= stop_t:
            before = ras.data.copy()
        bx, by = MR.blade_point(r[1], r[2], r[3], (RG.DECK_CENTRE_X_M, 0.0))
        if not cutting(r[0]):
            last = None
            continue
        a = last or (bx, by)
        if math.hypot(bx - a[0], by - a[1]) <= MR.MAX_STEP_M:
            ras.stamp_segment(a[0], a[1], bx, by, r_blade)
        if before is not None and resume_t is not None and r[0] >= resume_t and \
                hl_state(r[0]) == 'MOWING':
            d = math.hypot(bx - a[0], by - a[1])
            total += d
            c = int(math.floor((bx - ras.ox) / ras.res))
            w = int(math.floor((by - ras.oy) / ras.res))
            if 0 <= w < before.shape[0] and 0 <= c < before.shape[1] and before[w, c]:
                recut += d
        last = (bx, by)
    # the last sample with an area (afterwards the mission is over and both read empty)
    rep = [p for p in progress if len(p) > 6 and p[6] is not None]
    out = {'gt_percent': round(ras.percent(lawn) or 0.0, 1),
           'mission_percent': rep[-1][6] if rep else None,
           'plan_percent': rep[-1][5] if rep else None}
    if stop_t is not None and resume_t is not None:
        out.update({'stop_t': round(stop_t - t0, 1), 'resume_t': round(resume_t - t0, 1),
                    'mowed_after_resume_m': round(total, 1), 'recut_after_resume_m':
                    round(recut, 1)})
    return out


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

    # ---- (a) boundary from ground truth: the body (padded footprint), and the point
    body = load_body()
    soft = body.soft()
    modes, dflt = edge_modes(run)
    areas = getattr(load_world, 'areas', [])
    corr_poly = polys[-1] if polys else None    # the dock corridor (load_world appends it)
    use_point = os.environ.get('SIM_GATE_MEASURE', 'body') == 'point'
    per, per_pt, per_bi = {}, {}, {}
    worst = []
    for r in gt[::2]:                          # 25 Hz
        st = state_at(r[0])
        d_pt = outside(r[1], r[2], polys)
        bd = measure_body(body, soft, areas, modes, dflt, r[1], r[2])
        if bd is None or (corr_poly and A.point_in_polygon(r[1], r[2], corr_poly)):
            d_body = d_pt       # docking / undocking (the body enters the dock), drawn paths
        else:
            d_body = max(d_pt, RB.body_outside(bd, r[1], r[2], r[3], polys))
        # body judged in MOWING only (coverage, 2026-10-17 ws/corner-nose); transits, return,
        # docking stay point-based (base_link), the body there is info (a_body_info)
        judged = d_pt        # 2026-10-21: (a) is base_link; the body is criterion (i)
        for tab, d in ((per, judged), (per_pt, d_pt), (per_bi, d_body)):
            e = tab.setdefault(st, {'n': 0, 'max': 0.0, 'gt10': 0, 'at': None})
            e['n'] += 1
            if d > e['max']:
                e['max'], e['at'] = d, [round(r[0] - t0, 1), round(r[1], 2), round(r[2], 2),
                                        round(math.degrees(r[3]))]
            if d > OUT_LIMIT:
                e['gt10'] += 1
        d = judged
        if d > OUT_LIMIT:
            worst.append((r[0] - t0, st, d, r[1], r[2]))
    n_out = sum(e['gt10'] for e in per.values())
    a_max = max((e['max'] for e in per.values()), default=0.0)
    res['criteria']['a'] = {'pass': n_out == 0, 'samples_over': n_out, 'max_m': round(a_max, 3),
                            'measure': 'point (base_link)',
                            'body': body.describe(), 'edge_modes': modes or dflt,
                            'per_phase': {k: {'max_m': round(v['max'], 3), 'over': v['gt10'],
                                              'at': v['at']} for k, v in per.items()}}
    res['a_body_info'] = {k: {'max_m': round(v['max'], 3), 'over': v['gt10'], 'at': v['at']}
                          for k, v in per_bi.items()}
    res['a_point'] = {'samples_over': sum(e['gt10'] for e in per_pt.values()),
                      'max_m': round(max((e['max'] for e in per_pt.values()), default=0.0), 3),
                      'per_phase': {k: {'max_m': round(v['max'], 3), 'over': v['gt10'],
                                        'at': v['at']} for k, v in per_pt.items()}}
    # ---- (i) body overhang beyond the line in MOWING + TRANSIT (owner 2026-10-21), measured body, no
    # padding: soft areas side <= 0.25, nose/tail <= 0.25 (pivot sweep reported); hard areas
    # the whole body inside with a 0.10 m margin (sweep included).
    SOFT_SIDE, SOFT_NOSE, HARD_MARGIN = 0.25, 0.25, 0.10
    bsp = {'side': [0.0, None], 'nose': [0.0, None], 'sweep': [0.0, None]}
    hard_min = math.inf
    hard_at = None
    # MOWING and TRANSIT (step 3 round 2 review: the 07:05 field boundary trip was in TRANSIT)
    mow = [r for r in gt if state_at(r[0]) in ('MOWING', 'TRANSIT')]
    # pivoting in place: over +-0.25 s |v| < 0.04 m/s and |w| > 0.08 rad/s; a sample within
    # 0.6 s of one counts as sweep (a pivot's start / stop ramps are part of the pivot)
    piv_t = []
    for j in range(0, len(mow), 5):
        a, b = mow[max(0, j - 12)], mow[min(len(mow) - 1, j + 12)]
        dt = b[0] - a[0]
        if dt < 0.2 or dt > 1.0:
            continue
        v = math.hypot(b[1] - a[1], b[2] - a[2]) / dt
        w = abs(math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3]))) / dt
        if v < 0.04 and w > 0.08:
            piv_t.append(mow[j][0])

    def near_pivot(t):
        k = bisect.bisect_left(piv_t, t - 0.6)
        return k < len(piv_t) and piv_t[k] <= t + 0.6

    # Plan-relative (round 3, owner option (c)): in MOWING a sample is judged against the body
    # overhang of the PLANNED pose it tracks (nearest sub-path point around the progress index,
    # the planned heading; a pivot: the planned position at the actual yaw) - lap 0 on a driven
    # arc, row ends and pivot sweeps are planned to overhang; excess over the plan <=
    # I_TRACK_M passes. TRANSIT samples are judged absolute (<= I_TRANSIT_M, the soft nose
    # trip). The absolute maxima are reported as before.
    I_TRACK_M, I_TRANSIT_M = 0.10, 0.40
    # one ~/mow_plan per cross / repeat run (and per resume): judge a sample against the plan
    # live at its time, not the last one (a cross's run 1 was scored against run 2's rows)
    mplans = sorted(D['mow_plan'], key=lambda p: p[0])
    mplan_t = [p[0] for p in mplans]

    def mplan_idx(t):
        return max(0, bisect.bisect_right(mplan_t, t) - 1) if mplans else None

    def mplan_window(j):
        return (mplan_t[j] if j > 0 else -math.inf,
                mplan_t[j + 1] if j + 1 < len(mplans) else math.inf)

    _pplans = {}

    def pplan(j):
        if j not in _pplans:
            msg = json.loads(mplans[j][1])
            subs = msg['subpaths']
            js = edge_joins(D, *mplan_window(j))
            subs = [joined([(q[0], q[1]) for q in sp], js[k]) if k in js else sp
                    for k, sp in enumerate(subs)]
            _pplans[j] = (subs, [G.cumulative_lengths([(q[0], q[1]) for q in sp])
                                 for sp in subs], msg.get('types') or [])
        return _pplans[j]

    def plan_pose(t, x, y, yaw=None):
        h = hl_at(t)
        j = mplan_idx(t)
        if j is None:
            return None
        psubs, pcums, ptypes = pplan(j)
        if h is None or h[1] != 'MOWING' or h[4] is None or h[4] < 0 or \
                h[4] >= len(psubs) or h[5] is None or h[5] < 0:
            return None
        sp, cum = psubs[h[4]], pcums[h[4]]
        if len(sp) < 2:
            return None
        i = min(h[5], len(sp) - 1)
        lo, hi = i, i
        while lo > 0 and cum[i] - cum[lo] < 1.5:
            lo -= 1
        while hi < len(sp) - 1 and cum[hi] - cum[i] < 1.5:
            hi += 1
        best = None
        cands = []
        ty = ptypes[h[4]] if h[4] < len(ptypes) and ptypes[h[4]] else ''
        for j in range(lo, hi):
            ax, ay, bx, by = sp[j][0], sp[j][1], sp[j + 1][0], sp[j + 1][1]
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy
            if L2 < 1e-12:
                continue
            u = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
            px, py = ax + u * dx, ay + u * dy
            d = math.hypot(px - x, py - y)
            # a planned pivot vertex within 0.25 m: every yaw between its headings is
            # planned there (the robot turns in place / settles), judge it at the actual yaw
            piv = any(ty[q] in 'pP' for q in range(max(0, j - 5), min(len(ty), j + 7))) \
                if len(ty) == len(sp) else False
            cands.append((d, px, py, math.atan2(dy, dx), piv))
        if not cands:
            return None
        # at a plan vertex the incoming and outgoing segments are equally near: take the one
        # whose heading matches the robot (round 4: the outgoing leg of a 97 deg corner was
        # matched before the robot had turned, so the planned overhang read 0)
        dmin = min(c[0] for c in cands)
        near = [c for c in cands if c[0] <= dmin + 0.03]
        if yaw is not None:
            best = min(near, key=lambda c: abs(math.atan2(math.sin(c[3] - yaw),
                                                          math.cos(c[3] - yaw))))
        else:
            best = min(near)
        return best[1:]
    exc = {'side': [-1.0, None], 'nose': [-1.0, None], 'sweep': [-1.0, None]}
    tr_max, tr_at = 0.0, None
    for j in range(0, len(mow), 5):                          # 10 Hz
        r = mow[j]
        mode = measure_body(body, 'soft', areas, modes, dflt, r[1], r[2])
        if mode is None or (corr_poly and A.point_in_polygon(r[1], r[2], corr_poly)):
            continue
        if mode is body:                                      # hard area
            c = RB.body_clearance(body, r[1], r[2], r[3], polys, HARD_MARGIN)
            if c < hard_min:
                hard_min, hard_at = c, [round(r[0] - t0, 1), round(r[1], 2), round(r[2], 2)]
            continue
        pv = near_pivot(r[0])
        sp = body_split(body, r[1], r[2], r[3], polys, pv)
        at = [round(r[0] - t0, 1), round(r[1], 2), round(r[2], 2), round(math.degrees(r[3]))]
        if sp and sp[1] > bsp[sp[0]][0]:
            bsp[sp[0]] = [sp[1], at]
        if not sp:
            continue
        if state_at(r[0]) != 'MOWING':
            if sp[1] > tr_max:
                tr_max, tr_at = sp[1], at
            continue
        pp = plan_pose(r[0], r[1], r[2], r[3])
        if pp is None:
            continue
        ideal = body_split(body, pp[0], pp[1], r[3] if (pv or pp[3]) else pp[2], polys, pv)
        e = sp[1] - (ideal[1] if ideal else 0.0)
        if e > exc[sp[0]][0]:
            exc[sp[0]] = [e, at + [round(ideal[1] if ideal else 0.0, 3)]]
    i_pass = max(v[0] for v in exc.values()) <= I_TRACK_M and tr_max <= I_TRANSIT_M and \
        (hard_min >= HARD_MARGIN if math.isfinite(hard_min) else True)
    res['criteria']['i'] = {'pass': i_pass, 'body': body.describe(),
                            'side_m': round(bsp['side'][0], 3), 'side_at': bsp['side'][1],
                            'nose_m': round(bsp['nose'][0], 3), 'nose_at': bsp['nose'][1],
                            'sweep_m': round(bsp['sweep'][0], 3), 'sweep_at': bsp['sweep'][1],
                            'hard_min_clearance_m': (round(hard_min, 3)
                                                     if math.isfinite(hard_min) else None),
                            'hard_at': hard_at,
                            'excess_side_m': round(exc['side'][0], 3), 'excess_side_at': exc['side'][1],
                            'excess_nose_m': round(exc['nose'][0], 3), 'excess_nose_at': exc['nose'][1],
                            'excess_sweep_m': round(exc['sweep'][0], 3),
                            'excess_sweep_at': exc['sweep'][1],
                            'transit_max_m': round(tr_max, 3), 'transit_at': tr_at}
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
    hb = grep(r'HARD BOUNDARY|BOUNDARY OUT')
    # boundary lines (virtual walls, 2026-11-02): a line crossing is a BOUNDARY OUT trip whose
    # reason names the line; counted in trips and separately
    res['criteria']['b'] = {'pass': not hb, 'trips': len(hb),
                            'line_trips': sum(1 for _t, m in hb if 'boundary line' in m),
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
    for pj in range(len(mplans)):
        tw0, tw1 = mplan_window(pj)
        gtw = [r for r in gt if tw0 <= r[0] < tw1]
        for k, sp in enumerate(json.loads(mplans[pj][1])['subpaths']):
            pts = [(p[0], p[1]) for p in sp]
            cum = G.cumulative_lengths(pts)
            for i, turn in G.route_corners(pts, CORNER_RAD):
                win = window(pts, cum, i, 1.6)
                dev, n = 0.0, 0
                for r in gtw:
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
    # (h) against the path actually sent (round 4): an edge-start tangent join ("edge start
    # join: sub K local L side S inset I over D m", mission_fsm._join_leg) offsets the leg's
    # first D m toward the interior; rebuild it here so the join is not counted as error.
    # Per ~/mow_plan (a cross / repeat run each), the one live at the sample's time.
    _hprep = {}

    def hprep(pj):
        if pj not in _hprep:
            msg = json.loads(mplans[pj][1])
            _hprep[pj] = (msg.get('types'),
                          xt_prep(msg['subpaths'], msg.get('types'),
                                  edge_joins(D, *mplan_window(pj))))
        return _hprep[pj]

    def xt_prep(subpaths, types_all, joins):
        prep = []
        for k, sp in enumerate(subpaths):
            pts = [(p[0], p[1]) for p in sp]
            if k in joins:
                pts = joined(pts, joins[k])
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
        return prep
    names = {'l': 'line', 'a': 'arc', 'p': 'pivot_vertex', 'c': 'corner', 'P': 'pivoting'}
    xt = {}
    for r in gt[::2]:
        h = hl_at(r[0])
        pj = mplan_idx(r[0])
        if pj is None:
            continue
        types_all, prep = hprep(pj)
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

    # ---- (j) / (k) sim fidelity (2026-11-03): slope world / RTK injection; n/a otherwise
    has_terrain, has_rtk = world_features(world)
    try:
        with open(os.path.join(run, 'run.txt')) as fh:
            has_rtk = has_rtk or any(l.startswith('rtk=1') for l in fh)
    except OSError:
        pass
    simx = D.get('simx', [])
    rolls = [abs(r[2]) for r in D.get('tilt', []) if r[2] is not None and r[0] >= t0]
    rolls += [abs(r[1]) for r in simx if r[1] is not None and r[0] >= t0]
    slips = slip_events([r for r in D.get('wodom', []) if r[0] >= t0], gt, t0) \
        if has_terrain else []
    slip_lines = grep(r'wheel slip on slope')
    stuck_lines = [(t, m) for t, m in bt if re.search(r'STUCK_NEEDS_HELP|needs help', m)]
    rec_lines = grep(r'boundary: recovery move|boundary: back inside|recovery plan|'
                     r'BOUNDARY OUT')
    res['fidelity'] = {'roll_max_deg': round(max(rolls), 1) if rolls else None,
                       'tilt_bands': sorted({r[1] for r in D.get('tilt', []) if r[1]}),
                       'slip_events': slips, 'mission_slip_trips': len(slip_lines),
                       'lines': ['%.1f %s' % (t, m[:200]) for t, m in
                                 sorted(slip_lines + stuck_lines + rec_lines)[:40]]}
    res['criteria']['j'] = {'pass': (not has_terrain) or (stuck == 0 and n_out == 0),
                            'applies': has_terrain, 'stuck': stuck, 'samples_over': n_out,
                            'roll_max_deg': res['fidelity']['roll_max_deg'],
                            'slip_events': len(slips), 'mission_slip_trips': len(slip_lines)}
    wins = rtk_windows([r for r in simx if r[0] >= t0], t0) if has_rtk else []
    loc = [[r[0] - t0] + list(r[1:]) for r in D.get('loc', []) if r[0] >= t0]
    cut = [(r[0] - t0, bool(r[4])) for r in D['status'] if r[0] >= t0]
    over_t = [w[0] for w in worst]
    rh = rtk_handling(wins, loc, bt, cut, over_t)
    res['criteria']['k'] = {'pass': (not has_rtk) or (bool(wins) and all(e['ok'] for e in rh)),
                            'applies': has_rtk, 'windows': rh}
    kills = load_kills(run)
    for k in kills:
        tk = k.get('t_kill')
        if tk is None:
            continue
        k['state_after_60s'] = state_at(t0 + tk + 60.0)
        k['recovered'] = bool(k.get('respawn_s') is not None and final not in (
            'STUCK_NEEDS_HELP', 'EMERGENCY', 'NAV_TO_DOCK_FAILED', 'UNDOCK_FAILED')
            and complete)
        k['lines'] = ['%.1f %s' % (t, m[:160]) for t, m in bt if tk - 1.0 <= t <= tk + 90.0][:12]
    res['kills'] = kills

    ok = all(c['pass'] for c in res['criteria'].values())
    res['verdict'] = 'PASS' if ok else 'FAIL'
    # INVALID (not FAIL): the controller missed its 20 Hz sim-time budget, i.e. the host could
    # not keep up with the sim rate (sim_gate.sh -x); the run says nothing about the stack
    reasons = []
    if 'loop_hz' in res and res['loop_hz']['median'] < LOOP_HZ_MIN:
        reasons.append('controller loop median %d Hz sim < %d Hz (host too slow for the sim '
                       'rate; rerun slower, sim_gate.sh -x)' % (res['loop_hz']['median'],
                                                                LOOP_HZ_MIN))
    try:
        with open(os.path.join(run, 'launch.log'), errors='replace') as fh:
            if rate_race(fh.readlines(), run_rate(run)):
                reasons.insert(0, RATE_RACE)
    except OSError:
        pass
    invalid = bool(reasons)
    if invalid:
        res['verdict'] = 'INVALID'
        res['invalid'] = reasons
    C = res['criteria']
    rows = [
        ('a', 'max %s outside (gt), samples > 0.10 m' % C['a']['measure'], '%.3f m, %d' % (
            C['a']['max_m'], C['a']['samples_over'])),
        ('b', 'hard-boundary trips', '%d (%d boundary line)' % (C['b']['trips'],
                                                                C['b'].get('line_trips', 0))),
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
        ('i', 'body over plan side / nose / sweep; transit (abs)', '%.3f / %.3f / %.3f; %.3f (abs %.3f / %.3f / %.3f, hard clr %s)' % (
            C['i']['excess_side_m'], C['i']['excess_nose_m'], C['i']['excess_sweep_m'],
            C['i']['transit_max_m'], C['i']['side_m'], C['i']['nose_m'], C['i']['sweep_m'],
            C['i']['hard_min_clearance_m'])),
        ('h', 'MOWING cross-track p99 line / arc / corner', ' / '.join(
            '%.3f' % C['h']['stats'][k]['p99'] if k in C['h']['stats'] else '-'
            for k in ('line', 'arc', 'corner'))),
        ('j', 'slope: STUCK / samples > 0.10 m (roll max, slips)', 'n/a' if not C['j']['applies']
         else '%d / %d (%s deg, %d windows, %d mission trips)' % (
             C['j']['stuck'], C['j']['samples_over'], C['j']['roll_max_deg'],
             C['j']['slip_events'], C['j']['mission_slip_trips'])),
        ('k', 'RTK loss: held <= 3 s / blade off / resumed / inside', 'n/a' if not C['k']['applies']
         else ' '.join('%s@%.0f:%s/%s/%s/%d' % (
             e['kind'], e['from'], 'y' if e['held'] else 'n',
             {True: 'y', False: 'n', None: '-'}[e['blade_off']],
             'y' if e.get('resumed', True) else 'n', e['excursions'])
             for e in C['k']['windows']) or 'no loss window seen'),
    ]
    cov = coverage_report(gt, D['status'], D['progress'], state_at, lawn_polygon(world), t0)
    if cov is not None:
        res['coverage'] = cov
    print('   %-2s %-44s %-40s %s' % ('', 'criterion', 'value', 'result'))
    for k, name, val in rows:
        print('   %-2s %-44s %-40s %s' % (k, name, val, 'PASS' if C[k]['pass'] else 'FAIL'))
    if cov is not None:
        print('   coverage (reported): lawn cut %.1f %% (ground truth), mission raster %s %%, '
              'plan %s %%%s' % (cov['gt_percent'], cov['mission_percent'], cov['plan_percent'],
                                '' if 'resume_t' not in cov else
                                '; resume +%.0f s: %.1f m mowed, %.1f m of it over ground cut '
                                'before the STOP' % (cov['resume_t'],
                                                     cov['mowed_after_resume_m'],
                                                     cov['recut_after_resume_m'])))
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
    print('   body over plan at [t, x, y, deg, planned]: side %s, nose %s, sweep %s; transit %s'
          % (C['i']['excess_side_at'], C['i']['excess_nose_at'], C['i']['excess_sweep_at'],
             C['i']['transit_at']))
    print('   body worst at: side %s, nose %s, sweep %s' % (C['i']['side_at'], C['i']['nose_at'],
                                                         C['i']['sweep_at']))
    print('   body (info, all phases): ' + ', '.join('%s %.3f' % (k, v['max_m'])
                                                  for k, v in res['a_body_info'].items()))
    print('   point (base_link) max outside %.3f m, %d samples > 0.10 m' % (
        res['a_point']['max_m'], res['a_point']['samples_over']))
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
    fd = res['fidelity']
    if C['j']['applies'] or fd['roll_max_deg']:
        print('   slope: roll max %s deg, tilt bands %s, slip windows %s' % (
            fd['roll_max_deg'], fd['tilt_bands'], fd['slip_events'][:10]))
        for line in fd['lines'][:20]:
            print('   ' + line)
    for e in C['k'].get('windows', []):
        print('   rtk %s %.1f..%.1f s: degraded %s, hold %s, resume %s, blade off %s, '
              'excursions %d%s' % (e['kind'], e['from'], e['to'], e['degraded_at'], e['hold_at'],
                                   e['resume_at'], e['blade_off'], e['excursions'],
                                   ' (%s)' % e['note'] if 'note' in e else ''))
    for k in res['kills']:
        if 't_kill' not in k:
            print('   kill %s: %s' % (k.get('node'), k.get('error')))
            continue
        print('   kill %s at +%.1f s: respawn %s s, state +60 s %s, recovered %s' % (
            k['node'], k['t_kill'], k.get('respawn_s'), k['state_after_60s'], k['recovered']))
        for line in k['lines']:
            print('     ' + line)
    for reason in reasons:
        print('   INVALID: %s' % reason)
    print('   VERDICT: %s' % res['verdict'])
    try:
        with open(os.path.join(run, 'result.json'), 'w') as fh:
            json.dump(res, fh, indent=1)
    except OSError as e:
        print('   (result.json not written: %s)' % e)
    return 4 if invalid else (0 if ok else 1)


if __name__ == '__main__':
    sys.exit(main(sys.argv[1], sys.argv[2]))
