"""Field incidents replayed into the mission FSM: each test was red on its pre-fix commit.

Fixtures: fixtures/<date>_<slug>.json, extracted with scripts/replay/fsm_replay.py from the
named bag + mission log (docs/DEVELOPMENT.md "Incident -> regression"). Each test feeds the
recorded inputs through the real MissionNode handlers, answers services / actions with the
recorded outcomes where the log has them, and asserts on the FSM's reaction.

| test | bag (splits) + log, UTC window | fix | red on |
|---|---|---|---|
| a | 20261102-021049 (3-4) + mission log, 02:13:50-02:19:53 | ws/heading-stale a58fee6 | d3230fd |
| b | 20261102-021049 (3-4), 02:14:20-02:15:45 | ws/boundary-recover 5e67b08 | d3230fd |
| c | incident 20261102-034715 (bag 20261102-034709_0), 03:47:08-03:47:20 | ws/dock-join e1a79db + ws/dock-join-tol | d3230fd |
| d | 20261028-023252 (9-14) + tracker log, 02:42:00-02:47:40 | e845029 + 644d368 | 0f91864 |
| e | 20261028-023252 (87-88) + mission log, 04:00:40-04:01:45 | 32ac8f8 + cc34b8d | 49cd802 |

Fixture provenance and what is derived rather than recorded: each fixture's ``notes``.
"""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import replay_lib as rl  # noqa: E402
from mower_mission import mission_fsm as f  # noqa: E402
from test_mission_fsm import Harness, _route_resp  # noqa: E402


def _logs(h, mark=0):
    return [e.text for e in h.since(mark, f.Log)]


def _states(h, mark=0):
    return [e.status['state_name'] for e in h.since(mark, f.PublishStatus)]


# ---------------------------------------------------------------------------------------
# (c) 2026-11-02 03:47 UTC: RETURN HOME from the edge-lap start of area 5, off the route graph
# ---------------------------------------------------------------------------------------
def test_c_offgraph_dock_return_joins_the_graph_instead_of_failing():
    fx = rl.load('20261102_offgraph_dock')
    rec = rl.recorded_log(fx, 'is off the route graph')
    assert rec and rl.recorded_log(fx, 'NAV_TO_DOCK_FAILED')       # what the field did
    h = Harness(auto_route=False)
    area = fx['areas'][0]
    h.fsm._region_areas = {area['index']: rl.area_dict(area)}
    h.fsm.inputs.dock_pose = tuple(fx['dock_pose'])
    r = rl.Replay(h, fx)
    r.apply(r.events[0] if r.events[0]['topic'] == '/odometry/filtered_map' else
            r.last('/odometry/filtered_map'))       # parked: the bag starts 1.9 s after START
    start = h.fsm.inputs.pose
    (t_cmd, _), = rl.recorded_log(fx, 'high_level_control(2)')
    r.run_until(t_cmd)
    m = h.mark()
    assert h.cmd(f.CMD_HOME)
    off = []
    for _ in range(40):
        r.run_until(h.t - r.base + 0.1)
        s = h.fsm._service
        if s is not None and s.name == f.SRV_PLAN_ROUTE:
            req = h._request_of(s.token).request
            sx, sy = req['start'][:2]
            if math.hypot(sx - start[0], sy - start[1]) < 0.3:
                # recorded: "start (-5.85, 1.95) is off the route graph"
                off.append(req)
                resp = _route_resp(ok=False, message='start (%.2f, %.2f) is off the route graph'
                                   % (sx, sy))
                h._apply(h.fsm.on_service_result(s.token, False, resp, now=h.t))
            else:   # a join point inside the area: on the graph (not recorded; synthetic)
                ax, ay, ayaw = h.fsm.inputs.dock_pose
                h._apply(h.fsm.on_service_result(s.token, True, _route_resp(
                    [(sx, sy, 0.0), (ax + 0.8 * math.cos(ayaw), ay + 0.8 * math.sin(ayaw),
                                     ayaw + math.pi)]), now=h.t))
            continue
        a = h.fsm._action
        if a is not None and a.name == f.ACT_NAV:
            goal = h.goal(f.ACT_NAV)['pose']
            h.fsm.inputs.pose = (goal[0], goal[1], h.fsm.inputs.pose[2])
            h.finish(f.ACT_NAV)
            continue
        if a is not None and a.name in (f.ACT_DOCK, f.ACT_FOLLOW) or h.name != 'RETURNING_HOME':
            break
    assert off, 'the recorded off-graph reply was never asked for'
    assert 'NAV_TO_DOCK_FAILED' not in _states(h, m), _logs(h, m)[-5:]
    assert any(e.name == f.ACT_NAV for e in h.since(m, f.StartAction)), \
        'no in-area join transit after the off-graph reply'


# ---------------------------------------------------------------------------------------
# (b) 2026-11-02 02:14:40 UTC: BOUNDARY OUT on a side slope (roll ~ -9.9 deg), area 5
# ---------------------------------------------------------------------------------------
def _odoms(r, t_from, t_until):
    return [e for e in r.events if e['topic'] == '/odometry/filtered_map'
            and t_from <= e['t'] <= t_until]


def test_b_side_slope_boundary_recovery_never_swings_the_body_further_out():
    fx = rl.load('20261102_side_slope_boundary')
    (t_trip, trip), = rl.recorded_log(fx, 'BOUNDARY OUT')
    (t_follow, _), = rl.recorded_log(fx, 'sending /follow_path goal')
    assert 'rear_left 0.38 m outside' in trip
    area = fx['areas'][0]
    poly = [tuple(p) for p in area['outer']]
    h = Harness(hard_boundary_body=True)
    h.areas = [rl.area_dict(area)]
    r = rl.Replay(h, fx)
    # the sub-path being mowed: the recorded mow_plan (sub-path 0 from the resume cursor)
    path = [(x, y, 0.0) for x, y in fx['plan']['poses']]
    r.apply(_odoms(r, t_follow, t_trip)[0])
    assert h.cmd(f.CMD_START)
    h.answer_services()
    for _ in range(20):                  # WAITING_FOR_RTK (fixed) -> PLANNING
        h.tick()
        h.answer_services()
        if h.fsm._action is not None:
            break
    assert h.fsm._action is not None, (h.name, _logs(h)[-6:])
    h.finish(f.ACT_PLAN, f.SUCCEEDED, {'success': True, 'drivable_subpaths': [path],
                                        'full_path': path})
    h.fsm.inputs.is_cutting = True
    r.base = h.t - t_follow
    r.k = next(k for k, e in enumerate(r.events) if e['t'] >= t_follow)
    for _ in range(int((t_trip + 2.0 - t_follow) * 10)):        # recorded inputs to the trip
        if h.fsm._hb is not None:
            break
        r.run_until(h.t - r.base + 0.1)
    assert h.fsm._hb is not None, 'the recorded track did not trip the body boundary'
    tripped_at = h.fsm.inputs.pose
    w0 = rl.worst_corner(tripped_at, poly)
    peak, m = w0, h.mark()
    # closed loop from here: the recorded poses follow the field's own (different) motions
    for _ in range(600):
        if h.fsm._hb is None or h.name != 'MOWING':
            break
        k = h.mark()
        h.tick()
        dc = h.since(k, f.DriveCmd)
        x, y, th = h.fsm.inputs.pose
        if dc:
            th += dc[-1].angular * 0.1
            x += dc[-1].linear * 0.1 * math.cos(th)
            y += dc[-1].linear * 0.1 * math.sin(th)
        rl.Replay.apply(r, {'topic': '/odometry/filtered_map',
                            'msg': {'x': x, 'y': y, 'yaw': th}})
        peak = max(peak, rl.worst_corner((x, y, th), poly))
    # field: the retrace's lining-up pivot took the worst corner from 0.42 to 0.69 m
    assert peak <= w0 + 0.05 + 1e-6, 'recovery swung the worst corner from %.3f to %.3f m' % (
        w0, peak)


# ---------------------------------------------------------------------------------------
# (a) 2026-11-02 02:19:48 UTC: START after ~6 min parked (STUCK_NEEDS_HELP at the area 5
# edge), "heading cog is 350 s old: calibration drive first", no clear 1.0 m: refused
# ---------------------------------------------------------------------------------------
def _aged_heading(st, dt):
    """The aligner's status ``dt`` s later with nothing new: only the ages grow."""
    st = dict(st)
    for k in ('age_s', 'last_cog_age_s'):
        if isinstance(st.get(k), (int, float)):
            st[k] = round(st[k] + dt, 1)
    return st


def _moved_since_fix(r, t_status, st):
    """Farthest recorded map pose from the pose at the last cog fix (heading_logic's
    moved_since_fix_m; the pre-fix aligner in the bag did not publish it)."""
    t_fix = t_status - float(st['last_cog_age_s'])
    od = [e for e in r.events if e['topic'] == '/odometry/filtered_map']
    anchor = min(od, key=lambda e: abs(e['t'] - t_fix))['msg']
    return max(math.hypot(e['msg']['x'] - anchor['x'], e['msg']['y'] - anchor['y'])
               for e in od if e['t'] >= t_fix)


def test_a_stale_cog_heading_after_a_parked_wait_does_not_refuse_start():
    import json
    fx = rl.load('20261102_stale_heading_start')
    (t_start, _), = [x for x in rl.recorded_log(fx, 'high_level_control(1)')][:1]
    assert rl.recorded_log(fx, 'heading cog is 350 s old: calibration drive first')
    assert rl.recorded_log(fx, 'WAITING_FOR_RTK -> IDLE (heading not aligned')   # the field
    area = fx['areas'][0]
    h = Harness()
    h.areas = [rl.area_dict(area)]
    h.heading_fresh = False                 # the recorded aligner status drives the heading
    r = rl.Replay(h, fx)
    r.run_until(t_start - 0.2)              # recorded inputs; the bag ends at 02:15:45
    last = r.last('/heading_aligner/status')
    st = json.loads(last['msg']['data'])
    assert st['source'] == 'cog'
    st = _aged_heading(st, t_start - last['t'])
    assert abs(st['last_cog_age_s'] - 350) < 1.5            # the log's "350 s old"
    # fields the post-fix aligner publishes (derived from the same recording): moved since
    # the fix from the map track; the IMU stayed continuous (stereo_gap_max_s < 1 s, no reset)
    st['moved_since_fix_m'] = round(_moved_since_fix(r, last['t'], st), 3)
    st['imu_continuous'] = float(st.get('stereo_gap_max_s', 0.0)) < 1.0
    r.apply({'topic': '/heading_aligner/status', 'msg': {'data': json.dumps(st)}})
    m = h.mark()
    assert h.cmd(f.CMD_START)
    for _ in range(80):                     # 4 s, as the field (START -> refusal took 3.9 s)
        h.answer_services()
        h.fsm.inputs.heading_stamp = h.t
        h.tick()
    refused = [t for t in _logs(h, m) if 'heading not aligned' in t]
    assert not refused, refused[0]
    assert h.name != 'IDLE', _logs(h, m)[-4:]


# ---------------------------------------------------------------------------------------
# (e) 2026-10-28 04:00:55 UTC: a dog beside a transit route corner, area 4: "route corner
# pivot sweep blocked" retries 1/3..3/3 (3 / 10 / 30 s) with obstacle_policy already none
# ---------------------------------------------------------------------------------------
def test_e_dog_at_a_transit_route_corner_waits_instead_of_retrying_into_an_abort():
    fx = rl.load('20261028_dog_route_corner')
    assert len(rl.recorded_log(fx, 'route corner pivot sweep blocked, retry')) == 3
    area = fx['areas'][0]
    rt = fx['route']
    t_check = rl.recorded_log(fx, 'pre-pivot check, route corner')[0][0]
    t_tr = max(t for t, _ in rl.recorded_log(fx, 'state MOWING -> TRANSIT') if t < t_check)
    h = Harness(auto_route=False)
    h.areas = [rl.area_dict(area)]
    r = rl.Replay(h, fx)
    r.run_until(t_tr - 0.5)
    x0, y0, yaw0 = h.fsm.inputs.pose
    assert math.hypot(x0 - rt['corner'][0], y0 - rt['corner'][1]) < 0.1
    hd = yaw0 + math.radians(rt['turn_deg'])
    leg = [(x0 + 0.1 * k * math.cos(hd), y0 + 0.1 * k * math.sin(hd), hd)
           for k in range(1, int(rt['leg_m'] * 10) + 1)]
    tx, ty, _ = leg[-1]
    sub = [(tx + 0.1 * k * math.cos(hd + 2.6), ty + 0.1 * k * math.sin(hd + 2.6), 0.0)
           for k in range(25)]                       # sub-path 11 from the leg start on
    assert h.cmd(f.CMD_START), (h.name, _logs(h)[-4:])
    for _ in range(20):
        h.answer_services()
        h.tick()
        if h.fsm._action is not None:
            break
    h.finish(f.ACT_PLAN, f.SUCCEEDED, {'success': True, 'drivable_subpaths': [sub],
                                        'full_path': sub})
    route = [(x0, y0, yaw0), (x0 + 0.05 * math.cos(yaw0), y0 + 0.05 * math.sin(yaw0), yaw0)] \
        + leg
    r.base = h.t - t_tr
    m = h.mark()
    for _ in range(int((fx['events'][-1]['t'] - t_tr) * 10)):
        s = h.fsm._service
        if s is not None and s.name == f.SRV_PLAN_ROUTE:   # every re-send: the same route
            h._apply(h.fsm.on_service_result(s.token, True, _route_resp(route), now=h.t))
        a = h.fsm._action
        if a is not None and a.name == f.ACT_SPIN:
            h.finish(f.ACT_SPIN)
        elif a is not None and a.name == f.ACT_FOLLOW and a.purpose != 'coverage':
            if h.fsm._route_legs is not None and h.fsm._route_legs['i'] == 0:
                # path conditioning keeps the 0.05 m stub as leg 0 (no stub trim): it ends
                # on the corner, whose pivot is the one under test
                h.finish(f.ACT_FOLLOW, arrive=False)
                continue
            break                                          # past the corner, on the leg
        if r.k >= len(r.events):
            break
        r.run_until(h.t - r.base + 0.1)
    retries = [t for t in _logs(h, m) if 'pivot sweep blocked, retry' in t]
    aborts = [e for e in h.since(m, f.RecordIncident) if e.kind == 'transit_abort']
    assert any('route corner' in t for t in _logs(h, m)), _logs(h, m)[-6:]
    assert not retries, retries
    assert not aborts


# ---------------------------------------------------------------------------------------
# (d) 2026-10-28 02:43-02:47 UTC: a 1.5 m ladder ("wood") at (-19.6, -11.3) in area 4,
# seen in parts, stacked r 0.85 pending zones (#180, #183 at 02:43:13, #317, #322 at
# 02:46:55-02:47:03; 8+ over the mow) instead of one grown zone. Obstacle tracker replay.
# ---------------------------------------------------------------------------------------
LADDER = (-19.6, -11.3)


def _tracker():
    import yaml
    from mower_map import obstacle_tracker_node as otn
    from mower_map.obstacle_tracker_core import TrackerCore
    cfg = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..',
                       'mower_map', 'config', 'obstacle_tracker.yaml')
    over = {}
    if os.path.exists(cfg):
        with open(cfg) as fh:
            over = yaml.safe_load(fh)['obstacle_tracker']['ros__parameters']
    over['auto_promote'] = True        # on in the field (the zones were proposed)
    return TrackerCore(otn.core_params(lambda k: over.get(k, otn.PARAMS.get(k))))


def test_d_ladder_seen_in_parts_makes_one_zone_not_a_stack():
    import numpy as np
    fx = rl.load('20261028_ladder_zones')
    field = [t for t, x in [(e['t'], e['log']) for e in fx['events'] if 'log' in e]
             if 'proposed' in x and 'wood' in x]
    assert len(field) >= 4                              # what the field tracker did
    area = fx['areas'][0]
    areas = [(area['index'], area['name'], [tuple(p) for p in area['outer']])]
    poses = [(e['t'], tuple(e['pose'])) for e in fx['events'] if 'pose' in e]
    c = _tracker()
    zones = {}
    pend = []
    for e in fx['events']:
        if 'det' in e:
            pend += [tuple(d) for d in e['det']]
            continue
        if 'pose' not in e:
            continue
        t = e['t']
        c.add_detections(pend, t)
        pend = []
        # the planned path ahead, approximated by the recorded track of the next 20 s
        ahead = np.array([p[:2] for tt, p in poses if t <= tt <= t + 20.0])
        res = c.step(np.empty((0, 2)), t, tuple(e['pose']), None, areas,
                     plans=(ahead,) if len(ahead) >= 2 else ())
        for tid in res['promote']:
            c.confirm_promoted(tid, t)                  # map_server accepted it (pending)
            tr = c.find_track(tid)
            zones[tid] = (t, tr.cx, tr.cy)
        for tid in res.get('resize', []):
            c.resize_sent(tid, True)
        for tid in res.get('remove', []):
            c.confirm_removed(tid)
    near = {tid: z for tid, z in zones.items()
            if math.hypot(z[1] - LADDER[0], z[2] - LADDER[1]) < 1.5}
    assert near, 'the replay made no zone at the ladder (inputs do not reproduce it)'
    assert len(near) <= 2, 'stacked zones at the ladder: %s' % sorted(
        (round(t, 1), round(x, 2), round(y, 2)) for t, x, y in near.values())


# ---------------------------------------------------------------------------------------
# (d) 2026-11-03 03:46:39 UTC (area 5, side slope roll 8-11 deg, edge lap inset 0.05): a
# rear_left trip during an edge-lap no-go detour; the recovery ended 0.06 m out (under the
# 0.35 limit), the resume went MOWING -> TRANSIT back to the chunk start (the detour step
# left start_local stale), the transit re-tripped 5 s later 0.04 m away and the spot rule
# ended in STUCK_NEEDS_HELP. Derived: the detour state at the recorded detour log line (the
# bag has no no-go zone topic); the plan is the recorded mow_plan from the leg-start join.
# ---------------------------------------------------------------------------------------
def test_d_slope_trip_in_an_edge_detour_rejoins_the_lap_without_a_retrip():
    fx = rl.load('20261103_slope_resume_retrip')
    (t_trip, trip), _second = rl.recorded_log(fx, 'BOUNDARY OUT')
    assert 'rear_left 0.35 m outside' in trip
    assert rl.recorded_log(fx, 'detour inward around it on the lap')
    assert rl.recorded_log(fx, 'left the boundary 2 times within 1.0 m')          # the field
    area = fx['areas'][0]
    poly = [tuple(p) for p in area['outer']]
    h = Harness(hard_boundary_body=True)
    h.areas = [rl.area_dict(area)]
    r = rl.Replay(h, fx)
    # from 3 s before the trip (on the detour; earlier, the field's plan-relative corner
    # allowance that this plan slice lacks kept a bend from tripping)
    t_from = t_trip - 3.0
    od = _odoms(r, t_from, t_trip)
    p0 = od[0]['msg']
    poses = fx['plan']['poses']
    j0 = min(range(len(poses)), key=lambda i: math.hypot(poses[i][0] - p0['x'],
                                                         poses[i][1] - p0['y']))
    path = [(x, y, 0.0) for x, y in poses[j0:]]
    r.apply(od[0])
    assert h.cmd(f.CMD_START)
    h.answer_services()
    for _ in range(20):
        h.tick()
        h.answer_services()
        if h.fsm._action is not None:
            break
    h.finish(f.ACT_PLAN, f.SUCCEEDED, {'success': True, 'drivable_subpaths': [path],
                                        'full_path': path})
    h.fsm.inputs.is_cutting = True
    assert h.name == 'MOWING'
    m = h.fsm.mission
    r.base = h.t - t_from
    r.k = next(k for k, e in enumerate(r.events) if e['t'] >= t_from)
    for _ in range(int((t_trip + 2.0 - t_from) * 10)):
        if h.fsm._hb is not None:
            break
        # the field robot was on the edge-lap detour ("poses 384 -> 413"; derived state)
        m.nogo_detour = {'from': 0, 'end': 413 - fx['plan']['from_pose'] - j0,
                         'why': 'no-go zone'}
        m.step = 'nogo_detour'
        r.run_until(h.t - r.base + 0.1)
    assert h.fsm._hb is not None, [t for t in _logs(h) if 'BOUNDARY' in t]
    assert 'rear_left' in [t for t in _logs(h) if 'BOUNDARY OUT' in t][0]
    assert h.fsm._hb['step'] == 'nogo_detour'
    mark = h.mark()

    def closed_loop(ticks):
        for _ in range(ticks):
            k = h.mark()
            h.tick()
            dc = h.since(k, f.DriveCmd)
            x, y, th = h.fsm.inputs.pose
            if dc:
                th += dc[-1].angular * 0.1
                x += dc[-1].linear * 0.1 * math.cos(th)
                y += dc[-1].linear * 0.1 * math.sin(th)
            rl.Replay.apply(r, {'topic': '/odometry/filtered_map',
                                'msg': {'x': x, 'y': y, 'yaw': th}})
            if h.fsm._hb is None:
                return
    closed_loop(600)
    assert h.fsm._hb is None and h.name == 'MOWING', (h.name, _logs(h, mark)[-8:])
    ev = h.fsm._hb_body_eval(h.fsm.inputs.pose, 'soft')
    assert ev['excess'] <= -0.10 + 1e-6, ev                # boundary_recover_margin_m
    assert 'TRANSIT' not in _states(h, mark), _logs(h, mark)
    # drive the rejoined lap (its sent poses) for 1 m: no re-trip, no STUCK
    for _ in range(40):
        a = h.fsm._action
        if a is not None and a.name == f.ACT_SPIN:
            x, y, th = h.fsm.inputs.pose
            d = h.goal(f.ACT_SPIN)['angle']
            for _k in range(10):
                th += d / 10.0
                rl.Replay.apply(r, {'topic': '/odometry/filtered_map',
                                    'msg': {'x': x, 'y': y, 'yaw': th}})
                h.tick()
                assert h.fsm._hb is None
            h.finish(f.ACT_SPIN, f.SUCCEEDED)
        if a is not None and a.name == f.ACT_FOLLOW:
            break
        h.tick()
    poses = h.goal(f.ACT_FOLLOW)['poses']
    acc, prev = 0.0, h.fsm.inputs.pose
    for q in poses[1:]:
        th = math.atan2(q[1] - prev[1], q[0] - prev[0])
        acc += math.hypot(q[0] - prev[0], q[1] - prev[1])
        prev = (q[0], q[1], th)
        rl.Replay.apply(r, {'topic': '/odometry/filtered_map',
                            'msg': {'x': q[0], 'y': q[1], 'yaw': th}})
        h.tick()
        assert h.fsm._hb is None and h.name == 'MOWING', _logs(h, mark)[-6:]
        assert rl.worst_corner(prev, poly) < 0.35
        if acc >= 1.0:
            break
    assert acc >= 1.0
