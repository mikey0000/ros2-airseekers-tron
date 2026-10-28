"""Option C (2026-10-23): no-go zones in the mission - mid-mow re-plan with progress kept,
the pre-pivot sweep check and its fallbacks (a)-(e), the edge-lap detour, selective costmap
clears. docs/analysis/2026-10-23_obstacle_nogo_options.md section C."""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import pytest                                              # noqa: E402

from mower_mission import mission_fsm as f                 # noqa: E402
from mower_mission import nogo                             # noqa: E402
from mower_mission.fsm.phases import nogo as nogo_phase    # noqa: E402
from test_mission_fsm import (Harness, follow_current, line, plan, square,  # noqa: E402
                              start_until_planning)
from test_path_conditioning import ring                    # noqa: E402

Z = nogo.Zone


def logs(h, mark=0):
    return [e.text for e in h.since(mark, f.Log)]


def starts(h, mark, name):
    return [e for e in h.since(mark, f.StartAction) if e.name == name]


def zones(h, *zs, marks=()):
    h.fsm.inputs.nogo_zones = nogo.ZoneSet(h.t, list(zs), list(marks))
    h.fsm.inputs.nogo_stamp = h.t


# =====================================================================
# pure geometry: plan_pivot fallbacks (a)-(d), blocked (e)
# =====================================================================
D90 = math.radians(90)


def test_pivot_clear_when_nothing_in_the_sweep():
    assert nogo.plan_pivot(0, 0, 0, D90, [Z(3.0, 3.0, 0.3)]).kind == 'clear'
    assert nogo.plan_pivot(0, 0, 0, D90, []).kind == 'clear'


def test_a_other_direction_when_only_the_planned_sweep_hits():
    p = nogo.plan_pivot(0, 0, 0, D90, [Z(0.45, 0.45, 0.08, 1, 'trunk')])
    assert p.kind == 'other' and len(p.steps) == 1
    assert p.steps[0][0] == 'spin' and abs(p.steps[0][1] - (D90 - 2 * math.pi)) < 1e-9
    assert 'trunk' in p.why and p.hits


def test_a_other_direction_refused_beyond_the_body_limit_then_b_back_up():
    # the long way round would put the body out (excess > the planned pivot's own)
    def body(poses):
        return 0.5 if min(q[2] for q in poses) < -0.1 else 0.0
    p = nogo.plan_pivot(0, 0, 0, D90, [Z(0.45, 0.45, 0.08)], body=body)
    assert p.kind == 'backup'
    assert p.steps == [('backup', 0.3), ('spin', D90)]


def test_b_back_up_is_capped_by_the_body_limit_behind():
    d = nogo.plan_pivot(0, 0, 0, D90, [Z(0.12, 0.62, 0.05)], backoff=lambda pose, dist: 0.2)
    assert d.kind == 'backup' and d.steps[0] == ('backup', 0.2)
    # no room behind (< 0.10 m): no back-up
    d = nogo.plan_pivot(0, 0, 0, D90, [Z(0.0, 0.62, 0.05)], backoff=lambda pose, dist: 0.0)
    assert d.kind != 'backup'


def test_c_k_turn_when_no_room_behind_at_the_vertex():
    def bo(pose, dist):            # the body limit is behind the robot only at yaw 0
        return 0.0 if abs(pose[2]) < 1e-6 else dist
    p = nogo.plan_pivot(0, 0, 0, D90, [Z(0.0, 0.62, 0.05), Z(0.75, 0.0, 0.05)], backoff=bo)
    assert p.kind == 'kturn'
    (k1, a1), (k2, d2), (k3, a3) = p.steps
    assert (k1, k2, k3) == ('spin', 'backup', 'spin')
    assert abs(a1 + a3 - D90) < 1e-9 and d2 == pytest.approx(0.3)


def test_d_pivot_later_along_the_path():
    p = nogo.plan_pivot(0, 0, 0, D90, [Z(0.0, 0.62, 0.05)], backoff=lambda pose, d: 0.0,
                        kturn=False)
    assert p.kind == 'shift' and p.steps[0] == ('drive', 0.3) and p.steps[1][0] == 'spin'


def test_d_pivot_earlier_needs_room_behind():
    # shifts (-0.15,) only, no room behind: no shift
    p = nogo.plan_pivot(0, 0, 0, D90, [Z(0.0, 0.62, 0.05)], backoff=lambda pose, d: 0.0,
                        kturn=False, shifts=(-0.15,))
    assert p.kind == 'blocked'


def test_e_blocked_when_nothing_is_clear():
    p = nogo.plan_pivot(0, 0, 0, D90, [Z(0.0, 0.5, 0.4, 3, 'trunk')])
    assert p.kind == 'blocked' and p.steps == [] and p.hits[0].zone.id == 3


# =====================================================================
# the mission: pre-pivot gate on coverage pivots (fallbacks executed)
# =====================================================================
def _cov_pivot_h(monkeypatch, plans, zs=None, **params):
    """ring_h with the planned pivot's check answered by ``plans`` (a list, one per call)."""
    params.setdefault('path_conditioning', True)
    params.setdefault('pivot_backout_m', 0.0)
    calls = []

    def fake(x, y, yaw, delta, discs, **kw):
        calls.append((x, y, yaw, delta))
        return plans.pop(0) if plans else nogo.PivotPlan('clear')
    monkeypatch.setattr(nogo_phase.nogo, 'plan_pivot', fake)
    h = Harness(**params)
    h.areas = [square(-8, -4, 12)]
    r = ring()
    start_until_planning(h)
    y0 = math.atan2(r[1][1] - r[0][1], r[1][0] - r[0][0])
    h.fsm.inputs.pose = (r[0][0], r[0][1], y0 + math.radians(140))
    zones(h, *(zs or [Z(r[0][0] + 0.3, r[0][1] + 0.3, 0.1, 7, 'trunk', 'mark')]))
    sp = [(q[0], q[1], math.atan2(r[min(i + 1, len(r) - 1)][1] - q[1],
                                  r[min(i + 1, len(r) - 1)][0] - q[0]))
          for i, q in enumerate(r)]
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([sp, line(-4.0, 3.0, -1.0, 3.0)]))
    h.fsm.inputs.is_cutting = True
    h.tick()
    return h, calls


def test_cov_gate_a_pivots_the_other_way(monkeypatch):
    h, calls = _cov_pivot_h(monkeypatch, [nogo.PivotPlan('other', [('spin', -4.0)], 'x')])
    a = h.pending_action(f.ACT_SPIN)
    assert a.purpose == 'cov_pivot' and h.goal(f.ACT_SPIN)['angle'] == -4.0
    assert calls and h.fsm.mission.nogo_turns == 1
    assert any('pre-pivot check' in t and '(a)' in t for t in logs(h))


def test_cov_gate_b_backs_up_then_checks_again_and_pivots(monkeypatch):
    h, calls = _cov_pivot_h(monkeypatch, [nogo.PivotPlan('backup', [('backup', 0.3),
                                                                      ('spin', 1.0)], 'x')])
    a = h.pending_action(f.ACT_BACKUP)
    g = h.goal(f.ACT_BACKUP)
    assert a.purpose == 'nogo_step' and g['distance_m'] == 0.3 and g['speed_mps'] >= 0.06
    assert any('(b)' in t for t in logs(h))
    m = h.mark()
    h.finish(f.ACT_BACKUP)
    h.fsm.inputs.is_cutting = True
    h.tick(n=3)
    # back at the pivot: checked again from the new pose (clear now) -> the planned pivot
    assert len(calls) == 2
    assert h.pending_action(f.ACT_SPIN).purpose == 'cov_pivot'
    assert not starts(h, m, f.ACT_BACKUP)


def test_cov_gate_c_k_turn_runs_half_back_up_then_the_rest(monkeypatch):
    h, calls = _cov_pivot_h(monkeypatch, [nogo.PivotPlan(
        'kturn', [('spin', 0.8), ('backup', 0.25), ('spin', 0.8)], 'x')])
    assert h.pending_action(f.ACT_SPIN).purpose == 'nogo_step'
    assert h.goal(f.ACT_SPIN)['angle'] == 0.8
    h.finish(f.ACT_SPIN)
    assert h.pending_action(f.ACT_BACKUP).purpose == 'nogo_step'
    assert h.goal(f.ACT_BACKUP)['distance_m'] == 0.25
    h.finish(f.ACT_BACKUP)
    h.tick(n=3)
    assert h.pending_action(f.ACT_SPIN).purpose == 'cov_pivot'
    assert any('(c)' in t for t in logs(h))


def test_cov_gate_d_drives_the_pivot_point_later(monkeypatch):
    h, _calls = _cov_pivot_h(monkeypatch, [nogo.PivotPlan('shift', [('drive', 0.3),
                                                                      ('spin', 1.0)], 'x')])
    a = h.pending_action(f.ACT_DRIVE)
    assert a.purpose == 'nogo_step' and h.goal(f.ACT_DRIVE)['distance_m'] == 0.3
    assert any('(d)' in t for t in logs(h))


def test_cov_gate_maneuvers_are_capped_then_e_skips_the_leg(monkeypatch):
    bk = [('backup', 0.3), ('spin', 1.0)]
    h, calls = _cov_pivot_h(monkeypatch, [nogo.PivotPlan('backup', list(bk), 'x'),
                                          nogo.PivotPlan('backup', list(bk), 'x'),
                                          nogo.PivotPlan('backup', list(bk), 'x')])
    for _ in range(2):
        h.finish(f.ACT_BACKUP)
        h.tick(n=3)
    # third check: maneuvers used up (2 per pivot) -> (e): a mark is no zone to re-plan
    # around -> skip the leg (blocked_pivot_policy skip)
    assert len(calls) == 3
    assert any('(e)' in t and 'skip the leg' in t for t in logs(h))
    assert h.fsm.mission.pivot_skips == 1
    assert h.name != 'STUCK_NEEDS_HELP'


def test_cov_gate_e_replans_around_a_pending_zone_in_the_area(monkeypatch):
    r = ring()
    z = Z(r[0][0] + 0.5, r[0][1] + 0.2, 0.25, 9, 'trunk', 'pending')
    hit = nogo.nz.SweepHit(z, 0.0)
    h, _calls = _cov_pivot_h(monkeypatch, [nogo.PivotPlan('blocked', [], 'x', [hit])], zs=[z])
    a = h.pending_action(f.ACT_PLAN)
    assert a.purpose == 'nogo_replan'
    obs = h.goal(f.ACT_PLAN)['obstacles']
    assert len(obs) == 1 and len(obs[0]) == 16
    assert any(math.hypot(q[0] - z.x, q[1] - z.y) > z.r for q in obs[0])
    assert h.fsm.mission.nogo_replans[0] == 1 and h.blade == h.fsm.blade_on


# =====================================================================
# route corners
# =====================================================================
def _corner_route(deg, tgt):
    a = [(0.1 * i, 0.0, 0.0) for i in range(21)]
    hd = math.radians(deg)
    b = [(2.0 + 0.1 * i * math.cos(hd), 0.1 * i * math.sin(hd), hd) for i in range(1, 20)]
    return a + b + [tuple(tgt)]


def _route_corner_h(zs):
    from test_mission_fsm import _answer_route, _route_h, _route_resp
    h = _route_h()
    zones(h, *zs)
    _answer_route(h, resp=_route_resp(_corner_route(100.0, h.fsm.mission.transit_target)))
    h.finish(f.ACT_FOLLOW)                     # at the corner (2, 0), yaw 0
    return h


def test_route_corner_a_turns_the_other_way_round_a_trunk_in_the_sweep():
    h = _route_corner_h([Z(2.45, 0.45, 0.08, 4, 'trunk', 'pending')])
    a = h.pending_action(f.ACT_SPIN)
    ang = h.goal(f.ACT_SPIN)['angle']
    assert a.purpose == 'route_corner' and ang < -math.pi       # -260 deg
    assert abs(ang - (math.radians(100) - 2 * math.pi)) < 0.05
    assert any('route corner' in t and '(a)' in t for t in logs(h))


def test_route_corner_e_reroutes_once_then_pivots_as_planned():
    h = _route_corner_h([Z(2.0, 0.55, 0.45, 5, 'trunk', 'pending')])
    assert any('(e)' in t and 're-route' in t for t in logs(h))
    assert h.fsm.mission.nogo_route_aborts
    assert h.name != 'STUCK_NEEDS_HELP'


def _incidents(h, mark=0):
    return [(e.kind, e.detail) for e in h.since(mark, f.RecordIncident)]


def test_route_corner_dog_in_the_sweep_waits_then_continues_without_an_abort():
    # field 2026-10-28: a dog beside a route corner -> "pivot sweep blocked" -> transit
    # abort, 47 times. Now: the dynamic wait at the corner, then the transit continues.
    dog = Z(2.0, 0.55, 0.45, 7, 'dog', 'candidate')
    h = _route_corner_h([dog])
    assert h.fsm._dyn is not None and h.fsm._dyn.get('nogo')
    assert h.fsm.mission.step == 'dyn_wait'
    assert any('dog in the sweep: waiting' in t for t in logs(h))
    assert not [k for k, _ in _incidents(h) if k == 'transit_abort']
    mark = len(h.fx)
    h.tick(n=50)                               # 5 s: the dog is still there -> still waiting
    zones(h, dog)
    h.tick(n=20)
    assert h.fsm._dyn is not None and h.fsm.mission.step == 'dyn_wait'
    zones(h)                                   # the dog left the sweep
    for _ in range(40):
        zones(h)
        h.tick()
    assert h.fsm._dyn is None
    assert h.fsm.mission.step == 'transit' and h.name == 'TRANSIT'
    assert h.fsm._service is not None and h.fsm._service.name == f.SRV_PLAN_ROUTE
    assert not [k for k, _ in _incidents(h, mark) if k == 'transit_abort']
    assert not h.fsm.mission.nogo_route_aborts


def test_route_corner_dog_wait_times_out_into_the_re_route():
    dog = Z(2.0, 0.55, 0.45, 7, 'dog', 'candidate')
    h = _route_corner_h([dog])
    assert h.fsm._dyn is not None
    for _ in range(170):                       # > dynamic_wait_transit_s (15 s)
        zones(h, dog)
        h.tick()
    assert h.fsm._dyn is None
    assert ('transit_abort', 'route corner pivot sweep blocked') in _incidents(h)
    assert h.fsm.mission.nogo_route_aborts and h.name != 'STUCK_NEEDS_HELP'


def test_route_corner_static_zone_still_re_routes_without_a_wait():
    h = _route_corner_h([Z(2.0, 0.55, 0.45, 5, 'trunk', 'pending')])
    assert h.fsm._dyn is None
    assert any('(e)' in t and 're-route' in t for t in logs(h))


def test_living_sweep_hits_only_living_candidates():
    zs = [Z(0.45, 0.45, 0.2, 1, 'dog', 'candidate'), Z(0.45, 0.45, 0.2, 2, 'trunk', 'pending'),
          Z(0.45, 0.45, 0.2, state='mark')]
    assert nogo.living_sweep_hits(0, 0, 0, D90, zs) == ['dog']
    assert nogo.living_sweep_hits(0, 0, 0, D90, zs[1:]) == []
    assert nogo.living_sweep_hits(0, 0, 0, -D90, zs[:1]) == []   # the other way is clear


# =====================================================================
# mid-mow re-plan with progress kept
# =====================================================================
ROWS = [line(0.5, 1.0, 4.5, 1.0, 41), line(4.5, 2.0, 0.5, 2.0, 41), line(0.5, 3.0, 4.5, 3.0, 41)]


def _mowing_row2(**params):
    params.setdefault('nogo_replan_min_interval_s', 30.0)
    h = Harness(**params)
    start_until_planning(h)
    h.fsm.inputs.pose = (0.5, 1.0, 0.0)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([list(sp) for sp in ROWS]))
    follow_current(h)                          # row 0 done
    h.tick()
    if h.fsm._action is not None and h.fsm._action.name == f.ACT_NAV:
        h.finish(f.ACT_NAV)
    h.fsm.inputs.is_cutting = True
    h.tick()
    assert h.pending_action(f.ACT_FOLLOW)
    m = h.fsm.mission
    assert m.sub_i == 1
    h.fsm.inputs.pose = (2.5, 2.0, math.pi)    # half of row 1 mowed
    h.tick(n=3)
    assert m.progress_local == 20
    return h


def test_replan_around_a_new_zone_on_the_remaining_path_keeps_progress():
    h = _mowing_row2()
    m = h.fsm.mission
    mk = h.mark()
    zones(h, Z(1.5, 2.0, 0.2, 3, 'trunk', 'pending'))
    h.tick(dt=1.1)
    assert h.pending_action(f.ACT_PLAN).purpose == 'nogo_replan'
    assert any(isinstance(e, f.CancelActions) for e in h.since(mk))
    obs = h.goal(f.ACT_PLAN)['obstacles']
    assert len(obs) == 1 and nogo.zone_in_plan(Z(1.5, 2.0, 0.2), obs)
    assert any('no-go re-plan 1/3 of area 0' in t for t in logs(h, mk))
    # the planner splits row 1 at the hole and keeps rows 0 and 2
    new = [line(0.5, 1.0, 4.5, 1.0, 41), line(4.5, 2.0, 2.0, 2.0, 26),
           line(1.0, 2.0, 0.5, 2.0, 6), line(0.5, 3.0, 4.5, 3.0, 41)]
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan(new))
    ac = h.fsm.cursor.area(0)
    assert ac.completed == {0}                 # row 0 mowed; row 1's piece only in part
    assert m.sub_i == 1 and 20 <= m.start_local <= 21    # resumes where the robot was
    assert any('already mowed' in t for t in logs(h, mk))
    assert h.fsm.mission.nogo_replan is None


def test_replan_is_rate_limited_and_a_zone_in_the_plan_is_not_replanned():
    h = _mowing_row2()
    zones(h, Z(1.5, 2.0, 0.2, 3, 'trunk', 'pending'))
    h.tick(dt=1.1)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([list(sp) for sp in ROWS]))
    h.fsm.inputs.is_cutting = True
    h.tick(n=3)
    m0 = h.mark()
    # the same zone again: already a hole of the plan
    zones(h, Z(1.55, 2.0, 0.25, 3, 'trunk', 'pending'))
    h.tick(dt=1.1, n=3)
    assert not starts(h, m0, f.ACT_PLAN)
    # a second zone within nogo_replan_min_interval_s: no re-plan
    zones(h, Z(1.5, 3.0, 0.2, 4, 'stone', 'pending'))
    h.tick(dt=1.1, n=3)
    assert not starts(h, m0, f.ACT_PLAN)


def test_candidate_and_expired_zones_never_replan_and_the_plan_stays():
    h = _mowing_row2()
    m0 = h.mark()
    zones(h, Z(1.5, 2.0, 0.2, 3, 'trunk', 'candidate'))
    h.tick(dt=1.1, n=2)
    assert not starts(h, m0, f.ACT_PLAN)
    zones(h, Z(1.5, 2.0, 0.2, 3, 'trunk', 'pending'))
    h.fsm.inputs.nogo_stamp = h.t - 60.0       # stale message: ignored
    h.tick(dt=1.1, n=2)
    assert not starts(h, m0, f.ACT_PLAN)


def test_replan_failure_keeps_the_old_plan_and_resumes():
    h = _mowing_row2()
    zones(h, Z(1.5, 2.0, 0.2, 3, 'trunk', 'pending'))
    h.tick(dt=1.1)
    h.finish(f.ACT_PLAN, f.ABORTED, {'success': False, 'message': 'boom'})
    m = h.fsm.mission
    assert len(m.subpaths) == 3 and m.sub_i == 1 and m.start_local == 20
    assert any('keeping the old plan' in t for t in logs(h))


def test_mowed_progress_maps_done_rows_and_a_prefix():
    mowed = [[(0.5, 1.0), (4.5, 1.0)], [(4.5, 2.0), (2.5, 2.0)]]
    new = [line(0.5, 1.0, 4.5, 1.0, 41), line(4.5, 2.1, 0.5, 2.1, 41),
           line(0.5, 3.0, 4.5, 3.0, 41)]
    done, prefix = nogo.mowed_progress(new, mowed, tol=0.12)
    assert done == {0} and prefix == {1: 20}


# =====================================================================
# pending zones on a re-START (policy): holes for the current mow
# =====================================================================
def test_restart_plans_pending_zones_as_holes_and_replanned_zones_stay_holes():
    h = Harness()
    a = square(0, 0, 5)
    a['obstacles'] = [nogo.zone_polygon(2.0, 2.0, 0.25)]       # pending zone in the map
    h.areas = [a]
    start_until_planning(h)
    obs = h.goal(f.ACT_PLAN)['obstacles']
    assert len(obs) == 1                       # the pending zone is a hole of this mow
    m = h.fsm.mission
    m.nogo_extra[0] = [nogo.zone_polygon(3.0, 3.0, 0.2)]
    assert len(h.fsm._nogo_area_holes(0)) == 2  # + zones added mid-mow (re-plans, run 2)


# =====================================================================
# edge laps (B6): detour inward around it, rejoin, no stop-wait
# =====================================================================
def _edge_h():
    h = Harness()
    start_until_planning(h)
    h.fsm.inputs.pose = (0.0, 0.0, 0.0)
    lap = line(0.0, 0.0, 5.0, 0.0, 51)         # lap 0 on the south line
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([lap, line(4.5, 2.0, 0.5, 2.0)]))
    h.fsm.inputs.is_cutting = True
    h.tick()
    assert h.pending_action(f.ACT_FOLLOW) and h.fsm._edge_lap(0)
    return h


def test_edge_lap_abort_detours_inward_around_a_zone_and_rejoins():
    h = _edge_h()
    h.fsm.inputs.pose = (1.4, 0.0, 0.0)
    h.tick(n=2)
    zones(h, Z(2.5, 0.05, 0.1, 6, 'stone', 'candidate'))
    mk = h.mark()
    h.fsm.inputs.collision_stamp = h.t
    h.finish(f.ACT_FOLLOW, f.ABORTED)
    a = h.pending_action(f.ACT_FOLLOW)
    assert a.purpose == 'nogo_detour'
    poses = h.goal(f.ACT_FOLLOW)['poses']
    assert max(q[1] for q in poses) > 0.4      # inward (+y) past the stone's inner side
    assert min(q[1] for q in poses) >= -1e-9   # never outside the line
    assert not any('edge blocked: skip' in t for t in logs(h, mk))
    assert not any('path blocked' in t for t in logs(h, mk))    # no stop-wait
    end = h.fsm.mission.nogo_detour['end']
    h.finish(f.ACT_FOLLOW)
    a = h.pending_action(f.ACT_FOLLOW)
    assert a.purpose != 'nogo_detour' and h.fsm.mission.start_local == end
    assert any('rejoining the lap' in t for t in logs(h, mk))


def test_edge_lap_proactive_detour_for_a_zone_ahead_without_stopping():
    h = _edge_h()
    h.fsm.p.nogo_replan = False                # zone not re-planned: the detour handles it
    h.fsm.inputs.pose = (1.0, 0.0, 0.0)
    h.tick(n=2)
    zones(h, Z(2.6, 0.1, 0.1, 2, 'stone', 'candidate'))
    h.tick(dt=1.1)
    assert h.pending_action(f.ACT_FOLLOW).purpose == 'nogo_detour'
    assert h.fsm.mission.nogo_edge_detours == 1


def test_edge_detour_without_a_disc_is_none():
    lap = [(0.1 * i, 0.0, 0.0) for i in range(50)]
    assert nogo.edge_detour(lap, 0, [Z(2.0, 2.0, 0.2)], lambda x, y: y > 0) is None


# =====================================================================
# costmap clears on retries: only the 5 s obstacle layer
# =====================================================================
def test_retry_clear_is_selective_by_default_and_full_when_off():
    for sel in (True, False):
        h = Harness(clear_costmaps_selective=sel)
        start_until_planning(h)
        h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(2, 2, 4, 2)]))
        mk = h.mark()
        h.finish(f.ACT_NAV, f.ABORTED)
        c = [e for e in h.since(mk, f.CallService) if e.name == f.SRV_CLEAR_COSTMAPS]
        assert c and all(e.request['selective'] is sel for e in c)


def test_edge_detour_ignores_marks_and_too_large_offsets():
    h = _edge_h()
    h.fsm.inputs.pose = (1.0, 0.0, 0.0)
    h.tick(n=2)
    m0 = h.mark()
    zones(h, marks=[Z(2.6, 0.1, 0.1, state='mark')])       # 60 s marks: never a detour
    h.tick(dt=1.1)
    zones(h, Z(2.6, 0.4, 0.25, 2, 'stone', 'candidate'))   # needs 0.99 m > 0.60: none
    h.tick(dt=1.1)
    assert not [e for e in starts(h, m0, f.ACT_FOLLOW)]
    assert h.fsm.mission.nogo_edge_detours == 0


def test_replan_hole_is_grown_for_the_nose():
    h = _mowing_row2()
    zones(h, Z(1.5, 2.0, 0.2, 3, 'trunk', 'pending'))
    h.tick(dt=1.1)
    obs = h.goal(f.ACT_PLAN)['obstacles'][0]
    r = min(math.hypot(q[0] - 1.5, q[1] - 2.0) for q in obs)
    assert r >= 0.2 + h.fsm.p.nogo_plan_grow_m - 1e-6


def test_zone_in_plan_needs_half_of_the_disc_inside_a_hole():
    hole = nogo.zone_polygon(0.0, 0.0, 1.1)
    assert nogo.zone_in_plan(Z(0.0, 0.0, 0.85), [hole])
    assert nogo.zone_in_plan(Z(0.6, 0.0, 0.85), [hole])          # mostly inside: stacked
    assert not nogo.zone_in_plan(Z(1.5, 0.0, 0.85), [hole])      # mostly outside: new
    assert not nogo.zone_in_plan(Z(0.0, 0.0, 2.0), [hole])       # outgrew its hole
    assert not nogo.zone_in_plan(Z(0.0, 0.0, 0.3), [])


def test_route_corner_mark_under_a_dog_is_the_dog_and_clears_with_it():
    # field 2026-10-28: "sweeps dog (-23.82, -4.24) r 0.35, mark (-23.89, -4.22) r 0.35"
    dog = Z(2.0, 0.55, 0.35, 7, 'dog', 'candidate')
    mark = Z(1.95, 0.57, 0.35, state='mark')
    h = _route_corner_h([dog])
    zones(h, dog, marks=[mark])
    assert h.fsm._dyn is not None and h.fsm._dyn.get('nogo')
    for _ in range(40):                        # the dog candidate expired, the mark stays
        zones(h, marks=[mark])
        h.tick()
    assert h.fsm._dyn is None and h.fsm.mission.step == 'transit'
    m = len(h.fx)
    _answer_corner_route(h)
    zones(h, marks=[mark])
    h.finish(f.ACT_FOLLOW)
    assert h.pending_action(f.ACT_SPIN).purpose == 'route_corner'
    assert not [k for k, _ in _incidents(h, m) if k == 'transit_abort']


def _answer_corner_route(h):
    from test_mission_fsm import _answer_route, _route_resp
    _answer_route(h, resp=_route_resp(_corner_route(100.0, h.fsm.mission.transit_target)))
