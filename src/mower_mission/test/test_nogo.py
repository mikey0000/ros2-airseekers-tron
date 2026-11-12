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
from keep_harness import KeepHarness                       # noqa: E402

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
    params.setdefault('backout_m', 0.0)
    calls = []

    def fake(x, y, yaw, delta, discs, **kw):
        calls.append((x, y, yaw, delta))
        return plans.pop(0) if plans else nogo.PivotPlan('clear')
    monkeypatch.setattr(nogo_phase.nogo, 'plan_pivot', fake)
    h = KeepHarness(**params)      # the steps run with the blade on (owner rule 2026-10-23)
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
    assert h.blade and any('blade kept on' in t for t in logs(h))   # inside the area
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


def test_cov_gate_e_replans_around_an_accepted_zone_in_the_area(monkeypatch):
    r = ring()
    z = Z(r[0][0] + 0.5, r[0][1] + 0.2, 0.25, 9, 'trunk', 'accepted')
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
    # The sweep check runs at a route corner the conditioner pivots at (no fillet fits). The
    # toy 2 m legs would take the shipped r 0.8 m fillet (0.95 m tangents) round the 100 deg
    # corner, so a fillet radius whose tangents exceed the legs forces the pivot here.
    h = _route_h(path_conditioning_route_r_min_m=3.0)
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


def test_route_corner_e_reroutes_once_first():
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
    for k in range(21):                        # half of row 1 mowed
        h.fsm.inputs.pose = (4.5 - 0.1 * k, 2.0, math.pi)
        h.tick()
    h.tick(n=2)
    assert m.progress_local == 20
    return h


def test_replan_around_a_new_zone_on_the_remaining_path_keeps_progress():
    h = _mowing_row2()
    m = h.fsm.mission
    mk = h.mark()
    zones(h, Z(1.5, 2.0, 0.2, 3, 'trunk', 'accepted'))
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
    zones(h, Z(1.5, 2.0, 0.2, 3, 'trunk', 'accepted'))
    h.tick(dt=1.1)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([list(sp) for sp in ROWS]))
    h.fsm.inputs.is_cutting = True
    h.tick(n=3)
    m0 = h.mark()
    # the same zone again: already a hole of the plan
    zones(h, Z(1.55, 2.0, 0.25, 3, 'trunk', 'accepted'))
    h.tick(dt=1.1, n=3)
    assert not starts(h, m0, f.ACT_PLAN)
    # a second zone within nogo_replan_min_interval_s: no re-plan
    zones(h, Z(1.5, 3.0, 0.2, 4, 'stone', 'accepted'))
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
    zones(h, Z(1.5, 2.0, 0.2, 3, 'trunk', 'accepted'))
    h.tick(dt=1.1)
    h.finish(f.ACT_PLAN, f.ABORTED, {'success': False, 'message': 'boom'})
    m = h.fsm.mission
    assert len(m.subpaths) == 3 and m.sub_i == 1 and m.start_local == 20
    assert any('keeping the old plan' in t for t in logs(h))


# =====================================================================
# pending zones on a re-START (policy 2026-11-04): never holes; accepted ones are
# =====================================================================
def test_restart_plans_accepted_zones_as_holes_and_replanned_zones_stay_holes():
    h = Harness()
    a = square(0, 0, 5)
    a['obstacles'] = [nogo.zone_polygon(2.0, 2.0, 0.25)]       # accepted zone in the map
    h.areas = [a]
    start_until_planning(h)
    obs = h.goal(f.ACT_PLAN)['obstacles']
    assert len(obs) == 1                       # the accepted zone is a hole of this mow
    m = h.fsm.mission
    m.nogo_extra[0] = [nogo.zone_polygon(3.0, 3.0, 0.2)]
    assert len(h.fsm._nogo_area_holes(0)) == 2  # + zones added mid-mow (re-plans, run 2)


def test_restart_never_plans_a_pending_zone_as_a_hole():
    """Field 2026-11-04 00:02: a stuck-spot proposal (pending, #1097) became a hole on the
    STOP / START re-plan: plan 541 -> 526 m, cursor discarded, mow restarted at sub-path 1."""
    h = Harness()
    a = square(0, 0, 5)
    acc, pend = nogo.zone_polygon(2.0, 2.0, 0.25), nogo.zone_polygon(3.5, 3.5, 1.0)
    a['obstacles'] = [acc, pend]
    a['obstacle_pending'] = [False, True]
    h.areas = [a]
    start_until_planning(h)
    obs = h.goal(f.ACT_PLAN)['obstacles']
    assert obs == [list(acc)]
    m = h.fsm.mission
    assert m.areas[0]['obstacles'] == [acc] and m.areas[0]['pending_obstacles'] == [pend]
    assert any('1 pending (unreviewed) keepout' in t for t in logs(h))


def test_pending_zone_on_the_remaining_path_never_replans():
    h = _mowing_row2()
    m0 = h.mark()
    zones(h, Z(1.5, 2.0, 0.2, 3, 'trunk', 'pending'))
    h.tick(dt=1.1, n=3)
    assert not starts(h, m0, f.ACT_PLAN)
    assert h.fsm.mission.nogo_replans.get(0, 0) == 0


def test_cov_gate_e_pending_zone_in_the_sweep_skips_the_leg_no_replan(monkeypatch):
    r = ring()
    z = Z(r[0][0] + 0.5, r[0][1] + 0.2, 0.25, 9, 'trunk', 'pending')
    hit = nogo.nz.SweepHit(z, 0.0)
    h, _calls = _cov_pivot_h(monkeypatch, [nogo.PivotPlan('blocked', [], 'x', [hit])], zs=[z])
    assert h.fsm.mission.nogo_replans.get(0, 0) == 0
    assert h.fsm._action is None or h.fsm._action.name != f.ACT_PLAN
    assert any('(e)' in t and 'skip the leg' in t for t in logs(h))


# =====================================================================
# edge laps (B6): detour inward around it, rejoin, no stop-wait
# =====================================================================
def _edge_h(**params):
    h = Harness(**params)
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


def _branch(h, x, y0, y1, age=0.0):
    """Lethal local-costmap cells of a branch lying across the lap at ``x`` (no zone, no
    detection: stereo depth only), as mission_node feeds them (map frame)."""
    n = int(round((y1 - y0) / 0.05))
    h.fsm.inputs.costmap_lethal = [(x + 0.02 * (k % 2), y0 + 0.05 * k) for k in range(n + 1)]
    h.fsm.inputs.costmap_lethal_stamp = h.t - age


def _abort_on_branch(x, y0, y1, age=0.0, costmap=True):
    h = _edge_h(nogo_edge_costmap=costmap)       # off by default (2026-11-05)
    h.fsm.inputs.pose = (1.6, 0.0, 0.0)
    h.tick(n=2)
    _branch(h, x, y0, y1, age)
    mk = h.mark()
    h.fsm.inputs.collision_stamp = h.t          # MPPI "Optimizer fail" -> patience exceeded
    h.finish(f.ACT_FOLLOW, f.ABORTED)
    return h, mk


def test_edge_lap_costmap_only_obstacle_detours_tags_and_rejoins():
    """Field 2026-11-04 (area 3, 5 times): a branch across edge lap 0 seen only as stereo
    lethal cells -> follow_path aborted -> 'edge blocked: skip 2.0 m'. Now: detour inward
    round the cells, tag the obstacle on the map, rejoin the lap."""
    h, mk = _abort_on_branch(2.5, -0.3, 0.5)
    a = h.pending_action(f.ACT_FOLLOW)
    assert a.purpose == 'nogo_detour', logs(h, mk)
    poses = h.goal(f.ACT_FOLLOW)['poses']
    # the cells reach 0.5 m in: body half width + clearance past them (> the zone cap 0.6)
    assert max(q[1] for q in poses) >= 0.5 + h.fsm.p.robot_width_m / 2.0 + 0.1 - 0.02
    assert not any('edge blocked: skip' in t for t in logs(h, mk))
    inc = [e for e in h.since(mk, f.RecordIncident) if e.kind == 'edge_detour']
    assert inc and abs(inc[0].x - 2.5) < 0.15 and 'costmap obstacle' in inc[0].detail
    assert h.blade                              # owner policy: blade on in-area detours
    end = h.fsm.mission.nogo_detour['end']
    h.finish(f.ACT_FOLLOW)
    assert h.pending_action(f.ACT_FOLLOW).purpose != 'nogo_detour'
    assert h.fsm.mission.start_local == end
    assert any('rejoining the lap' in t for t in logs(h, mk))


def test_edge_lap_costmap_obstacle_right_ahead_backs_up_then_detours():
    """Sim 2026-11-04: the abort stops the robot at the branch, a detour from there aborted
    in 1 s (every MPPI trajectory collides): back up first, bend the detour from there."""
    h, mk = _abort_on_branch(2.0, -0.3, 0.5)       # cells 0.4 m ahead of the robot at 1.6
    # not Nav2 BackUp (its footprint check refuses with the nose on the cells): DriveCmd
    assert h.fsm._action is None and h.fsm.mission.step == 'nogo_detour'
    assert any('back up' in t and 'then detour' in t for t in logs(h, mk))
    cmds = [e for e in h.since(mk, f.DriveCmd)]
    assert cmds and cmds[-1].linear < 0.0 and cmds[-1].angular == 0.0
    x = 1.6
    for _ in range(4):                             # reversing: still going
        x -= 0.1
        h.fsm.inputs.pose = (x, 0.0, 0.0)
        h.tick()
        assert h.fsm._action is None and h.since(mk, f.DriveCmd)[-1].linear < 0.0
    h.fsm.inputs.pose = (1.1, 0.0, 0.0)            # backed up 0.5 m
    h.tick()
    a = h.pending_action(f.ACT_FOLLOW)
    assert a.purpose == 'nogo_detour'
    poses = h.goal(f.ACT_FOLLOW)['poses']
    assert abs(poses[0][0] - 1.1) < 0.06            # bent from the lap pose by the robot
    assert max(q[1] for q in poses) >= 0.5 + 0.235 + 0.1 - 0.02
    assert not any('edge blocked: skip' in t for t in logs(h, mk))


def test_edge_lap_costmap_obstacle_too_deep_for_a_detour_is_skipped():
    h, mk = _abort_on_branch(2.5, -0.3, 1.6)         # needs ~1.9 m > 1.5 m
    assert any('no inward detour fits' in t for t in logs(h, mk))
    assert any('edge blocked: skip' in t for t in logs(h, mk))


def test_edge_lap_stale_costmap_is_not_used():
    h, mk = _abort_on_branch(2.5, -0.3, 0.5, age=10.0)
    assert any('edge blocked: skip' in t for t in logs(h, mk))
    assert h.fsm.mission.nogo_edge_detours == 0


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


def test_edge_detour_ignores_marks_and_too_large_offsets():
    h = _edge_h()
    h.fsm.inputs.pose = (1.0, 0.0, 0.0)
    h.tick(n=2)
    m0 = h.mark()
    # interior 60 s marks: never a detour (perimeter marks, within nogo_perimeter_band_m of
    # the line, do since 2026-11-11: test_edge_lap_bends_inward_round_a_perimeter_mark...)
    zones(h, marks=[Z(2.6, 0.7, 0.5, state='mark')])
    h.tick(dt=1.1)
    zones(h, Z(2.6, 0.4, 0.25, 2, 'stone', 'candidate'))   # needs 0.99 m > 0.60: none
    h.tick(dt=1.1)
    assert not [e for e in starts(h, m0, f.ACT_FOLLOW)]
    assert h.fsm.mission.nogo_edge_detours == 0


def test_replan_hole_is_grown_for_the_nose():
    h = _mowing_row2()
    zones(h, Z(1.5, 2.0, 0.2, 3, 'trunk', 'accepted'))
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


def test_edge_detour_round_a_chain_of_discs_terminates_fast():
    """Sim 2026-11-04: the chain push loop never ended when abs(lat_in - o) rounded to just
    below reach after o = lat_in + reach (mission_node hung at the branch)."""
    import random
    import signal
    import time
    rnd = random.Random(7)
    lap = [(0.05 * i, 0.0, 0.0) for i in range(120)]
    reach = 0.071 + 0.235 + 0.10                # disc r + edge_detour half_width + clearance
    deep = next(y for y in (rnd.uniform(0.85, 0.95) for _ in range(10000))
                if abs(y - (y + reach)) < reach)    # the rounding that looped forever
    discs = [Z(1.5 + rnd.uniform(-0.05, 0.05), rnd.uniform(-0.1, 0.8), 0.071)
             for _ in range(300)] + [Z(1.5, deep, 0.071)]

    def boom(*_a):
        raise TimeoutError('edge_detour did not return')
    old = signal.signal(signal.SIGALRM, boom)
    signal.alarm(5)
    try:
        t0 = time.monotonic()
        res = nogo.edge_detour(lap, 0, discs, lambda x, y: y > 0, max_offset_m=1.5,
                               ramp_m=1.2)
        dt = time.monotonic() - t0
    finally:
        signal.alarm(0)
        signal.signal(signal.SIGALRM, old)
    assert res is not None and dt < 1.0
    poses, _end = res
    assert max(q[1] for q in poses) >= deep + reach - 0.02


def test_edge_lap_abort_on_a_dense_costmap_blob_is_bounded():
    """A wide blob of lethal cells (hedge, bed) on the lap: the disc set is merged (<= 0.2 m
    discs past 300 cells) and the abort is handled in well under a tick budget."""
    import time
    h = _edge_h(nogo_edge_costmap=True)
    h.fsm.inputs.pose = (1.6, 0.0, 0.0)
    h.tick(n=2)
    h.fsm.inputs.costmap_lethal = [(2.2 + 0.05 * i, -0.2 + 0.05 * j)
                                   for i in range(40) for j in range(40)]
    h.fsm.inputs.costmap_lethal_stamp = h.t
    polys = h.fsm._region()
    t0 = time.monotonic()
    cm = h.fsm._nogo_costmap_discs(16, polys)
    assert time.monotonic() - t0 < 0.5
    assert cm and len(cm) <= 300 and all(abs(z.r - 0.142) < 1e-9 for z in cm)
    mk = h.mark()
    h.fsm.inputs.collision_stamp = h.t
    t0 = time.monotonic()
    h.finish(f.ACT_FOLLOW, f.ABORTED)
    assert time.monotonic() - t0 < 1.0
    assert any('edge blocked: skip' in t or 'no inward detour fits' in t
               for t in logs(h, mk))     # 1.8 m deep: no detour fits, skipped


def test_deep_costmap_detour_rejoins_the_lap_at_a_shallow_angle():
    """Sim 2026-11-04 (branch-scen5 501-503 s): a 1.31 m detour over a 1.2 m ramp came back
    onto lap 0 at ~60 deg and the robot overshot 0.14 m outside the line. The costmap
    detour's ramp scales with its offset (nogo_edge_costmap_ramp_per_m)."""
    h = Harness(nogo_edge_costmap=True)
    h.areas = [square(0, 0, 12)]
    start_until_planning(h)
    h.fsm.inputs.pose = (0.0, 0.0, 0.0)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(0.0, 0.0, 12.0, 0.0, 121),
                                            line(11.5, 2.0, 0.5, 2.0)]))
    h.fsm.inputs.is_cutting = True
    h.tick()
    assert h.pending_action(f.ACT_FOLLOW) and h.fsm._edge_lap(0)
    h.fsm.inputs.pose = (1.6, 0.0, 0.0)
    h.tick(n=2)
    _branch(h, 3.0, -0.3, 0.9)                     # deep: ~1.3 m inward offset, far ahead
    h.fsm.inputs.collision_stamp = h.t
    h.finish(f.ACT_FOLLOW, f.ABORTED)
    poses = h.goal(f.ACT_FOLLOW)['poses']
    assert max(q[1] for q in poses) >= 0.9 + 0.071 + 0.235 + 0.10 - 0.03
    # the way in keeps the plain 1.2 m ramp (~0.4 m beside the robot here); a symmetric
    # long ramp started ~1.2 m beside it (sim 2026-11-05: both such detours aborted)
    assert abs(poses[0][1]) < 0.6, poses[0]
    pk = max(range(len(poses)), key=lambda k: poses[k][1])
    steep = max(abs(math.atan2(b[1] - a[1], b[0] - a[0]))
                for a, b in zip(poses[pk:-1], poses[pk + 1:]) if b[0] - a[0] > 1e-6)
    assert math.degrees(steep) <= 32.0, math.degrees(steep)
    # ends boundary_recover_margin_m inside the lap, then merges tangentially (sim
    # 2026-11-05: a rejoin straight onto the line swung the rear 0.35 m out)
    assert abs(poses[-1][1] - h.fsm.p.boundary_recover_margin_m) < 0.02
    h.fsm.inputs.pose = (poses[-1][0], poses[-1][1], 0.0)
    mk = h.mark()
    h.finish(f.ACT_FOLLOW, arrive=False)
    assert any('merging from 0.10 m inside' in t for t in logs(h, mk))
    assert not [e for e in h.since(mk, f.StartAction) if e.name == f.ACT_SPIN]   # no pivot
    nxt = h.goal(f.ACT_FOLLOW)['poses']
    assert abs(nxt[0][1] - 0.10) < 0.02 and abs(nxt[-1][1]) < 1e-6   # merges onto the lap


def test_costmap_lethal_points_maps_odom_cells_into_the_map_frame():
    """mission_node hands the grid + map <- odom; the cells are extracted on demand."""
    import numpy as np
    from mower_mission.fsm.common import costmap_lethal_points
    data = np.zeros((3, 4), dtype=np.int8)
    data[1, 2] = 100                                 # lethal: odom (1.0+2.5*0.5, 2.0+1.5*0.5)
    data[0, 0] = 99                                  # inscribed: not lethal
    lc = {'w': 4, 'h': 3, 'res': 0.5, 'ox': 1.0, 'oy': 2.0, 'data': data}
    a = math.pi / 2                                  # map <- odom: +90 deg, then (10, 20)
    pts = costmap_lethal_points((lc, (math.cos(a), math.sin(a), 10.0, 20.0)))
    assert len(pts) == 1
    ox, oy = 2.25, 2.75
    assert abs(pts[0][0] - (10.0 - oy)) < 1e-9 and abs(pts[0][1] - (20.0 + ox)) < 1e-9
    big = {'w': 100, 'h': 100, 'res': 0.05, 'ox': 0.0, 'oy': 0.0,
           'data': np.full((100, 100), 100, dtype=np.int8)}
    assert len(costmap_lethal_points((big, (1.0, 0.0, 0.0, 0.0)), cap=500)) == 500
    assert costmap_lethal_points(None) == []


def test_costmap_detour_back_up_follows_a_curved_lap_back():
    """Sim 2026-11-05 (branch-scen8 446-448 s): a straight reverse on lap 0 where the line
    curves took base_link 0.14 m outside; the back-up retraces the lap it just drove."""
    h = Harness(nogo_edge_costmap=True)
    h.areas = [square(-6, -6, 12)]
    start_until_planning(h)
    R = 4.0                                         # lap: arc of radius 4 about (0, 4)
    arc = [(R * math.sin(a), 4.0 - R * math.cos(a), a)
           for a in [k * 0.0125 for k in range(160)]]
    h.fsm.inputs.pose = arc[0]
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([arc, line(-2.0, 2.0, -4.0, 2.0)]))
    h.fsm.inputs.is_cutting = True
    h.tick()
    h.fsm.mission.edge_laps[(h.fsm.mission.area_idx, 0)] = True
    q = arc[40]
    h.fsm.inputs.pose = q
    h.tick(n=2)
    ahead = arc[50]                                 # the branch 0.5 m ahead on the arc
    h.fsm.inputs.costmap_lethal = [(ahead[0] + 0.05 * k * -math.sin(ahead[2]),
                                    ahead[1] + 0.05 * k * math.cos(ahead[2]))
                                   for k in range(-4, 8)]
    h.fsm.inputs.costmap_lethal_stamp = h.t
    mk = h.mark()
    h.fsm.inputs.collision_stamp = h.t
    h.finish(f.ACT_FOLLOW, f.ABORTED)
    assert h.fsm.mission.step == 'nogo_detour' and h.fsm._action is None, logs(h, mk)
    cmd = h.since(mk, f.DriveCmd)[-1]
    assert cmd.linear < 0.0 and abs(cmd.angular) > 0.01   # reversing along the arc


def test_costmap_edge_detour_off_skips_like_master():
    """2026-11-05: shipped off; on since 2026-11-12 (owner rule: step round foliage). Off: a
    costmap-only obstacle on an edge lap: the collision abort skips the stretch (master
    behaviour), no back-up, no detour, no costmap read."""
    h, mk = _abort_on_branch(2.0, -0.3, 0.5, costmap=False)
    assert h.fsm.p.nogo_edge_costmap is False
    assert Harness().fsm.p.nogo_edge_costmap is True        # the shipped default
    txt = logs(h, mk)
    assert any('edge blocked: skip' in t for t in txt), txt
    assert not any('costmap cells' in t or 'back up' in t for t in txt)
    assert h.fsm.mission.nogo_edge_detours == 0


# =====================================================================
# 2026-11-11: perimeter objects stay obstacles; edge laps bend inward round them
# =====================================================================
def test_perimeter_marks_keep_their_place_at_true_size_interior_marks_unchanged():
    h = _edge_h()                              # square(0, 0, 5), lap 0 on the south line
    out = Z(2.5, -0.38, 0.30, state='mark')    # a pot 0.38 m beyond the line (r 0.2 + 0.1)
    inner = Z(2.5, 2.0, 0.30, state='mark')    # an interior mark
    zones(h, marks=[out, inner])
    got = {(z.x, z.y): z.r for z in h.fsm._nogo_discs()}
    assert abs(got[(2.5, -0.38)] - 0.25) < 1e-9 and abs(got[(2.5, 2.0)] - 0.30) < 1e-9


def test_edge_lap_bends_inward_round_a_perimeter_mark_the_body_would_touch():
    # sim 2026-11-11 (owner_area3_edge1111 v1): a pot whose near face is 0.18 m beyond the
    # line; the lap-0 body overhangs 0.235 m and hit it twice (STUCK). The lap now bends
    # inward round the mark (true size + clearance), blade on, at most 0.30 m
    h = _edge_h()
    h.fsm.inputs.pose = (1.0, 0.0, 0.0)
    h.tick(n=2)
    zones(h, marks=[Z(2.5, -0.38, 0.30, state='mark')])
    mk = h.mark()
    h.tick(dt=1.0, n=2)
    a = h.pending_action(f.ACT_FOLLOW)
    assert a is not None and a.purpose == 'nogo_detour'
    poses = h.goal(f.ACT_FOLLOW)['poses']
    off = max(q[1] for q in poses)
    assert 0.20 < off <= 0.40 + 1e-6           # 0.235 + 0.20 - (0.38 - 0.25) = 0.305
    assert any('detour inward' in t for t in logs(h, mk))


def test_edge_lap_ignores_a_perimeter_mark_the_body_cannot_touch():
    h = _edge_h()
    h.fsm.inputs.pose = (1.0, 0.0, 0.0)
    h.tick(n=2)
    zones(h, marks=[Z(2.5, -0.75, 0.25, state='mark')])   # face 0.55 m out: body side 0.32 clear
    h.tick(dt=1.0, n=2)
    a = h.fsm._action
    assert a is None or a.purpose != 'nogo_detour'


def test_perimeter_mark_detour_is_clamped_not_dropped_when_the_need_exceeds_the_cap():
    # sim 2026-11-11 (inside world v3): one scattered mark 0.1 m further in made the need
    # exceed the cap; the detour came only once the robot was beside the pot
    h = _edge_h()
    h.fsm.inputs.pose = (1.0, 0.0, 0.0)
    h.tick(n=2)
    zones(h, marks=[Z(2.5, -0.20, 0.30, state='mark')])   # needs 0.49 > 0.40
    h.tick(dt=1.0, n=2)
    a = h.pending_action(f.ACT_FOLLOW)
    assert a is not None and a.purpose == 'nogo_detour'
    assert abs(max(q[1] for q in h.goal(f.ACT_FOLLOW)['poses']) - 0.40) < 0.02


# =====================================================================
# 2026-11-12: owner rule (2026-11-11) - step round it, rejoin, carry on
# =====================================================================
def test_route_corner_sweep_still_blocked_by_a_static_object_reverses_never_pivots():
    """Sim 2026-11-11 inside world v5: after the re-route the route-corner gate pivoted "as
    planned" with a pot in the sweep (2 bumps, STUCK). Now: reverse (the mission's own
    DriveCmd reverse, not Nav2's BackUp) and re-route from there; at the same corner again:
    abort the transit without the pivot."""
    trunk = Z(2.0, 0.75, 0.30, 5, 'trunk', 'pending')
    h = _route_corner_h([trunk])
    assert any('(e)' in t and 're-route the transit' in t for t in logs(h))
    from test_mission_fsm import _answer_route, _route_resp

    def corner_again():
        for _ in range(400):
            if h.fsm._service is not None:
                break
            zones(h, trunk)
            h.tick()
        mk = h.mark()
        h.fsm.inputs.pose = (0.0, 0.0, 0.0)
        _answer_route(h, resp=_route_resp(_corner_route(100.0, h.fsm.mission.transit_target)))
        zones(h, trunk)
        h.fsm.inputs.pose = (2.0, 0.0, 0.0)
        h.finish(f.ACT_FOLLOW)                 # at the corner (2, 0), yaw 0
        return mk
    mk = corner_again()
    assert h.fsm._action is None or h.fsm._action.name != f.ACT_SPIN, logs(h, mk)
    assert not any('pivot as planned' in t for t in logs(h, mk))
    assert any('static object in the sweep: reverse' in t for t in logs(h, mk)), logs(h, mk)
    assert h.fsm.mission.step == 'nogo_detour' and h.fsm._action is None
    cmds = list(h.since(mk, f.DriveCmd))
    assert cmds and cmds[-1].linear < 0.0
    h.fsm.inputs.pose = (1.4, 0.0, 0.0)        # reversed 0.6 m
    h.tick()
    assert any('re-routing the transit from here' in t for t in logs(h, mk))
    assert ('transit_abort', 'route corner pivot sweep blocked: reversed, re-routing') in \
        _incidents(h, mk)
    mk = corner_again()                        # the same corner again: no second reverse
    assert h.fsm._action is None or h.fsm._action.name != f.ACT_SPIN
    assert any('reversed here already' in t for t in logs(h, mk)), logs(h, mk)
    assert h.name != 'STUCK_NEEDS_HELP'


def test_route_corner_dynamic_or_clear_sweep_keeps_the_old_branches():
    assert nogo.static_hits([]) == []
    hit = nogo.PivotPlan('blocked', [], '', [])
    assert nogo.static_hits(hit.hits) == []


def _detour_on_branch():
    h, mk = _abort_on_branch(2.5, -0.3, 0.5)
    assert h.pending_action(f.ACT_FOLLOW).purpose == 'nogo_detour', logs(h, mk)
    return h, mk


def test_edge_detour_aborted_beside_the_object_reverses_and_bends_again_not_skips():
    """Sim 2026-11-11 inside world v3 / v5: the detour follow aborted beside the pot (the
    collision monitor stopped MPPI on the bend) and the stretch was skipped. Now: reverse
    along the detour just driven, bend again from there (costmap cells), no skip."""
    h, mk = _detour_on_branch()
    d = h.fsm.mission.nogo_detour
    poses = d['poses']
    k = len(poses) // 2
    h.fsm.inputs.pose = (poses[k][0], poses[k][1], 0.3)
    h.fsm.inputs.collision_stamp = h.t
    h.finish(f.ACT_FOLLOW, f.ABORTED)
    txt = logs(h, mk)
    assert not any('edge blocked: skip' in t for t in txt), txt
    assert any('the detour stopped beside it: back up' in t for t in txt), txt
    assert h.fsm.mission.step == 'nogo_detour' and h.fsm.mission.nogo_detour.get('backup')
    assert h.fsm.mission.nogo_detour['again'] == 1
    cmds = list(h.since(mk, f.DriveCmd))
    assert cmds and cmds[-1].linear < 0.0
    # the reverse runs back along the detour poses already driven (inside), not to the lap
    path = h.fsm.mission.nogo_detour['path']
    assert path[0][1] > 0.05
    h.fsm.inputs.pose = (path[-1][0], path[-1][1], 0.0)
    h.tick()
    a = h.pending_action(f.ACT_FOLLOW)
    assert a is not None and a.purpose == 'nogo_detour', logs(h, mk)
    assert h.fsm.mission.nogo_detour['again'] == 1
    assert any('again 1/2' in t for t in logs(h, mk))


def test_edge_detour_aborted_again_and_again_skips_after_the_limit():
    h, mk = _detour_on_branch()
    for n in (1, 2):
        d = h.fsm.mission.nogo_detour
        h.fsm.inputs.pose = (d['poses'][2][0], d['poses'][2][1], 0.0)
        h.fsm.inputs.collision_stamp = h.t
        h.finish(f.ACT_FOLLOW, f.ABORTED)
        assert not any('edge blocked: skip' in t for t in logs(h, mk))
        for _ in range(200):
            if h.fsm._action is not None and h.fsm._action.name == f.ACT_FOLLOW:
                break
            b = h.fsm.mission.nogo_detour
            if b is not None and b.get('backup'):
                h.fsm.inputs.pose = (b['path'][-1][0], b['path'][-1][1], 0.0)
            h.tick()
        assert h.fsm.mission.nogo_detour['again'] == n, logs(h, mk)
    d = h.fsm.mission.nogo_detour
    h.fsm.inputs.pose = (d['poses'][2][0], d['poses'][2][1], 0.0)
    h.finish(f.ACT_FOLLOW, f.ABORTED)
    assert any('edge blocked: skip' in t for t in logs(h, mk))


def test_edge_detour_again_off_skips_as_before():
    h, mk = _abort_on_branch(2.5, -0.3, 0.5)
    h.fsm.p.nogo_edge_detour_again = 0
    d = h.fsm.mission.nogo_detour
    h.fsm.inputs.pose = (d['poses'][3][0], d['poses'][3][1], 0.0)
    h.finish(f.ACT_FOLLOW, f.ABORTED)
    assert any('edge detour aborted: skipping the stretch' in t for t in logs(h, mk))


def test_collision_monitor_stop_held_on_an_edge_lap_steps_round_it_not_skips():
    """Field 2026-11-11 area 3 west edge: three collision-monitor stops on foliage inside the
    line, each one a skipped stretch. Owner rule: step round it on the inward side, rejoin
    the lap and carry on."""
    h = _edge_h()
    h.fsm.inputs.pose = (1.0, 0.0, 0.0)
    h.tick(n=2)
    _branch(h, 1.75, -0.3, 0.25)               # leaves over the line, 0.75 m ahead
    mk = h.mark()
    h.fsm.inputs.cm_stop_since = h.t
    for _ in range(int(h.fsm.p.edge_blocked_hold_s * 10) + 5):
        _branch(h, 1.75, -0.3, 0.25)
        h.tick()
        if h.fsm.mission.step == 'nogo_detour':
            break
    txt = logs(h, mk)
    assert not any('edge blocked: skip' in t for t in txt), txt
    assert any('collision_monitor stop held' in t and ('back up' in t or 'detour' in t)
               for t in txt), txt
    assert h.fsm.mission.step == 'nogo_detour'


def test_collision_monitor_stop_held_with_costmap_off_skips_as_before():
    h = _edge_h(nogo_edge_costmap=False)
    h.fsm.inputs.pose = (1.0, 0.0, 0.0)
    h.tick(n=2)
    _branch(h, 1.75, -0.3, 0.25)
    mk = h.mark()
    h.fsm.inputs.cm_stop_since = h.t
    h.tick(n=int(h.fsm.p.edge_blocked_hold_s * 10) + 5)
    assert any('edge blocked: skip' in t and 'collision_monitor stop held' in t
               for t in logs(h, mk))


def test_back_up_command_never_drives_forward():
    """Sim 2026-11-12 (foliage v1): a back-up whose first target lay ahead of the robot
    drove forward (_track_cmd) into the next pot. The back-up only ever reverses."""
    from mower_mission.fsm.holds import HoldsMixin
    v, w = HoldsMixin._reverse_cmd((0.0, 0.0, 0.0), (0.3, 0.1), 0.1)
    assert v < 0.0 and w == 0.0
    v, w = HoldsMixin._reverse_cmd((0.0, 0.0, 0.0), (-0.5, 0.1), 0.1)
    assert v < 0.0 and w < 0.0                 # rear swings toward +y: turn clockwise


def test_edge_detour_on_an_inner_lap_bends_toward_the_interior_whichever_side_it_is():
    """Sim 2026-11-12 (foliage v2, lap 2 0.45 m in): both points 0.3 m either side of the lap
    are inside, so ``inside`` alone took the left side, which is outward when the lap runs
    clockwise; the detour 'would put the body 0.59 m beyond its limit' and the stretch was
    skipped. ``depth`` picks the deeper side."""
    sq = [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0), (0.0, 5.0)]
    from mower_mission import body_boundary as bb

    def inside(x, y):
        return bb.point_outside(x, y, [sq]) <= 0.0

    def depth(x, y):
        out = bb.point_outside(x, y, [sq])
        return -out if out > 0.0 else bb.nearest_edge_point(x, y, [sq])[2]
    lap = [(4.5 - 0.05 * k, 0.45, math.pi) for k in range(81)]   # westward: left is outward
    disc = Z(3.0, 0.2, 0.15, state='mark')
    res = nogo.edge_detour(lap, 0, [disc], inside, depth=depth, max_offset_m=0.6)
    assert res is not None
    assert max(q[1] for q in res[0]) > 0.45 + 0.1                 # bent north (inward)
    assert min(q[1] for q in res[0]) >= 0.45 - 1e-9


def test_edge_detour_starts_from_the_robots_inward_offset():
    """Sim 2026-11-12 (foliage v3): a detour that began on the lap while the robot merged out
    from 0.10 m inside swung it outward on the slope. The bend starts at the robot's offset."""
    lap = [(0.05 * k, 0.0, 0.0) for k in range(81)]
    disc = Z(1.5, -0.3, 0.1, state='mark')
    plain = nogo.edge_detour(lap, 0, [disc], lambda x, y: y > 0, max_offset_m=0.6)
    held = nogo.edge_detour(lap, 0, [disc], lambda x, y: y > 0, max_offset_m=0.6,
                            start_offset_m=0.10)
    assert plain[0][0][1] < 0.10 - 1e-6
    assert abs(held[0][0][1] - 0.10) < 1e-6
    assert all(b[1] >= a[1] - 1e-9 for a, b in zip(plain[0], held[0]))


def test_edge_detour_holds_the_bend_past_a_disc_near_the_end_of_the_lookahead():
    """Sim 2026-11-12 inside v6: a proactive detour started 3 m before the pot; need was only
    computed up to the 2.5 m lookahead, so the return ramp began before the pot and the nose
    met it. The bend holds while the disc is beside the body."""
    lap = [(0.05 * k, 0.0, 0.0) for k in range(161)]
    disc = Z(3.0, -0.35, 0.2, state='mark')         # beside the body from 2.35 m on
    res = nogo.edge_detour(lap, 0, [disc], lambda x, y: y > 0, max_offset_m=0.6,
                           lookahead_m=2.5, clearance=0.2)
    need = 0.2 + 0.235 + 0.2 - 0.35
    poses = res[0]
    at = [q[1] for q in poses if 2.35 <= q[0] <= 3.15]  # the disc beside the body box
    assert at and min(at) >= need - 1e-6, (min(at), need)


def test_edge_detour_success_with_most_of_it_left_sends_the_rest():
    """Sim 2026-11-12 foliage v1 / v5: a detour 'reached the goal' 10 ms after it was sent;
    the mission rejoined the lap at its end and the robot drove into the pot."""
    h, mk = _abort_on_branch(2.5, -0.3, 0.5)
    d = h.fsm.mission.nogo_detour
    h.fsm.inputs.pose = (d['poses'][1][0], d['poses'][1][1], 0.0)
    h.finish(f.ACT_FOLLOW, arrive=False)
    a = h.fsm._action
    assert a is not None and a.purpose == 'nogo_detour', logs(h, mk)
    assert any('sending the rest' in t for t in logs(h, mk))
    end = h.fsm.mission.nogo_detour['end']
    last = h.fsm.mission.nogo_detour['poses'][-1]
    h.fsm.inputs.pose = (last[0], last[1], 0.0)
    h.finish(f.ACT_FOLLOW)
    assert h.fsm.mission.start_local == end
    assert any('rejoining the lap' in t for t in logs(h, mk))
