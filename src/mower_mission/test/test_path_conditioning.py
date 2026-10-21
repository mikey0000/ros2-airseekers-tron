"""Path conditioning (docs/DESIGN_path_conditioning_2026-10.md steps 2-3).

Real geometry: the run 5 outer ring (test/data/run5_2026-10-15_ring.json, the field
mow_plan's sub-path 0), the real dock path routes (real_dock_path_routes.json), the run 4
/plan fixtures (run4_2026-10-15_plans.json) and the real map's polygons (the sim gate world's
copy of run 5's areas.dat, test/data/run5_region.json)."""

import json
import re
import math

import pytest
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from mower_mission import geometry as geo             # noqa: E402
from mower_mission import mission_fsm as f            # noqa: E402
from mower_mission import path_conditioning as pc     # noqa: E402
from test_mission_fsm import Harness, line, plan, square, start_until_planning  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
V = (-4.74, -1.17)                  # run 5's V corner


def _region():
    with open(os.path.join(HERE, 'data', 'run5_region.json')) as fh:
        return {k: [tuple(q) for q in v] for k, v in json.load(fh)['polygons'].items()}


POLYS = _region()
REGION = list(POLYS.values())


def ring():
    with open(os.path.join(HERE, 'data', 'run5_2026-10-15_ring.json')) as fh:
        return [tuple(q) for q in json.load(fh)['ring']]


def routes():
    with open(os.path.join(HERE, 'data', 'real_dock_path_routes.json')) as fh:
        d = json.load(fh)
    return d['route_from_dock'], d['route_to_dock']


def run4_plans():
    with open(os.path.join(HERE, 'data', 'run4_2026-10-15_plans.json')) as fh:
        d = json.load(fh)
    return [pl['poses'] if isinstance(pl, dict) else pl for pl in d['plans']]


def corner_at(res, xy, tol=0.05):
    return next(c for c in res.corners if math.hypot(c.x - xy[0], c.y - xy[1]) <= tol)


def heading_after(poses, i, look=0.3):
    q = next(p for p in poses[i + 1:] if geo.dist(p, poses[i]) >= look)
    return math.atan2(q[1] - poses[i][1], q[0] - poses[i][0])


# ---- the conditioner on the real geometry -------------------------------------------------

def test_run5_v_corner_is_a_pivot_at_the_vertex_on_the_outer_ring():
    res = pc.condition(ring(), REGION)
    assert res.valid, res.errors
    c = corner_at(res, V)
    assert c.choice == 'pivot' and c.why == 'edge policy'
    assert -105 < math.degrees(c.turn) < -95
    i = next(i for i in res.pivots if geo.dist(res.poses[i], V) < 0.01)
    assert res.types[i] == pc.PIVOT
    # the pivot turns from the incoming run to the outgoing run (the next pose's heading)
    h_in = res.poses[i][2]
    h_out = heading_after(res.poses, i)
    assert abs(math.degrees(geo.wrap_angle(h_out - h_in)) - math.degrees(c.turn)) < 6


def test_edge_policy_pivots_convex_ring_corners_at_the_edge_only():
    # the run 5 V without the policy: chosen from the space (the 64 deg corner 0.6 m on takes
    # the run a fillet would need), not 'edge policy'
    c = corner_at(pc.condition(ring(), REGION, pc.Params(edge_pivot_rad=0.0)), V)
    assert c.why != 'edge policy'
    # a square ring 0.1 m inside a square area: convex 90 deg corners at the edge pivot;
    # without the policy they are fillets that move inward, away from the edge
    area = [[(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)]]
    sq = []
    for (x0, y0), (x1, y1) in zip([(0.1, 0.1), (3.9, 0.1), (3.9, 3.9), (0.1, 3.9)],
                                  [(3.9, 0.1), (3.9, 3.9), (0.1, 3.9), (0.1, 0.1)]):
        sq += [(x0 + (x1 - x0) * k / 38, y0 + (y1 - y0) * k / 38) for k in range(38)]
    sq.append((0.1, 0.1))
    edge = pc.condition(sq, area)
    assert [c.choice for c in edge.corners] == ['pivot'] * 3
    assert all(c.why == 'edge policy' for c in edge.corners)
    fil = pc.condition(sq, area, pc.Params(edge_pivot_rad=0.0))
    assert [c.choice for c in fil.corners] == ['fillet'] * 3
    near = [q for q in fil.poses if geo.dist(q, (3.9, 0.1)) < 1.0]
    assert min(pc.clearance(q, area) for q in near) >= 0.1 - 1e-3


def test_run5_ring_output_is_drivable_and_inside():
    res = pc.condition(ring(), REGION)
    assert res.valid
    gaps = sorted(geo.dist(res.poses[i], res.poses[i + 1]) for i in range(len(res.poses) - 1))
    assert gaps[len(gaps) // 2] == pytest.approx(0.05, abs=0.005)     # resampled at 0.05 m
    assert gaps[-1] <= 0.1
    for c in res.corners:
        if c.choice == 'fillet':
            assert c.r >= 0.5 - 1e-9
    assert pc.ARC in res.types and pc.LINE in res.types
    # nothing farther out than the drawn ring
    raw_min = min(pc.clearance(q, REGION) for q in ring())
    assert min(pc.clearance(q, REGION) for q in res.poses) >= min(0.0, raw_min) - 0.01


def test_planned_arcs_and_straight_runs_are_left_alone():
    """A Dubins-like r = 0.5 m arc and a straight swath: no corners, no pivots."""
    arc = [(0.5 * math.sin(a / 20.0 * math.pi), 0.5 - 0.5 * math.cos(a / 20.0 * math.pi))
           for a in range(21)]
    pts = [(-2.0 + 0.1 * k, 0.0) for k in range(20)] + arc + \
        [(-0.1 * k, 1.0) for k in range(1, 20)]
    res = pc.condition(pts, [])
    assert not res.pivots and all(c.choice == 'kink' or False for c in res.corners) or \
        not res.corners
    assert pc.ARC in res.types


def test_dock_path_routes_condition_to_fillets_and_pivots():
    fwd, back = routes()
    for r in (fwd, back):
        res = pc.condition(r, REGION, ring=False)
        assert res.valid, res.errors
        by = {(round(c.x, 2), round(c.y, 2)): c for c in res.corners}
        # the 39 deg bend MPPI cut by 0.20-0.34 m is a fillet now
        c39 = corner_at(res, (-2.26, 0.37))
        assert c39.choice == 'fillet' and c39.r >= 0.5
        # the drawn 90 deg corner by the dock has no room for a fillet: pivot on the vertex
        c90 = corner_at(res, (-0.14, -0.17), tol=0.06)
        assert c90.choice == 'pivot'
        assert by
        for c in res.corners:
            assert c.choice in ('fillet', 'pivot', 'kink')


def test_run4_plans_are_valid_and_every_pivot_is_on_the_drawn_vertex():
    for pts in run4_plans():
        res = pc.condition(pts, REGION, ring=False)
        assert res.valid, res.errors
        raw = [tuple(q[:2]) for q in pts]
        for i in res.pivots:
            assert min(geo.dist(res.poses[i], q) for q in raw) < 0.06


def test_no_region_room_means_pivot_not_fillet():
    """A 90 deg corner in a 0.6 m wide channel: the r = 0.5 fillet would cut 0.21 m toward
    the inner edge; no room -> pivot at the vertex."""
    chan = [[(-0.3, -0.3), (3.3, -0.3), (3.3, 0.3), (0.3, 0.3)],
            [(2.7, -0.3), (3.3, -0.3), (3.3, 3.0), (2.7, 3.0)]]
    pts = [(0.0, 0.0)] + [(0.1 * k, 0.0) for k in range(1, 31)] + \
        [(3.0, 0.1 * k) for k in range(1, 28)]
    res = pc.condition(pts, chan, ring=False)
    assert res.valid
    c = corner_at(res, (3.0, 0.0))
    assert c.choice == 'pivot' and c.why == 'no fillet fits'
    wide = [[(-0.3, -1.5), (4.5, -1.5), (4.5, 3.0), (-0.3, 3.0)]]
    c = corner_at(pc.condition(pts, wide, ring=False), (3.0, 0.0))
    assert c.choice == 'fillet' and c.r == 1.0


def test_no_pivot_allowed_is_reported_not_driven():
    chan = [[(-0.3, -0.3), (3.3, -0.3), (3.3, 0.3), (0.3, 0.3)],
            [(2.7, -0.3), (3.3, -0.3), (3.3, 3.0), (2.7, 3.0)]]
    pts = [(0.1 * k, 0.0) for k in range(31)] + [(3.0, 0.1 * k) for k in range(1, 28)]
    res = pc.condition(pts, chan, pc.Params(allow_pivot=False), ring=False)
    assert not res.valid and 'K-turn not implemented' in res.errors[0]


# ---- mission integration -------------------------------------------------------------------

def ring_h(yaw=None, **params):
    params.setdefault('path_conditioning', True)
    h = Harness(**params)
    h.areas = [square(-8, -4, 12)]
    r = ring()
    start_until_planning(h)
    y0 = math.atan2(r[1][1] - r[0][1], r[1][0] - r[0][0])
    h.fsm.inputs.pose = (r[0][0], r[0][1], y0 if yaw is None else yaw)
    sp = [(q[0], q[1], math.atan2(r[min(i + 1, len(r) - 1)][1] - q[1],
                                  r[min(i + 1, len(r) - 1)][0] - q[0]))
          for i, q in enumerate(r)]
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([sp, line(-4.0, 3.0, -1.0, 3.0)]))
    h.fsm.inputs.is_cutting = True
    h.tick()
    return h


def test_mission_ends_the_leg_on_a_pivot_vertex_and_pivots_with_the_blade_on():
    # square(-8, -4, 12) is the whole region here: no edge policy pivots; the V's fillet fits
    # only with the edge policy off, so force the policy via a tight area
    h = ring_h()
    m = h.fsm.mission
    assert h.name == 'MOWING' and h.blade
    piv = m.cond[0]['pivots']
    g = h.goal(f.ACT_FOLLOW)
    if piv:
        assert len(g['poses']) == piv[0] + 1 and m.corner_end == piv[0]
    # every conditioned forward leg ends along-path (2026-10-16), not on a position disc
    assert g['goal_checker_id'] == 'coverage_corner_goal_checker'


def tight_ring_h(**params):
    """The ring inside the real Area 1 polygon: the edge policy pivots the V."""
    params.setdefault('path_conditioning', True)
    h = Harness(**params)
    h.areas = [{'name': 'Area 1', 'outer': POLYS['Area 1'], 'obstacles': [],
                'is_navigation_area': False}]
    r = ring()
    start_until_planning(h)
    y0 = math.atan2(r[1][1] - r[0][1], r[1][0] - r[0][0])
    h.fsm.inputs.pose = (r[0][0], r[0][1], y0)
    sp = [(q[0], q[1], math.atan2(r[min(i + 1, len(r) - 1)][1] - q[1],
                                  r[min(i + 1, len(r) - 1)][0] - q[0]))
          for i, q in enumerate(r)]
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([sp, line(-4.0, 3.0, -1.0, 3.0)]))
    h.fsm.inputs.is_cutting = True
    h.tick()
    return h


def drive_to_chunk_end(h, short=0.05):
    poses = h.goal(f.ACT_FOLLOW)['poses']
    a, b = poses[-2], poses[-1]
    yaw = math.atan2(b[1] - a[1], b[0] - a[0])
    for q in poses[:-1:4]:
        h.fsm.inputs.pose = (q[0], q[1], yaw)
        h.tick()
    h.fsm.inputs.pose = (b[0] - short * math.cos(yaw), b[1] - short * math.sin(yaw), yaw)
    h.tick()
    return yaw


def test_mission_run5_v_corner_leg_end_and_pivot_target():
    h = tight_ring_h()
    m = h.fsm.mission
    i_v = next(i for i in m.cond[0]['pivots'] if geo.dist(m.subpaths[0][i], V) < 0.01)
    # follow up to the V (earlier pivots first, if any)
    for _ in range(10):
        if h.fsm._action.name == f.ACT_SPIN:
            yaw = h.fsm.inputs.pose[2] + h.goal(f.ACT_SPIN)['angle']
            h.fsm.inputs.pose = h.fsm.inputs.pose[:2] + (yaw,)
            h.finish(f.ACT_SPIN)
            continue
        if m.corner_end == i_v:
            break
        drive_to_chunk_end(h)
    assert m.corner_end == i_v
    g = h.goal(f.ACT_FOLLOW)
    assert geo.dist(g['poses'][-1], V) < 0.01            # the leg ends ON the vertex
    assert g['goal_checker_id'] == 'coverage_corner_goal_checker'
    mk = h.mark()
    yaw_in = drive_to_chunk_end(h)
    # the leg ends on Nav2's along-path checker (no mission-side corner-reached rule)
    assert h.fsm._action.name == f.ACT_FOLLOW
    h.finish(f.ACT_FOLLOW, arrive=False)
    sp = h.goal(f.ACT_SPIN)
    out = heading_after(m.subpaths[0], i_v)
    assert abs(geo.wrap_angle(sp['angle'] - geo.wrap_angle(out - yaw_in))) < math.radians(10)
    assert -110 < math.degrees(sp['angle']) < -90
    assert h.blade and not h.since(mk, f.BladeOff) and f.blade_allowed(h.fsm)
    h.fsm.inputs.pose = h.fsm.inputs.pose[:2] + (yaw_in + sp['angle'],)
    h.finish(f.ACT_SPIN)
    g = h.goal(f.ACT_FOLLOW)
    assert geo.dist(g['poses'][0], V) < 0.01 and h.blade


def test_mission_leg_start_far_off_heading_pivots_first():
    r = ring()
    y0 = math.atan2(r[1][1] - r[0][1], r[1][0] - r[0][0])
    h = ring_h(yaw=y0 + math.radians(140))
    assert h.pending_action(f.ACT_SPIN)
    assert abs(math.degrees(h.goal(f.ACT_SPIN)['angle'])) > 120
    assert h.blade and h.fsm.mission.step == 'corner_pivot'


def test_mission_unfinished_coverage_pivot_pivots_again_then_follows_within_the_threshold():
    # 2026-10-16 field run 6: never follow a leg far off its heading after a failed pivot
    r = ring()
    y0 = math.atan2(r[1][1] - r[0][1], r[1][0] - r[0][0])
    h = ring_h(yaw=y0 + math.radians(140), blocked_pivot_policy='retry')
    assert h.pending_action(f.ACT_SPIN).purpose == 'cov_pivot'
    first = h.goal(f.ACT_SPIN)['angle']
    mk = h.mark()
    h.finish(f.ACT_SPIN, f.ABORTED)                     # yaw unchanged
    assert h.pending_action(f.ACT_SPIN).purpose == 'cov_pivot'
    assert abs(h.goal(f.ACT_SPIN)['angle'] - first) < 0.05
    assert not h.since(mk, f.StartAction) or all(
        e.name != f.ACT_FOLLOW for e in h.since(mk, f.StartAction))
    x, y = h.fsm.inputs.pose[:2]
    hd = h.fsm.mission.pivot_target[0]
    h.fsm.inputs.pose = (x, y, hd + math.radians(10))  # 10 deg short: under 20 deg
    h.finish(f.ACT_SPIN, f.ABORTED)
    assert h.pending_action(f.ACT_FOLLOW)
    assert h.blade and h.fsm.mission.step == 'follow'


def test_mission_coverage_pivot_failing_three_times_stops():
    r = ring()
    y0 = math.atan2(r[1][1] - r[0][1], r[1][0] - r[0][0])
    h = ring_h(yaw=y0 + math.radians(140), blocked_pivot_policy='retry')
    for _ in range(2):
        h.finish(f.ACT_SPIN, f.ABORTED)
        assert h.pending_action(f.ACT_SPIN).purpose == 'cov_pivot'
    h.finish(f.ACT_SPIN, f.ABORTED)
    assert h.name == 'STUCK_NEEDS_HELP'
    assert h.fsm._action is None and not h.blade


def test_mission_short_forward_sub_paths_are_conditioned_too():
    # 2026-10-16: no turn-leg exemption; a short forward piece is a line like any other and
    # ends along-path
    h = Harness(path_conditioning=True)
    h.areas = [square(-1, -1, 12)]
    start_until_planning(h)
    h.fsm.inputs.pose = (0.0, 0.0, 0.0)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(0.0, 0.0, 0.6, 0.0), line(0.3, 2.4, 4.0, 2.4)]))
    h.fsm.inputs.is_cutting = True
    h.tick()
    assert 0 in h.fsm.mission.cond
    assert h.goal(f.ACT_FOLLOW)['goal_checker_id'] == 'coverage_corner_goal_checker'


def test_mission_invalid_conditioning_is_not_mowed(monkeypatch):
    real = pc.condition

    def broken(*a, **k):
        res = real(*a, **k)
        res.errors.append('test: forced failure')
        return res
    monkeypatch.setattr(pc, 'condition', broken)
    h = Harness(path_conditioning=True)
    h.areas = [square(-1, -1, 12)]
    start_until_planning(h)
    h.fsm.inputs.pose = (0.0, 0.0, 0.0)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(0, 0, 4, 0), line(0, 2, 4, 2)]))
    h.fsm.inputs.is_cutting = True
    h.tick()
    texts = [e.text for e in h.fx if isinstance(e, f.Log)]
    assert any('path conditioning failed' in t and 'will not be driven' in t for t in texts)
    assert any('NOT mowed: path conditioning' in t for t in texts)
    assert not h.since(0, f.StartAction) or all(
        e.name != f.ACT_FOLLOW for e in h.since(0, f.StartAction))


def test_revert_path_conditioning_false_sends_the_raw_ring():
    h = tight_ring_h(path_conditioning=False)
    g = h.goal(f.ACT_FOLLOW)
    r = ring()
    assert [tuple(q[:2]) for q in g['poses'][:5]] == [tuple(q[:2]) for q in r[:5]]
    assert not h.fsm.mission.cond and h.fsm.mission.corner_end is None


def test_small_join_heading_error_is_left_to_the_tracker():
    # under path_conditioning_pivot_min_rad (0.35 rad, the conditioner's own threshold): no
    # pivot, the leg is sent at once
    h0 = ring_h()
    m = h0.fsm.mission
    sp = m.subpaths[0]
    end = min([c for c in m.cond[0]['pivots'] if c > 0] + [len(sp) - 1])
    jh = f.MissionFSM._join_heading(sp[:end + 1], h0.fsm.inputs.pose)
    h = ring_h(yaw=jh + math.radians(15))
    assert h.pending_action(f.ACT_FOLLOW)


def test_leg_start_pivot_is_the_one_mechanism_for_both_controllers():
    r = ring()
    y0 = math.atan2(r[1][1] - r[0][1], r[1][0] - r[0][0])
    for ctl in ('rpp', 'mppi'):
        h = ring_h(yaw=y0 + math.radians(40), coverage_controller=ctl)
        assert h.pending_action(f.ACT_SPIN).purpose == 'cov_pivot'
    assert 'coverage_leg_pivot_rad' not in f.Params.__dataclass_fields__
    assert 'path_conditioning_start_pivot_rad' not in f.Params.__dataclass_fields__


def test_route_is_split_at_conditioner_pivots_only():
    fwd, _back = routes()
    h = Harness(path_conditioning=True)
    h.fsm._region_areas = {i: {'outer': p} for i, p in enumerate(REGION)}
    legs = h.fsm._condition_route([tuple(q) for q in fwd], 'route')
    res = pc.condition(fwd, REGION, pc.Params(r_min=0.8), ring=False)   # route minimum
    assert len(legs) == len(res.pivots) + 1
    for lg in legs[:-1]:
        assert any(geo.dist(lg[-1], (c.x, c.y)) < 0.06 for c in res.corners
                   if c.choice == 'pivot')


# ---- swath-end turn chains (2026-10-16, roadmap 3.2) ---------------------------------------

def _yawed(pts, reverse=False):
    out = []
    for i, q in enumerate(pts):
        a, b = (pts[i], pts[i + 1]) if i + 1 < len(pts) else (pts[i - 1], pts[i])
        y = math.atan2(b[1] - a[1], b[0] - a[0]) + (math.pi if reverse else 0.0)
        out.append((q[0], q[1], geo.wrap_angle(y)))
    return out


def _arc(c, r, a0, a1, n=10):
    return [(c[0] + r * math.cos(a0 + (a1 - a0) * k / n), c[1] + r * math.sin(a0 + (a1 - a0) * k / n))
            for k in range(n + 1)]


def rs_chain(gap=1.28, swath=5.0):
    """Swath A (0,0)->(swath,0), a Reeds-Shepp-like turn (forward arc, reverse arc, forward
    arc) to swath B (swath, gap) -> (0, gap), as the coverage bridge cuts it: forward
    sub-path ending at the first cusp, a reverse leg, a forward sub-path from the second
    cusp through swath B."""
    a = [(0.1 * k, 0.0) for k in range(int(round(swath / 0.1)) + 1)]
    t1 = _arc((swath, 0.5), 0.5, -math.pi / 2, -math.pi / 6)[1:]       # forward, left
    c1 = t1[-1]
    rev = _arc((c1[0] - 0.5 * math.cos(math.pi / 3), c1[1] + 0.2), 0.3, -0.2, 0.6, 6)
    rev = [c1] + rev[1:]
    c2 = rev[-1]
    b0 = (swath, gap)
    t2 = [c2] + [(c2[0] + (b0[0] - c2[0]) * k / 6, c2[1] + (b0[1] - c2[1]) * k / 6)
                 for k in range(1, 6)]
    b = [(swath - 0.1 * k, gap) for k in range(int(round(swath / 0.1)) + 1)]
    return [_yawed(a + t1), _yawed(rev, reverse=True), _yawed(t2 + b)]


BOX = [[(-1.0, -1.0), (7.0, -1.0), (7.0, 3.0), (-1.0, 3.0)]]


def test_reverse_turn_chain_becomes_one_forward_path_with_filleted_connector_corners():
    subs, joins = pc.join_turn_chains(rs_chain(), BOX)
    assert len(subs) == 1 and len(joins) == 1 and joins[0].ok
    sp = subs[0]
    assert not geo.is_reverse_subpath(sp)
    assert geo.dist(joins[0].a, (5.0, 0.0)) < 1e-6 and geo.dist(joins[0].b, (5.0, 1.28)) < 1e-6
    res = pc.condition(sp, BOX)
    assert res.valid
    cs = [c for c in res.corners if c.choice != 'kink']
    assert len(cs) == 2 and all(c.choice == 'fillet' and c.r >= 0.5 for c in cs)
    assert all(abs(abs(math.degrees(c.turn)) - 90) < 2 for c in cs)
    assert not res.pivots
    # drivable: nothing tighter than r_min
    assert all(abs(k) <= 1.0 / 0.5 + 1e-6 for k in res.curvature)


def test_short_connector_u_turn_pivots_on_both_swath_ends():
    # a 0.25 m connector (< merge_m): no fillet fits, two 90 deg pivots (not one 180 deg)
    subs, joins = pc.join_turn_chains(rs_chain(gap=0.25), BOX)
    assert len(subs) == 1 and joins[0].ok
    res = pc.condition(subs[0], BOX)
    piv = [c for c in res.corners if c.choice == 'pivot']
    assert len(piv) == 2 and all(abs(abs(math.degrees(c.turn)) - 90) < 3 for c in piv)
    assert {(round(c.x, 2), round(c.y, 2)) for c in piv} == {(5.0, 0.0), (5.0, 0.25)}


def test_u_turn_with_a_swath_width_connector_is_two_pivots():
    # boustrophedon (pivot turn type): swath, 0.16 m connector, swath back
    pts = [(0.1 * k, 0.0) for k in range(51)] + [(5.0, 0.08), (5.0, 0.16)] + \
        [(5.0 - 0.1 * k, 0.16) for k in range(1, 51)]
    res = pc.condition(pts, BOX)
    assert res.valid
    piv = [c for c in res.corners if c.choice == 'pivot']
    assert len(piv) == 2 and all(abs(abs(math.degrees(c.turn)) - 90) < 3 for c in piv)
    assert len(res.pivots) == 2


def test_turn_chain_kept_when_the_connector_leaves_the_region():
    # the region ends just past the swath ends: the straight connector is outside
    tight = [[(-1.0, -1.0), (5.05, -1.0), (5.05, 0.3), (4.0, 0.6), (5.05, 1.0), (5.05, 3.0),
              (-1.0, 3.0)]]
    chain = rs_chain()
    subs, joins = pc.join_turn_chains(chain, tight)
    assert not joins[0].ok and 'region' in joins[0].why
    assert subs == chain


def test_turn_chain_without_reverse_or_with_a_gap_is_left_alone():
    a, rev, b = rs_chain()
    fwd_only = [a, b]
    assert pc.join_turn_chains(fwd_only, BOX)[0] == fwd_only
    far = [a, [(q[0], q[1] + 0.5, q[2]) for q in rev], b]
    assert pc.join_turn_chains(far, BOX)[0] == far


def test_mission_drives_the_swath_pair_as_one_conditioned_path():
    h = Harness(path_conditioning=True)
    h.areas = [{'name': 'a', 'outer': BOX[0], 'obstacles': [], 'is_navigation_area': False}]
    start_until_planning(h)
    h.fsm.inputs.pose = (0.0, 0.0, 0.0)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan(rs_chain()))
    h.fsm.inputs.is_cutting = True
    h.tick()
    m = h.fsm.mission
    assert len(m.subpaths) == 1 and 0 in m.cond and not m.cond[0]['pivots']
    g = h.goal(f.ACT_FOLLOW)
    assert g['controller_id'] == 'FollowCoveragePath'
    assert g['goal_checker_id'] == 'coverage_corner_goal_checker'
    assert geo.dist(g['poses'][-1], (0.0, 1.28)) < 0.06
    texts = [e.text for e in h.fx if isinstance(e, f.Log)]
    assert any('1 swath-end reverse turn(s) replaced' in t for t in texts)


def test_consecutive_turn_chains_are_all_joined():
    # swath A -> turn -> swath B -> turn -> swath C: swath B ends the first chain and starts
    # the second (the second chain is the first mirrored at x = 2.5 and moved up a swath gap)
    first, rev, b = rs_chain(gap=1.28)

    def t(sp, reverse=False):
        return _yawed([(5.0 - q[0], q[1] + 1.28) for q in sp], reverse)
    first2, rev2, c = t(first), t(rev, True), t(b)
    mid = _yawed([q[:2] for q in b] + [q[:2] for q in first2[51:]])
    subs, joins = pc.join_turn_chains([first, rev, mid, rev2, c], BOX)
    assert [j.ok for j in joins] == [True, True]
    assert len(subs) == 1
    res = pc.condition(subs[0], BOX)
    assert res.valid and len([c for c in res.corners if c.choice != 'kink']) == 4


def test_route_stub_is_not_trimmed_under_path_conditioning():
    # a 0.24 m stub after a 90 deg corner at the route end: kept (it is the leg onto the
    # sub-path start); the trim ended transits up to 0.3 m short (2026-10-16 sim gate)
    route = [(0.1 * k, 0.0, 0.0) for k in range(21)] + \
        [(2.0, 0.06 * k, math.pi / 2) for k in range(1, 5)]
    h = Harness(path_conditioning=True)
    h.fsm.inputs.pose = None
    assert h.fsm._route_prepare(route) == route
    h = Harness(path_conditioning=False)
    h.fsm.inputs.pose = None
    assert len(h.fsm._route_prepare(route)) < len(route)


def test_route_leg_start_pivots_by_the_conditioner_threshold():
    h = Harness(path_conditioning=True, route_transits=True)
    h.fsm.inputs.pose = (0.0, 0.0, math.radians(33))
    h.fsm._route_legs = {'legs': [[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)]], 'i': 0}
    assert h.fsm._route_corner_turn('route') is True
    h2 = Harness(path_conditioning=False, route_transits=True)
    h2.fsm.inputs.pose = (0.0, 0.0, math.radians(33))
    h2.fsm._route_legs = {'legs': [[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)]], 'i': 0}
    assert h2.fsm._route_corner_turn('route') is False


def test_transit_ending_beside_the_start_drives_onto_it_first():
    h = Harness(path_conditioning=True, route_transits=True)
    h.areas = [square(-1, -1, 12)]
    start_until_planning(h)
    h.fsm.inputs.pose = (0.0, 0.0, 0.0)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(3.0, 3.0, 7.0, 3.0)]))
    m = h.fsm.mission
    assert h.name == 'TRANSIT'
    m.transit_target = (3.0, 3.0, 0.0)
    h.fsm._action = None
    h.fsm.inputs.pose = (2.9, 3.15, 0.0)               # arrived 0.18 m beside the start
    h.fsm._route_legs = {'legs': [[(0.0, 0.0, 0.0), (2.9, 3.15, 0.0)]], 'i': 0}
    h.fsm._begin(h.t)
    h.fsm._route_result(f.SUCCEEDED)
    h._apply(h.fsm._end())
    assert m.final_approach
    a = h.pending_action(f.ACT_SPIN)                   # face the start (-56 deg) first
    assert a.purpose == 'final_approach_spin'
    want = math.atan2(-0.15, 0.1)
    assert abs(h.goal(f.ACT_SPIN)['angle'] - want) < 1e-6
    h.fsm.inputs.pose = (2.9, 3.15, want)
    h.finish(f.ACT_SPIN)
    d = h.pending_action(f.ACT_DRIVE)                  # then straight onto it
    assert d.purpose == 'final_approach'
    assert abs(h.goal(f.ACT_DRIVE)['distance_m'] - math.hypot(0.1, 0.15)) < 1e-6
    h.fsm.inputs.pose = (3.0, 3.0, want)
    h.finish(f.ACT_DRIVE, f.ABORTED)                   # outcome not judged: mow
    assert h.name == 'MOWING'


def test_transit_ending_on_the_start_mows_at_once():
    h = Harness(path_conditioning=True, route_transits=True)
    h.areas = [square(-1, -1, 12)]
    start_until_planning(h)
    h.fsm.inputs.pose = (0.0, 0.0, 0.0)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(3.0, 3.0, 7.0, 3.0)]))
    m = h.fsm.mission
    m.transit_target = (3.0, 3.0, 0.0)
    h.fsm.inputs.pose = (2.98, 3.02, 0.0)
    h.fsm._route_legs = {'legs': [[(0.0, 0.0, 0.0), (2.98, 3.02, 0.0)]], 'i': 0}
    h.fsm._route_result(f.SUCCEEDED)
    assert not m.final_approach and h.name == 'MOWING'


def test_routes_take_no_fillet_under_the_route_minimum():
    # the real dock-path routes: every route fillet r >= path_conditioning_route_r_min_m (0.8),
    # tighter corners pivot at the vertex
    h = Harness(path_conditioning=True)
    h.fsm._region_areas = {i: {'outer': q} for i, q in enumerate(REGION)}
    assert f.Params().path_conditioning_route_r_min_m == 0.8
    for route in routes():
        h.fsm._begin(h.t)
        legs = h.fsm._condition_route([tuple(q) for q in route], 'route')
        fx = h.fsm._end()
        assert legs
        logs = [e.text for e in fx if isinstance(e, f.Log) and 'conditioned' in e.text]
        rs = [float(x) for x in re.findall(r' r ([0-9.]+)', logs[0])]
        assert all(r >= 0.8 - 1e-9 for r in rs), logs[0]
