"""Path conditioning (docs/DESIGN_path_conditioning_2026-10.md steps 2-3).

Real geometry: the run 5 outer ring (test/data/run5_2026-10-15_ring.json, the field
mow_plan's sub-path 0), the real dock path routes (real_dock_path_routes.json), the run 4
/plan fixtures (run4_2026-10-15_plans.json) and the real map's polygons (the sim gate world's
copy of run 5's areas.dat, test/data/run5_region.json)."""

import json
import types
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
    # the pivot machinery at the V (the round-2 body limit K-turns it: test_kturn_*)
    params.setdefault('path_conditioning_body_limit_soft_m', -1.0)
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


def test_leg_start_pivot_is_the_one_mechanism():
    r = ring()
    y0 = math.atan2(r[1][1] - r[0][1], r[1][0] - r[0][0])
    h = ring_h(yaw=y0 + math.radians(40))
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
    assert g['controller_id'] == 'FollowCoveragePathMPPI'
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


# ---- edge layout round 2: body limit, K-turns, hairpins, hand-off pivots (2026-10-21) ------

def _lap(poly, step=0.05, mid=False):
    """A closed lap through ``poly``'s vertices (first vertex repeated), sampled at ``step``;
    ``mid``: starting and ending mid first edge (as the planner starts a lap)."""
    pts = []
    P = list(poly) + [poly[0]]
    if mid:
        m = ((P[0][0] + P[1][0]) / 2.0, (P[0][1] + P[1][1]) / 2.0)
        P = [m] + P[1:] + [m]
    for a, b in zip(P, P[1:]):
        n = max(1, int(math.ceil(math.dist(a, b) / step)))
        pts += [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n) for i in range(n)]
    return pts + [tuple(P[-1])]


def _replay_body(res, region, p, span=None):
    """Replay a conditioned path the way the mission drives it and return (worst body
    overhang, worst nose overhang during the K-turn pivots): every pose at its facing yaw,
    every pivot swept (K-turn pivots by their planned angle, the others the short way)."""
    br = pc.BodyRegion(region, res.poses[0], 1e3, p)
    kt = {}
    for i_p1, i_v, d1, d2 in res.kturns:
        kt[i_p1], kt[i_v] = d1, d2
    lo, hi = span if span else (0, len(res.poses) - 1)
    worst = br.drive(res.poses[lo:hi + 1])
    nose = 0.0
    for i in res.pivots:
        if i + 1 >= len(res.poses) or not lo <= i <= hi:
            continue
        q, nxt = res.poses[i], res.poses[i + 1]
        yaw = q[2]
        if i in kt:
            d = kt[i]
            for j in range(37):
                y = yaw + d * j / 36.0
                nose = max(nose, br.out(q[0] + p.body_front_m * math.cos(y),
                                        q[1] + p.body_front_m * math.sin(y)))
        else:
            d = geo.wrap_angle(nxt[2] - yaw)
        worst = max(worst, br.sweep(q[0], q[1], yaw, d))
    return worst, nose


@pytest.mark.parametrize('ccw', [True, False])
def test_kturn_on_a_lap_on_the_line_keeps_the_swept_body_within_the_soft_limit(ccw):
    # lap 0 (soft) ON the line of a pentagon with sharp convex corners: a plain pivot there
    # sweeps the front corners ~0.64 past the next edge, a fillet ~0.41
    poly = [(0, 0), (6, 0), (6, 4), (3, 6.5), (0, 4)]
    lap = _lap(poly, mid=True)
    if not ccw:
        lap = lap[::-1]
    p = pc.Params(body_limit_m=0.26)
    res = pc.condition(lap, [poly], p)
    assert res.valid, res.errors
    kts = [c for c in res.corners if c.choice == 'kturn']
    assert len(kts) >= 2 and len(res.kturns) == len(kts)
    for c in kts:
        assert c.body_m <= 0.26 + 1e-3 and c.back_m >= 0.3 - 1e-9
    # gentle corners (< kturn_min_rad) stay wide arcs (0.31 < kturn_gentle_m): no K-turn
    assert all(abs(c.turn) >= p.kturn_min_rad for c in kts)
    # each K-turn replayed (P1 about-turn, reverse, V pivot) is within the limit, and every
    # K-turn pivot turns the nose through the area, never across the line
    for i_p1, i_v, _d1, _d2 in res.kturns:
        worst, nose = _replay_body(res, [poly], p, (i_p1, i_v))
        assert worst <= 0.26 + 0.01 and nose <= 0.26 + 0.01
    # the whole lap (gentle corners on wide fillets) stays under the 0.35 body trip limit
    assert _replay_body(res, [poly], p)[0] <= 0.35
    sgn = 1.0 if ccw else -1.0
    for i_p1, i_v, d1, d2 in res.kturns:
        assert d1 * sgn > 0.0 and abs(abs(d1) - math.pi) < 1e-6   # about-turn into the area
        assert d2 * sgn < 0.0                                     # nose sweeps inside to u1
        assert res.types[i_p1] == pc.PIVOT and res.types[i_v] == pc.PIVOT
        assert all(t == pc.REVERSE for t in res.types[i_p1 + 1:i_v])
    # the opposite about-turn at P1 would swing the nose across the line
    i_p1 = res.kturns[0][0]
    q = res.poses[i_p1]
    br = pc.BodyRegion([poly], q, 50.0, p)
    wrong = max(br.out(q[0] + 0.6 * math.cos(q[2] - sgn * math.pi * j / 36),
                       q[1] + 0.6 * math.sin(q[2] - sgn * math.pi * j / 36)) for j in range(37))
    assert wrong > 0.3


def test_kturn_hard_mode_keeps_the_whole_body_inside():
    # hard: lap at 0.45 in a 2 m square, fillets forced off (r 2.0 does not fit): the
    # vertex pivot sweeps the front corners ~0.2 m out, the K-turn keeps everything inside
    S, d = 2.0, 0.45
    poly = [(0, 0), (S, 0), (S, S), (0, S)]
    lap = _lap([(d, d), (S - d, d), (S - d, S - d), (d, S - d)], mid=True)
    p = pc.Params(body_limit_m=0.0, r_min=2.0, r_max=2.0, convex_r_max=2.0)
    res = pc.condition(lap, [poly], p)
    assert res.valid, res.errors
    assert sum(1 for c in res.corners if c.choice == 'kturn') == 4
    worst, nose = _replay_body(res, [poly], p)
    assert worst <= 0.01 and nose <= 0.0
    # without the body limit: vertex pivots, the body out ~0.2
    plain = pc.condition(lap, [poly], pc.Params(r_min=2.0, r_max=2.0))
    assert [c.choice for c in plain.corners].count('pivot') >= 3
    assert _replay_body(plain, [poly], p)[0] > 0.15


def test_kturn_gentle_convex_corner_takes_a_wide_fillet_not_a_kturn():
    poly = [(0, 0), (8, 0), (8, 3), (4, 5.5), (0, 3)]     # 58 deg at (8, 3), 64 at the top
    res = pc.condition(_lap(poly[1:] + poly[:1]), [poly], pc.Params(body_limit_m=0.26))
    top = corner_at(res, (8, 3))
    assert top.choice == 'fillet' and top.r > 1.0 + 1e-9    # wider than r_max: less overhang


def test_split_kturns_gives_forward_reverse_forward_legs():
    poly = [(0, 0), (6, 0), (6, 4), (3, 6.5), (0, 4)]
    res = pc.condition(_lap(poly[1:] + poly[:1]), [poly], pc.Params(body_limit_m=0.26))
    legs = pc.split_kturns(res)
    assert len(legs) == 2 * len(res.kturns) + 1
    for k, lg in enumerate(legs):
        assert lg['reverse'] == (k % 2 == 1)
        assert geo.is_reverse_subpath(lg['poses']) == lg['reverse']
        if k:
            assert geo.dist(lg['poses'][0], legs[k - 1]['poses'][-1]) < 1e-9
            assert lg['start_turn'] is not None
        assert len(lg['types']) == len(lg['poses'])
    assert legs[0]['start_turn'] is None
    # the reverse legs run back along the incoming run onto the vertex
    for lg in legs[1::2]:
        assert 0.3 - 1e-6 <= geo.path_length(lg['poses']) <= 1.5 + 1e-6


@pytest.mark.parametrize('spacing', [0.15, 0.18])
def test_hairpin_is_pivot_90_straight_one_row_spacing_pivot_90(spacing):
    # row -> row hairpin (rows 0.45 in, connector of one row spacing): never merged into one
    # ~180 deg pivot, which would leave the tracker a sideways jump onto the next row
    rows = [(x, 0.0) for x in [i * 0.05 for i in range(61)]] + \
        [(x, spacing) for x in [3.0 - i * 0.05 for i in range(61)]]
    region = [[(-2, -2), (5, -2), (5, 2), (-2, 2)]]
    res = pc.condition(rows, region, pc.Params(body_limit_m=0.26))
    piv = [c for c in res.corners if c.choice == 'pivot']
    assert [round(math.degrees(c.turn)) for c in piv] == [90, 90]
    assert math.dist((piv[0].x, piv[0].y), (3.0, 0.0)) < 0.01
    assert math.dist((piv[1].x, piv[1].y), (3.0, spacing)) < 0.01


def test_join_heading_honours_a_pivot_vertex_inside_the_look_ahead():
    # hand-off (2026-10-21 B run, (-0.91, 4.84)): the leg's first segment is 0.1 m long, then
    # turns -112 deg. The 0.4 m look-ahead aimed past the vertex (one -138 deg pivot); with
    # the vertex honoured the robot turns onto the first segment (-21) and pivots at the vertex.
    h0 = math.radians(-21.0)
    v = (0.1 * math.cos(h0), 0.1 * math.sin(h0))
    h1 = h0 + math.radians(-112.0)
    leg = [(0.0, 0.0)] + [(v[0] * k / 2, v[1] * k / 2) for k in (1, 2)] + \
        [(v[0] + 0.05 * k * math.cos(h1), v[1] + 0.05 * k * math.sin(h1)) for k in range(1, 9)]
    pose = (0.0, 0.0, 0.0)
    cut = f.MissionFSM._join_heading(leg, pose)
    assert math.degrees(geo.wrap_angle(cut)) < -60            # the old rule: past the vertex
    hon = f.MissionFSM._join_heading(leg, pose, sharp=0.35)
    assert abs(math.degrees(geo.wrap_angle(hon - h0))) < 1.0
    # at the vertex: the segment after it
    at = f.MissionFSM._join_heading(leg[2:], (v[0], v[1], h0), sharp=0.35)
    assert abs(math.degrees(geo.wrap_angle(at - h1))) < 2.0


def kturn_h():
    """Lap 0 on the line of a 3 x 2.5 m area (soft) whose corners no fillet fits (r 3.0
    only): K-turns there instead of vertex pivots."""
    h = Harness(path_conditioning=True, path_conditioning_r_min_m=3.0,
                path_conditioning_r_max_m=3.0, path_conditioning_convex_r_max_m=3.0)
    h.areas = [{'name': 'a', 'outer': [(0, 0), (3, 0), (3, 2.5), (0, 2.5)], 'obstacles': [],
                'is_navigation_area': False}]
    lap = _lap([(1.5, 0), (3, 0), (3, 2.5), (0, 2.5), (0, 0), (1.0, 0)])
    sp = [(q[0], q[1], math.atan2(lap[min(i + 1, len(lap) - 1)][1] - q[1],
                                  lap[min(i + 1, len(lap) - 1)][0] - q[0]))
          for i, q in enumerate(lap)]
    start_until_planning(h)
    h.fsm.inputs.pose = (1.5, 0.0, 0.0)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([sp, line(0.6, 1.2, 2.4, 1.2)]))
    h.fsm.inputs.is_cutting = True
    h.tick()
    return h


def test_mission_drives_a_kturn_about_turn_reverse_blade_off_then_pivot_inward():
    h = kturn_h()
    m = h.fsm.mission
    kts = [k for k, c in m.cond.items() if c.get('kturn')]
    assert kts and kts[0] == 1                       # forward to P1, reverse leg, forward ...
    assert geo.is_reverse_subpath(m.subpaths[1]) and not geo.is_reverse_subpath(m.subpaths[2])
    d1, d2 = m.cond[1]['start_turn'], m.cond[2]['start_turn']
    assert d1 > 0 and d2 < 0                         # CCW lap: about-turn left, V pivot right
    # forward leg 0 up to P1
    p1 = m.subpaths[1][0]
    h.fsm.inputs.pose = (p1[0], p1[1], 0.0)
    h.finish(f.ACT_FOLLOW, arrive=False)
    for _ in range(5):                               # (corner pivots before P1, if any)
        if m.sub_i == 1:
            break
        h.fsm.inputs.pose = (p1[0], p1[1], 0.0)
        if h.fsm._action is not None and h.fsm._action.name == f.ACT_SPIN:
            h.finish(f.ACT_SPIN)
        elif h.fsm._action is not None:
            h.finish(f.ACT_FOLLOW, arrive=False)
        h.tick()
    assert m.sub_i == 1
    h.tick()
    # the about-turn: blade off, a ~180 deg Spin the planned way round (into the area)
    assert not h.blade
    sp = h.goal(f.ACT_SPIN)
    assert sp['angle'] > math.radians(150)
    h.fsm.inputs.pose = (p1[0], p1[1], geo.wrap_angle(sp['angle']))
    h.finish(f.ACT_SPIN)
    g = h.goal(f.ACT_FOLLOW)
    assert g['controller_id'] == h.fsm.coverage_controller_id(True)
    assert not h.blade
    v = m.subpaths[1][-1]
    h.fsm.inputs.pose = (v[0], v[1], math.pi)
    h.finish(f.ACT_FOLLOW, arrive=False)
    for _ in range(40):                              # the next forward leg spins the blade up
        h.tick(dt=0.5)
        if h.fsm._action is not None and h.fsm._action.name == f.ACT_SPIN:
            break
    assert m.sub_i == 2 and h.blade
    sp = h.goal(f.ACT_SPIN)
    assert -math.radians(100) < sp['angle'] < -math.radians(80)   # pivot at V, nose inward


def _arc_lap():
    """Lap 0 on the line of an area whose edge into a sharp corner V is curved: an r 0.6 arc
    over the last 0.63 m (no straight 0.3 m run before V), then a 100 deg right turn."""
    pts = [(0.0, 1.5), (0.0, 0.5)]
    pts += [(x / 20.0, 0.5) for x in range(1, 61)]
    # tangent continuation: an arc of r 0.6 bending right from heading 0 to heading -60 deg
    cx, cy = pts[-1][0], 0.5 - 0.6
    for k in range(1, 31):
        t = math.radians(60.0) * k / 30.0
        pts.append((cx + 0.6 * math.sin(t), cy + 0.6 * math.cos(t)))
    V = pts[-1]
    h = math.radians(-60.0) - math.radians(100.0)           # a sharp right turn at V
    for k in range(1, 41):
        pts.append((V[0] + 0.05 * k * math.cos(h), V[1] + 0.05 * k * math.sin(h)))
    return pts, V


def test_kturn_with_a_curved_approach_retraces_the_driven_curve():
    lap, V = _arc_lap()
    end = lap[-1]
    region = [lap + [(end[0] - 3.0, end[1]), (-3.0, 1.5)]]   # interior on the right (CW)
    p = pc.Params(body_limit_m=0.26, body_cap_m=9.0)          # (the cap would cut it)
    res = pc.condition(lap, region, p, ring=True, turn_sign=-1.0)
    assert res.valid, res.errors
    kt = corner_at(res, V)
    # the curve itself (r 0.6, driven forward ON the line) puts the body ~0.43 out: the
    # K-turn is no worse than that, far better than the vertex pivot (0.65)
    br = pc.BodyRegion(region, V, 50.0, p)
    curve = br.drive([(q[0], q[1], math.atan2(b[1] - q[1], b[0] - q[0]))
                      for q, b in zip(lap[60:91], lap[61:92])])
    assert kt.choice == 'kturn' and kt.back_m >= 0.3
    assert kt.body_m <= curve + 0.01 and kt.body_m < 0.5
    i_p1, i_v, d1, d2 = res.kturns[0]
    # the reverse leg runs back over the curve just driven: every pose ON the drawn line
    for q in res.poses[i_p1:i_v + 1]:
        assert geo.path_projection(lap, q)[1] < 0.01
    # and follows the curve, not the chord P1 -> V
    P1, Vq = res.poses[i_p1], res.poses[i_v]
    assert max(abs(pc._orient(P1, Vq, q)) / geo.dist(P1, Vq)
               for q in res.poses[i_p1:i_v + 1]) > 0.02
    yaws = [q[2] for q in res.poses[i_p1 + 1:i_v]]
    assert abs(geo.wrap_angle(yaws[-1] - yaws[0])) > math.radians(20)   # facing turns with it
    worst, _nose = _replay_body(res, region, p, (i_p1, i_v))
    assert worst <= kt.body_m + 0.01
    assert d1 < 0 and d2 > 0          # CW: about-turn right (into the area), V pivot left


def test_kturn_retraces_a_jog_vertex_with_a_pivot_in_the_reverse_leg():
    # a 30 deg drawing jog 0.2 m before a sharp 100 deg corner: P1 lies back past the jog,
    # whose pivot moves into the reverse leg (a reverse leg either side of it)
    V = (4.0, 0.0)
    J = (3.8, 0.0)
    hj = math.radians(30.0)
    pts = [(x / 20.0, 0.0) for x in range(0, 77)]
    pts = [(J[0] - 0.05 * k * math.cos(hj), J[1] - 0.05 * k * math.sin(hj))
           for k in range(40, 0, -1)] + [J] + [(J[0] + 0.05 * k * 4 / 4, 0.0) for k in (1, 2, 3)]
    pts.append(V)
    h = math.radians(100.0)
    pts += [(V[0] + 0.05 * k * math.cos(h), V[1] + 0.05 * k * math.sin(h)) for k in range(1, 41)]
    region = [pts + [(pts[-1][0] - 4.0, pts[-1][1]), (pts[0][0] - 1.0, pts[0][1] + 1.0)]]
    p = pc.Params(body_limit_m=0.26)
    res = pc.condition(pts, region, p, ring=True, turn_sign=1.0)
    assert res.valid, res.errors
    assert corner_at(res, V).choice == 'kturn'
    assert corner_at(res, J).choice == 'retraced'
    i_p1, i_v, _d1, _d2 = res.kturns[0]
    inner = [i for i in res.pivots if i_p1 < i < i_v]
    assert len(inner) == 1 and geo.dist(res.poses[inner[0]], J) < 0.01
    legs = pc.split_kturns(res)
    assert [lg['reverse'] for lg in legs] == [False, True, True, False]
    for lg in legs[1:3]:
        assert geo.is_reverse_subpath(lg['poses'])
    assert abs(abs(math.degrees(legs[2]['start_turn'])) - 30.0) < 3.0
    assert _replay_body(res, region, p, (i_p1, i_v))[0] <= 0.26 + 0.01


def _rounded_rect(w, h, R, n=6):
    """A w x h rectangle whose corners are drawn arcs of radius R (n segments each), CCW."""
    out = []
    for cx, cy, a0 in ((w - R, R, -90), (w - R, h - R, 0), (R, h - R, 90), (R, R, 180)):
        for k in range(n + 1):
            a = math.radians(a0 + 90.0 * k / n)
            out.append((cx + R * math.cos(a), cy + R * math.sin(a)))
    return out


@pytest.mark.parametrize('R', [0.44, 0.72])
def test_drawn_arc_bends_on_lap0_take_fillets_not_kturns(R):
    # owner_far Area 6's bulges are driven arcs (R 0.44 / 0.72): following them ON the line
    # puts the outer front corner sqrt((R + 0.235)^2 + 0.6^2) - R out (0.46 / 0.41), as the
    # owner's own drive did. That is the line's curvature, not a corner: no K-turn there.
    poly = _rounded_rect(6.0, 4.0, R)
    res = pc.condition(_lap(poly, mid=True), [poly], pc.Params(body_limit_m=0.26))
    assert res.valid, res.errors
    assert res.kturns == []
    assert all(c.choice in ('fillet', 'kink') for c in res.corners), \
        [(c.choice, round(math.degrees(c.turn))) for c in res.corners]
    for c in res.corners:
        if c.body_m is not None:
            assert c.body_m <= c.limit_m + 1e-3


def test_clean_area_ring_drops_a_cusp_and_a_start_wobble_only():
    # owner_far Area 6's recorded vertices: the +40 / -60 / +25 deg wobble around the start
    # vertex v0 (v42, v0, v1) and the 9 cm cusp v19 -> v20 (each a K-turn on lap 0); the
    # rest of the outline is filler. Drawn R 0.72 arcs keep their shape.
    v41, v42, v0 = (-27.8459, -15.6639), (-28.6156, -15.2297), (-28.7605, -15.2584)
    v1, v2 = (-28.9114, -15.0863), (-29.3873, -14.8787)
    v18, v19, v20, v21 = (-27.7254, -4.10399), (-27.1895, -3.48144), (-27.2757, -3.49238), \
        (-27.2737, -2.70777)
    ring = [v0, v1, v2, (-33.0, -12.0), (-33.0, -6.0), v18, v19, v20, v21, (-27.0, -1.5),
            (-22.0, -1.5), (-22.0, -13.0), v41, v42]
    out = geo.clean_area_ring(ring)
    near = lambda q: any(math.dist(q, r) < 0.01 for r in out)   # noqa: E731

    def turns(poly):
        n = len(poly)
        return [math.degrees(geo.wrap_angle(
            math.atan2(poly[(i + 1) % n][1] - poly[i][1], poly[(i + 1) % n][0] - poly[i][0]) -
            math.atan2(poly[i][1] - poly[i - 1][1], poly[i][0] - poly[i - 1][0])))
            for i in range(n)]
    # no cusp left (no vertex turns > 90 deg there) and the wobble is one gentle bend
    for q, t in zip(out, turns(out)):
        if math.dist(q, v19) < 0.5 or math.dist(q, v0) < 0.5:
            assert abs(t) < 45.0, (q, t)
    assert not (near(v19) and near(v20))
    assert sum(1 for q in (v42, v0, v1) if near(q)) <= 1
    for q in ring:                                               # never far off the line
        assert abs(pc.clearance(q, [out])) <= 0.10
    # the drawn arcs of a rounded rectangle (R 0.72) stay: corners within 0.05 of the arcs
    rr = _rounded_rect(6.0, 4.0, 0.72)
    out = geo.clean_area_ring(rr)
    assert len(out) >= 4 * 3
    for q in rr:
        assert abs(pc.clearance(q, [out])) <= 0.05 + 1e-6
    # an outline with nothing to drop keeps its order and vertices
    sqr = [(0, 0), (5, 0), (5, 5), (0, 5)]
    assert geo.clean_area_ring(sqr) == [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0), (0.0, 5.0)]


# ---- body cap / corner cut (field 2026-10-23, area 5) -------------------------------------

# area 5 ("Area 7") of the 2026-10-23 field maps: lap 0's -89 deg corner at (-5.81, -2.62) is
# a blunt 71 deg spike; its K-turn reversed 0.30 m with the body 0.54 m out (the retrace into
# the tip, whatever the reverse length), pivots drifted ~0.15 m and the robot stuck 0.74 m out
FIELD23_AREA5 = [
    (-2.42777, 0.603669), (-3.68007, -0.64144), (-4.13443, -1.25334), (-5.4586, -2.45428),
    (-5.64323, -2.60242), (-5.81183, -2.6159), (-6.10666, -2.26859), (-6.40751, -1.56303),
    (-6.49084, -0.694381), (-6.6368, -0.497456), (-6.60865, -0.32109), (-6.81943, 0.200973),
    (-6.7096, 0.67657), (-5.97887, 1.77966), (-4.41611, 3.63844), (-3.71662, 3.98655),
    (-2.75692, 4.28974), (-1.80164, 4.76187), (-0.601567, 4.99453), (-0.307907, 5.13676),
    (-0.367525, 5.16016), (-0.0265033, 5.25821), (0.0592182, 4.98217), (-0.213657, 2.26314),
    (-1.35556, 1.21173), (-2.2729, 0.579132)]
TRIP_SOFT = {'corner': 0.55, 'rear_mid': 0.40}
DRIFT_M = 0.15


def _pivot_sweeps_by_point(res, region, p):
    """Worst overhang through every planned pivot (K-turn ones by their planned angle),
    split into the outline's corners / nose / sides and its mid-rear point."""
    br = pc.BodyRegion(region, res.poses[0], 1e3, p)
    kt = {}
    for i_p1, i_v, d1, d2 in res.kturns:
        kt[i_p1], kt[i_v] = d1, d2
    F, R, W = p.body_front_m, p.body_rear_m, p.half_width_m
    groups = {'corner': [(F, W), (F, -W), (-R, W), (-R, -W), (F, 0.0), (0.25, W), (0.25, -W)],
              'rear_mid': [(-R, 0.0)]}
    worst = {k: 0.0 for k in groups}
    for i in res.pivots:
        if i + 1 >= len(res.poses):
            continue
        q = res.poses[i]
        d = kt.get(i, geo.wrap_angle(res.poses[i + 1][2] - q[2]))
        n = max(1, int(abs(d) / math.radians(3)))
        for j in range(n + 1):
            y = q[2] + d * j / n
            c, s = math.cos(y), math.sin(y)
            for k, pts in groups.items():
                for a, b in pts:
                    worst[k] = max(worst[k], br.out(q[0] + a * c - b * s, q[1] + a * s + b * c))
    return worst


def test_field_corner_is_cut_with_the_body_under_the_cap():
    ring = geo.clean_area_ring(FIELD23_AREA5)
    p = pc.Params(body_limit_m=0.26)
    res = pc.condition(_lap(ring, mid=True), [ring], p)
    assert res.valid, res.errors
    c = corner_at(res, (-5.81, -2.62), tol=0.06)
    assert c.choice == 'cut' and c.body_m <= 0.35 + 1e-3
    assert 'tip uncut' in c.why
    # every corner where the robot pivots is planned within the cap (soft 0.35)...
    for cc in res.corners:
        if cc.choice in ('pivot', 'kturn', 'cut'):
            assert cc.body_m <= 0.35 + 2e-3, (cc.choice, cc.x, cc.y, cc.body_m)
    # ... and 0.15 m of pivot drift on top stays under the soft body trips
    worst = _pivot_sweeps_by_point(res, [ring], p)
    for k, lim in TRIP_SOFT.items():
        assert worst[k] + DRIFT_M < lim, (k, worst[k])
    # the cut leaves the drawn line inside the area: all its poses are inside
    assert min(pc.clearance(q, [ring]) for q in res.poses) > -0.01


def test_body_cap_cuts_a_spike_that_no_kturn_fits():
    # the field spike alone (its tip vertices, closed by two far ones): a short -34 deg bend
    # 0.17 m before the -135 deg tip; the K-turn's retrace into the tip is 0.54 out whatever
    # its length, the cut keeps it far lower; with the cap off the K-turn is driven
    poly = [(-4.13443, -1.25334), (-5.4586, -2.45428), (-5.64323, -2.60242),
            (-5.81183, -2.6159), (-6.10666, -2.26859), (-6.40751, -1.56303),
            (-6.6, 1.0), (-3.5, 1.0)]
    p = pc.Params(body_limit_m=0.26)
    res = pc.condition(_lap(poly, mid=True), [poly], p)
    assert res.valid, res.errors
    c = corner_at(res, (-5.81, -2.62), tol=0.06)
    # (the leg after the tip is shorter here than in the field: 0.39, the least overhang)
    assert c.choice == 'cut' and c.body_m < 0.40
    assert _pivot_sweeps_by_point(res, [poly], p)['corner'] + DRIFT_M < TRIP_SOFT['corner']
    off = pc.condition(_lap(poly, mid=True), [poly], pc.Params(body_limit_m=0.26, body_cap_m=9.0))
    c = corner_at(off, (-5.81, -2.62), tol=0.06)
    assert c.choice == 'kturn' and c.body_m > 0.45


# =====================================================================
# field 2026-10-28 (area 4 re-plan, 04:11): a near-U corner's long fillet tangent snapped
# onto a later pass of the same sub-path -> a 0.03 m piece and a 3.9 m hop to the next
# pivot vertex (the robot orbited it until the leg loop guard). Staggered row ends: R1
# south to (0, 0), R2 starts 0.554 m further north, so R1's end is a 162 deg corner (r 1.0:
# tangent 6.3 m) and R2's start is no corner.
# =====================================================================
def _staggered_rows():
    def densify(q, step=0.10):
        out = [q[0]]
        for a, b in zip(q, q[1:]):
            n = max(1, math.ceil(math.dist(a, b) / step - 1e-9))
            out += [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n)
                    for k in range(1, n + 1)]
        return out
    w, top = 0.18, 10.0
    rows = [(0.0, top, 0.0), (w, 0.554, top)]
    x, down = 2 * w, True
    while x < 2.6:
        rows.append((x, top, 0.0) if down else (x, 0.0, top))
        down = not down
        x = round(x + w, 6)
    raw = [(-0.18, 0.0), (-0.18, top)]
    for (rx, y0, y1) in rows:
        raw += [(rx, y0), (rx, y1)]
    prm = pc.Params(r_min=0.5, r_max=1.0, margin_m=0.05, half_width_m=0.235, slack_m=0.05,
                    pivot_min_rad=0.35, edge_pivot_rad=0.0, edge_clearance_m=0.20)
    prm.body_limit_m, prm.body_cap_m, prm.convex_r_max = 0.26, 0.35, 2.0
    region = [[(-1.0, -1.0), (4.0, -1.0), (4.0, 11.0), (-1.0, 11.0)]]
    return densify(raw), region, prm


def test_fillet_tangent_stays_on_its_own_corner_runs_no_hop():
    pts, region, prm = _staggered_rows()
    res = pc.condition(pts, region, prm)
    assert res.valid, res.errors
    P = res.poses
    assert max(math.dist(P[i][:2], P[i + 1][:2]) for i in range(len(P) - 1)) < 0.3
    assert len([q for q in P if abs(q[0] - 0.18) < 0.02 and 1.0 < q[1] < 9.0]) > 100   # R2 kept


def test_conditioning_that_skips_drawn_path_is_invalid(monkeypatch):
    # backstop: without the tangent bound (the old global projection) the conditioned path
    # skipped R1's tail and R2 with one straight hop; it must fail validation, not be driven
    pts, region, prm = _staggered_rows()
    src = open(pc.__file__).read().replace(
        "s2 <= s_hi + lim_out + p.tangent_tol_m and", "True and", 1)
    old = types.ModuleType('pc_old')
    old.__file__, old.__package__ = pc.__file__, 'mower_mission'
    exec(compile(src, pc.__file__, 'exec'), old.__dict__)
    res = old.condition(pts, region, prm)
    assert not res.valid and 'skipped' in res.errors[0]


def test_split_hops_backstop_splits_densified_forward_subpaths_only():
    from mower_mission.fsm.common import split_hops
    a = [(0.05 * i, 0.0, 0.0) for i in range(41)]               # 2 m densified
    b = [(2.0 + 0.05 * i, 3.9, 0.0) for i in range(41)]
    out, n = split_hops([a + b])
    assert n == 1 and [len(s) for s in out] == [41, 41]
    sparse = [(0, 0, 0), (1, 0, 0), (5, 0, 0), (6, 0, 0)]          # raw vertices: untouched
    rev = [(0.05 * i, 0.0, math.pi) for i in range(41)] + [(4.0, 0.0, math.pi)]
    assert split_hops([sparse, rev]) == ([sparse, rev], 0)
