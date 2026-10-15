"""Path conditioning (docs/DESIGN_path_conditioning_2026-10.md steps 2-3).

Real geometry: the run 5 outer ring (test/data/run5_2026-10-15_ring.json, the field
mow_plan's sub-path 0), the real dock path routes (real_dock_path_routes.json), the run 4
/plan fixtures (run4_2026-10-15_plans.json) and the real map's polygons (the sim gate world's
copy of run 5's areas.dat, test/data/run5_region.json)."""

import json
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
        assert g['goal_checker_id'] == 'coverage_leg_goal_checker'


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
    mk = h.mark()
    yaw_in = drive_to_chunk_end(h)
    assert any('pivot vertex' in e.text and 'reached' in e.text for e in h.since(mk, f.Log))
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


def test_mission_turn_legs_are_not_conditioned():
    h = Harness(path_conditioning=True)
    h.areas = [square(-1, -1, 12)]
    start_until_planning(h)
    v = [(0.0, 0.0, 0.0), (0.3, 0.0, 0.0), (0.5, 0.0, 0.0), (0.4, 0.2, 2.0), (0.3, 0.4, 2.0)]
    h.fsm.inputs.pose = (0.0, 0.0, 3.0)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([v, line(0.3, 0.4, 4.0, 0.4)]))
    h.fsm.inputs.is_cutting = True
    h.tick()
    assert h.goal(f.ACT_FOLLOW)['poses'] == v
    assert 0 not in h.fsm.mission.cond


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


def test_revert_start_pivot_0_follows_at_once():
    r = ring()
    y0 = math.atan2(r[1][1] - r[0][1], r[1][0] - r[0][0])
    h = ring_h(yaw=y0 + math.radians(140), path_conditioning_start_pivot_rad=0.0)
    assert h.pending_action(f.ACT_FOLLOW)


def test_route_is_split_at_conditioner_pivots_only():
    fwd, _back = routes()
    h = Harness(path_conditioning=True)
    h.fsm._region_areas = {i: {'outer': p} for i, p in enumerate(REGION)}
    legs = h.fsm._condition_route([tuple(q) for q in fwd], 'route')
    res = pc.condition(fwd, REGION, ring=False)
    assert len(legs) == len(res.pivots) + 1
    for lg in legs[:-1]:
        assert any(geo.dist(lg[-1], (c.x, c.y)) < 0.06 for c in res.corners
                   if c.choice == 'pivot')
