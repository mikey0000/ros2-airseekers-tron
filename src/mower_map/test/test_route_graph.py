# SPDX-License-Identifier: GPL-3.0-or-later
"""ROS-free tests for mower_map.route_graph (run: python3 -m pytest src/mower_map/test)."""

import math
import os

import numpy as np
import pytest

from mower_map import areas as core
from mower_map import route_graph as rg

DATA = os.path.join(os.path.dirname(__file__), 'data')
CLR = 0.35


# ------------------------------------------------------------------ helpers

def _sim():
    areas, _ = core.load_areas_file(os.path.join(DATA, 'sim_garden_paths.dat'))
    dock = core.load_dock_file(os.path.join(DATA, 'sim_garden_dock.yaml'))
    return areas, dock


def _field():
    areas, _ = core.load_areas_file(os.path.join(DATA, 'field_2026-10-08_paths.dat'))
    dock = core.load_dock_file(os.path.join(DATA, 'field_2026-10-08_dock.yaml'))
    return areas, dock


def _square(n=10.0, obstacles=None):
    poly = [(0.0, 0.0), (n, 0.0), (n, n), (0.0, n)]
    return core.Area('lawn', poly, obstacles=list(obstacles or []))


def _box(cx, cy, h):
    return core.Obstacle(polygon=[(cx - h, cy - h), (cx + h, cy - h),
                                  (cx + h, cy + h), (cx - h, cy + h)])


def _dist_edges(poly, pts):
    a = np.array(pts, dtype=float)
    return core.distance_to_polygon_edges(a[:, 0], a[:, 1], poly)


def _near(route, p, r):
    return [i for i, q in enumerate(route.poses) if math.hypot(q[0] - p[0], q[1] - p[1]) <= r]


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def check_inside(route, areas, clearance, extra_polys=()):
    """Poses stay in the area / path union, out of obstacles, and keep ``clearance`` from
    the edges of their own area except near route ends and kind changes.
    ``extra_polys``: further walkable polygons (the graph's synthetic dock corridor)."""
    poses = route.poses
    xs = np.array([p[0] for p in poses])
    ys = np.array([p[1] for p in poses])
    inside = np.zeros(len(poses), dtype=bool)
    edge_d = np.full(len(poses), np.inf)
    for a in areas:
        if len(a.polygon) < 3:
            continue
        inside |= core.points_in_polygon(xs, ys, a.polygon)
        edge_d = np.minimum(edge_d, core.distance_to_polygon_edges(xs, ys, a.polygon))
    for q in extra_polys:
        inside |= core.points_in_polygon(xs, ys, q)
        edge_d = np.minimum(edge_d, core.distance_to_polygon_edges(xs, ys, q))
    obstacles = [o.polygon for a in areas for o in a.obstacles if len(o.polygon) >= 3]
    for i in range(len(poses)):
        assert inside[i] or edge_d[i] <= 0.05, 'pose %d %s outside all areas' % (i, poses[i][:2])
    for o in obstacles:
        assert not core.points_in_polygon(xs, ys, o).any(), 'pose inside an obstacle'
    # clearance for in-area poses away from route ends / kind changes
    excl = [poses[0][:2], poses[-1][:2]]
    excl += [poses[i][:2] for i in range(len(poses)) if route.kinds[i] != 'area']
    ex = np.array(excl)
    for i, p in enumerate(poses):
        if route.kinds[i] != 'area':
            continue
        if np.hypot(ex[:, 0] - p[0], ex[:, 1] - p[1]).min() <= clearance + 0.15:
            continue
        pts = [p[:2]]
        d = _dist_edges(areas[route.areas[i]].polygon, pts)[0]
        assert d >= clearance - 0.02, 'pose %d %s only %.3f from area edge' % (i, p[:2], d)
        for o in obstacles:
            d = _dist_edges(o, pts)[0]
            assert d >= clearance - 0.02, 'pose %d %s only %.3f from obstacle' % (i, p[:2], d)
    for p, q in zip(poses, poses[1:]):
        assert math.hypot(q[0] - p[0], q[1] - p[1]) <= 0.11
    return True


def _min_dist_to_polygon(route, poly):
    return float(_dist_edges(poly, [p[:2] for p in route.poses]).min())


def _follows(route, i0, i1, line, tol):
    ok = []
    for p in route.poses[i0:i1 + 1]:
        best = min(math.hypot(p[0] - a[0], p[1] - a[1]) for a in line)
        ok.append(best)
    return max(ok)


def _line_dist(p, line):
    cum = rg._cumulative(line)
    return rg._project(line, cum, p)[0]


def _dock_polys_of(areas):
    d = core.load_dock_file(os.path.join(DATA, 'sim_garden_dock.yaml'))
    return [p.polygon for p in rg.RouteGraph([], d).paths if p.index == -1]


def _dock_polys(g):
    # the dock -> approach corridor is synthesised by the graph, it is not in `areas`
    return [p.polygon for p in g.paths if p.index == -1]


def _names(route):
    return [lg['name'] for lg in route.legs]


# ------------------------------------------------------------------ (a) sim map

SIM_GOAL = (20.5, -16.5, 0.0)
P1_END = (9.69, -0.44)
P2_START = (8.59, -5.37)


def _exit_index(route, chan, x0):
    """Index of the last pose on the path ``chan`` (within 0.45 m of its centreline) before
    the route first leaves it; poses with x < x0 (the dock leg) are skipped."""
    last = None
    for i, p in enumerate(route.poses):
        if p[0] < x0 - 1e-6 and last is None:
            continue
        if _line_dist(p[:2], chan) > 0.45:
            break
        last = i
    return last


def _check_sim_route(route, areas, g):
    assert isinstance(route, rg.Route)
    chan = [tuple(p) for p in areas[2].channel]
    # 2026-10-09 mid-path portals + area_weight 8: the route rides Path 1 to near the foot
    # of the hop to Path 2 (round the drawn triangle), no longer to Path 1's end
    i1 = _exit_index(route, chan, chan[0][0])
    assert i1 is not None and route.poses[i1][0] >= 7.3, route.poses[i1]
    i2 = _near(route, P2_START, 0.3)
    assert i2 and i2[0] > i1
    names = _names(route)
    order = [names.index('Path 1')]
    assert 'lawn' in names[order[0]:]
    order.append(names.index('lawn', order[0]))
    order.append(names.index('Path 2', order[1]))
    assert any(lg['kind'] == 'area' and lg['area'] == 1 for lg in route.legs[order[2]:])
    check_inside(route, areas, CLR, _dock_polys(g))
    tri = areas[0].obstacles[0].polygon
    assert _min_dist_to_polygon(route, tri) >= 0.33


def test_sim_dock_approach_to_area1():
    areas, dock = _sim()
    g = rg.RouteGraph(areas, dock)
    _check_sim_route(g.plan((0.8, 0.0, 0.0), SIM_GOAL), areas, g)


def test_sim_dock_pose_to_area1():
    areas, dock = _sim()
    g = rg.RouteGraph(areas, dock)
    _check_sim_route(g.plan((0.0, 0.0, 0.0), SIM_GOAL), areas, g)


def test_sim_final_yaw_and_spacing():
    areas, dock = _sim()
    route = rg.RouteGraph(areas, dock).plan((0.8, 0.0, 0.0), (20.5, -16.5, 1.25))
    assert isinstance(route, rg.Route)
    assert abs(_wrap(route.poses[-1][2] - 1.25)) < 1e-9
    assert route.path_length_m > 0 and route.length_m >= route.path_length_m
    assert route.single_area == -1


def test_sim_start_on_path_rides_it_whatever_the_weight():
    """Owner rule 2026-10-09 (paths END TO END): a start on a path band is driven along
    the path, even when lawn metres are as cheap as path metres."""
    areas, dock = _sim()
    g = rg.RouteGraph(areas, dock, rg.RouteParams(area_weight=1.0))
    route = g.plan((0.8, 0.0, 0.0), SIM_GOAL)
    assert isinstance(route, rg.Route)
    assert route.legs[0]['kind'] == rg.PATH
    chan = [tuple(p) for p in areas[2].channel]
    i1 = _exit_index(route, chan, chan[0][0])
    assert math.hypot(route.poses[i1][0] - P1_END[0], route.poses[i1][1] - P1_END[1]) <= 0.4


# ------------------------------------------------------------- (a2) redrawn sim map

def _sim_v2():
    areas, _ = core.load_areas_file(os.path.join(DATA, 'sim_garden_paths_v2.dat'))
    dock = core.load_dock_file(os.path.join(DATA, 'sim_garden_dock.yaml'))
    return areas, dock


V2_ENDS = [(10.4582, -0.2161), (10.5279, -0.407447)]   # Path 1 / Dock path ends


def test_sim_v2_dock_rides_path1_end_to_end():
    """Redrawn map (Path 1 + Dock path on top of each other): from the dock approach the
    route rides the path network to its END, hops the lawn to Path 2's START, follows
    Path 2 into area 1 (owner: paths are driven end to end)."""
    areas, dock = _sim_v2()
    route = rg.RouteGraph(areas, dock).plan((0.82, 0.0, 0.0), (23.68, -9.01, 0.0))
    _check_end_to_end(areas, route, V2_ENDS)


def test_fillet_rounds_acute_path_bend_inside_the_band():
    """Path 2's 65 deg bend at (8.106, -13.089) (old sim map): MPPI left the band by
    0.13 m there. The route now rounds it (no pose at the vertex) inside the band."""
    areas, dock = _sim()
    vertex = (8.106, -13.0885)
    on = rg.RouteGraph(areas, dock, rg.RouteParams(fillet_radius_m=1.0)).plan(
        (0.8, 0.0, 0.0), SIM_GOAL)
    off = rg.RouteGraph(areas, dock).plan(           # default: off (MPPI cut fillets more)
        (0.8, 0.0, 0.0), SIM_GOAL)
    d_on = min(math.hypot(p[0] - vertex[0], p[1] - vertex[1]) for p in on.poses)
    d_off = min(math.hypot(p[0] - vertex[0], p[1] - vertex[1]) for p in off.poses)
    assert d_off < 0.01 and d_on > 0.1
    near = [p for p in on.poses if math.hypot(p[0] - vertex[0], p[1] - vertex[1]) < 1.0]
    assert near and all(core.point_in_polygon(p[0], p[1], areas[3].polygon) for p in near)
    # heading change between 0.1 m poses stays below a 0.3 m radius turn
    for a, b in zip(on.poses, on.poses[1:-1]):
        if math.hypot(a[0] - vertex[0], a[1] - vertex[1]) < 1.0:
            assert abs(_wrap(b[2] - a[2])) <= 0.1 / 0.3 + 1e-6


# ------------------------------------------------------------------ (b) field map

def test_field_dock_to_lawn_section():
    areas, dock = _field()
    g = rg.RouteGraph(areas, dock)
    start = (0.394004, -1.29168, 1.757547)
    route = g.plan(start, (-19.0, -6.0, 0.0))
    assert isinstance(route, rg.Route)
    assert _names(route) == ['dock', 'Dock path', 'Area 1', 'Path 2', 'lawn section']
    i1 = _near(route, (-1.34, 4.41), 0.3)
    assert i1
    i1 = i1[0]
    i2 = _near(route, (-18.51, -0.52), 0.3)
    assert i2 and i2[0] > i1
    chan = [tuple(p) for p in areas[3].channel]
    for p in route.poses[i1:i2[0] + 1]:
        assert _line_dist(p[:2], chan) <= 0.45
    check_inside(route, areas, CLR, _dock_polys(g))


def test_field_reverse_to_dock_approach():
    areas, dock = _field()
    g = rg.RouteGraph(areas, dock)
    goal = g.dock_goal()
    assert goal is not None
    route = g.plan((-19.0, -6.0, 0.0), goal)
    assert isinstance(route, rg.Route)
    last = route.poses[-1]
    # the approach pose lies on the dock path: the last pose is its centreline projection,
    # which can differ from the goal by up to the 1 mm pose de-duplication in Route
    assert math.hypot(last[0] - goal[0], last[1] - goal[1]) < 2e-3
    assert abs(_wrap(last[2] - _wrap(dock.yaw + math.pi))) < 1e-6
    check_inside(route, areas, CLR, _dock_polys(g))


# ------------------------------------------------------------------ (c) obstacle detour

def test_obstacle_detour():
    box = _box(5.0, 5.0, 1.0)
    areas = [_square(obstacles=[box])]
    route = rg.RouteGraph(areas).plan((1.0, 5.0, 0.0), (9.0, 5.0, 0.0))
    assert isinstance(route, rg.Route)
    assert _min_dist_to_polygon(route, box.polygon) >= 0.33
    xs = np.array([p[0] for p in route.poses])
    ys = np.array([p[1] for p in route.poses])
    assert not core.points_in_polygon(xs, ys, box.polygon).any()
    assert route.length_m > 8.0
    assert route.single_area == 0
    assert set(route.kinds) == {'area'}
    check_inside(route, areas, CLR)


def test_no_obstacle_is_straight():
    route = rg.RouteGraph([_square()]).plan((1.0, 5.0, 0.0), (9.0, 5.0, 0.0))
    assert isinstance(route, rg.Route)
    assert abs(route.length_m - 8.0) < 0.01


# ------------------------------------------------------------------ (d) concave area

def test_concave_u_shape_bends_round_notch():
    u = [(0, 0), (10, 0), (10, 10), (7, 10), (7, 3), (3, 3), (3, 10), (0, 10)]
    areas = [core.Area('u', [(float(x), float(y)) for x, y in u])]
    route = rg.RouteGraph(areas).plan((1.5, 9.0, 0.0), (8.5, 9.0, 0.0))
    assert isinstance(route, rg.Route)
    check_inside(route, areas, CLR)
    assert min(p[1] for p in route.poses) < 3.0
    assert route.length_m > 7.0


# ------------------------------------------------------------------ (e) off graph

def test_start_off_graph_returns_reason():
    r = rg.RouteGraph([_square()]).plan((50.0, 50.0, 0.0), (5.0, 5.0, 0.0))
    assert isinstance(r, str) and 'start' in r


def test_goal_off_graph_returns_reason():
    r = rg.RouteGraph([_square()]).plan((5.0, 5.0, 0.0), (50.0, 50.0, 0.0))
    assert isinstance(r, str) and 'goal' in r


def test_disconnected_areas_no_route():
    a = _square()
    b = core.Area('far', [(30.0, 0.0), (40.0, 0.0), (40.0, 10.0), (30.0, 10.0)])
    r = rg.RouteGraph([a, b]).plan((5.0, 5.0, 0.0), (35.0, 5.0, 0.0))
    assert isinstance(r, str)


# ------------------------------------------------------------------ (f) near-edge

def test_near_edge_start_and_goal_kept_exactly():
    areas = [_square()]
    route = rg.RouteGraph(areas).plan((0.15, 5.0, 0.0), (9.85, 5.0, 0.0))
    assert isinstance(route, rg.Route)
    assert math.hypot(route.poses[0][0] - 0.15, route.poses[0][1] - 5.0) < 1e-6
    assert math.hypot(route.poses[-1][0] - 9.85, route.poses[-1][1] - 5.0) < 1e-6
    check_inside(route, areas, CLR)


# ------------------------------------------------------------------ (g) params

def test_params_from_dict_ignores_unknown():
    p = rg.RouteParams.from_dict({'clearance_m': 0.5, 'unknown': 1})
    assert p.clearance_m == 0.5
    assert not hasattr(p, 'unknown')
    assert p.area_weight == 8.0
    assert rg.RouteParams.from_dict(None).clearance_m == 0.35


def test_start_on_drawn_obstacle_edge_escapes_by_a_link():
    """Sim 2026-10-09: the Nav2 fallback left the robot on the edge of the lawn's drawn
    triangle obstacle at (8.16, -2.73); the route must still start there (a blade-off link
    out of the obstacle), not report 'off the route graph' for every later transit."""
    areas, dock = _sim()
    g = rg.RouteGraph(areas, dock)
    route = g.plan((8.16, -2.73, 0.0), SIM_GOAL)
    assert isinstance(route, rg.Route), route
    assert route.kinds[0] == rg.LINK and route.legs[0]['kind'] == rg.LINK
    assert route.legs[0]['length'] < 1.0
    assert _names(route)[-2:] == ['Path 2', 'area 1']
    # a goal inside the obstacle stays unreachable (only the start may escape)
    assert isinstance(g.plan(SIM_GOAL, (8.5, -2.8, 0.0)), str)


# ------------------------------------------------- (h) overlapping / duplicated paths

def _sim_v3():
    areas, _ = core.load_areas_file(os.path.join(DATA, 'sim_garden_paths_v3.dat'))
    dock = core.load_dock_file(os.path.join(DATA, 'sim_garden_dock.yaml'))
    return areas, dock


V2_GOAL = (23.68, -9.01, 0.0)


def _check_end_to_end(areas, route, ends, p2_name='Path 2', p2_index=None):
    """Route from the dock: starts on a drawn path, leaves the first path network (the
    union of the other drawn paths' bands) within 0.4 m of one of its ``ends``, reaches
    Path 2 near its START, follows it into area 1, and stays inside the polygons."""
    assert isinstance(route, rg.Route), route
    assert route.legs[0]['kind'] == rg.PATH, route.summary()
    if p2_index is None:
        p2_index = next(i for i, a in enumerate(areas) if a.name == p2_name)
    p2 = [tuple(p) for p in areas[p2_index].channel]
    lines = [[tuple(p) for p in a.channel] for i, a in enumerate(areas)
             if a.channel and i != p2_index]
    # the exit: the last pose of the first run of path poses (the dock leg included)
    last = None
    for i, k in enumerate(route.kinds):
        if k != rg.PATH:
            break
        last = i
    assert last is not None
    q = route.poses[last]
    for p in route.poses[:last + 1]:             # that run is on the first path network
        if p[0] >= 0.5:
            assert min(_line_dist(p[:2], ln) for ln in lines) <= 0.45, p
    assert min(math.hypot(q[0] - e[0], q[1] - e[1]) for e in ends) <= 0.4, q
    # joins Path 2 at its START or where it crosses the lawn boundary (owner 2026-10-09:
    # "left at its END or at a JUNCTION with an AREA or another PATH")
    lawn = next(a for a in areas if a.name == 'lawn').polygon
    E = rg._ring_edges(lawn)
    joins = [p2[0]] + [(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
                       for a, b in zip(p2, p2[1:]) for t in rg._seg_intersections(a, b, E)]
    j = next(i for i in range(last + 1, len(route.poses))
             if _line_dist(route.poses[i][:2], p2) <= 0.05)
    q2 = route.poses[j]
    assert min(math.hypot(q2[0] - p[0], q2[1] - p[1]) for p in joins) <= 0.6, (q2, joins)
    assert route.legs[-1]['kind'] == rg.AREA and route.legs[-1]['area'] == 1, route.summary()
    assert route.legs[-2]['kind'] == rg.PATH and route.legs[-2]['area'] == p2_index
    check_inside(route, areas, CLR, _dock_polys_of(areas))


def test_sim_v3_dock_path_on_top_of_path1():
    """Owner GUI sim 2026-10-09: "Dock path" drawn almost on top of Path 1. The route
    starts on one of them, rides to the network's end, Path 2 from its start."""
    areas, dock = _sim_v3()
    assert {a.name for a in areas} >= {'Path 1', 'Dock path', 'Path 2'}
    g = rg.RouteGraph(areas, dock)
    route = g.plan((0.82, 0.0, 0.0), V2_GOAL)
    _check_end_to_end(areas, route, V2_ENDS)
    assert _names(route)[0] in ('Path 1', 'Dock path')


def _sim_v4():
    areas, _ = core.load_areas_file(os.path.join(DATA, 'sim_garden_paths_v4.dat'))
    dock = core.load_dock_file(os.path.join(DATA, 'sim_garden_dock.yaml'))
    return areas, dock


@pytest.mark.parametrize('start', [(0.82, 0.0, 0.0), (0.0, 0.0, 0.0)])
def test_sim_v4_end_to_end_from_the_dock(start):
    """Owner's map 2026-10-09 (both paths named "Path 2"; the first starts at the dock):
    dock -> along the first path to its END (9.18, -0.29) -> lawn hop -> second path's
    START (5.37, -4.85) -> along it -> area 1. Also from the dock pose itself."""
    areas, dock = _sim_v4()
    p_first = next(i for i, a in enumerate(areas) if a.channel and a.channel[0][0] < 1.0)
    p_second = next(i for i, a in enumerate(areas) if a.channel and i != p_first)
    route = rg.RouteGraph(areas, dock).plan(start, V2_GOAL)
    _check_end_to_end(areas, route, [(9.18145, -0.291765)], p2_index=p_second)
    first = [lg for lg in route.legs if lg['kind'] == rg.PATH and lg['area'] != -1][0]
    assert first['area'] == p_first


def test_start_on_band_in_the_same_area_may_hop_directly():
    """In-area mowing transit that begins on a path band (not at the dock): the direct
    lawn leg stays allowed (no ride to the path's far end and back)."""
    areas, dock = _sim()
    route = rg.RouteGraph(areas, dock).plan((5.0, -0.2, 0.0), (5.0, -3.0, 0.0))
    assert isinstance(route, rg.Route) and route.length_m < 3.5, route


def test_goal_on_path_band_is_reached_along_the_path():
    areas, dock = _sim_v2()
    route = rg.RouteGraph(areas, dock).plan((5.0, -4.0, 0.0), (20.5, 3.0, 0.0))
    assert isinstance(route, str)              # not in any polygon: off graph
    route = rg.RouteGraph(areas, dock).plan((23.68, -9.01, 0.0), (6.0, -0.3, 3.1))
    assert isinstance(route, rg.Route), route
    assert route.legs[-1]['kind'] == rg.PATH, route.summary()


def _v2_base():
    areas, dock = _sim_v2()
    return [a for a in areas if a.name in ('lawn', '', 'Path 2')], dock


def _path(name, line, w=0.7):
    return core.Area(name, core.buffer_polyline(line, w / 2.0), True, [], list(line), w)


SYNTH = {   # case: (path A, path B, the network ends the route may leave from)
    'identical': ([(0.8, 0.0), (10.4, -0.3)], [(0.8, 0.0), (10.4, -0.3)], [(10.4, -0.3)]),
    'partial_overlap': ([(0.8, 0.0), (6.0, -0.15)], [(3.0, -0.08), (10.4, -0.3)],
                        [(10.4, -0.3)]),
    'end_inside_band': ([(0.8, 0.0), (3.0, 0.0)], [(2.9, 0.15), (10.4, -0.3)],
                        [(10.4, -0.3)]),
    'shallow_crossing': ([(0.8, 0.0), (10.4, -0.6)], [(0.8, -0.5), (10.4, 0.3)],
                         [(10.4, -0.6), (10.4, 0.3), (0.8, -0.5)]),
    'near_duplicate': ([(0.8, 0.0), (4.244, -0.326), (10.458, -0.216)],
                       [(0.8, 0.0), (10.528, -0.407)], [(10.458, -0.216), (10.528, -0.407)]),
}


@pytest.mark.parametrize('case', sorted(SYNTH))
def test_overlapping_paths_never_break_the_route(case):
    base, dock = _v2_base()
    a, b, ends = SYNTH[case]
    areas = base + [_path('A', a), _path('B', b)]
    g = rg.RouteGraph(areas, dock)
    _check_end_to_end(areas, g.plan((0.82, 0.0, 0.0), V2_GOAL), ends)


def test_overlapping_paths_switch_along_the_overlap():
    """Two paths overlapping only in their middle: a route from A's start to B's end
    switches inside the overlap (no lawn leg; the lawn is not on the way)."""
    lawn = core.Area('lawn', [(-50, -50), (-40, -50), (-40, -40), (-50, -40)])
    a = _path('A', [(0.0, 0.0), (6.0, 0.0)])
    b = _path('B', [(3.0, 0.1), (9.0, 0.1), (9.0, 6.0)])
    g = rg.RouteGraph([lawn, a, b], None)
    route = g.plan((0.0, 0.0, 0.0), (9.0, 6.0, 1.57))
    assert isinstance(route, rg.Route), route
    assert all(lg['kind'] == rg.PATH for lg in route.legs)
    assert route.length_m < 15.5                     # 3 + ~6 + 6, not back via A's end


def test_overlap_fuzz_routes_always_found():
    """Random second paths over / along / across Path 1's position never break the dock ->
    area 1 route (a seed-fixed fuzz of what the owner may draw)."""
    import random
    rnd = random.Random(7)
    base, dock = _v2_base()
    p1 = [(0.8, 0.0), (4.244, -0.3261), (10.4582, -0.2161)]
    for _ in range(40):
        x0 = 0.8 + rnd.choice([0.0, 0.0, rnd.uniform(-0.3, 3.0)])
        pts = [(x0, rnd.uniform(-0.3, 0.3)), (rnd.uniform(4.0, 11.5), rnd.uniform(-0.6, 0.3))]
        if rnd.random() < 0.3:
            pts.insert(1, ((pts[0][0] + pts[1][0]) / 2, rnd.uniform(-0.5, 0.2)))
        if rnd.random() < 0.2:
            pts = pts[::-1]
        areas = base + [_path('Path 1', p1), _path('Dock path', pts)]
        route = rg.RouteGraph(areas, dock).plan((0.82, 0.0, 0.0), V2_GOAL)
        assert isinstance(route, rg.Route), (pts, route)
        assert route.legs[0]['kind'] == rg.PATH, (pts, route.summary())


def test_path_through_an_area_is_left_at_the_boundary_crossing():
    """Owner 2026-10-09: a path is left at its end or at a junction with an AREA (where
    its centreline crosses the area boundary) or another path; never mid-way inside."""
    lawn = core.Area('lawn', [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)])
    pth = _path('through', [(-5.0, 5.0), (15.0, 5.0)])
    yard = core.Area('yard', [(-8.0, 3.0), (-3.0, 3.0), (-3.0, 7.0), (-8.0, 7.0)])
    g = rg.RouteGraph([lawn, yard, pth], None)
    route = g.plan((-4.5, 5.0, 0.0), (5.0, 1.0, 0.0))   # yard start, on the path band
    assert isinstance(route, rg.Route), route
    assert route.legs[0]['kind'] == rg.PATH
    i = max(k for k, kd in enumerate(route.kinds) if kd == rg.PATH)
    exit_x = route.poses[i][0]
    assert abs(exit_x - 0.0) <= 0.5, route.poses[i]          # left at the crossing x = 0
    assert route.legs[-1]['kind'] == rg.AREA and route.legs[-1]['area'] == 0


# ------------------------------------------------------- (i) transit variation

def _modes(areas, mode):
    return {i: mode for i in range(len(areas))}


def _inside_all(route, areas):
    xs = np.array([p[0] for p in route.poses])
    ys = np.array([p[1] for p in route.poses])
    ok = np.zeros(len(xs), dtype=bool)
    for a in areas:
        ok |= core.points_in_polygon(xs, ys, a.polygon)
    return ok


@pytest.mark.parametrize('mode', ['none', 'lanes', 'perimeter', 'mixed'])
def test_variation_is_deterministic_and_inside(mode):
    areas, dock = _sim_v4()
    g = rg.RouteGraph(areas, dock)
    base = g.plan((0.82, 0.0, 0.0), V2_GOAL)
    seen = set()
    for k in range(6):
        r1 = g.plan((0.82, 0.0, 0.0), V2_GOAL, k, _modes(areas, mode))
        r2 = rg.RouteGraph(areas, dock).plan((0.82, 0.0, 0.0), V2_GOAL, k, _modes(areas, mode))
        assert r1.poses == r2.poses and r1.variant == r2.variant      # deterministic per run
        ok = _inside_all(r1, areas)
        ok[:2] = True                                                  # dock approach start
        assert ok.all()
        assert r1.length_m <= 2.0 * base.length_m
        seen.add(round(r1.length_m, 3))
        if k == 0 or mode == 'none':
            assert r1.poses == base.poses and r1.variant == ''
    if mode != 'none':
        assert len(seen) >= 2                                          # the line does vary


def test_lane_offsets_clipped_to_the_band():
    """A 2 m wide drawn path: lanes of up to 1.0 - 0.29 - 0.03 = 0.68 m; every pose keeps
    the robot's half width inside the band. A 0.7 m band (0.03 m) has no lanes."""
    lawn = core.Area('lawn', [(-1, -6), (19, -6), (19, 6), (-1, 6)])
    wide = _path('wide', [(0.0, 0.0), (15.0, 0.0)], w=2.0)
    g = rg.RouteGraph([lawn, wide], None)
    offs = []
    for k in range(5):
        r = g.plan((0.0, 0.0, 0.0), (15.0, 0.0, 0.0), k, {0: 'lanes', 1: 'lanes'})
        assert isinstance(r, rg.Route)
        ys = [abs(p[1]) for p in r.poses]
        assert max(ys) <= 1.0 - 0.29 + 1e-6
        offs.append(round(max(p[1] for p in r.poses) + min(p[1] for p in r.poses), 2))
    assert len(set(offs)) >= 3                                         # 0, +a, -a ...
    narrow = _path('narrow', [(0.0, 0.0), (15.0, 0.0)], w=0.7)
    g = rg.RouteGraph([lawn, narrow], None)
    r = g.plan((0.0, 0.0, 0.0), (15.0, 0.0, 0.0), 1, {0: 'lanes', 1: 'lanes'})
    assert max(abs(p[1]) for p in r.poses) < 1e-6


def test_perimeter_variant_bounded_and_both_directions():
    """Square lawn, hop along one side: cw and ccw ring routes exist; only those within
    perimeter_max_factor x the direct leg are used; the variants cycle per run."""
    lawn = core.Area('lawn', [(0, 0), (10, 0), (10, 10), (0, 10)])
    g = rg.RouteGraph([lawn], None)
    direct = g.plan((2.0, 1.5, 0.0), (8.0, 1.5, 0.0))
    got = []
    for k in range(3):
        r = g.plan((2.0, 1.5, 0.0), (8.0, 1.5, 0.0), k, {0: 'perimeter'})
        assert r.length_m <= 2.0 * direct.length_m + 1e-6
        assert _inside_all(r, [lawn]).all()
        got.append(r.variant)
    assert got[0] == '' and 'perimeter' in got[1]
    reg = g.regions[0]
    cw = g._perimeter([(2.0, 1.5), (8.0, 1.5)], reg, False)
    ccw = g._perimeter([(2.0, 1.5), (8.0, 1.5)], reg, True)
    assert cw and ccw
    assert rg._polyline_length(ccw) < rg._polyline_length(cw)          # ccw: along y = 0.4
