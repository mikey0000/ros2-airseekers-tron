# SPDX-License-Identifier: GPL-3.0-or-later
"""Boundary lines (virtual walls, 2026-11-02): wire format, areas.dat persistence, the body
footprint vs line hit test, the planner keepouts (masks, route graph)."""

import json
import math

import numpy as np
import pytest

from mower_map import areas as core
from mower_map import boundary_lines as bl
from mower_map import route_graph as rg

FRONT, REAR, HALF = 0.60, 0.10, 0.235


def wall(points, lid='bl-1', name='drop-off', width=0.0):
    return bl.BoundaryLine(lid, [tuple(map(float, p)) for p in points], name, width)


def depth(line, x, y, yaw):
    return bl.body_depth(line, bl.body_corners(x, y, yaw, FRONT, REAR, HALF))


# ------------------------------------------------------------------ wire format

def test_json_round_trip_assigns_unique_ids_and_validates():
    ok, msg, lines = bl.parse_json(json.dumps({'version': 1, 'lines': [
        {'id': 'a', 'name': 'bed\nedge', 'points': [[0, 0], [1, 0], [1, 0.005], [2, 1]],
         'width_m': 0.2},
        {'id': 'a', 'points': [[0, 1], [3, 1]]},
        {'points': [[5, 5], [6, 6]]}]}))
    assert ok, msg
    assert [ln.id for ln in lines] == ['a', 'line-1', 'line-2']
    assert lines[0].name == 'bed edge'
    assert lines[0].points == [(0.0, 0.0), (1.0, 0.0), (2.0, 1.0)]   # 5 mm vertex merged
    ok2, _m, again = bl.parse_json(bl.to_json(lines))
    assert ok2 and [(ln.id, ln.points, ln.width_m) for ln in again] == \
        [(ln.id, ln.points, ln.width_m) for ln in lines]


@pytest.mark.parametrize('payload, why', [
    ({'lines': [{'id': 'x', 'points': [[0, 0]]}]}, 'at least 2'),
    ({'lines': [{'id': 'x', 'points': [[0, 0], [1, 0]], 'width_m': 3.0}]}, 'width_m'),
    ({'lines': [{'id': 'x', 'points': [[0, 0], [1, 0]], 'width_m': -0.1}]}, 'width_m'),
    ({'version': 2, 'lines': []}, 'newer'),
    ({'lines': [{'id': 'x', 'points': [[0, 'a'], [1, 0]]}]}, 'bad points'),
])
def test_json_rejects_bad_sets(payload, why):
    ok, msg, lines = bl.parse_json(json.dumps(payload))
    assert not ok and why in msg and lines == []


def test_empty_payload_means_no_lines():
    assert bl.parse_json('') == (True, 'ok', [])
    assert bl.parse_json('{"version": 1, "lines": []}') == (True, 'ok', [])


# ------------------------------------------------------------------ areas.dat

def _area():
    return core.Area('lawn', [(0.0, 0.0), (10.0, 0.0), (10.0, 6.0), (0.0, 6.0)])


def test_areas_dat_keeps_lines_versioned_and_with_the_datum(tmp_path):
    lines = [wall([(5, -1), (5, 4.5)], name='flower bed', width=0.25),
             wall([(1, 1), (2, 2), (3, 1)], lid='bl-2', name='')]
    path = str(tmp_path / 'areas.dat')
    core.save_areas_file(path, [_area()], -36.85, 174.76, lines)
    text = open(path).read()
    assert 'boundary_lines_version: 1' in text and 'boundary_line_count: 2' in text
    areas, datum = core.load_areas_file(path)
    assert [a.name for a in areas] == ['lawn'] and datum == pytest.approx((-36.85, 174.76))
    got, version = core.load_boundary_lines_file(path)
    assert version == 1
    assert [(ln.id, ln.name, ln.width_m) for ln in got] == \
        [('bl-1', 'flower bed', 0.25), ('bl-2', '', 0.0)]
    assert got[1].points == [(1.0, 1.0), (2.0, 2.0), (3.0, 1.0)]


def test_areas_dat_without_lines_is_unchanged_upstream_format():
    assert core.format_areas_dat([_area()], 1.0, 2.0, []) == core.format_areas_dat([_area()], 1.0, 2.0)
    assert 'boundary' not in core.format_areas_dat([_area()])
    assert core.parse_boundary_lines_dat(core.format_areas_dat([_area()])) == ([], 0)


def test_areas_dat_skips_a_broken_line_but_keeps_the_others():
    text = core.format_areas_dat([_area()], boundary_lines=[wall([(0, 0), (1, 0)]),
                                                            wall([(0, 1), (1, 1)], lid='b')])
    text = text.replace('boundary_line_0_points: 0,0;1,0', 'boundary_line_0_points: 0,0')
    lines, _v = core.parse_boundary_lines_dat(text)
    assert [ln.id for ln in lines] == ['b']


def test_map_store_clear_keeps_the_lines():
    st = core.MapStore()
    st.add_area('lawn', _area().polygon)
    st.boundary_lines = [wall([(5, -1), (5, 4.5)])]
    st.clear()
    assert st.areas == [] and len(st.boundary_lines) == 1


# ------------------------------------------------------------------ body vs line

def test_body_clear_of_a_line_reports_the_gap():
    ln = wall([(2.0, -3.0), (2.0, 3.0)])
    # facing the line: nose at x = 0.6 + 1.0 = 1.6 -> 0.40 m short of x = 2
    assert depth(ln, 1.0, 0.0, 0.0) == pytest.approx(-0.40, abs=1e-9)
    # width 0.3: the band edge is 0.15 closer
    assert bl.body_depth(wall([(2.0, -3.0), (2.0, 3.0)], width=0.3),
                         bl.body_corners(1.0, 0.0, 0.0)) == pytest.approx(-0.25, abs=1e-9)


def test_body_crossing_a_line_trips_with_the_separating_depth():
    ln = wall([(2.0, -3.0), (2.0, 3.0)])
    assert depth(ln, 1.5, 0.0, 0.0) == pytest.approx(0.10, abs=1e-9)    # nose 0.10 across
    assert depth(ln, 1.9, 0.0, 0.0) == pytest.approx(0.20, abs=1e-9)    # rear 0.20 short
    hits = bl.body_hits([ln, wall([(9, 9), (9, 10)], lid='far')], (1.5, 0.0, 0.0),
                        FRONT, REAR, HALF)
    assert [(round(d, 3), h.id) for d, h in hits] == [(0.1, 'bl-1')]


def test_short_line_between_body_corners_is_still_a_crossing():
    # a 0.2 m line poking into the left side of the body: no corner or body point is
    # "beyond" it, but the footprint covers part of it
    ln = wall([(0.25, 0.40), (0.25, 0.10)])
    assert depth(ln, 0.0, 0.0, 0.0) > 0.1
    assert bl.body_hits([ln], (0.0, 0.0, 0.0), FRONT, REAR, HALF)


def test_driving_into_a_line_crosses_exactly_when_the_nose_reaches_it():
    ln = wall([(3.0, -1.0), (3.0, 1.0)])
    xs = np.arange(1.5, 3.0, 0.01)
    first = next(x for x in xs if bl.body_hits([ln], (x, 0.0, 0.0), FRONT, REAR, HALF))
    assert first == pytest.approx(2.40, abs=0.011)
    # yawed 45 deg: a front corner reaches it first
    c = bl.body_corners(2.0, 0.0, math.radians(45.0), FRONT, REAR, HALF)
    reach = max(p[0] for p in c)
    assert depth(ln, 2.0, 0.0, math.radians(45.0)) == pytest.approx(reach - 3.0, abs=1e-9)


def test_body_beside_the_end_of_a_line_is_clear():
    ln = wall([(0.0, 1.0), (0.0, 3.0)])           # ends 0.765 m left of the body's side
    assert depth(ln, 0.0, 0.0, 0.0) == pytest.approx(-math.hypot(0.0, 1.0 - HALF), abs=1e-9)
    assert bl.clearance([ln], (0.0, 0.0, 0.0)) == pytest.approx(1.0 - HALF, abs=1e-9)


# ------------------------------------------------------------------ planner keepouts

def test_line_keepout_goes_only_to_the_areas_it_touches():
    a = _area()
    b = core.Area('back', [(20.0, 0.0), (30.0, 0.0), (30.0, 6.0), (20.0, 6.0)])
    ln = wall([(5, -1), (5, 4.5)])
    out = core.with_boundary_lines([a, b], [ln], keepout_m=0.3)
    assert [o.name for o in out[0].obstacles] == ['boundary line bl-1']
    assert out[1].obstacles == [] and a.obstacles == []          # copies, store untouched
    k = out[0].obstacles[0].polygon
    xs = [p[0] for p in k]
    assert min(xs) == pytest.approx(4.85) and max(xs) == pytest.approx(5.15)
    # width_m widens the planner band too
    k2 = core.with_boundary_lines([a], [wall([(5, -1), (5, 4.5)], width=0.4)], 0.3)
    assert max(p[0] for p in k2[0].obstacles[0].polygon) == pytest.approx(5.35)


def test_keepout_mask_is_lethal_along_the_line():
    a = _area()
    areas = core.with_boundary_lines([a], [wall([(5, -1), (5, 4.5)])], 0.3)
    spec = core.grid_for_polygons([a.polygon], 0.05, 0.5)
    mask = core.build_keepout_mask(areas, spec)
    nav = core.build_nav_mask(areas, spec, 0.35, 0.0)
    for y in (0.5, 2.0, 4.0):
        r, c = spec.index_of(5.0, y)
        assert mask[r, c] == core.LETHAL and nav[r, c] == core.LETHAL
    r, c = spec.index_of(5.0, 5.5)                  # the gap above the line end stays free
    assert mask[r, c] == core.FREE


def _crosses(route, ln):
    pts = [(p[0], p[1]) for p in route.poses]
    for p, q in zip(pts[:-1], pts[1:]):
        for a, b in bl.segments(ln):
            if bl._segments_cross(p, q, a, b):
                return True
    return False


def test_route_graph_goes_round_a_line_not_across_it():
    a = _area()
    ln = wall([(5, -1), (5, 4.5)])                  # wall from the south edge, gap in the north
    plain = rg.RouteGraph([a]).plan((2.0, 1.0, 0.0), (8.0, 1.0, 0.0))
    assert not isinstance(plain, str) and _crosses(plain, ln)    # straight across without it
    g = rg.RouteGraph(core.with_boundary_lines([a], [ln], 0.3))
    route = g.plan((2.0, 1.0, 0.0), (8.0, 1.0, 0.0))
    assert not isinstance(route, str), route
    assert not _crosses(route, ln)
    assert max(p[1] for p in route.poses) > 4.5     # through the gap at the north end


def test_route_graph_has_no_route_through_a_full_wall():
    a = _area()
    g = rg.RouteGraph(core.with_boundary_lines([a], [wall([(5, -1), (5, 7)])], 0.3))
    route = g.plan((2.0, 1.0, 0.0), (8.0, 1.0, 0.0))
    assert isinstance(route, str) or not _crosses(route, wall([(5, -1), (5, 7)]))


def test_keepout_default_is_the_same_everywhere():
    """map_server PARAMS / yaml, mission Params / yaml and the library share one default."""
    import os
    import yaml
    root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    with open(os.path.join(root, 'mower_map', 'config', 'map_server.yaml')) as fh:
        ms = yaml.safe_load(fh)['map_server_node']['ros__parameters']
    with open(os.path.join(root, 'mower_mission', 'config', 'mission.yaml')) as fh:
        mi = yaml.safe_load(fh)['behavior_tree_node']['ros__parameters']
    assert bl.DEFAULT_KEEPOUT_M == pytest.approx(2 * (HALF + 0.05))
    assert ms['boundary_line_keepout_m'] == mi['boundary_line_keepout_m'] == bl.DEFAULT_KEEPOUT_M


def _min_pose_dist(route, ln):
    return min(bl.nearest_point(ln, p[0], p[1])[2] for p in route.poses)


def test_route_graph_never_uses_a_gap_narrower_than_the_body():
    """(c) a wall across the area leaves a gap at the south edge. 0.32 m (narrower than the
    0.47 m body): no route through it. 1.3 m: the route uses it, base_link >= body half
    width + 0.05 off the true line all the way."""
    a = _area()
    narrow = wall([(5, 0.6), (5, 7)])                # band ends 0.315 above the edge
    g = rg.RouteGraph(core.with_boundary_lines([a], [narrow]))
    route = g.plan((2.0, 3.0, 0.0), (8.0, 3.0, 0.0))
    assert isinstance(route, str) or (not _crosses(route, narrow) and
                                      _min_pose_dist(route, narrow) >= HALF + 0.05)
    wide = wall([(5, 1.6), (5, 7)])                  # band ends 1.315 above the edge
    route = rg.RouteGraph(core.with_boundary_lines([a], [wide])).plan((2.0, 3.0, 0.0),
                                                                       (8.0, 3.0, 0.0))
    assert not isinstance(route, str), route
    assert not _crosses(route, wide)
    assert _min_pose_dist(route, wide) >= HALF + 0.05
    assert min(p[1] for p in route.poses) < 1.3      # it did go through the gap
