# Copyright 2026 michael
# SPDX-License-Identifier: Apache-2.0
"""Unit tests for mower_coverage_bridge.splitter (pure Python, no ROS)."""

import math

import pytest

from mower_coverage_bridge import splitter as sp
from mower_coverage_bridge.splitter import SEGMENT_RING, SEGMENT_SWATH

OP = 0.18  # planner default swath spacing


# --------------------------------------------------------------------------
# Fixtures that reproduce mower_coverage_node's pose layout
# --------------------------------------------------------------------------

def ring(x0, y0, x1, y1):
    """Closed rectangle loop starting mid-longest-edge, first == last."""
    mid = ((x0 + x1) / 2.0, y0)
    return [mid, (x1, y0), (x1, y1), (x0, y1), (x0, y0), mid]


def serpentine(x0, x1, y0, n, dy=OP):
    """n horizontal 2-pose swaths in boustrophedon order."""
    out = []
    for i in range(n):
        y = y0 + i * dy
        out += [(x0, y), (x1, y)] if i % 2 == 0 else [(x1, y), (x0, y)]
    return out


def rect_plan():
    """10 x 6 m field: 2 headland rings + 5 swaths, as the node lays it out."""
    rings = ring(0.09, 0.09, 9.91, 5.91) + ring(0.27, 0.27, 9.73, 5.73)
    # First swath starts right next to where the inner ring closed.
    swaths = serpentine(5.0, 9.6, 0.45, 5)
    return rings + swaths


def densified_serpentine(x0, x1, y0, n, step=0.1, dy=OP):
    out = []
    for i in range(n):
        y = y0 + i * dy
        a, b = ((x0, y), (x1, y)) if i % 2 == 0 else ((x1, y), (x0, y))
        out += sp.densify([a, b], step)
    return out


# --------------------------------------------------------------------------
# Segmentation
# --------------------------------------------------------------------------

def test_structural_rect_one_segment_per_ring_and_swath():
    segs, mode = sp.split_segments(rect_plan(), ring_count=2, swath_count=5)
    assert mode == sp.MODE_STRUCTURAL
    assert [s.kind for s in segs] == [SEGMENT_RING] * 2 + [SEGMENT_SWATH] * 5
    assert all(len(s.points) == 6 for s in segs[:2])
    assert all(len(s.points) == 2 for s in segs[2:])
    assert sp.is_closed(segs[0].points) and sp.is_closed(segs[1].points)


def test_rings_detected_by_closure_without_counts():
    rings, nxt = sp.detect_rings(rect_plan())
    assert len(rings) == 2
    assert nxt == 12


def test_counts_unknown_falls_back_to_sparse_pairing():
    segs, mode = sp.split_segments(rect_plan())
    assert mode == sp.MODE_STRUCTURAL
    assert sum(s.kind == SEGMENT_SWATH for s in segs) == 5


def test_count_mismatch_still_segments_sparse_plan():
    segs, _ = sp.split_segments(rect_plan(), ring_count=2, swath_count=7)
    assert [s.kind for s in segs].count(SEGMENT_SWATH) == 5
    assert [s.kind for s in segs].count(SEGMENT_RING) == 2


def test_plain_distance_rule_would_be_wrong_on_sparse_paths():
    # Every swath EDGE is longer than 0.6 m, so the planner path cannot be
    # split by pose spacing alone: the structural rule must win.
    pts = serpentine(0.0, 5.0, 0.0, 4)
    segs, mode = sp.split_segments(pts, ring_count=0, swath_count=4)
    assert mode == sp.MODE_STRUCTURAL
    assert len(segs) == 4
    assert all(math.isclose(s.length, 5.0) for s in segs)


def test_heuristic_densified_serpentine_one_segment_per_swath():
    pts = densified_serpentine(0.0, 3.0, 0.0, 4)
    segs, mode = sp.split_segments(pts)
    assert mode == sp.MODE_HEURISTIC
    assert len(segs) == 4
    assert all(s.kind == SEGMENT_SWATH for s in segs)
    assert all(math.isclose(s.length, 3.0, abs_tol=1e-9) for s in segs)


def test_heuristic_splits_on_large_gap():
    a = sp.densify([(0.0, 0.0), (2.0, 0.0)], 0.1)
    b = sp.densify([(3.0, 0.0), (5.0, 0.0)], 0.1)  # 1 m gap on the same line
    segs = sp.split_heuristic(a + b)
    assert len(segs) == 2
    assert segs[0][-1] == (2.0, 0.0) and segs[1][0] == (3.0, 0.0)


def test_heuristic_splits_on_in_place_reversal():
    a = sp.densify([(0.0, 0.0), (2.0, 0.0)], 0.1)
    b = sp.densify([(2.0, 0.0), (1.0, 0.0)], 0.1)[1:]  # drive straight back
    segs = sp.split_heuristic(a + b, turn_split_deg=150.0)
    assert len(segs) == 2


def test_heuristic_keeps_right_angle_corners():
    pts = sp.densify([(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)], 0.1)
    assert len(sp.split_heuristic(pts)) == 1


def test_duplicate_poses_are_dropped():
    pts = [(0.0, 0.0), (0.0, 0.0), (4.0, 0.0), (4.0, OP), (4.0, OP), (0.0, OP)]
    segs, _ = sp.split_segments(pts, ring_count=0, swath_count=2)
    assert len(segs) == 2


def test_empty_and_single_pose():
    assert sp.split_segments([])[0] == []
    assert sp.split_segments([(1.0, 1.0)])[0] == []
    assert sp.plan([]).subpaths == []


# --------------------------------------------------------------------------
# Joining into drivable sub-paths
# --------------------------------------------------------------------------

def test_rect_plan_is_one_drivable_subpath():
    res = sp.plan(rect_plan(), 2, 5)
    assert len(res.subpaths) == 1
    assert res.subpaths[0].segment_indices == list(range(7))
    # Sub-path = all segments + straight connectors between them.
    connectors = sum(sp.dist(a.points[-1], b.points[0])
                     for a, b in zip(res.segments[:-1], res.segments[1:]))
    total = sum(s.length for s in res.segments) + connectors
    assert math.isclose(res.subpaths[0].length, total, rel_tol=1e-9)


@pytest.mark.parametrize('gap,expected', [(0.59, 1), (0.6, 1), (0.61, 2), (5.0, 2)])
def test_join_threshold_is_inclusive_at_transit_gap(gap, expected):
    segs = [sp.Segment(SEGMENT_SWATH, [(0.0, 0.0), (3.0, 0.0)]),
            sp.Segment(SEGMENT_SWATH, [(3.0, gap), (0.0, gap)])]
    assert len(sp.join_subpaths(segs, 0.6)) == expected


def test_l_shape_trimmed_swaths_become_separate_subpaths():
    # L-shaped field: lower arm x 0..10, y 0..3; upper arm x 0..4, y 3..8.
    # A sweep line at y=2 lies fully in the lower arm. Sweep lines in the
    # notch band were trimmed by the field into two pieces each - left arm
    # x 0.2..3.8 and the far end x 9.0..9.8 (a pond-shaped bite in between) -
    # and the planner orders pieces on the same line consecutively.
    swaths = [
        [(0.2, 2.6), (9.8, 2.6)],     # full-width swath
        [(9.8, 2.78), (9.0, 2.78)],   # piece 2 of the next line (right lobe)
        [(3.8, 2.78), (0.2, 2.78)],   # piece 1, same sweep line, 5.2 m gap
        [(0.2, 2.96), (3.8, 2.96)],   # next line, left arm only
        [(9.0, 2.96), (9.8, 2.96)],   # same sweep line, far piece again
    ]
    pts = [p for s in swaths for p in s]
    res = sp.plan(pts, ring_count=0, swath_count=5)
    assert res.mode == sp.MODE_STRUCTURAL
    assert len(res.segments) == 5
    assert [s.segment_indices for s in res.subpaths] == [[0, 1], [2, 3], [4]]
    # Pieces on the same sweep line never share a sub-path.
    for sub in res.subpaths:
        ys = [round(res.segments[i].points[0][1], 2) for i in sub.segment_indices]
        assert len(ys) == len(set(ys))
    gaps = sp.transit_gaps(res.subpaths)
    assert all(g > 0.6 for g in gaps)
    assert math.isclose(gaps[0], 5.2, abs_tol=1e-9)


def test_connector_crossing_a_hole_starts_new_subpath():
    segs = [sp.Segment(SEGMENT_SWATH, [(0.0, 0.0), (3.0, 0.0)]),
            sp.Segment(SEGMENT_SWATH, [(3.0, 0.5), (0.0, 0.5)])]
    hole = [(2.9, 0.2), (3.1, 0.2), (3.1, 0.3), (2.9, 0.3)]
    assert len(sp.join_subpaths(segs, 0.6)) == 1
    assert len(sp.join_subpaths(segs, 0.6, holes=[hole])) == 2


def test_connector_leaving_the_field_starts_new_subpath_but_touching_does_not():
    # Concave notch between two swath ends: the straight join leaves the field.
    boundary = [(0, 0), (10, 0), (10, 2), (9.8, 2), (9.8, 1.1), (9.7, 1.1),
                (9.7, 2), (0, 2)]
    across = [sp.Segment(SEGMENT_SWATH, [(9.5, 1.5), (9.65, 1.5)]),
              sp.Segment(SEGMENT_SWATH, [(9.9, 1.5), (9.95, 1.5)])]
    assert len(sp.join_subpaths(across, 0.6, fences=[boundary])) == 2
    # Swath ends lying exactly ON the boundary (headland_passes = -1).
    on_line = [sp.Segment(SEGMENT_SWATH, [(5.0, 0.0), (10.0, 0.0)]),
               sp.Segment(SEGMENT_SWATH, [(10.0, 0.18), (5.0, 0.18)])]
    assert len(sp.join_subpaths(on_line, 0.6, fences=[[(0, 0), (10, 0), (10, 2), (0, 2)]])) == 1


# --------------------------------------------------------------------------
# Output shaping and stats
# --------------------------------------------------------------------------

def test_densify_bounds_edge_length_and_keeps_vertices():
    pts = [(0.0, 0.0), (1.05, 0.0), (1.05, 0.05)]
    out = sp.densify(pts, 0.1)
    assert out[0] == pts[0] and out[-1] == pts[-1]
    assert (1.05, 0.0) in out
    assert max(sp.dist(a, b) for a, b in zip(out[:-1], out[1:])) <= 0.1 + 1e-12
    assert math.isclose(sp.path_length(out), sp.path_length(pts))
    assert sp.densify(pts, 0.0) == pts


def test_yaws_point_along_the_path():
    y = sp.yaws([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)])
    assert y[0] == pytest.approx(0.0)
    assert y[1] == pytest.approx(math.pi / 2)
    assert y[2] == pytest.approx(math.pi / 2)


def test_swath_angle_folded_to_half_turn():
    segs = [sp.Segment(SEGMENT_RING, ring(0, 0, 2, 1)),
            sp.Segment(SEGMENT_SWATH, [(1.0, 1.0), (0.0, 0.0)])]  # heading -135 deg
    assert sp.swath_angle_deg(segs) == pytest.approx(45.0)
    assert sp.swath_angle_deg(segs[:1]) is None


def test_summary_mentions_counts():
    res = sp.plan(rect_plan(), 2, 5)
    text = sp.summary(res)
    assert '2 ring(s) + 5 swath(s) = 7 segment(s)' in text
    assert '1 drivable sub-path(s)' in text
    assert res.ring_count == 2 and res.swath_count == 5


# --------------------------------------------------------------------------
# Regression on real mower_coverage_node output (captured 2026-10-06,
# Fields2Cover 2.1, defaults: op_width 0.18, 2 headland rings)
# --------------------------------------------------------------------------

def _load(name):
    import json
    import os
    with open(os.path.join(os.path.dirname(__file__), 'data', name)) as f:
        return [tuple(p) for p in json.load(f)]


def test_real_rect_10x6_auto():
    res = sp.plan(_load('rect_10x6_auto_planner_path.json'), ring_count=2, swath_count=30,
                  fences=[[(0, 0), (10, 0), (10, 6), (0, 6)]])
    assert res.mode == sp.MODE_STRUCTURAL
    assert (res.ring_count, res.swath_count, len(res.segments)) == (2, 30, 32)
    # Rings joined (0.18 m apart); swaths joined by their U-turns; one
    # blade-off transit from the inner ring's end to the first swath.
    assert [len(s.segment_indices) for s in res.subpaths] == [2, 30]


def test_real_l_shape_swaths_trimmed_into_pieces():
    # L = (0,0) (10,0) (10,3) (4,3) (4,8) (0,8), mow_angle_deg 135: sweep
    # lines cross the notch, so the planner emits two pieces per line.
    boundary = [(0, 0), (10, 0), (10, 3), (4, 3), (4, 8), (0, 8)]
    res = sp.plan(_load('l_shape_angle135_planner_path.json'), ring_count=2, swath_count=66,
                  fences=[boundary])
    assert res.mode == sp.MODE_STRUCTURAL
    assert len(res.segments) == 68
    assert len(res.subpaths) > 2
    assert all(g > 0.6 for g in sp.transit_gaps(res.subpaths))
    for sub in res.subpaths:
        idx = sub.segment_indices
        for i, j in zip(idx[:-1], idx[1:]):
            assert sp.dist(res.segments[i].points[-1], res.segments[j].points[0]) <= 0.6
