"""Douglas-Peucker, transit decision and resume-cursor format (no ROS)."""

import math

from mower_mission import geometry as geo
from mower_mission.resume import HEADER, AreaCursor, ResumeCursor, absolute_to_local


def test_dp_straight_line_collapses_to_endpoints():
    pts = [(x * 0.1, 0.0) for x in range(50)]
    assert geo.douglas_peucker(pts, 0.05) == [pts[0], pts[-1]]


def test_dp_keeps_corner_and_drops_noise():
    pts = [(x * 0.1, 0.01 * ((-1) ** x)) for x in range(21)]          # noisy edge to (2, 0)
    pts += [(2.0, y * 0.1) for y in range(1, 21)]                     # edge up to (2, 2)
    out = geo.douglas_peucker(pts, 0.05)
    assert out[0] == pts[0] and out[-1] == pts[-1]
    assert len(out) == 3
    assert geo.dist(out[1], (2.0, 0.0)) < 0.11


def test_dp_short_inputs_unchanged():
    assert geo.douglas_peucker([(0, 0), (1, 1)], 0.05) == [(0, 0), (1, 1)]


def test_dp_long_track_no_recursion_error():
    pts = [(math.cos(t / 1000.0), math.sin(t / 1000.0)) for t in range(20000)]
    out = geo.douglas_peucker(pts, 0.001)
    assert 3 < len(out) < 2000


def test_simplify_ring_square_closes_without_duplicate():
    # Drive a 4 x 3 m rectangle, sampled every 0.1 m, ending near the start.
    track = []
    for x in range(40):
        track.append((x * 0.1, 0.0))
    for y in range(30):
        track.append((4.0, y * 0.1))
    for x in range(40, 0, -1):
        track.append((x * 0.1, 3.0))
    for y in range(30, 0, -1):
        track.append((0.0, y * 0.1))
    track.append((0.02, 0.01))                    # back at the start
    ring = geo.simplify_ring(track, 0.05)
    assert len(ring) == 4
    assert geo.dist(ring[0], ring[-1]) > 1.0      # implicitly closed, no repeat
    assert abs(geo.polygon_area(ring) - 12.0) < 0.2


def test_polygon_area_and_path_length():
    assert geo.polygon_area([(0, 0), (2, 0), (2, 2), (0, 2)]) == 4.0
    assert geo.path_length([(0, 0, 0), (3, 4, 0), (3, 5, 0)]) == 6.0


def test_transit_decision():
    assert geo.needs_transit(None, (0, 0), 0.6)            # unknown pose: safe path
    assert not geo.needs_transit((0, 0, 0), (0.5, 0.3), 0.6)
    assert geo.needs_transit((0, 0, 0), (0.61, 0.0), 0.6)


def test_nearest_index_respects_start():
    poses = [(0, 0, 0), (1, 0, 0), (2, 0, 0), (0.1, 0, 0)]
    assert geo.nearest_index(poses, (0, 0)) == 0
    assert geo.nearest_index(poses, (0, 0), start=1) == 3


def test_heading_of_fills_missing_yaw():
    poses = [(0, 0, 0.0), (0, 1, 0.0)]
    assert abs(geo.heading_of(poses, 0) - math.pi / 2) < 1e-9
    assert geo.heading_of([(0, 0, 1.0), (1, 0, 0)], 0) == 1.0


def test_fingerprint_changes_with_geometry():
    a = [[(0, 0, 0), (1, 0, 0)]]
    b = [[(0, 0, 0), (1, 0.01, 0)]]
    assert geo.plan_fingerprint(a) == geo.plan_fingerprint([[(0, 0, 5), (1, 0, 9)]])
    assert geo.plan_fingerprint(a) != geo.plan_fingerprint(b)
    assert geo.plan_fingerprint([]) != 0


def test_resume_round_trip():
    c = ResumeCursor()
    c.current_command = 1
    c.single_area_target = 2
    c.current_area = 2
    c.completed_areas = {0, 1}
    c.areas[2] = AreaCursor(120, 1234567890123, 57, {0, 1, 3})
    text = c.dumps()
    assert text.splitlines()[0] == HEADER
    back = ResumeCursor.loads(text)
    assert back.dumps() == text
    assert back.areas[2] == c.areas[2]
    assert (back.current_command, back.single_area_target, back.current_area,
            back.completed_areas) == (1, 2, 2, {0, 1})
    assert back.available


def test_resume_matches_upstream_layout():
    c = ResumeCursor()
    c.current_command = 1
    c.current_area = 0
    c.areas[0] = AreaCursor(10, 99, -1, {2})
    lines = c.dumps().splitlines()
    assert lines == ['mowgli_coverage_resume v2', 'current_command 1', 'current_area 0',
                     'completed_areas', 'area 0 10 99 -1 completed 2']


def test_resume_tolerates_garbage_and_unknown_tags():
    assert ResumeCursor.loads('') is None
    assert ResumeCursor.loads('mowgli_coverage_resume v1\ncurrent_command 1') is None
    text = (HEADER + '\ncross_hatch_area 0 1 -1 0 0 -1 0 0\ncurrent_command 1\n'
            'area x y z\narea 3 10 5 4 completed 0 1\n')
    c = ResumeCursor.loads(text)
    assert c.current_command == 1 and c.areas[3] == AreaCursor(10, 5, 4, {0, 1})


def test_absolute_to_local():
    sps = [[1, 2, 3], [4, 5], [6, 7, 8, 9]]
    assert absolute_to_local(sps, 0) == (0, 0)
    assert absolute_to_local(sps, 3) == (1, 0)
    assert absolute_to_local(sps, 8) == (2, 3)
    assert absolute_to_local(sps, 9) is None


def test_config_yaml_matches_fsm_parameter_types():
    import os
    yaml = __import__('pytest').importorskip('yaml')
    from mower_mission.mission_fsm import Params
    path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        'config', 'mission.yaml')
    with open(path) as fh:
        d = yaml.safe_load(fh)['behavior_tree_node']['ros__parameters']
    p = Params()
    for name in p.__dataclass_fields__:
        if name == 'dock_pose':
            continue
        assert name in d, name
        assert type(d[name]).__name__ == type(getattr(p, name)).__name__, name  # noqa: E721


def test_chunk_end_avoids_ending_near_earlier_poses():
    # closed 4 x 2 m ring (0.1 m spacing) followed by a swath back past the start
    ring = []
    for x in range(40):
        ring.append((x * 0.1, 0.0, 0.0))
    for y in range(20):
        ring.append((4.0, y * 0.1, 0.0))
    for x in range(40, 0, -1):
        ring.append((x * 0.1, 2.0, 0.0))
    for y in range(20, -1, -1):
        ring.append((0.0, y * 0.1, 0.0))        # back at (0, 0)
    path = ring + [(0.3, 0.3 + 0.1 * k, 0.0) for k in range(10)]
    e = geo.chunk_end(path, 0, 100.0, 0.5)
    assert e < len(ring) - 5                     # cannot end at the ring start
    for j in range(0, e):
        if geo.path_length(path[j:e + 1]) >= 1.0:
            assert geo.dist(path[j], path[e]) > 0.5
    e2 = geo.chunk_end(path, e, 100.0, 0.5)      # the next chunk makes progress
    assert e2 > e
    assert geo.chunk_end(path, 0, 0.0, 0.5) == len(path) - 1   # disabled


def test_chunk_end_respects_max_length():
    path = [(0.1 * k, 0.0, 0.0) for k in range(300)]
    e = geo.chunk_end(path, 0, 10.0, 0.5)
    assert 99 <= e <= 101


def test_area_skip_rows_round_trip():
    from mower_mission.resume import ResumeCursor
    cur = ResumeCursor()
    cur.current_command = 1
    cur.area(2).skipped = [(6, 8), (30, 34)]
    text = cur.dumps()
    assert 'area_skip 2 6 8' in text and 'area_skip 2 30 34' in text
    back = ResumeCursor.loads(text)
    assert back.areas[2].skipped == [(6, 8), (30, 34)] and back.available


def test_inside_area_respects_holes_and_leave_margin():
    from mower_mission import geometry as g
    area = {'outer': [(0, 0), (4, 0), (4, 4), (0, 4)],
            'obstacles': [[(1, 1), (2, 1), (2, 2), (1, 2)]]}
    assert g.inside_area((3, 3), area)
    assert not g.inside_area((1.5, 1.5), area)
    assert not g.inside_area((4.2, 2), area)
    assert g.inside_area((4.2, 2), area, leave_m=0.3)


# --- recording closure (overshoot past the start) ---------------------------

# Area 1 as stored on the mower 2026-10-06 (recorded drive, 175 samples ->
# 16 vertices): the drive passed its start (vertex 0) and stopped ~0.6 m
# further on, leaving a 175 deg sliver spike where the ring closes.
_AREA1 = [(0.389398, 1.76668), (0.329541, 2.23991), (0.141112, 2.43469),
          (-0.869764, 2.28123), (-1.55917, 2.01274), (-2.7828, 1.01557),
          (-3.006, 0.657267), (-3.07431, 0.315402), (-2.90828, -0.904132),
          (-2.70292, -1.68169), (-2.48501, -1.75286), (-1.37076, -1.00271),
          (-0.969795, -0.462225), (-0.657529, 0.252227), (0.146422, 1.03497),
          (0.671379, 2.31698)]


def _densify(poly, step=0.05):
    out = []
    for a, b in zip(poly, poly[1:]):
        n = max(1, int(geo.dist(a, b) / step))
        out += [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(n)]
    out.append(poly[-1])
    return out


def _max_turn_deg(ring):
    worst = 0.0
    n = len(ring)
    for i in range(n):
        a, b, c = ring[i - 1], ring[i], ring[(i + 1) % n]
        h1 = math.atan2(b[1] - a[1], b[0] - a[0])
        h2 = math.atan2(c[1] - b[1], c[0] - b[0])
        worst = max(worst, abs(math.degrees((h2 - h1 + math.pi) % (2 * math.pi) - math.pi)))
    return worst


def test_simplify_ring_without_trim_reproduces_area1_spike():
    ring = geo.simplify_ring(_densify(_AREA1), 0.05)
    assert _max_turn_deg(ring) > 170.0


def test_simplify_ring_trims_overshoot_past_start():
    ring = geo.simplify_ring(_densify(_AREA1), 0.05, close_radius=0.5)
    assert _max_turn_deg(ring) < 120.0
    assert all(geo.dist(p, (0.671379, 2.31698)) > 0.3 for p in ring)
    assert abs(geo.polygon_area(ring) - geo.polygon_area(_AREA1[:-1])) < 0.2


def test_trim_overshoot_square():
    sq = [(0, 0), (2, 0), (2, 2), (0, 2), (0, 0), (0.5, 0)]   # drove 0.5 m past the start
    track = _densify(sq)
    ring = geo.simplify_ring(track, 0.05, close_radius=0.5)
    assert len(ring) == 4
    assert all(min(geo.dist(p, c) for c in [(0, 0), (2, 0), (2, 2), (0, 2)]) < 0.06 for p in ring)


def test_trim_overshoot_leaves_open_and_short_tracks_alone():
    u = _densify([(0, 0), (3, 0), (3, 3), (2, 3)])            # never returns near the start
    assert geo.trim_closure_overshoot(u, 0.5) == u
    sq = _densify([(0, 0), (2, 0), (2, 2), (0, 2), (0, 0.3)])  # stops short of the start
    assert geo.trim_closure_overshoot(sq, 0.5) == sq


def test_simplify_open_path_keeps_endpoints_no_closure():
    from mower_mission import geometry as g
    pts = [(x * 0.1, 0.0) for x in range(30)] + [(3.0, y * 0.1) for y in range(1, 20)]
    pts += [pts[-1]]                                    # duplicate dropped
    out = g.simplify_open_path(pts, 0.05)
    assert out == [(0.0, 0.0), (2.9, 0.0), (3.0, 0.1), (3.0, 1.9)] or (
        out[0] == (0.0, 0.0) and out[-1] == (3.0, 1.9) and len(out) <= 4)


def test_buffer_polyline_band():
    from mower_mission import geometry as g
    ring = g.buffer_polyline([(0, 0), (4, 0)], 0.35)
    assert ring == [(0, 0.35), (4, 0.35), (4, -0.35), (0, -0.35)]
    assert abs(g.polygon_area(ring) - 4 * 0.7) < 1e-9
    corner = g.buffer_polyline([(0, 0), (4, 0), (4, 3)], 0.35)
    assert len(corner) == 6
    assert abs(corner[1][0] - (4 - 0.35)) < 1e-9 and abs(corner[1][1] - 0.35) < 1e-9
    assert g.buffer_polyline([(0, 0)], 0.35) == []
