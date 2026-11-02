# SPDX-License-Identifier: GPL-3.0-or-later
"""Boundary lines (virtual walls, 2026-11-02) in the mission: the hard boundary guard trips on
the footprint touching the TRUE line (reason names it) and retraces until the body is clear;
coverage goals get each line overlapping the area as a thin hole. Blade lines (2026-11-03):
the blade disc, not the body, trips, and the hole is the disc-sized band."""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from mower_mission import mission_fsm as f   # noqa: E402
from mower_mission import walls              # noqa: E402
from test_hard_boundary import feed, run_retrace, transit_h  # noqa: E402
from test_mission_fsm import Harness, line, plan, square, start_until_planning  # noqa: E402

WALL_X = 1.75     # the robot starts at (1.0, 0) facing east: the nose (+0.60) reaches it at x 1.15


def wall(x=WALL_X, lid='bl-1', name='drop-off', width=0.0, mode='body'):
    return walls.BoundaryLine(lid, [(x, -2.0), (x, 2.0)], name, width, mode)


def drive_east(h, x_to, step=0.02):
    x = h.fsm.inputs.pose[0]
    while x < x_to - 1e-9 and h.fsm._hb is None:
        x = round(min(x_to, x + step), 6)
        feed(h, (x, 0.0, 0.0), out=0.0)
        h.tick()


def test_body_touching_a_line_trips_names_it_and_retraces_until_clear():
    h = transit_h()
    h.fsm.inputs.boundary_lines = [wall()]
    m = h.mark()
    drive_east(h, 1.5)
    x = h.fsm.inputs.pose[0]
    assert h.fsm._hb is not None and x == 1.16                  # nose 0.01 past the line
    logs = [e.text for e in h.since(m, f.Log)]
    assert any('BOUNDARY OUT: body crosses boundary line bl-1 "drop-off" by 0.01 m' in t
               for t in logs), logs
    assert h.since(m, f.CancelActions) and h.since(m, f.BladeOff)
    assert [e.kind for e in h.since(m, f.RecordIncident)][:1] == ['boundary']
    assert h.fsm._hb['line'] == 'bl-1' and h.fsm._hb['mode'] is None
    run_retrace(h)
    assert h.fsm._hb is None
    nose = h.fsm.inputs.pose[0] + 0.60
    assert nose <= WALL_X - 0.02 + 1e-6                         # clear by hard_boundary_body_clear_m
    assert h.fsm.mission.transit_fails == 1                     # a transit abort, retried


def test_width_moves_the_trip_earlier():
    h = transit_h()
    h.fsm.inputs.boundary_lines = [wall(width=0.4)]
    drive_east(h, 1.5)
    # band edge at 1.55: the nose (1.62 on the first guard tick) is already in it
    assert h.fsm._hb is not None and h.fsm.inputs.pose[0] == 1.02


def test_line_beside_the_path_and_disabled_lines_do_not_trip():
    h = transit_h()
    h.fsm.inputs.boundary_lines = [walls.BoundaryLine('side', [(0.0, 0.5), (3.0, 0.5)])]
    drive_east(h, 1.5)                                       # body side at y 0.235: clear
    assert h.fsm._hb is None
    h = transit_h(boundary_lines_enabled=False)
    h.fsm.inputs.boundary_lines = [wall()]
    drive_east(h, 1.5)
    assert h.fsm._hb is None


def test_line_deleted_during_the_retrace_counts_as_clear():
    h = transit_h()
    h.fsm.inputs.boundary_lines = [wall()]
    drive_east(h, 1.5)
    assert h.fsm._hb is not None
    h.fsm.inputs.boundary_lines = []
    run_retrace(h, max_ticks=40)
    assert h.fsm._hb is None


def test_coverage_goal_gets_the_line_as_a_hole():
    h = Harness()
    h.areas = [square(0, 0, 5)]
    h.fsm.inputs.boundary_lines = [walls.BoundaryLine('bed', [(2.5, -1.0), (2.5, 3.5)]),
                                   walls.BoundaryLine('far', [(20.0, 0.0), (21.0, 0.0)])]
    start_until_planning(h)
    holes = h.goal(f.ACT_PLAN)['obstacles']
    assert len(holes) == 1                                   # the far line is not in this area
    xs = [p[0] for p in holes[0]]
    assert math.isclose(min(xs), 2.215) and math.isclose(max(xs), 2.785)   # keepout 0.57
    h2 = Harness(boundary_line_keepout_m=0.6)
    h2.areas = [square(0, 0, 5)]
    h2.fsm.inputs.boundary_lines = [walls.BoundaryLine('bed', [(2.5, -1.0), (2.5, 3.5)],
                                                       '', 0.2)]
    start_until_planning(h2)
    xs = [p[0] for p in h2.goal(f.ACT_PLAN)['obstacles'][0]]
    assert math.isclose(max(xs) - min(xs), 0.8)              # width_m + keepout


def test_lines_off_means_no_holes():
    h = Harness(boundary_lines_enabled=False)
    h.areas = [square(0, 0, 5)]
    h.fsm.inputs.boundary_lines = [walls.BoundaryLine('bed', [(2.5, -1.0), (2.5, 3.5)])]
    start_until_planning(h)
    assert h.goal(f.ACT_PLAN)['obstacles'] == []


def test_area_holes_and_parse():
    assert walls.parse('not json') is None
    lines = walls.parse('{"version": 1, "lines": [{"id": "x", "points": [[0, 0], [1, 0]]}]}')
    assert [ln.id for ln in lines] == ['x']
    assert walls.area_holes(lines, [(5, 5), (6, 5), (6, 6)], 0.3) == []
    assert len(walls.area_holes(lines, [(-1, -1), (2, -1), (2, 1), (-1, 1)], 0.3)) == 1


def test_blade_line_trips_when_the_disc_reaches_it_and_retraces_until_the_disc_is_clear():
    h = transit_h()
    h.fsm.inputs.boundary_lines = [wall(mode='blade', name='flower bed')]
    m = h.mark()
    drive_east(h, 1.6)
    x = h.fsm.inputs.pose[0]
    # disc front edge = x + 0.23 + 0.11: past 1.75 first at x 1.42 (the nose was 0.27 across)
    assert h.fsm._hb is not None and x == 1.42
    logs = [e.text for e in h.since(m, f.Log)]
    assert any('BOUNDARY OUT: blade crosses boundary line bl-1 "flower bed" by 0.01 m' in t
               for t in logs), logs
    assert h.since(m, f.BladeOff)
    assert h.fsm._hb['line'] == 'bl-1'
    run_retrace(h)
    assert h.fsm._hb is None
    assert h.fsm.inputs.pose[0] + 0.34 <= WALL_X - 0.02 + 1e-6    # disc clear by body_clear_m


def test_blade_line_beside_the_path_lets_the_body_overhang():
    # line at y 0.15: the body (half width 0.235) overhangs it, the disc (0.11) does not
    h = transit_h()
    side = [(0.0, 0.15), (3.0, 0.15)]
    h.fsm.inputs.boundary_lines = [walls.BoundaryLine('bed', side, '', 0.0, 'blade')]
    drive_east(h, 1.5)
    assert h.fsm._hb is None
    h = transit_h()
    h.fsm.inputs.boundary_lines = [walls.BoundaryLine('pond', side, '', 0.0, 'body')]
    drive_east(h, 1.5)
    assert h.fsm._hb is not None                             # a body line trips at once


def test_blade_radius_from_map_server_moves_the_trip():
    h = transit_h()
    h.fsm.inputs.boundary_lines = [wall(mode='blade')]
    h.fsm.inputs.boundary_blade_radius = 0.165               # 330 mm disc: edge 0.395 ahead
    drive_east(h, 1.6)
    assert h.fsm._hb is not None and h.fsm.inputs.pose[0] == 1.36


def test_coverage_goal_blade_hole_is_disc_sized():
    def hole_width(lines, radius=None, **params):
        h = Harness(**params)
        h.areas = [square(0, 0, 5)]
        h.fsm.inputs.boundary_lines = lines
        h.fsm.inputs.boundary_blade_radius = radius
        start_until_planning(h)
        return sorted(round(max(p[0] for p in o) - min(p[0] for p in o), 4)
                      for o in h.goal(f.ACT_PLAN)['obstacles'])
    bed = walls.BoundaryLine('bed', [(2.5, -1.0), (2.5, 3.5)], '', 0.0, 'blade')
    pond = walls.BoundaryLine('pond', [(1.0, -1.0), (1.0, 3.5)])
    assert hole_width([bed, pond]) == [0.32, 0.57]           # 2 x (0.11 + 0.05), body 0.57
    assert hole_width([bed], 0.165) == [0.43]                # map_server's 330 mm disc
    assert hole_width([bed], 0.165, boundary_line_blade_keepout_m=0.5) == [0.5]


def test_parse_reads_the_mode_and_the_blade_radius():
    text = ('{"version": 2, "blade_radius_m": 0.165, "lines": ['
            '{"id": "a", "points": [[0, 0], [1, 0]], "mode": "blade"},'
            '{"id": "b", "points": [[0, 1], [1, 1]]}]}')
    assert [(ln.id, ln.mode) for ln in walls.parse(text)] == [('a', 'blade'), ('b', 'body')]
    assert walls.blade_radius_of(text) == 0.165
    assert walls.blade_radius_of('{"version": 1, "lines": []}') is None
    assert walls.blade_keepout() == walls.blade_keepout(None, -1.0) == 0.32
    assert walls.describe(0.02, walls.parse(text)[0]) == 'blade crosses boundary line a by 0.02 m'


# keep the imported helpers referenced (flake8)
_ = (line, plan)
