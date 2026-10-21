"""Step 3 round 2 (2026-10-21): body-aware boundary-out check with the stock recovery
(reverse 0.3 m straight, resume), the outside_m jump filter, Spin drift, repeated bumper
hits."""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from mower_mission import body_boundary as bb   # noqa: E402
from mower_mission import mission_fsm as f      # noqa: E402
from test_mission_fsm import Harness, line, plan, square, start_until_planning  # noqa: E402

SQ = [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0), (0.0, 5.0)]
SOFT = bb.limits_for('soft')
HARD = bb.limits_for('hard')


# ---------------------------------------------------------------- pure geometry

def test_soft_lap0_on_the_line_does_not_trip():
    ev = bb.evaluate((2.0, 0.0, 0.0), [SQ], SOFT)        # base_link on the line, heading along
    assert abs(ev['out'] - 0.235) < 1e-9 and ev['excess'] < -0.1


def test_soft_lap0_drift_trips_on_a_side_corner():
    ev = bb.evaluate((2.0, -0.13, 0.0), [SQ], SOFT)      # 0.13 drift: corners 0.365 out
    assert ev['excess'] > 0.0 and ev['point'] in ('front_right', 'rear_right')


def test_soft_row_end_nose_and_pivot_sweep_do_not_trip():
    # rows end 0.45 inside: nose 0.15 past; a pivot there sweeps the front corners <= 0.195
    for deg in range(0, 361, 5):
        ev = bb.evaluate((4.55, 2.0, math.radians(deg)), [SQ], SOFT)
        assert ev['excess'] < -0.1, deg


def test_hard_trips_beyond_0_10_and_rear_points_recover_forward():
    assert bb.evaluate((2.0, 0.34, 0.0), [SQ], HARD)['excess'] < 0.0     # corners 0.105 in
    ev = bb.evaluate((0.02, 2.0, math.radians(67)), [SQ], HARD)          # rear-left 0.235 out
    assert ev['excess'] > 0.0 and ev['point'] == 'rear_left' and ev['rear']


def test_union_of_area_and_path_counts_as_inside():
    path = [(5.0, 1.8), (9.0, 1.8), (9.0, 2.2), (5.0, 2.2)]
    ev = bb.evaluate((4.8, 2.0, 0.0), [SQ, path], SOFT)                 # nose on the path
    assert ev['outs']['nose'] == 0.0


# ---------------------------------------------------------------- mission integration

def feed(h, pose, out=0.0):
    i = h.fsm.inputs
    i.pose = tuple(pose)
    i.outside_m = out
    i.outside_stamp = h.t


def mowing_h(subpaths, pose, **params):
    params.setdefault('hard_boundary_body', True)
    h = Harness(**params)
    h.areas = [square(0, 0, 5)]
    feed(h, pose)
    start_until_planning(h)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan(subpaths))
    h.fsm.inputs.is_cutting = True
    feed(h, pose)
    h.tick()
    return h


def integrate(h, max_ticks=200):
    """Integrate /cmd_vel_emergency DriveCmds while the boundary recovery runs."""
    cmds = []
    for _ in range(max_ticks):
        if h.fsm._hb is None:
            break
        m = h.mark()
        h.tick()
        dc = h.since(m, f.DriveCmd)
        x, y, yaw = h.fsm.inputs.pose
        if dc:
            v, w = dc[-1].linear, dc[-1].angular
            cmds.append((v, w))
            yaw += w * 0.1
            x += v * 0.1 * math.cos(yaw)
            y += v * 0.1 * math.sin(yaw)
        feed(h, (x, y, yaw))
    return cmds


def test_nose_trip_reverses_0_3_straight_slowly_then_resumes():
    h = mowing_h([line(1, 2, 4.9, 2, n=41), line(1, 3, 4, 3)], (1.0, 2.0, 0.0))
    assert h.name == 'MOWING' and h.pending_action(f.ACT_FOLLOW) and h.blade
    m = h.mark()
    x = 1.0
    while h.fsm._hb is None and x < 4.9:
        x += 0.05
        feed(h, (x, 2.0, 0.0))
        h.tick()
    assert h.fsm._hb is not None and h.fsm._hb['mode'] == 'soft' and not h.blade
    assert x > 4.7                   # nose 0.35 past at x 4.75: no trip before the limit
    x0 = x
    cmds = integrate(h)
    moving = [c for c in cmds if c[0] or c[1]]
    assert moving and all(v < 0.0 and abs(v) <= 0.10 + 1e-9 and w == 0.0 for v, w in moving)
    assert abs((x0 - h.fsm.inputs.pose[0]) - 0.30) < 0.06          # 0.3 m straight back
    assert h.fsm._hb_count == 1
    h.tick(n=3)
    assert h.name == 'MOWING' and h.fsm.mission.sub_i == 0         # resumed the same lap
    assert [e.kind for e in h.since(m, f.RecordIncident)][:1] == ['boundary']


def test_side_drift_not_fixed_by_the_straight_move_falls_back_to_the_retrace():
    h = mowing_h([line(1, 0, 4, 0), line(1, 3, 4, 3)], (1.0, 0.0, 0.0))
    for k in range(1, 30):           # drift out sideways while driving east
        feed(h, (1.0 + 0.05 * k, -0.01 * k, 0.0))     # lateral slide
        h.tick()
        if h.fsm._hb is not None:
            break
    assert h.fsm._hb is not None
    for _ in range(60):
        if h.fsm._hb is None or h.fsm._hb['stage'] == 'drive':
            break
        integrate(h, max_ticks=1)
    assert h.fsm._hb is not None and h.fsm._hb['stage'] == 'drive'   # retrace fallback


def test_hard_rear_trip_moves_forward():
    h = mowing_h([line(1, 2, 4, 2)], (1.0, 2.0, 0.0))
    h.fsm.mission.settings['edge_mode'] = 'hard'
    feed(h, (0.02, 2.0, math.radians(67)))
    h.tick()
    k = h.fsm._hb
    assert k is not None and k['mode'] == 'hard' and k['dir'] > 0
    cmds = integrate(h, max_ticks=10)
    assert any(v > 0.0 for v, _ in cmds)


def test_outside_jump_is_ignored_and_not_counted():
    h = mowing_h([line(1, 2, 4, 2)], (1.0, 2.0, 0.0), hard_boundary_body=False)
    feed(h, (1.2, 2.0, 0.0), out=0.0)
    h.tick()
    feed(h, (1.25, 2.0, 0.0), out=23.37)      # GUI map reload mid-way (field 2026-10-21)
    h.tick(n=5)
    assert h.fsm._hb is None and h.fsm._hb_count == 0
    feed(h, (1.3, 2.0, 0.0), out=0.0)
    h.tick()
    assert h.fsm._hb is None and h.name == 'MOWING'


def test_spin_drift_stops_the_pivot_and_replans_the_approach():
    h = mowing_h([line(1, 2, 4, 2), line(1, 3, 4, 3)], (1.0, 2.0, math.pi),
                 path_conditioning=True)
    a = h.pending_action(f.ACT_SPIN)
    assert a.purpose == 'cov_pivot'
    feed(h, (1.05, 1.95, 2.5))
    h.tick()
    assert h.fsm._action is not None and h.fsm._action.name == f.ACT_SPIN    # 0.07 m: fine
    m = h.mark()
    feed(h, (1.1, 1.85, 2.0))                 # synthetic slip: 0.18 m from the Spin start
    h.tick()
    assert h.since(m, f.CancelActions)
    assert h.name == 'TRANSIT' and h.fsm.mission.step == 'transit'
    assert 'spin_drift' in [e.kind for e in h.since(m, f.RecordIncident)]


def test_three_bumper_hits_within_a_metre_skip_the_stretch():
    h = mowing_h([line(0.5, 2, 4.5, 2), line(0.5, 3, 4.5, 3)], (0.5, 2.0, 0.0))
    assert h.pending_action(f.ACT_FOLLOW)
    feed(h, (4.0, 2.0, 0.0))
    for k in range(2):
        h.fsm.inputs.bumper_hits += 1
        feed(h, (4.0 + 0.1 * k, 2.0, 0.0))
        h.tick()
        assert h.fsm.mission.sub_i == 0 and h.fsm._action is not None
    m = h.mark()
    h.fsm.inputs.bumper_hits += 1
    h.tick()
    assert 'bumper_skip' in [e.kind for e in h.since(m, f.RecordIncident)]
    assert h.fsm.mission.sub_i == 1              # the rest of the row skipped
