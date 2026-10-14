"""Fix 10 (turn-leg completion cap, leg loop guard) and fix 11 (route cusp about-turn, bounded
costmap fallback) from the 2026-10-14 run 2 field test (docs/wheel_smoothness_field_tests.md)."""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from mower_mission import mission_fsm as f   # noqa: E402
from test_mission_fsm import Harness, mowing, plan, start_until_planning  # noqa: E402


def logs(h, mark=0):
    return [e.text for e in h.since(mark, f.Log)]


def starts(h, mark, name):
    return [e for e in h.since(mark, f.StartAction) if e.name == name]


# =====================================================================
# fix 10: turn-leg completion cap
# =====================================================================
RYAW = math.atan2(-0.35, 0.1)          # reverse leg poses face against its travel


def _k_turn_on_rev(h, fwd2=None):
    """swath ending at a cusp -> reverse leg -> forward leg (+ a last swath). Returns with
    the reverse leg's follow_path running and the robot at its start."""
    start_until_planning(h)
    fwd = [(0.2, 0.0, 0.0)] + [(1.0 + 0.1 * k, 0.0, 0.0) for k in range(11)]   # ends (2.0, 0)
    rev = [(2.0, 0.0, RYAW), (1.95, 0.175, RYAW), (1.9, 0.35, RYAW)]
    fwd2 = fwd2 or [(1.9, 0.35, math.pi), (0.0, 0.36, math.pi)]
    last = [(0.0, 1.0, 0.0), (4.0, 1.0, 0.0)]
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([fwd, rev, fwd2, last]))
    h.fsm.inputs.is_cutting = True
    h.tick()
    assert h.goal(f.ACT_FOLLOW)['poses'][-1] == fwd[-1]
    h.fsm.inputs.pose = (2.0, 0.0, 0.0)          # at the cusp: the leg goal checker ends it
    h.finish(f.ACT_FOLLOW, arrive=False)
    assert h.goal(f.ACT_FOLLOW)['poses'] == rev
    return fwd, rev, fwd2


def test_turn_leg_end_reached_with_yaw_off_completes_after_the_time_cap():
    h = Harness()
    _fwd, rev, fwd2 = _k_turn_on_rev(h)
    tok = h.pending_action(f.ACT_FOLLOW).token
    h.fsm.inputs.pose = (1.9, 0.35, RYAW + 1.0)  # at the end in xy, 1.0 rad off: no skip
    m = h.mark()
    h.tick(dt=0.1, n=199)                        # 19.9 s, yaw constant (no rotation)
    assert h.pending_action(f.ACT_FOLLOW).token == tok
    assert not h.since(m, f.CancelActions)
    h.tick(dt=0.1, n=2)
    g = h.goal(f.ACT_FOLLOW)
    assert g['poses'] == fwd2 and h.name == 'MOWING'
    assert 1 in h.fsm.cursor.areas[0].completed
    done = [t for t in logs(h, m) if 'leg done' in t]
    assert len(done) == 1 and 'time cap' in done[0] and 'sub-path 1' in done[0]
    assert h.since(m, f.CancelActions)


def test_turn_leg_end_reached_completes_after_one_full_rotation():
    """The run 2 loop: pose index at the leg's last pose, the robot looping 0.4 m away
    (outside turn_leg_skip_m, so the yaw-pass skip can never fire) and turning."""
    h = Harness()
    _fwd, rev, fwd2 = _k_turn_on_rev(h)
    h.fsm.inputs.pose = (1.9, 0.35, RYAW + 1.0)
    h.tick()                                     # end reached (pose index at the last pose)
    m = h.mark()
    yaw = RYAW + 1.0
    for k in range(70):                          # 0.1 rad per 0.1 s tick, 0.4 m off the end
        yaw += 0.1
        h.fsm.inputs.pose = (1.9, 0.75, yaw)
        h.tick()
        if h.since(m, f.CancelActions):
            break
    assert 60 <= k < 66                          # ~6.28 rad, after ~6.3 s (< the 20 s cap)
    assert h.goal(f.ACT_FOLLOW)['poses'] == fwd2
    done = [t for t in logs(h, m) if 'leg done' in t]
    assert len(done) == 1 and 'rotation cap' in done[0]


def test_turn_leg_end_with_yaw_met_is_unchanged():
    """Yaw within tolerance at the end: the leg goal checker ends it (0.2 s later) and the
    caps stay out of it; an abort there is skipped on the re-send as before (turn_leg_skip)."""
    h = Harness()
    _fwd, rev, fwd2 = _k_turn_on_rev(h)
    h.fsm.inputs.pose = (1.92, 0.33, RYAW + 0.2)
    m = h.mark()
    h.tick(dt=0.1, n=2)                          # 0.2 s at the end
    assert h.goal(f.ACT_FOLLOW)['poses'] == rev and not h.since(m, f.CancelActions)
    h.finish(f.ACT_FOLLOW, arrive=False)
    assert h.goal(f.ACT_FOLLOW)['poses'] == fwd2
    assert not [t for t in logs(h, m) if 'leg done' in t]

    h2 = Harness(transit_retry_delay_s=0.1)
    _k_turn_on_rev(h2)
    h2.fsm.inputs.pose = (1.95, 0.2, RYAW + 0.2)    # 0.16 m short of the end (pose 1)
    h2.tick(dt=0.1, n=2)
    m = h2.mark()
    h2.finish(f.ACT_FOLLOW, f.ABORTED)
    h2.tick(dt=0.1, n=2)                         # retry after 0.1 s: turn_leg_skip, next leg
    assert h2.goal(f.ACT_FOLLOW)['poses'][0] == (1.9, 0.35, math.pi)
    done = [t for t in logs(h2, m) if 'leg done' in t]
    assert len(done) == 1 and 'no re-send' in done[0] and 'cap' not in done[0]


def test_turn_leg_caps_off_never_end_the_leg():
    h = Harness(turn_leg_done_after_s=0.0, turn_leg_done_after_rot_rad=0.0,
                leg_loop_abort_s=0.0)
    _k_turn_on_rev(h)
    tok = h.pending_action(f.ACT_FOLLOW).token
    h.fsm.inputs.pose = (1.9, 0.35, RYAW + 1.0)
    h.tick(dt=1.0, n=120)
    assert h.pending_action(f.ACT_FOLLOW).token == tok


def test_turn_leg_cap_resets_per_leg():
    """The reverse leg ends on its time cap; the next (short forward) leg reaches its end at
    once with the yaw off and gets its own 20 s from then."""
    fwd2 = [(1.9, 0.35, math.pi), (1.0, 0.36, math.pi)]      # 0.9 m <= turn_leg_max_m
    h = Harness()
    _k_turn_on_rev(h, fwd2=fwd2)
    h.fsm.inputs.pose = (1.9, 0.35, RYAW + 1.0)
    h.tick(dt=0.1, n=201)
    assert h.goal(f.ACT_FOLLOW)['poses'] == fwd2
    tok = h.pending_action(f.ACT_FOLLOW).token
    h.fsm.inputs.pose = (1.0, 0.36, math.pi + 1.0)
    m = h.mark()
    h.tick(dt=0.1, n=199)
    assert h.pending_action(f.ACT_FOLLOW).token == tok
    h.tick(dt=0.1, n=2)
    assert 2 in h.fsm.cursor.areas[0].completed
    assert h.name == 'TRANSIT' and h.fsm.mission.sub_i == 3     # on to the last swath
    done = [t for t in logs(h, m) if 'leg done' in t]
    assert len(done) == 1 and 'sub-path 2' in done[0] and 'time cap' in done[0]


# =====================================================================
# fix 10: leg loop guard
# =====================================================================
def test_follow_without_pose_index_progress_is_aborted_and_retried():
    h = Harness(transit_retry_delay_s=3.0)
    mowing(h)                                    # line (0, 0) -> (4, 0), 0.4 m poses
    for k in range(1, 4):                        # drives 1.2 m, then loops in place
        h.fsm.inputs.pose = (0.4 * k, 0.0, 0.0)
        h.tick(dt=1.0)
    tok = h.pending_action(f.ACT_FOLLOW).token
    assert h.fsm.mission.progress_local == 3
    m = h.mark()
    h.tick(dt=1.0, n=89)
    assert h.pending_action(f.ACT_FOLLOW).token == tok and not h.since(m, f.CancelActions)
    h.tick(dt=1.0)
    assert h.fsm._action is None and h.since(m, f.CancelActions)
    assert h.fsm.mission.step == 'retry_wait' and h.fsm.mission.follow_fails == 1
    assert 'retry 1/3' in h.fsm.sub_state
    assert any('leg loop guard' in t for t in logs(h, m))
    assert any(t.startswith('follow_path finished: aborted') for t in logs(h, m))
    h.tick(dt=1.0, n=4)
    g = h.goal(f.ACT_FOLLOW)                     # the existing retry: from the reached pose
    assert abs(g['poses'][0][0] - 1.2) < 1e-6


def test_loop_guard_restarts_with_every_new_goal_and_does_not_fire_in_a_hold():
    """Localization hold for 100 s (the follow is cancelled): no loop abort while held; the
    resumed goal gets its own leg_loop_abort_s."""
    h = Harness()
    mowing(h)
    h.tick(dt=1.0, n=50)
    m = h.mark()
    i = h.fsm.inputs
    for _ in range(100):
        i.loc_state, i.loc_reason, i.loc_stamp = 'degraded', 'rtk float', h.t
        h.tick(dt=1.0)
    assert h.fsm.mission.step == 'loc_hold'
    for _ in range(5):
        i.loc_state, i.loc_stamp = 'ok', h.t
        h.tick(dt=1.0)
    assert h.fsm.mission.step == 'follow'
    tok = h.pending_action(f.ACT_FOLLOW).token
    h.tick(dt=1.0, n=85)
    assert h.pending_action(f.ACT_FOLLOW).token == tok
    assert not any('leg loop guard' in t for t in logs(h, m))
    h.tick(dt=1.0, n=6)
    assert any('leg loop guard' in t for t in logs(h, m))


def test_loop_guard_does_not_fire_in_a_dynamic_obstacle_wait():
    h = Harness(dynamic_wait_s=120.0)
    mowing(h)
    h.tick(dt=1.0, n=80)
    m = h.mark()
    i = h.fsm.inputs
    for _ in range(40):                          # person right in front for 40 s
        i.obstacle_kind, i.obstacle_class = 'dynamic', 'person'
        i.obstacle_distance, i.obstacle_bearing, i.obstacle_stamp = 0.8, 0.0, h.t
        h.tick(dt=1.0)
    assert h.fsm.mission.step == 'dyn_wait'
    assert not any('leg loop guard' in t for t in logs(h, m))


def test_loop_guard_does_not_fire_in_a_stuck_escape():
    h = Harness()
    mowing(h)
    h.fsm.inputs.pose = (1.0, 0.0, 0.0)
    h.tick(dt=1.0, n=80)                         # 80 s at pose index 2..3
    m = h.mark()
    h.fsm.inputs.stuck = True
    h.tick()
    h.fsm.inputs.stuck = False
    assert h.fsm._stk is not None
    h.tick(n=10)
    for k in range(1, 40):                       # the escape reverses slowly (~20 s)
        h.fsm.inputs.pose = (1.0 - 0.01 * k, 0.0, 0.0)
        h.tick(dt=0.5)
        if h.fsm._stk is None:
            break
    assert h.fsm._stk is None and h.name == 'MOWING'
    h.tick()
    assert h.pending_action(f.ACT_FOLLOW)
    assert not any('leg loop guard' in t for t in logs(h, m))
    tok = h.pending_action(f.ACT_FOLLOW).token
    h.tick(dt=1.0, n=80)                         # the resumed goal starts its own timer
    assert h.pending_action(f.ACT_FOLLOW).token == tok
