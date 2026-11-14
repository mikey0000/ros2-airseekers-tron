"""RTK dropout, field 2026-11-13 (docs/analysis/2026-11-13_rtk_dropout.md).

1. Once a localization hold has stopped the mission, it resumes only on RTK FIXED held
   loc_fixed_settle_s; float passes the sigma gate but does not resume.
2. A boundary trip on an untrusted pose (the fix wait pending, a pose jump with the wheels
   still) is a localization event: no recovery moves, the hold waits for fixed, and the
   boundary is checked again on the settled pose. A real excursion still trips as before.
3. No resume path (here: the boundary recovery) bypasses the fix wait.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from mower_mission import mission_fsm as f   # noqa: E402
from test_body_boundary import feed, mowing_h   # noqa: E402
from test_loc_hold import held, hold, mow_follow, run, set_loc   # noqa: E402
from test_mission_fsm import Harness, line   # noqa: E402

SINGLE = 'ekf_map sigma 0.16 m > 0.15 m (rtk single)'
LEG = [line(1, 2, 4.9, 2, n=41), line(1, 3, 4, 3)]     # east along y = 2 in the 5 m square
W = (3.0, 2.0)                                          # wheel odometry standing at (3, 2)


def step(h, pose, state='ok', rtk='fixed', wheel=None, reason='', n=1, dt=0.1):
    """Tick ``n`` times at ``pose`` with a fresh status and, given, wheel odometry."""
    for _ in range(n):
        feed(h, pose)
        set_loc(h, state, reason=reason, rtk=rtk)
        if wheel is not None:
            h.fsm.inputs.wheel_xy = tuple(wheel)
        h.tick(dt=dt)


def follow_starts(h, mark):
    return [e for e in h.since(mark, f.StartAction) if e.name == f.ACT_FOLLOW]


def logged(h, mark, text):
    return [e.text for e in h.since(mark, f.Log) if text in e.text]


def moving(h, mark):
    return [e for e in h.since(mark, f.DriveCmd) if e.linear or e.angular]


def strip_mow(rtk='fixed', **params):
    """Following the y = 2 leg from (1, 2) to (3, 2) (wheel odometry = the pose), then
    there with the status ok on ``rtk`` for longer than the settle window."""
    h = mowing_h(LEG, (1.0, 2.0, 0.0), **params)
    x = 1.0
    while x < 3.0 - 1e-9:
        x += 0.05
        step(h, (x, 2.0, 0.0), rtk=rtk, wheel=(x, 2.0))
    step(h, (3.0, 2.0, 0.0), rtk=rtk, wheel=W, n=70)
    assert h.name == 'MOWING' and h.fsm.mission.step == 'follow' and h.blade
    return h


# ---- 1: resume only on a settled RTK fix -----------------------------------------------------
def test_float_does_not_resume_after_a_dropout_hold():
    h = Harness()
    mow_follow(h)
    hold(h)
    m = h.mark()
    run(h, 'ok', 60.0, rtk='float')                  # sigma ok, RTK float: no resume
    assert held(h) and not h.blade and not follow_starts(h, m)
    assert 'waiting for RTK fixed' in h.fsm.sub_state


def test_fixed_resumes_only_after_the_settle_time():
    h = Harness()
    mow_follow(h)
    hold(h)
    run(h, 'ok', 10.0, rtk='float')
    run(h, 'ok', 4.5, rtk='fixed')                   # < loc_fixed_settle_s 5 s
    assert held(h)
    run(h, 'ok', 0.5, rtk='float')                   # back to float: the settle restarts
    run(h, 'ok', 4.5, rtk='fixed')
    assert held(h)
    m = h.mark()
    run(h, 'ok', 1.0, rtk='fixed')                   # 5.5 s fixed in one run
    assert h.fsm._loc is None and h.fsm.mission.step != 'loc_hold'
    h.tick()
    assert follow_starts(h, m) and h.blade


def test_float_wait_times_out_like_the_lost_wait():
    """Fixed never comes back: the existing loc_lost_dock_s give-up (stays, no RTK fixed)."""
    h = Harness()
    mow_follow(h)
    hold(h)
    run(h, 'ok', 605.0, dt=5.0, rtk='float')
    assert h.name == 'MOWING_INCOMPLETE' and h.fsm.mission is None
    assert 'localization not ok' in h.fsm.sub_state and not h.blade


def test_require_fixed_off_restores_the_old_resume():
    h = Harness(loc_resume_require_fixed=False)
    mow_follow(h)
    hold(h)
    run(h, 'ok', 3.5, rtk='float')
    assert h.fsm._loc is None


def test_float_without_a_dropout_hold_keeps_mowing():
    """The fix wait is for a mission a hold stopped: plain float mowing is not held."""
    h = Harness()
    mow_follow(h)
    m = h.mark()
    run(h, 'ok', 20.0, rtk='float')
    assert h.fsm.mission.step == 'follow' and h.fsm._loc is None
    assert not h.since(m, f.BladeOff)


# ---- 2: boundary trips on an untrusted pose --------------------------------------------------
def test_incident_float_jump_during_the_hold_runs_no_recovery():
    """The field sequence: single -> hold; float comes back 0.3 m off and the map pose jumps
    south with the wheels still -> the body is "out", but no recovery move; float never
    resumes; fixed settles with the pose back on the leg -> resume, nothing counted."""
    h = strip_mow()
    step(h, (3.0, 2.0, 0.0), 'degraded', 'single', W, SINGLE)
    assert held(h) and not h.blade
    step(h, (3.0, 2.0, 0.0), 'degraded', 'single', W, SINGLE, n=140)
    m = h.mark()
    jumped = (3.0, -0.25, 0.0)                       # side corners ~0.5 m outside
    step(h, jumped, 'ok', 'float', W, n=110)
    assert h.fsm._hb is None and h.fsm._hb_count == 0
    assert held(h) and not h.blade
    assert not moving(h, m) and not follow_starts(h, m)
    assert not h.since(m, f.RecordIncident)
    assert len(logged(h, m, 'pose not trusted')) == 1      # the trip was seen, logged once
    m = h.mark()
    step(h, (3.0, 2.0, 0.0), 'ok', 'fixed', W, n=60)    # fixed, settled, on the leg
    assert h.fsm._loc is None and h.fsm._hb is None and h.fsm._hb_count == 0
    assert follow_starts(h, m) and h.blade


def test_pose_jump_without_wheel_motion_on_float_is_a_localization_event():
    """No hold before: mowing on float (sigma ok), the pose jumps 2.3 m south with the
    wheels still -> stop, blade off, wait for fixed. Fixed then puts the robot genuinely
    out: that trip is acted on (re-evaluated on the settled pose)."""
    h = strip_mow('float')
    m = h.mark()
    step(h, (3.0, -0.3, 0.0), 'ok', 'float', W)
    assert h.fsm._hb is None and held(h) and not h.blade
    assert h.since(m, f.ZeroBurst) and not moving(h, m)
    assert logged(h, m, 'wheels 0.00 m')
    step(h, (3.0, -0.3, 0.0), 'ok', 'float', W, n=200)
    assert h.fsm._hb is None and held(h) and h.fsm._hb_count == 0
    # the settled fix says the robot really is that far out
    step(h, (3.0, -0.3, 0.0), 'ok', 'fixed', W, n=60)
    assert h.fsm._hb is not None and h.fsm._hb_count == 1
    assert not h.blade


def test_genuine_excursion_on_float_still_trips():
    """Mowing on float, the robot drives out (wheels move with the pose): the guard trips
    and recovers exactly as before."""
    h = strip_mow('float')
    x = 3.0
    while h.fsm._hb is None and x < 4.9:
        x += 0.05
        step(h, (x, 2.0, 0.0), 'ok', 'float', (x, 2.0))
    assert h.fsm._hb is not None and h.fsm._hb['mode'] == 'soft' and h.fsm._loc is None
    assert x > 4.75 and h.fsm._hb_count == 1 and not h.blade


def test_genuine_excursion_on_fixed_without_wheel_odometry_still_trips():
    h = strip_mow()
    h.fsm.inputs.wheel_xy = None
    x = 3.0
    while h.fsm._hb is None and x < 4.9:
        x += 0.05
        feed(h, (x, 2.0, 0.0))
        set_loc(h, 'ok', rtk='fixed')
        h.tick()
    assert h.fsm._hb is not None and x > 4.75 and h.fsm._loc is None


def test_fast_drive_out_on_fixed_with_still_wheels_still_trips():
    """RTK fixed and settled: the pose is trusted even when the wheels disagree (slide)."""
    h = strip_mow()
    x = 3.0
    while h.fsm._hb is None and x < 4.9:
        x += 0.1
        step(h, (x, 2.0, 0.0), 'ok', 'fixed', W)
    assert h.fsm._hb is not None and h.fsm._loc is None


def test_trip_right_after_an_rtk_class_change_is_deferred():
    h = strip_mow()
    step(h, (3.0, 2.0, 0.0), 'ok', 'float', W, n=10)     # fixed -> float 1 s ago
    m = h.mark()
    step(h, (4.85, 2.0, 0.0), 'ok', 'float', (1.85, 0.0))         # nose past, wheels agree
    assert h.fsm._hb is None and held(h) and not moving(h, m)
    assert logged(h, m, 'RTK float for only')


def test_defer_off_restores_the_old_guard():
    h = strip_mow('float', hard_boundary_loc_defer=False)
    step(h, (3.0, -0.3, 0.0), 'ok', 'float', W)
    assert h.fsm._hb is not None


# ---- 3: no resume path bypasses the fix wait --------------------------------------------------
def _recover(h, state, rtk, max_ticks=300):
    """Integrate the recovery's DriveCmds (wheels move with the pose) until it ends."""
    for _ in range(max_ticks):
        if h.fsm._hb is None:
            return
        m = h.mark()
        x, y, yaw = h.fsm.inputs.pose
        step(h, (x, y, yaw), state, rtk, (x, y))
        dc = h.since(m, f.DriveCmd)
        if dc:
            v, w = dc[-1].linear, dc[-1].angular
            yaw += w * 0.1
            x += v * 0.1 * math.cos(yaw)
            y += v * 0.1 * math.sin(yaw)
        feed(h, (x, y, yaw))


def test_boundary_recovery_cannot_resume_before_rtk_fixed_settles():
    """A genuine trip under RTK fixed; RTK drops to single during the recovery and comes
    back as float: back inside, the recovery holds for fixed instead of resuming."""
    h = strip_mow()
    x = 3.0
    while h.fsm._hb is None and x < 4.9:
        x += 0.05
        step(h, (x, 2.0, 0.0), 'ok', 'fixed', (x, 2.0))
    assert h.fsm._hb is not None and h.fsm._loc is None
    m = h.mark()
    step(h, h.fsm.inputs.pose, 'degraded', 'single', (x, 2.0), SINGLE, n=3)
    _recover(h, 'ok', 'float')
    assert h.fsm._hb is None
    assert held(h) and not h.blade and not follow_starts(h, m)
    assert logged(h, m, 'boundary recovery done before RTK fixed settled')
    pose = h.fsm.inputs.pose
    step(h, pose, 'ok', 'float', (pose[0], 2.0), n=100)
    assert held(h) and not follow_starts(h, m)
    step(h, pose, 'ok', 'fixed', (pose[0], 2.0), n=60)
    assert h.fsm._loc is None and h.fsm._hb is None
    assert h.fsm.mission.step not in ('loc_hold', None)
    assert follow_starts(h, m) or h.pending_action(f.ACT_NAV)

