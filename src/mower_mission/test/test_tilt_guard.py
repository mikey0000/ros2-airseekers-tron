"""Tilt guard in the mission FSM: caution speed cap, steep back-out, STUCK_NEEDS_HELP, critical."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from mower_mission import mission_fsm as f   # noqa: E402
from test_mission_fsm import Harness, mowing  # noqa: E402


def drives(h, m):
    return [(e.linear, e.angular) for e in h.since(m, f.DriveCmd)]


def tilt(h, band, roll=None, pitch=None, axis='roll'):
    i = h.fsm.inputs
    i.tilt_band = band
    i.tilt_roll_deg = roll
    i.tilt_pitch_deg = pitch
    i.tilt_axis = axis
    i.tilt_stamp = h.t


def step(h, band, roll=None, n=1, dt=0.1, pose=None):
    """n ticks with a fresh /tilt/status each time (and an optional pose)."""
    for _ in range(n):
        if pose is not None:
            h.fsm.inputs.pose = pose
        h.fsm.inputs.tilt_stamp = h.t + dt
        h.fsm.inputs.tilt_band = band
        h.fsm.inputs.tilt_roll_deg = roll
        h.fsm.inputs.tilt_axis = 'roll'
        h.tick(dt=dt)


def speed_calls(h, m):
    return [e.request['params'] for e in h.since(m, f.CallService)
            if e.name == f.SRV_SET_PARAMS and e.request['node'] == f.PARAM_NODE_CONTROLLER]


def lay_track(h, pts):
    """Drive the given poses with band ok so the trail fills."""
    for p in pts:
        step(h, 'ok', pose=p)


def line_x(x0, x1, y=0.0, n=20):
    return [(x0 + (x1 - x0) * k / n, y, 0.0) for k in range(n + 1)]


def limit_at(h, pose=(1.0, 0.0, 0.0), roll=23):
    h.fsm.inputs.pose = pose
    m = h.mark()
    step(h, 'limit', roll)
    return m


def start_backout(h, track=None, pose=(1.0, 0.0, 0.0)):
    """Mowing, track laid, limit hit; returns the mark and runs through the 0.5 s pause."""
    mowing(h)
    lay_track(h, track if track is not None else line_x(0.0, 1.0))
    m = limit_at(h, pose)
    step(h, 'limit', 23, n=3)
    assert not drives(h, m)
    step(h, 'limit', 23, n=3)
    return m


def backed_out_once(h):
    """mowing + limit + reverse to x=0.65 with band caution: a finished back-out."""
    m = start_backout(h)
    for k in range(1, 8):
        step(h, 'limit' if k < 3 else 'caution', 20, pose=(1.0 - 0.05 * k, 0.0, 0.0))
        if h.fsm._tob is None:
            break
    assert h.fsm._tob is None
    return m


def test_caution_caps_speed_once_then_restores():
    """Caution while mowing caps the controller speed once; ok restores it."""
    h = Harness()
    mowing(h)
    m = h.mark()
    step(h, 'caution', 17, n=5)
    assert speed_calls(h, m) == [{'FollowCoveragePath.desired_linear_vel': 0.15,
                                  'FollowPath.vx_max': 0.15}]
    assert h.blade and h.name == 'MOWING' and not h.since(m, f.CancelActions)
    assert h.fsm.status()['sub_state_name'].startswith('slope caution (roll 17 deg): slowed')
    m2 = h.mark()
    step(h, 'ok', 5, n=5)
    cut = min(float(h.fsm.mission.settings['cut_speed_mps']), float(h.fsm.p.cut_speed_max_mps))
    assert speed_calls(h, m2) == [{'FollowPath.vx_max': 0.3,
                                   'FollowCoveragePath.desired_linear_vel': cut}]
    assert h.fsm.status()['sub_state_name'] == h.fsm.sub_state
    assert h.blade and h.name == 'MOWING'


def test_stale_status_or_guard_disabled_does_nothing():
    """A status older than 2 s is unknown; tilt_guard=False ignores a fresh limit."""
    h = Harness()
    mowing(h)
    m = h.mark()
    tilt(h, 'caution', 17)
    h.fsm.inputs.tilt_stamp = h.t - 3.0         # status is 3 s old
    h.tick(n=5)
    assert h.fsm._tilt_band() == 'unknown'
    assert h.name == 'MOWING' and h.blade and not h.since(m, f.CancelActions)
    assert not speed_calls(h, m) and not h.fsm._tilt_capped
    h = Harness(tilt_guard=False)
    mowing(h)
    m = h.mark()
    step(h, 'limit', 23, n=10)
    assert h.name == 'MOWING' and h.blade and h.fsm._tob is None
    assert not h.since(m, f.CancelActions) and not h.since(m, f.RecordIncident)
    assert not speed_calls(h, m)


def test_limit_while_mowing_backs_out_straight_and_resumes():
    """Limit while following: stop, blade off, steep incident, reverse along the track, resume."""
    h = Harness()
    mowing(h)
    lay_track(h, line_x(0.0, 1.0))
    m = limit_at(h)
    assert h.since(m, f.CancelActions) and h.since(m, f.ZeroBurst) and not h.blade
    assert [e.active for e in h.since(m, f.SuppressStuckGuard)] == [True]
    inc = h.since(m, f.RecordIncident)
    assert len(inc) == 1 and inc[0].kind == 'steep' and (inc[0].x, inc[0].y) == (1.0, 0.0)
    assert h.fsm.sub_state == 'steep slope (roll 23 deg): stopping'
    step(h, 'limit', 23, n=3)
    assert not drives(h, m)                     # 0.5 s pause
    step(h, 'limit', 23, n=3)
    d = drives(h, m)
    assert d and d[-1][0] == -0.1 and abs(d[-1][1]) < 0.02
    assert 'backing out' in h.fsm.sub_state
    for k in range(1, 4):                       # x = 0.95 .. 0.85, limit till 0.8
        step(h, 'limit', 23, pose=(1.0 - 0.05 * k, 0.0, 0.0))
    step(h, 'limit', 23, pose=(0.8, 0.0, 0.0))
    step(h, 'caution', 18, pose=(0.75, 0.0, 0.0))   # 0.25 m: not far enough yet
    assert h.fsm._tob is not None
    step(h, 'caution', 18, pose=(0.7, 0.0, 0.0))    # 0.30 m: done
    assert h.fsm._tob is None
    assert drives(h, m)[-1] == (0.0, 0.0)
    assert [e.active for e in h.since(m, f.SuppressStuckGuard)] == [True, False]
    assert h.since(m, f.StartAction) or h.name in ('TRANSIT', 'MOWING')
    assert len(h.fsm._tilt_hist) == 1


def test_backout_steers_the_rear_toward_the_track():
    """Reverse steering follows an offset track; a track ahead is followed forward."""
    h = Harness()
    track = [(0.05 * k, 0.2, 0.0) for k in range(17)]       # y = +0.2, x up to 0.8
    m = start_backout(h, track=track, pose=(1.0, 0.0, 0.0))
    v, w = drives(h, m)[-1]
    assert v == -0.1 and w < 0                  # target behind-left: yr > 0, v < 0 -> w < 0
    h = Harness()
    track = [(1.5 - 0.05 * k, 0.0, 0.0) for k in range(21)]  # drove in reverse, x 1.5 -> 0.5
    m = start_backout(h, track=track, pose=(0.5, 0.0, 0.0))
    v, w = drives(h, m)[-1]
    assert v == 0.1 and abs(w) < 0.02


def test_still_steep_after_max_distance_needs_help():
    """Still limit after tilt_backout_max_m: STUCK_NEEDS_HELP, blade off, motion stopped."""
    h = Harness()
    m = start_backout(h)
    for k in range(1, 30):
        step(h, 'limit', 23, pose=(1.0 - 0.05 * k, 0.0, 0.0))
        if h.name == 'STUCK_NEEDS_HELP':
            break
    assert h.name == 'STUCK_NEEDS_HELP' and not h.blade
    assert h.fsm.sub_state == 'steep slope: needs help at (1.00, 0.00)'
    assert not h.fsm.motion_enabled() and h.fsm._tob is None
    assert drives(h, m)[-1] == (0.0, 0.0)
    assert [e.active for e in h.since(m, f.SuppressStuckGuard)] == [True, False]


def test_no_progress_during_backout_needs_help():
    """No movement for > 3 s while backing out: STUCK_NEEDS_HELP."""
    h = Harness()
    start_backout(h)
    assert h.fsm._tob is not None
    step(h, 'limit', 23, n=8, dt=0.5)
    assert h.name == 'STUCK_NEEDS_HELP' and not h.blade
    assert h.fsm.sub_state == 'steep slope: needs help at (1.00, 0.00)'


def test_same_spot_again_needs_help_and_stop_clears():
    """A second limit within 0.75 m of a backed-out spot needs help; STOP clears the history."""
    h = Harness()
    backed_out_once(h)
    assert len(h.fsm._tilt_hist) == 1
    m = h.mark()
    step(h, 'limit', 23, pose=(0.9, 0.1, 0.0))
    assert h.name == 'STUCK_NEEDS_HELP' and not h.blade and h.fsm._tob is None
    assert not drives(h, m)
    assert h.fsm.sub_state == 'steep slope: needs help at (0.90, 0.10)'
    step(h, 'ok', 5, n=3)
    assert h.name == 'STUCK_NEEDS_HELP'
    h.cmd(f.CMD_STOP)
    assert h.name in f.IDLE_NAMES and h.fsm._tilt_hist == []


def test_critical_is_an_emergency_and_clears_to_idle():
    """Critical tilt: EMERGENCY, blade off, mission interrupted; ok clears it."""
    h = Harness()
    mowing(h)
    m = h.mark()
    step(h, 'critical', 31)
    assert h.name == 'EMERGENCY' and not h.blade
    assert 'tilt critical (roll 31 deg)' in h.fsm.sub_state
    assert h.since(m, f.ZeroBurst) and h.since(m, f.CancelActions)
    assert h.fsm.mission is None
    step(h, 'critical', 31, n=3)
    assert h.name == 'EMERGENCY'
    step(h, 'ok', 5, n=2)
    assert h.name in f.IDLE_NAMES


def test_critical_aborts_a_running_backout():
    """Critical during the back-out stops it: zero drive, stuck guard re-armed."""
    h = Harness()
    m0 = start_backout(h)
    assert h.fsm._tob is not None
    m = h.mark()
    step(h, 'critical', 31, pose=(0.95, 0.0, 0.0))
    assert h.fsm._tob is None and h.name == 'EMERGENCY'
    assert [e.active for e in h.since(m, f.SuppressStuckGuard)] == [False]
    assert drives(h, m)[0] == (0.0, 0.0)
    del m0


def test_manual_mowing_limit_blade_off_only():
    """Manual mowing: limit turns the blade off, keeps the phase, never drives; blade-on refused."""
    h = Harness()
    h.cmd(f.CMD_MANUAL_MOW)
    assert h.blade and h.name == 'MANUAL_MOWING'
    m = h.mark()
    step(h, 'limit', 23, n=3)
    assert not h.blade and h.name == 'MANUAL_MOWING'
    assert h.fsm.sub_state == f.SUB_MANUAL_BLADE_OFF
    assert h.since(m, f.BladeOff) and not h.since(m, f.DriveCmd) and h.fsm._tob is None
    ok, msg, fx = h.fsm.manual_blade(True, h.t)
    h._apply(fx)
    assert not ok and 'steep slope' in msg and not h.blade
    step(h, 'ok', 5)
    ok, msg, fx = h.fsm.manual_blade(True, h.t)
    h._apply(fx)
    assert ok and h.blade


def test_limit_in_idle_does_nothing():
    """Limit while idle: no actions, no drive, no blade, no incident."""
    h = Harness()
    m = h.mark()
    step(h, 'limit', 23, n=10)
    assert h.name in f.IDLE_NAMES and h.fsm._tob is None
    for kind in (f.DriveCmd, f.BladeOff, f.BladeOn, f.CancelActions, f.RecordIncident,
                 f.StartAction, f.ZeroBurst, f.SuppressStuckGuard):
        assert not h.since(m, kind), kind
