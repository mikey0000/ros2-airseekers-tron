"""DriveLoadDetector (pure logic) against traces shaped like the 2026-10-21c field data."""

from mower_control.motor_load import DriveLoadDetector, replay


def _drive(det, t0, seconds, v_cmd, ground, frames, rpm=(0.0, 0.0)):
    """10 Hz motion, one motor frame per second from ``frames`` [(left_a, right_a)]."""
    out = []
    for k in range(int(seconds * 10)):
        t = t0 + k * 0.1
        det.on_motion(t, v_cmd, ground)
        if k % 10 == 0 and k // 10 < len(frames):
            det.on_motor(t, *frames[k // 10], left_rpm=rpm[0], right_rpm=rpm[1])
        ev = det.update(t)
        if ev:
            out.append((t, ev))
    return out


def test_wheel_spin_0637_fires_as_spin():
    det = DriveLoadDetector()
    _drive(det, 0.0, 2.0, -0.12, 0.10, [(1.1, 2.5), (21.1, 2.3)])   # load rising, still moving
    ev = _drive(det, 2.0, 3.0, -0.12, 0.01, [(19.6, 9.4), (15.5, 15.1), (12.2, 10.4)],
                rpm=(18.0, 2.0))
    assert len(ev) == 1
    assert ev[0][1]['kind'] == 'spin' and ev[0][1]['load_a'] > 20.0


def test_normal_forward_and_pivots_do_not_fire():
    det = DriveLoadDetector()
    # p99 forward load at speed: ground follows the command
    assert not _drive(det, 0.0, 10.0, 0.2, 0.2, [(8.0, 8.5)] * 10)
    # pivot at 0.3 rad/s draws 15-23 A with no linear command
    assert not _drive(det, 10.0, 10.0, 0.0, 0.01, [(14.0, 9.3)] * 10)
    # bumper-like slowdown: 17 A, ground 0.11 of 0.17 commanded
    assert not _drive(det, 20.0, 3.0, 0.17, 0.11, [(10.5, 6.5)] * 3)


def test_stall_kind_holdoff_and_stale():
    det = DriveLoadDetector(holdoff_s=10.0)
    ev = _drive(det, 0.0, 5.0, 0.15, 0.0, [(12.0, 12.0)] * 5, rpm=(0.0, 1.0))
    assert len(ev) == 1 and ev[0][1]['kind'] == 'stall'   # holdoff suppresses repeats
    det2 = DriveLoadDetector()
    det2.on_motor(0.0, 15.0, 15.0)
    det2.on_motion(3.0, 0.15, 0.0)
    assert det2.update(3.0) is None                       # frame 3 s old: no decision


def test_confirm_frames():
    det = DriveLoadDetector(confirm_frames=2)
    ev = _drive(det, 0.0, 3.0, 0.15, 0.0, [(12.0, 12.0), (5.0, 5.0), (12.0, 12.0)])
    assert not ev
    ev = _drive(det, 3.0, 3.0, 0.15, 0.0, [(12.0, 12.0), (12.0, 12.0)])
    assert len(ev) == 1


def test_replay_matches_live_feeding():
    """replay() over recorded rows fires where the live feed does (0637 wheel spin shape)."""
    motion = [(k * 0.1, -0.12, 0.10 if k < 20 else 0.01) for k in range(50)]
    frames = [(0.0, 1.1, 2.5, 0.0, 0.0), (1.0, 21.1, 2.3, 0.0, 0.0), (2.0, 19.6, 9.4, 18.0, 2.0),
              (3.0, 15.5, 15.1, 18.0, 2.0), (4.0, 12.2, 10.4, 18.0, 2.0)]
    ev = replay(motion, frames)
    assert len(ev) == 1
    t, e = ev[0]
    assert 2.0 <= t <= 4.0 and e['kind'] == 'spin' and e['load_a'] > 20.0
    # same rows, swapped left/right: the rule only uses the sum, so the B10 label
    # question does not change the result
    swapped = [(f[0], f[2], f[1], f[4], f[3]) for f in frames]
    assert [(round(t2, 3), e2['kind'], e2['load_a']) for t2, e2 in replay(motion, swapped)] == \
        [(round(t, 3), e['kind'], e['load_a'])]

