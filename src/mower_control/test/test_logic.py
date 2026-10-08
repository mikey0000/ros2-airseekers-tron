"""Node logic without spinning: slew/watchdog, slip latch, IMU window evaluation."""

import math
import types

import pytest

from mower_control import cmd_vel_slew, imu_cal, slip_detector


def _logger():
    log = types.SimpleNamespace(lines=[])
    for level in ('debug', 'info', 'warn', 'warning', 'error'):
        setattr(log, level, lambda text, _l=level: log.lines.append((_l, text)))
    return log


# ---------------------------------------------------------------------- cmd_vel_slew
def _slew_node(**overrides):
    log = _logger()
    node = types.SimpleNamespace(
        _max_linear_accel=0.5, _max_angular_accel=1.0, _cmd_timeout=0.5,
        _log_period=1.0, _cur_lin=0.0, _cur_ang=0.0, _tgt=None, _last_cmd_time=None,
        _last_tick=None, _next_log=0.0, get_logger=lambda: log, log=log)
    node.__dict__.update(overrides)
    return node


def _step(node, now):
    return cmd_vel_slew.CmdVelSlewNode.step(node, now)


def test_slew_ramps_and_passes_through():
    node = _slew_node()
    assert _step(node, 100.0) is None                     # first tick only arms dt
    node._tgt, node._last_cmd_time = (1.0, 0.1, 0.2, 0.3, 0.4, -2.0), 100.0
    out = _step(node, 100.05)
    assert out == pytest.approx((0.025, 0.1, 0.2, 0.3, 0.4, -0.05))
    for k in range(1, 9):
        node._last_cmd_time = 100.05 + 0.05 * k
        out = _step(node, 100.05 + 0.05 * k)
    assert out[0] == pytest.approx(0.225) and out[5] == pytest.approx(-0.45)


def test_watchdog_exact_stop_and_timing():
    node = _slew_node()
    _step(node, 10.0)
    node._tgt, node._last_cmd_time = (0.4, 0.0, 0.0, 0.0, 0.0, 0.0), 10.0
    for k in range(1, 11):
        out = _step(node, 10.0 + 0.05 * k)                # 0.5 s since the command: not stale
    assert out[0] == pytest.approx(0.25)
    out = _step(node, 10.55)                              # > cmd_timeout: exact zero, no slew
    assert out[0] == 0.0 and out[5] == 0.0
    assert node._cur_lin == 0.0
    node._last_cmd_time = 10.6                            # fresh command: slews up from zero
    out = _step(node, 10.6)
    assert out[0] == pytest.approx(0.025)


def test_applied_log_throttled():
    node = _slew_node()
    _step(node, 0.0)
    for k in range(1, 41):                                # 2 s at 20 Hz
        node._tgt, node._last_cmd_time = (0.2, 0.0, 0.0, 0.0, 0.0, 0.0), 0.05 * k  # commanded
        _step(node, 0.05 * k)
    applied = [t for lvl, t in node.log.lines if t.startswith('cmd_vel applied')]
    assert len(applied) == 2
    node = _slew_node(_log_period=0.0)
    _step(node, 0.0)
    for k in range(1, 41):
        node._tgt, node._last_cmd_time = (0.2, 0.0, 0.0, 0.0, 0.0, 0.0), 0.05 * k
        _step(node, 0.05 * k)
    assert len([t for _l, t in node.log.lines if t.startswith('cmd_vel applied')]) == 40


def test_applied_log_silent_at_rest():
    node = _slew_node(_log_period=0.0)
    _step(node, 0.0)
    for k in range(1, 21):                                # nothing commanded: no chatter
        _step(node, 0.05 * k)
    assert not [t for _l, t in node.log.lines if t.startswith('cmd_vel applied')]


# ---------------------------------------------------------------------- slip_detector
def _slip_node():
    pub = []
    node = types.SimpleNamespace(
        _latched=False, _cmd=None, _meas=None, _require_rtk_fixed=True, _fix_status=None,
        _fix_time=None, _fix_warned=False, _fix_timeout=5.0, _slip_threshold=0.3,
        _slip_window=2.0, _violation_start=None, get_logger=_logger, pub=pub,
        _publish_dig=pub.append)
    return node


def test_slip_latches_after_window_with_rtk_fixed():
    node = _slip_node()
    step = slip_detector.SlipDetectorNode.step
    node._cmd, node._meas = (0.5, 0.0), (0.0, 0.0)
    node._fix_status, node._fix_time = 'quality=RTK_FIXED', 0.0
    step(node, 0.0)
    assert node.pub == [False]
    step(node, 1.0)
    step(node, 2.1)
    assert node._latched and node.pub[-1] is True and node.pub == [False, False, True]
    step(node, 2.2)
    assert node.pub[-1] is True


def test_slip_gated_without_rtk_fixed_or_stale_fix():
    node = _slip_node()
    step = slip_detector.SlipDetectorNode.step
    node._cmd, node._meas = (0.5, 0.0), (0.0, 0.0)
    node._fix_status, node._fix_time = 'quality=FLOAT', 0.0
    for t in (0.0, 1.0, 3.0):
        step(node, t)
    assert not node._latched and node.pub == []
    node._fix_status, node._fix_time = 'solution=INS_RTKFIXED', 0.0
    step(node, 10.0)                                      # stale /fix_status
    assert not node._latched and node.pub == []


# ---------------------------------------------------------------------- imu_cal
THRESH = dict(gravity=9.81, accel_tolerance=0.5, gyro_variance_max=1e-4,
              accel_variance_max=0.01, gyro_bias_max=0.2, require_gyro_noise=False)


def _drive(det, t0, t1, cmd, pose_fn, wheel_v, dt=0.1, suppressed=False):
    """Feed the detector from t0 to t1; returns the first event (or None)."""
    ev, t = None, t0
    while t <= t1 + 1e-9:
        e = det.update(t, cmd, pose_fn(t), t, wheel_v, suppressed=suppressed)
        ev = ev or e
        t += dt
    return ev


def test_stuck_detects_commanded_motion_without_ekf_progress():
    det = slip_detector.StuckDetector()
    ev = _drive(det, 0.0, 3.8, (0.2, 0.0), lambda t: (1.0, 2.0, 0.0), 0.0)
    assert ev is None and not det.stuck                   # window not full yet
    ev = _drive(det, 3.9, 4.2, (0.2, 0.0), lambda t: (1.0 + 0.001 * t, 2.0, 0.0), 0.0)
    assert det.stuck and ev['reason'] == 'stuck'
    assert ev['x'] == pytest.approx(1.0, abs=0.01) and ev['commanded'] == [0.2, 0.0]


def test_stuck_pivot_in_place_without_turning():
    det = slip_detector.StuckDetector()                   # the 2026-10-07 case: w only
    ev = _drive(det, 0.0, 4.5, (0.0, 0.4), lambda t: (0.0, 0.0, 0.01 * t), 0.0)
    assert ev and ev['reason'] == 'stuck'


def test_no_stuck_when_moving_turning_or_not_commanded():
    det = slip_detector.StuckDetector()
    assert _drive(det, 0.0, 10.0, (0.2, 0.0), lambda t: (0.2 * t, 0.0, 0.0), 0.2) is None
    det = slip_detector.StuckDetector()
    assert _drive(det, 0.0, 10.0, (0.0, 0.3), lambda t: (0.0, 0.0, 0.3 * t), 0.0) is None
    det = slip_detector.StuckDetector()
    assert _drive(det, 0.0, 10.0, (0.0, 0.05), lambda t: (0.0, 0.0, 0.0), 0.0) is None
    det = slip_detector.StuckDetector()                   # stale EKF pose: no verdict
    t = 0.0
    while t < 10.0:
        assert det.update(t, (0.2, 0.0), (0.0, 0.0, 0.0), 0.0, 0.2) is None
        t += 0.1


def test_spinning_wheel_to_ekf_ratio():
    det = slip_detector.StuckDetector()
    # EKF creeps 0.08 m in 4 s (> min progress) while the wheels report 0.8 m
    ev = _drive(det, 0.0, 4.3, (0.2, 0.0), lambda t: (0.02 * t, 0.0, 0.0), 0.2)
    assert ev and ev['reason'] == 'spinning' and ev['wheel_m'] > 0.6


def test_stuck_clears_on_move_or_suppress():
    det = slip_detector.StuckDetector()
    _drive(det, 0.0, 4.5, (0.2, 0.0), lambda t: (0.0, 0.0, 0.0), 0.0)
    assert det.stuck
    _drive(det, 4.6, 6.0, (0.0, 0.0), lambda t: (0.2, 0.0, 0.0), 0.0)
    assert det.stuck                                      # 0.2 m < clear 0.3 m
    _drive(det, 6.1, 6.2, (0.0, 0.0), lambda t: (0.35, 0.0, 0.0), 0.0)
    assert not det.stuck
    _drive(det, 7.0, 11.5, (0.2, 0.0), lambda t: (0.35, 0.0, 0.0), 0.0)
    assert det.stuck
    _drive(det, 11.6, 11.7, (-0.1, 0.0), lambda t: (0.35, 0.0, 0.0), 0.0, suppressed=True)
    assert not det.stuck
    # while suppressed it never latches
    assert _drive(det, 12.0, 20.0, (-0.1, 0.0), lambda t: (0.35, 0.0, 0.0), 0.0,
                  suppressed=True) is None


def test_stuck_rearms_after_escape_suppress():
    """Escape -> suppress false -> commanded + no progress must latch again within 4 s."""
    det = slip_detector.StuckDetector()
    _drive(det, 0.0, 4.5, (0.1, 0.2), lambda t: (-1.18, 3.73, 0.0), 0.0)
    assert det.stuck
    _drive(det, 4.6, 8.0, (-0.1, 0.0), lambda t: (-1.18, 3.73, 0.0), 0.0, suppressed=True)
    assert not det.stuck
    ev = _drive(det, 8.1, 12.3, (0.1, 0.2), lambda t: (-1.18, 3.73, 0.0), 0.0)
    assert ev and ev['reason'] == 'stuck'


def test_stuck_latches_through_dithering_rotate_to_heading():
    """2026-10-07 live: stalled RPP rotate-to-heading, |w| 0.01-0.3 with sign flips and
    sub-threshold / zero samples every ~1 s; the window must not restart on each dip."""
    det = slip_detector.StuckDetector()
    ws = [-0.155, -0.011, -0.183, 0.176, 0.103, -0.178, -0.049, 0.169, 0.151, 0.040,
          -0.191, 0.0, 0.081, -0.042, 0.141, -0.070, 0.161, 0.066, -0.056]
    ev, t = None, 0.0
    while t < 19.0 and ev is None:
        w = ws[int(t)]
        ev = det.update(t, (0.0, w), (-1.18, 3.73, 0.0), t, 0.0)
        t += 0.05
    assert ev and ev['reason'] == 'stuck' and t < 8.0


def test_stuck_window_restarts_after_a_real_stop():
    det = slip_detector.StuckDetector()
    assert _drive(det, 0.0, 3.0, (0.2, 0.0), lambda t: (0.0, 0.0, 0.0), 0.0) is None
    assert _drive(det, 3.1, 5.0, (0.0, 0.0), lambda t: (0.0, 0.0, 0.0), 0.0) is None
    assert _drive(det, 5.1, 8.0, (0.2, 0.0), lambda t: (0.0, 0.0, 0.0), 0.0) is None


def _stuck_node(det):
    pub, events = [], []
    log = _logger()
    node = types.SimpleNamespace(
        _stuck=det, _reset_req=False, _cmd=(0.2, 0.0), _ekf_pose=(1.0, 1.0, 0.0), _ekf_t=0.0,
        _meas=(0.0, 0.0), _suppressed=False, _phase='TRANSIT', _stuck_last=None,
        get_logger=lambda: log,
        _stuck_pub=types.SimpleNamespace(publish=lambda b: pub.append(b == slip_detector._DIG_TRUE)),
        _stuck_event_pub=types.SimpleNamespace(publish=lambda m: events.append(m.data)))
    return node, pub, events


def test_stuck_step_publishes_latch_event_and_reset():
    import json
    node, pub, events = _stuck_node(slip_detector.StuckDetector())
    t = 0.0
    while t < 4.5:
        node._ekf_t = t
        slip_detector.SlipDetectorNode.stuck_step(node, t)
        t += 0.1
    assert pub[-1] is True and len(events) == 1
    ev = json.loads(events[0])
    assert ev['phase'] == 'TRANSIT' and ev['reason'] == 'stuck' and 'ts' in ev
    node._reset_req = True
    slip_detector.SlipDetectorNode.stuck_step(node, t)
    assert pub[-1] is False


def _wit_at_rest(n=100):
    # What the JY61P on the mower reports standing still: gyro exactly 0.0 on all axes,
    # accel ~(-0.61, -0.02, 9.85) with ~0.01-0.02 m/s^2 noise.
    acc = [(-0.6087 + 0.01 * ((i % 5) - 2), -0.0184 + 0.009 * ((i % 3) - 1),
            9.8485 + 0.02 * ((i % 7) - 3) / 3) for i in range(n)]
    return [a + (0.0, 0.0, 0.0) for a in acc]


def test_dead_banded_gyro_window_is_plausible():
    stats = imu_cal.window_stats(_wit_at_rest())
    assert imu_cal.window_rejections(stats, **THRESH) == []
    # The original MowgliNext rule (non-zero gyro variance on every axis) rejects it:
    strict = dict(THRESH, require_gyro_noise=True)
    assert imu_cal.window_rejections(stats, **strict) == [
        'gyro axis with zero variance (require_gyro_noise)']
    gyro, accel = imu_cal.biases_from_stats(stats, 9.81)
    assert gyro == [0.0, 0.0, 0.0]
    # the accel bias is the |accel| excess along gravity (~0.057 m/s^2 here)
    assert sum(v * v for v in accel) ** 0.5 == pytest.approx(stats['mag_a'] - 9.81)
    imu_cal.validate_calibration({'gyro_bias_radps': gyro, 'accel_bias_mps2': accel}, 0.2)


def test_moving_or_frozen_windows_rejected():
    moving = [(0.5 * (i % 2), 0.0, 9.81 + (i % 3) * 0.3, 0.0, 0.0, 0.2 * (i % 2))
              for i in range(100)]
    reasons = imu_cal.window_rejections(imu_cal.window_stats(moving), **THRESH)
    assert any('gyro variance' in r for r in reasons)
    assert any('accel variance' in r for r in reasons)
    frozen = [(0.0, 0.0, 9.81, 0.0, 0.0, 0.0)] * 100
    assert imu_cal.window_rejections(imu_cal.window_stats(frozen), **THRESH) == [
        'accelerometer frozen (zero variance)']
    upside = [(0.0, 0.0, -9.81 + 0.01 * (i % 2), 0.0, 0.0, 0.0) for i in range(100)]
    assert imu_cal.window_rejections(imu_cal.window_stats(upside), **THRESH) == []


def test_retry_backoff_caps_at_ten_minutes():
    delays = [imu_cal.retry_delay(k, 60.0, 600.0) for k in range(1, 7)]
    assert delays == [60.0, 120.0, 240.0, 480.0, 600.0, 600.0]


@pytest.mark.parametrize('data, reason', [
    ({'gyro_bias_radps': [0, 0, 0], 'accel_bias_mps2': [0, 0, 0]}, 'all-zero'),
    ({'gyro_bias_radps': [0.3, 0, 0], 'accel_bias_mps2': [0, 0, 0.1]}, 'gyro bias'),
    ({'gyro_bias_radps': [0.1, 0]}, 'malformed'),
    (None, 'malformed'),
])
def test_validate_calibration_rejects(data, reason):
    with pytest.raises(ValueError, match=reason):
        imu_cal.validate_calibration(data, 0.2)


# ---------------------------------------------------------------------- straight-line shaping
def test_shaper_defaults_are_neutral():
    sh = cmd_vel_slew.DriveShaper()
    for lin, ang in ((0.3, 0.0), (0.3, 0.01), (0.0, 0.5), (-0.3, -0.02)):
        assert sh.shape(lin, ang, yaw=1.0, gyro_z=0.2) == ang
    assert not sh.holding


def test_trim_only_while_driving_and_signed_with_linear():
    sh = cmd_vel_slew.DriveShaper(angular_trim_radps=0.03)
    assert sh.shape(0.3, 0.0) == pytest.approx(0.03)
    assert sh.shape(-0.3, 0.0) == pytest.approx(-0.03)
    assert sh.shape(0.01, 0.5) == 0.5                     # turning on the spot: untouched
    assert sh.shape(0.3, 0.2) == pytest.approx(0.23)


def test_deadband_zeroes_small_angular_only_while_driving():
    sh = cmd_vel_slew.DriveShaper(angular_deadband_radps=0.05)
    assert sh.shape(0.3, 0.04) == 0.0
    assert sh.shape(0.3, -0.049) == 0.0
    assert sh.shape(0.3, 0.06) == 0.06
    assert sh.shape(0.0, 0.04) == 0.04                    # spot turn kept


def test_heading_hold_engages_corrects_and_releases():
    sh = cmd_vel_slew.DriveShaper(heading_hold=True, heading_hold_kp=1.0, heading_hold_kd=0.0,
                                  angular_deadband_radps=0.05)
    assert sh.shape(0.3, 0.02, yaw=0.5, gyro_z=0.0) == 0.0   # locks yaw 0.5, no error yet
    assert sh.holding and sh.yaw_ref == 0.5
    assert sh.shape(0.3, 0.0, yaw=0.45) == pytest.approx(0.05)  # drifted right: steer left
    assert sh.shape(0.3, 0.3, yaw=0.45) == 0.3               # operator steers: released
    assert not sh.holding
    assert sh.shape(0.3, 0.0, yaw=1.0) == 0.0                # re-locks at the new heading
    assert sh.yaw_ref == 1.0
    assert sh.shape(0.02, 0.0, yaw=0.9) == 0.0 and not sh.holding  # stopped: released
    sh.shape(0.3, 0.0, yaw=0.0)
    assert sh.shape(0.3, 0.0, yaw=None) == 0.0 and not sh.holding  # stale IMU: released


def test_heading_hold_clamp_damping_and_wrap():
    sh = cmd_vel_slew.DriveShaper(heading_hold=True, heading_hold_kp=2.0, heading_hold_kd=0.5,
                                  heading_hold_max_radps=0.15)
    sh.shape(0.3, 0.0, yaw=math.pi - 0.01)
    assert sh.shape(0.3, 0.0, yaw=-math.pi + 0.01) == pytest.approx(-0.04)  # across +-pi
    assert sh.shape(0.3, 0.0, yaw=0.0) == pytest.approx(0.15)              # clamped
    assert sh.shape(0.3, 0.0, yaw=math.pi - 0.01, gyro_z=0.1) == pytest.approx(-0.05)
    assert sh.shape(0.3, 0.0, yaw=math.pi - 0.01, gyro_z=-10.0) == pytest.approx(0.15)


def test_step_applies_shaper_and_resets_on_watchdog():
    sh = cmd_vel_slew.DriveShaper(heading_hold=True)
    node = _slew_node(_shaper=sh, _imu=(0.2, 0.0), _imu_time=100.0, _max_angular_accel=100.0)
    _step(node, 100.0)
    node._tgt, node._last_cmd_time = (0.3, 0, 0, 0, 0, 0.0), 100.0
    _step(node, 100.05)
    assert sh.holding and sh.yaw_ref == 0.2
    node._imu = (0.1, 0.0)
    node._imu_time = node._last_cmd_time = 100.1
    out = _step(node, 100.1)
    assert out[5] == pytest.approx(0.1)
    _step(node, 101.0)                                       # stale command: exact stop
    assert not sh.holding


def test_settings_overrides(tmp_path):
    f = tmp_path / 'robot.yaml'
    f.write_text('mowgli:\n  ros__parameters:\n    angular_trim_radps: 0.02\n'
                 '    heading_hold: true\n    heading_hold_kp: 2\n    other: 1\n'
                 '    angular_deadband_radps: "x"\n')
    assert cmd_vel_slew.settings_overrides(str(f)) == {
        'angular_trim_radps': 0.02, 'heading_hold': True, 'heading_hold_kp': 2.0}
    assert cmd_vel_slew.settings_overrides(str(tmp_path / 'missing.yaml')) == {}


# ---- 2026-10-08 false stuck latches, replayed from mows/20261008-031816 bag ----
# 1791433239.6: (cmd v, cmd w, ekf dx, ekf dy, ekf yaw, odom v) every 0.1 s, 6.0 s before the latch
_PIVOT_DITHER = [
    (0.00, -0.05, 0.016, 0.001, -1.630, 0.00),
    (0.00, -0.15, 0.018, -0.000, -1.630, 0.00),
    (0.00, -0.25, 0.020, -0.001, -1.631, 0.00),
    (0.00, -0.33, 0.006, 0.000, -1.635, -0.00),
    (0.00, -0.29, 0.009, 0.001, -1.644, -0.00),
    (0.00, -0.19, 0.009, 0.001, -1.649, 0.00),
    (0.00, -0.29, 0.012, 0.002, -1.666, -0.00),
    (0.00, -0.39, 0.012, 0.002, -1.674, -0.01),
    (0.00, -0.48, 0.014, 0.003, -1.695, -0.01),
    (0.00, -0.50, 0.014, 0.005, -1.749, -0.01),
    (0.00, -0.50, 0.011, 0.006, -1.771, 0.01),
    (0.00, -0.50, 0.011, 0.005, -1.791, 0.00),
    (0.00, -0.49, 0.012, 0.001, -1.801, -0.01),
    (0.00, -0.42, 0.002, 0.001, -1.807, -0.01),
    (0.00, -0.35, 0.005, -0.001, -1.812, 0.00),
    (0.00, -0.25, 0.004, -0.001, -1.815, -0.01),
    (0.00, -0.15, 0.007, -0.001, -1.823, -0.00),
    (0.00, -0.05, 0.007, 0.000, -1.826, -0.00),
    (0.00, 0.05, 0.007, 0.000, -1.827, 0.00),
    (0.00, -0.05, 0.007, -0.001, -1.827, 0.00),
    (0.00, -0.15, 0.007, -0.001, -1.827, 0.00),
    (0.00, -0.25, 0.008, 0.001, -1.830, 0.00),
    (0.00, -0.35, 0.008, 0.001, -1.832, -0.00),
    (0.00, -0.38, 0.005, -0.000, -1.843, 0.00),
    (0.00, -0.34, 0.006, -0.002, -1.859, -0.01),
    (0.00, -0.24, 0.006, -0.001, -1.873, 0.03),
    (0.00, -0.14, 0.005, -0.005, -1.940, 0.01),
    (0.00, -0.10, 0.001, -0.008, -1.981, -0.01),
    (0.00, -0.06, 0.001, -0.007, -1.991, 0.01),
    (0.00, 0.04, 0.001, -0.004, -2.004, -0.00),
    (0.00, 0.14, 0.002, -0.007, -2.003, -0.00),
    (0.00, 0.24, 0.002, -0.007, -2.002, -0.00),
    (0.00, 0.34, 0.005, -0.006, -1.996, -0.00),
    (0.00, 0.37, 0.002, -0.003, -1.987, -0.00),
    (0.00, 0.41, 0.003, -0.003, -1.973, -0.00),
    (0.00, 0.47, 0.003, -0.003, -1.964, 0.00),
    (0.00, 0.50, 0.002, -0.005, -1.926, -0.01),
    (0.00, 0.50, 0.004, -0.005, -1.878, -0.01),
    (0.00, 0.50, 0.007, -0.005, -1.839, -0.01),
    (0.00, 0.45, 0.008, -0.007, -1.811, -0.00),
    (0.00, 0.35, 0.007, -0.009, -1.805, -0.00),
    (0.00, 0.25, 0.004, -0.009, -1.795, -0.00),
    (0.00, 0.15, 0.004, -0.006, -1.778, 0.00),
    (0.00, 0.15, 0.006, -0.003, -1.758, 0.01),
    (0.00, 0.25, 0.008, -0.005, -1.741, 0.01),
    (0.00, 0.34, 0.011, -0.007, -1.733, -0.00),
    (0.00, 0.24, 0.011, -0.007, -1.727, 0.00),
    (0.00, 0.14, 0.011, -0.007, -1.694, 0.00),
    (0.00, 0.04, 0.013, -0.009, -1.677, -0.00),
    (0.00, -0.06, 0.014, -0.009, -1.670, -0.00),
    (0.00, -0.16, 0.015, -0.007, -1.670, -0.00),
    (0.00, -0.26, 0.014, -0.008, -1.673, -0.00),
    (0.00, -0.34, 0.013, -0.010, -1.677, 0.00),
    (0.00, -0.37, 0.008, -0.004, -1.687, -0.00),
    (0.00, -0.42, 0.008, -0.004, -1.705, 0.00),
    (0.00, -0.50, 0.008, -0.006, -1.745, -0.01),
    (0.00, -0.50, 0.008, -0.006, -1.762, 0.00),
    (0.00, -0.45, 0.006, -0.003, -1.801, 0.00),
    (0.00, -0.35, 0.008, -0.004, -1.814, -0.00),
    (0.00, -0.25, 0.009, -0.008, -1.822, -0.00),
    (0.00, -0.25, 0.012, -0.006, -1.834, 0.01),
]

# 1791430214.6: (cmd v, cmd w, ekf dx, ekf dy, ekf yaw, odom v) every 0.1 s, 6.0 s before the latch
_CREEP_SHUFFLE = [
    (0.00, -0.25, 0.005, 0.024, -0.451, -0.01),
    (0.00, -0.25, 0.007, 0.022, -0.463, 0.00),
    (0.00, -0.25, 0.010, 0.021, -0.480, -0.00),
    (0.00, -0.25, 0.008, 0.025, -0.510, -0.00),
    (0.05, -0.17, 0.004, 0.027, -0.556, 0.02),
    (0.10, -0.18, 0.002, 0.031, -0.585, 0.05),
    (0.10, -0.18, 0.003, 0.034, -0.590, 0.11),
    (0.09, -0.13, -0.003, 0.024, -0.603, 0.11),
    (0.09, -0.09, -0.002, 0.014, -0.628, 0.08),
    (0.08, -0.03, 0.007, 0.016, -0.642, 0.04),
    (0.08, -0.03, 0.014, 0.003, -0.639, 0.06),
    (0.10, 0.07, 0.018, -0.013, -0.641, 0.08),
    (0.05, 0.17, 0.021, -0.021, -0.643, 0.08),
    (0.00, 0.25, 0.025, -0.015, -0.637, 0.04),
    (0.00, 0.30, 0.033, -0.027, -0.622, 0.00),
    (0.00, 0.30, 0.039, -0.042, -0.603, -0.01),
    (0.02, 0.35, 0.047, -0.049, -0.581, 0.00),
    (0.05, 0.45, 0.049, -0.045, -0.551, 0.03),
    (0.05, 0.55, 0.046, -0.033, -0.520, 0.06),
    (0.05, 0.55, 0.054, -0.037, -0.486, 0.05),
    (0.05, 0.50, 0.062, -0.041, -0.459, 0.04),
    (0.05, 0.40, 0.076, -0.040, -0.441, 0.04),
    (0.05, 0.30, 0.088, -0.048, -0.424, 0.03),
    (0.03, 0.20, 0.092, -0.051, -0.408, 0.05),
    (-0.03, 0.10, 0.094, -0.059, -0.394, 0.01),
    (-0.05, 0.00, 0.097, -0.061, -0.387, -0.02),
    (-0.05, 0.00, 0.102, -0.060, -0.381, -0.03),
    (-0.05, 0.00, 0.101, -0.061, -0.377, -0.04),
    (-0.05, 0.00, 0.095, -0.058, -0.373, -0.04),
    (-0.05, 0.00, 0.090, -0.056, -0.370, -0.04),
    (-0.05, 0.00, 0.084, -0.056, -0.367, -0.04),
    (-0.00, 0.00, 0.080, -0.047, -0.364, -0.03),
    (0.05, 0.00, 0.076, -0.044, -0.363, -0.00),
    (0.05, 0.00, 0.076, -0.047, -0.363, 0.02),
    (0.05, 0.00, 0.078, -0.031, -0.365, 0.03),
    (0.05, 0.00, 0.081, -0.034, -0.367, 0.02),
    (0.05, 0.00, 0.082, -0.041, -0.372, 0.04),
    (0.05, 0.00, 0.091, -0.038, -0.376, 0.04),
    (0.05, 0.00, 0.092, -0.045, -0.378, 0.03),
    (-0.00, 0.00, 0.100, -0.047, -0.379, 0.03),
    (-0.05, 0.00, 0.099, -0.049, -0.379, 0.00),
    (-0.05, 0.00, 0.103, -0.054, -0.378, -0.03),
    (-0.05, 0.00, 0.107, -0.051, -0.378, -0.04),
    (-0.05, 0.00, 0.104, -0.051, -0.377, -0.03),
    (-0.05, 0.00, 0.099, -0.045, -0.376, -0.03),
    (-0.05, 0.00, 0.094, -0.045, -0.374, -0.04),
    (-0.05, 0.00, 0.090, -0.041, -0.373, -0.03),
    (-0.02, 0.00, 0.086, -0.038, -0.372, -0.04),
    (0.03, 0.00, 0.081, -0.038, -0.373, -0.02),
    (0.05, 0.00, 0.078, -0.036, -0.374, 0.02),
    (0.05, 0.00, 0.077, -0.031, -0.372, 0.03),
    (0.05, 0.00, 0.075, -0.027, -0.373, 0.03),
    (0.05, 0.00, 0.080, -0.035, -0.376, 0.03),
    (0.05, 0.00, 0.087, -0.038, -0.379, 0.04),
    (0.05, 0.00, 0.095, -0.032, -0.382, 0.03),
    (0.05, 0.00, 0.093, -0.041, -0.384, 0.04),
    (0.00, 0.00, 0.100, -0.044, -0.384, 0.03),
    (-0.05, 0.00, 0.101, -0.047, -0.384, 0.01),
    (-0.05, 0.00, 0.106, -0.047, -0.384, -0.04),
    (-0.05, 0.00, 0.108, -0.045, -0.384, -0.03),
    (-0.05, 0.00, 0.104, -0.049, -0.383, -0.03),
]

# 1791430259.7: (cmd v, cmd w, ekf dx, ekf dy, ekf yaw, odom v) every 0.1 s, 6.0 s before the latch
_REVERSE_ARC = [
    (0.10, -0.16, -0.006, 0.101, 1.671, 0.07),
    (0.10, -0.16, -0.007, 0.112, 1.661, 0.08),
    (0.10, -0.16, -0.011, 0.123, 1.643, 0.11),
    (0.10, -0.17, -0.011, 0.131, 1.622, 0.08),
    (0.10, -0.16, -0.010, 0.139, 1.612, 0.08),
    (0.10, -0.16, -0.009, 0.150, 1.599, 0.11),
    (0.10, -0.16, -0.009, 0.160, 1.580, 0.10),
    (0.10, -0.16, -0.008, 0.162, 1.571, 0.07),
    (0.10, -0.16, -0.009, 0.175, 1.557, 0.06),
    (0.10, -0.17, -0.008, 0.187, 1.539, 0.10),
    (0.10, -0.18, -0.010, 0.196, 1.494, 0.12),
    (0.10, -0.18, -0.010, 0.195, 1.470, 0.12),
    (0.10, -0.17, -0.009, 0.210, 1.455, 0.07),
    (0.10, -0.18, -0.007, 0.229, 1.448, 0.07),
    (0.05, -0.25, -0.005, 0.249, 1.427, 0.11),
    (0.00, -0.26, -0.005, 0.256, 1.407, 0.06),
    (0.00, -0.27, -0.003, 0.256, 1.400, -0.01),
    (0.00, -0.26, -0.004, 0.266, 1.389, -0.01),
    (0.00, -0.30, -0.001, 0.276, 1.365, 0.01),
    (0.00, -0.30, 0.002, 0.269, 1.322, -0.00),
    (0.00, -0.25, 0.002, 0.263, 1.303, 0.01),
    (0.03, -0.20, 0.003, 0.263, 1.275, -0.01),
    (0.07, -0.19, 0.010, 0.264, 1.250, 0.01),
    (0.10, -0.16, 0.015, 0.269, 1.220, 0.07),
    (0.09, -0.14, 0.020, 0.270, 1.180, 0.11),
    (0.08, -0.10, 0.021, 0.271, 1.165, 0.08),
    (0.08, -0.08, 0.021, 0.279, 1.159, 0.06),
    (0.09, -0.11, 0.029, 0.291, 1.150, 0.08),
    (0.09, -0.11, 0.032, 0.302, 1.135, 0.10),
    (0.09, -0.14, 0.029, 0.303, 1.128, 0.10),
    (0.10, -0.16, 0.031, 0.312, 1.117, 0.07),
    (0.08, -0.25, 0.034, 0.328, 1.119, 0.09),
    (0.03, -0.20, 0.039, 0.339, 1.116, 0.09),
    (0.00, -0.30, 0.036, 0.347, 1.114, 0.03),
    (0.00, -0.30, 0.040, 0.354, 1.109, -0.01),
    (0.00, -0.30, 0.046, 0.360, 1.102, -0.01),
    (0.00, -0.30, 0.048, 0.363, 1.094, -0.01),
    (0.00, -0.30, 0.046, 0.358, 1.086, 0.02),
    (0.00, -0.30, 0.045, 0.354, 1.074, -0.03),
    (0.00, -0.30, 0.041, 0.348, 1.062, 0.00),
    (0.00, -0.30, 0.040, 0.345, 1.049, 0.03),
    (0.00, -0.30, 0.035, 0.338, 1.045, -0.00),
    (0.00, -0.30, 0.036, 0.333, 1.035, 0.02),
    (0.00, -0.30, 0.037, 0.332, 1.027, -0.01),
    (0.00, -0.30, 0.039, 0.327, 1.002, -0.01),
    (0.00, -0.30, 0.038, 0.324, 0.973, 0.00),
    (0.00, -0.30, 0.037, 0.316, 0.944, 0.02),
    (0.00, -0.30, 0.035, 0.306, 0.914, -0.01),
    (0.00, -0.30, 0.038, 0.300, 0.898, 0.01),
    (0.00, -0.30, 0.041, 0.301, 0.867, 0.01),
    (0.00, -0.30, 0.043, 0.303, 0.845, -0.00),
    (0.00, -0.30, 0.047, 0.304, 0.830, 0.00),
    (-0.05, -0.30, 0.048, 0.304, 0.808, -0.01),
    (-0.10, -0.30, 0.047, 0.304, 0.763, -0.06),
    (-0.13, -0.30, 0.046, 0.300, 0.704, -0.11),
    (-0.13, -0.30, 0.041, 0.294, 0.636, -0.13),
    (-0.13, -0.30, 0.041, 0.289, 0.600, -0.12),
    (-0.13, -0.30, 0.039, 0.280, 0.590, -0.08),
    (-0.13, -0.30, 0.034, 0.264, 0.584, -0.13),
    (-0.13, -0.30, 0.020, 0.252, 0.557, -0.15),
    (-0.13, -0.30, 0.010, 0.253, 0.508, -0.15),
]


def _replay(det, rows, x0=-20.0, y0=-1.0, dt=0.1):
    """Feed bag rows (cmd v, cmd w, ekf dx, ekf dy, ekf yaw, odom v); first event or None."""
    ev = None
    for k, (v, w, dx, dy, yaw, ov) in enumerate(rows):
        t = 100.0 + k * dt
        ev = ev or det.update(t, (v, w), (x0 + dx, y0 + dy, yaw), t, ov)
    return ev


def test_old_rule_would_latch_on_the_replays():
    """Sanity: with the path checks disabled (pre-fix rule) every replay latches. The bag's
    EKF stamps lag the live 10 Hz tick by ~0.1 s, so the pivot replay peaks at 10.1 deg
    max deviation where the live detector saw 9.9 deg: allow 10.5 deg here."""
    for rows in (_PIVOT_DITHER, _CREEP_SHUFFLE):
        det = slip_detector.StuckDetector(min_path_m=1e9, min_yaw_path_deg=1e9,
                                          min_heading_deg=10.5)
        assert (_replay(det, rows) or {}).get('reason') == 'stuck'
    det2 = slip_detector.StuckDetector()
    det2._paths = lambda: (0.0, 0.0)
    assert (_replay(det2, _REVERSE_ARC) or {}).get('reason') == 'spinning'


def test_no_false_stuck_on_dithering_pivot_that_turns():
    """33239.6: RPP rotate-to-heading flips w +-0.5 every ~1.2 s, IMU/EKF follow (yaw path
    ~39 deg in 4 s) but max deviation from the window start stays < 10 deg."""
    assert _replay(slip_detector.StuckDetector(), _PIVOT_DITHER) is None


def test_no_false_stuck_on_forward_back_creep():
    """30214.6: goal approach shuffles v +-0.05 m/s; EKF chord 0.04 m but path ~0.2 m."""
    assert _replay(slip_detector.StuckDetector(), _CREEP_SHUFFLE) is None


def test_no_false_spinning_on_reverse_arc():
    """30259.7: reverse arc v -0.13 w -0.3 turning ~45 deg; wheels 0.20 m vs EKF chord
    ~0.06 m tripped the 3x ratio, but the EKF path is ~0.3 m."""
    assert _replay(slip_detector.StuckDetector(), _REVERSE_ARC) is None


def test_real_pivot_stall_with_ekf_jitter_still_latches():
    """Yesterday's stall (w >= 0.16, no body motion) with parked RTK/EKF jitter (+-4 mm,
    +-0.05 deg) must still latch within the 4 s window."""
    import random
    rnd = random.Random(7)
    det = slip_detector.StuckDetector()
    ev, t = None, 0.0
    while t < 6.0 and ev is None:
        w = 0.16 if int(t) % 2 else -0.3
        pose = (rnd.uniform(-0.004, 0.004), rnd.uniform(-0.004, 0.004),
                math.radians(rnd.uniform(-0.05, 0.05)))
        ev = det.update(t, (0.0, w), pose, t, 0.0)
        t += 0.05
    assert ev and ev['reason'] == 'stuck' and t < 4.6
