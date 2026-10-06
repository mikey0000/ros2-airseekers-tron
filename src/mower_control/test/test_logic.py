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
        _step(node, 0.05 * k)
    applied = [t for lvl, t in node.log.lines if t.startswith('cmd_vel applied')]
    assert len(applied) == 2
    node = _slew_node(_log_period=0.0)
    _step(node, 0.0)
    for k in range(1, 41):
        _step(node, 0.05 * k)
    assert len([t for _l, t in node.log.lines if t.startswith('cmd_vel applied')]) == 40


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
