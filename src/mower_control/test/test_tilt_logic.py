"""tilt_logic: gravity-referenced attitude, low-pass, debounce, hysteresis, stale, status."""

import json
import math

import pytest

from mower_control import tilt_logic as tl
from mower_control.tilt_logic import TiltBands

R = math.radians


def feed(b, t0, dur, roll_deg, pitch_deg, dt=0.04):
    """Feed constant attitude at 25 Hz from t0 for dur seconds; return the final t."""
    n = int(round(dur / dt))
    t = t0
    for i in range(1, n + 1):
        t = t0 + i * dt
        b.update(t, R(roll_deg), R(pitch_deg))
    return t


def feed_bands(b, t0, dur, roll_deg, pitch_deg, dt=0.04):
    """Like feed, but returns (final t, list of band names after every sample)."""
    bands, t = [], t0
    for i in range(1, int(round(dur / dt)) + 1):
        t = t0 + i * dt
        bands.append(b.update(t, R(roll_deg), R(pitch_deg)))
    return t, bands


def _quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


# ---------------------------------------------------------------- pure helpers
@pytest.mark.parametrize('yaw', [0.0, 1.0, -2.5])
def test_quat_round_trip_ignores_yaw(yaw):
    """Roll/pitch come back from a ZYX quaternion regardless of yaw."""
    r, p = tl.quat_to_roll_pitch(*_quat(R(12), R(-20), yaw))
    assert math.degrees(r) == pytest.approx(12, abs=1e-6)
    assert math.degrees(p) == pytest.approx(-20, abs=1e-6)


def test_gravity_tilt_zero_offsets_identity():
    """With no mounting offsets the angles pass through unchanged."""
    r, p = tl.gravity_tilt(R(10), R(-15))
    assert math.degrees(r) == pytest.approx(10, abs=1e-6)
    assert math.degrees(p) == pytest.approx(-15, abs=1e-6)


def test_gravity_tilt_roll_offset_180_keeps_pitch():
    """A 180 deg roll offset maps sensor roll 175 to body roll -5 and keeps pitch."""
    r, p = tl.gravity_tilt(R(175), R(10), mount_roll=R(180))
    assert math.degrees(r) == pytest.approx(-5, abs=1e-6)
    assert math.degrees(p) == pytest.approx(10, abs=1e-6)


def test_gravity_tilt_small_offsets_are_added():
    """Small mounting offsets are added to roll / pitch."""
    r, _ = tl.gravity_tilt(R(-2), 0.0, mount_roll=R(2))
    assert math.degrees(r) == pytest.approx(0, abs=1e-6)
    _, p = tl.gravity_tilt(0.0, R(5), mount_pitch=R(3))
    assert math.degrees(p) == pytest.approx(8, abs=1e-6)


# ---------------------------------------------------------------- band behaviour
def test_level_ground_is_ok():
    """Level ground stays 'ok' at level 0."""
    b = TiltBands()
    t = feed(b, 0.0, 3.0, 0, 0)
    assert b.band(t) == 'ok' and b.status(t)['level'] == 0


def test_caution_needs_debounce_and_names_axis():
    """Sustained roll 17 enters 'caution' only after the 1 s debounce (plus filter settling)."""
    b = TiltBands()
    t = feed(b, 0.0, 1.0, 0, 0)
    t = feed(b, t, 0.8, 17, 0)
    assert b.band(t) == 'ok'
    t = feed(b, t, 0.8, 17, 0)
    assert b.band(t) == 'caution' and b.axis == 'roll'


def test_root_bump_roll_spike_never_leaves_ok():
    """A 0.2 s roll spike to 35 deg never leaves 'ok'."""
    b = TiltBands()
    t = feed(b, 0.0, 1.0, 0, 0)
    t, b1 = feed_bands(b, t, 0.2, 35, 0)
    t, b2 = feed_bands(b, t, 3.0, 0, 0)
    assert set(b1 + b2) == {'ok'}


def test_root_bump_pitch_spike_stays_ok():
    """A 0.4 s pitch spike to 28 deg on level ground never leaves 'ok'."""
    b = TiltBands()
    t = feed(b, 0.0, 1.0, 0, 0)
    t, b1 = feed_bands(b, t, 0.4, 0, 28)
    t, b2 = feed_bands(b, t, 3.0, 0, 0)
    assert set(b1 + b2) == {'ok'}


def test_escalation_straight_to_critical_skips_caution_debounce():
    """A step to roll 33 reaches 'critical' on its own 0.3 s debounce, faster than caution's 1 s."""
    b = TiltBands()
    t = feed(b, 0.0, 1.0, 0, 0)
    t1 = feed(b, t, 0.96, 33, 0)
    # filter reaches 30 deg at ~0.6 s, + 0.3 s debounce: well under the 1.0 s caution debounce
    assert b.band(t1) == 'critical' and b.axis == 'roll'


def test_pitch_and_roll_have_separate_thresholds():
    """Pitch 20 -> caution (axis pitch); roll 20 -> caution; roll 23 -> limit; pitch 23 -> caution."""
    def run(roll, pitch):
        b = TiltBands()
        t = feed(b, 0.0, 1.0, 0, 0)
        t = feed(b, t, 4.0, roll, pitch)
        return b.band(t), b.axis
    assert run(0, 20) == ('caution', 'pitch')
    assert run(20, 0) == ('caution', 'roll')
    assert run(23, 0) == ('limit', 'roll')
    assert run(0, 23)[0] == 'caution'
    assert run(-23, 0) == ('limit', 'roll')
    assert run(0, -20) == ('caution', 'pitch')


def test_hysteresis_and_staged_release():
    """'limit' holds above 22-3, releases to caution after clear_s, then to ok after another clear_s."""
    b = TiltBands()
    t = feed(b, 0.0, 1.0, 0, 0)
    t = feed(b, t, 4.0, 23, 0)
    assert b.band(t) == 'limit'
    t = feed(b, t, 5.0, 20.5, 0)
    assert b.band(t) == 'limit'
    t = feed(b, t, 1.5, 18, 0)
    assert b.band(t) == 'limit'
    t = feed(b, t, 1.5, 18, 0)
    assert b.band(t) == 'caution'
    t = feed(b, t, 1.0, 5, 0)
    assert b.band(t) == 'caution'
    t = feed(b, t, 2.0, 5, 0)
    assert b.band(t) == 'ok'


def test_release_timer_restarts_inside_hysteresis_band():
    """Re-entering the hysteresis band (>= 12 deg) restarts the clear_s release timer."""
    b = TiltBands()
    t = feed(b, 0.0, 1.0, 0, 0)
    t = feed(b, t, 3.0, 17, 0)
    assert b.band(t) == 'caution'
    t = feed(b, t, 1.5, 10, 0)
    t = feed(b, t, 0.5, 14, 0)
    t = feed(b, t, 1.5, 10, 0)
    assert b.band(t) == 'caution'
    t = feed(b, t, 1.0, 10, 0)
    assert b.band(t) == 'ok'


# ---------------------------------------------------------------- stale
def test_stale_gives_unknown_and_level_minus_one():
    """No sample for > stale_s -> band 'unknown', stale True, level -1."""
    b = TiltBands()
    t = feed(b, 0.0, 1.0, 0, 0)
    assert b.band(t + 0.4) == 'ok'
    st = b.status(t + 0.6)
    assert b.band(t + 0.6) == 'unknown'
    assert st['stale'] is True and st['level'] == -1 and st['band'] == 'unknown'


def test_new_sample_after_gap_restarts_filter():
    """After a stale gap the filter restarts at the new value (no smoothing from the old one)."""
    b = TiltBands()
    t = feed(b, 0.0, 1.0, 0, 0)
    b.update(t + 1.0, R(25), 0.0)
    assert b.roll == pytest.approx(25, abs=1e-6)


# ---------------------------------------------------------------- inverted board
def _alternating(b, n=100, dt=0.04):
    t = 0.0
    for i in range(n):
        t = i * dt
        b.update(t, R(179 if i % 2 == 0 else -179), 0.0)
    return t


def test_inverted_board_without_offset_is_critical():
    """An inverted board jittering +-179 deg keeps |roll| > 150 (wrap-safe), inverted(), critical."""
    b = TiltBands()
    t = _alternating(b)
    assert abs(b.roll) > 150 and b.inverted()
    assert b.band(t) == 'critical'


def test_inverted_board_with_180_offset_is_ok():
    """The same input with mount_roll_offset_deg=180 reads level."""
    b = TiltBands(mount_roll_offset_deg=180)
    t = _alternating(b)
    assert not b.inverted() and b.band(t) == 'ok'


# ---------------------------------------------------------------- status / ctor
def test_status_keys_and_json():
    """status() has the documented keys and is JSON-serializable."""
    b = TiltBands()
    t = feed(b, 0.0, 1.0, 3, 2)
    st = b.status(t)
    assert set(st) == {'band', 'level', 'roll_deg', 'pitch_deg', 'raw_roll_deg',
                       'raw_pitch_deg', 'axis', 'stale', 'imu_age_s', 'since_s',
                       'thresholds'}
    assert set(st['thresholds']) == {'roll', 'pitch', 'hysteresis'}
    json.dumps(st)
    json.dumps(TiltBands().status(0.0))   # before any sample (None values)


def test_constructor_validation():
    """Non-increasing thresholds raise ValueError, unknown kwargs KeyError."""
    with pytest.raises(ValueError):
        TiltBands(roll_thresholds_deg=(20.0, 20.0, 30.0))
    with pytest.raises(ValueError):
        TiltBands(pitch_thresholds_deg=(30.0, 25.0, 18.0))
    with pytest.raises(KeyError):
        TiltBands(bogus=1)
