"""Front camera blocked monitor (stock parity 2026-11-03)."""

import datetime

import numpy as np

from mower_vision.camera_blocked import BlockedMonitor, image_stats, is_daylight


def mono(img):
    img = np.asarray(img, dtype=np.uint8)
    return img.tobytes(), img.shape[1], img.shape[0], img.shape[1], 'mono8'


def test_stats_mono_and_bgr():
    rng = np.random.default_rng(0)
    grass = rng.integers(40, 200, size=(64, 80), dtype=np.uint8)
    mean, std = image_stats(*mono(grass), decimate=4)
    assert 100 < mean < 140 and std > 30
    flat = np.full((64, 80, 3), 20, dtype=np.uint8)
    assert image_stats(flat.tobytes(), 80, 64, 240, 'bgr8') == (20.0, 0.0)
    assert image_stats(b'\x00' * 10, 80, 64, 80, 'mono8') is None      # short buffer
    assert image_stats(flat.tobytes(), 80, 64, 240, 'jpeg') is None


def test_daylight_window():
    noon = datetime.datetime(2026, 11, 3, 12, 0)
    night = datetime.datetime(2026, 11, 3, 22, 0)
    assert is_daylight([8.0, 18.0], noon) and not is_daylight([8.0, 18.0], night)
    assert is_daylight([], night)


def test_blocked_after_hold_and_clears_on_recovery():
    m = BlockedMonitor(blocked_s=10.0, clear_s=3.0)
    covered, good = (90.0, 2.0), (110.0, 40.0)
    assert not m.update(0.0, covered, True)
    assert not m.update(9.0, covered, True) and not m.blocked
    assert m.update(10.0, covered, True) and m.blocked and 'uniform' in m.why
    assert not m.update(11.0, good, True) and m.blocked              # clear_s hold
    assert not m.update(12.0, None, True) and m.blocked              # no frame: hold
    assert m.update(14.0, good, True) and not m.blocked


def test_dark_and_glitch_frames():
    m = BlockedMonitor(blocked_s=5.0)
    m.update(0.0, (5.0, 3.0), True)
    m.update(3.0, (110.0, 40.0), True)                               # one good frame resets
    assert not m.update(7.0, (5.0, 3.0), True)
    assert m.update(12.0, (5.0, 3.0), True) and 'dark' in m.why


def test_inactive_clears_at_once():
    m = BlockedMonitor(blocked_s=1.0)
    m.update(0.0, (255.0, 0.0), True)
    m.update(2.0, (255.0, 0.0), True)
    assert m.blocked
    assert m.update(3.0, None, False) and not m.blocked
