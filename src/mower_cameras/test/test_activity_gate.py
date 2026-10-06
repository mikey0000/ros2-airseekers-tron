# SPDX-License-Identifier: GPL-3.0-or-later
from mower_cameras import activity_gate as g


def test_low_power_only_for_idle_modes():
    assert g.is_low_power('docked_idle') and g.is_low_power('idle')
    for a in (None, '', 'manual', 'mowing', 'docking', 'bogus'):
        assert not g.is_low_power(a)


def test_capture_period():
    assert g.capture_period(1 / 15, False) == 1 / 15
    assert g.capture_period(1 / 15, True) == 1.0
    assert g.capture_period(0.0, True) == 1.0            # stereo: every frame -> 1 fps
    assert g.capture_period(1 / 15, True, gui_watching=True) == 1 / 15
    assert g.capture_period(2.0, True) == 2.0            # never faster than configured


def test_imu():
    assert g.imu_decimation(False) == 1 and g.imu_batch_wait(False) == 0.0
    assert g.imu_decimation(True) == 2 and 0 < g.imu_batch_wait(True) <= 0.2


def test_watch_fail_active_and_resume():
    seen = []
    w = g.ActivityWatch(seen.append)
    assert not w.low_power                 # no message yet -> full rate
    assert w.update('docked_idle') and w.low_power
    assert not w.update('mowing')          # leaving idle takes effect immediately
    w.update('mowing')
    assert seen == ['docked_idle', 'mowing']
