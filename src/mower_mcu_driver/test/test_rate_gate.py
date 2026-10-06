# SPDX-License-Identifier: GPL-3.0-or-later
from mower_mcu_driver.rate_gate import ChangeOrPeriodGate, activity_low_power


def test_period_and_change():
    g = ChangeOrPeriodGate()
    assert g.allow((1,), 0.0, 0.1)
    assert not g.allow((1,), 0.05, 0.1)
    assert g.allow((2,), 0.06, 0.1)          # change goes through immediately
    assert not g.allow((2,), 0.10, 0.1)
    assert g.allow((2,), 0.17, 0.1)


def test_zero_period_always():
    g = ChangeOrPeriodGate()
    assert all(g.allow((1,), t * 0.01, 0.0) for t in range(5))


def test_activity():
    assert activity_low_power('idle') and activity_low_power('docked_idle')
    for a in (None, '', 'manual', 'mowing', 'docking', 'x'):
        assert not activity_low_power(a)
