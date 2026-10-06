# SPDX-License-Identifier: GPL-3.0-or-later
from mower_gui_bridge.gui_relay import Throttle, period_for


def test_rate_cap_per_key():
    t = Throttle(period_for(2.0))
    assert t.due(0.0, 'a')
    assert not t.due(0.1, 'a')
    assert t.due(0.1, 'b')            # independent keys (cameras)
    assert t.due(0.5, 'a')
    n = sum(t.due(1.0 + i * 0.05, 'c') for i in range(40))   # 20 Hz for 2 s
    assert 4 <= n <= 5


def test_change_passes_immediately():
    t = Throttle(1.0)
    assert t.due(0.0)
    assert not t.due(0.2)
    assert t.due(0.3, changed=True)
    assert not t.due(0.5)


def test_zero_rate_is_unthrottled():
    t = Throttle(period_for(0.0))
    assert all(t.due(i * 0.001) for i in range(10))
