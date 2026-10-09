# SPDX-License-Identifier: GPL-3.0-or-later
from mower_gui_bridge.gui_relay import ChangeOrKeepalive, LatestByName, Throttle, period_for


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


def test_change_or_keepalive_first_value_and_changes_pass_at_once():
    g = ChangeOrKeepalive(period_for(1.0))
    assert g.due(False, 0.0)                 # first value
    assert not g.due(False, 0.08)            # 13 Hz repeats are dropped
    assert g.due(True, 0.15)                 # a change goes out immediately
    assert g.due(False, 0.2)                 # and the change back too
    assert not g.due(False, 0.5)


def test_change_or_keepalive_rate_on_a_steady_13hz_source():
    g = ChangeOrKeepalive(period_for(1.0))
    n = sum(g.due(False, i / 13.0) for i in range(13 * 10))    # 10 s of the same value
    assert 10 <= n <= 11


def test_change_or_keepalive_after_a_gap_sends_at_once():
    g = ChangeOrKeepalive(period_for(1.0))
    assert g.due(True, 0.0)
    assert g.due(True, 30.0)                 # GUI resubscribed after a pause


class _St:
    def __init__(self, name, level):
        self.name, self.level = name, level


def test_latest_by_name_merges_and_clears():
    m = LatestByName()
    m.add('gps', _St('gps', 0))
    m.add('imu', _St('imu', 0))
    m.add('gps', _St('gps', 2))              # latest wins, first-seen order kept
    assert len(m) == 2
    out = m.take()
    assert [(s.name, s.level) for s in out] == [('gps', 2), ('imu', 0)]
    assert m.take() == [] and len(m) == 0    # only what updated since the last take


def test_latest_by_name_rate_reduction_9hz_to_1hz():
    m = LatestByName()
    names = ['a', 'b', 'c', 'd', 'e', 'f']
    published = 0
    for tick in range(10):                   # 10 s, 9 arrays/s across 6 publishers
        for k in range(9):
            n = names[(tick * 9 + k) % 6]
            m.add(n, _St(n, 0))
        if m.take():
            published += 1
    assert published == 10
