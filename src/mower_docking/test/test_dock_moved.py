"""Charger-moved check (stock parity 2026-11-03)."""

from mower_alerts import rules
from mower_docking.dock_moved import dock_moved, notice


def test_moved_when_farther_than_warn():
    r = dock_moved((1.0, 0.6, 0.0), (1.0, 0.0, 0.0), True, True, 0.5)
    assert r['moved'] and r['distance_m'] == 0.6 and r['stored'] == [1.0, 0.0]
    inc, n = rules.parse_notice(notice(r))
    assert inc == 'dock_moved' and n.oneshot and 'appears to have moved' in n.text


def test_not_moved_within_warn():
    r = dock_moved((1.0, 0.3, 0.0), (1.0, 0.0, 0.0), True, True, 0.5)
    assert r is not None and not r['moved']


def test_not_evaluated_without_rtk_fixed_measured_pose_or_poses():
    assert dock_moved((5.0, 0.0, 0.0), (0.0, 0.0, 0.0), False, True, 0.5) is None
    assert dock_moved((5.0, 0.0, 0.0), (0.0, 0.0, 0.0), True, False, 0.5) is None
    assert dock_moved(None, (0.0, 0.0, 0.0), True, True, 0.5) is None
    assert dock_moved((5.0, 0.0, 0.0), None, True, True, 0.5) is None
    assert dock_moved((5.0, 0.0, 0.0), (0.0, 0.0, 0.0), True, True, 0.0) is None
