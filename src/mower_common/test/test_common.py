"""mower_common angle helpers and phases (pure python)."""

import math

import pytest

from mower_common import angles, phases


@pytest.mark.parametrize('a', [0.0, 1.0, -1.0, math.pi, -math.pi, 3 * math.pi, -7.5, 12.0])
def test_wrap_range(a):
    for f in (angles.wrap_angle, angles.wrap_angle_trig):
        w = f(a)
        assert -math.pi - 1e-12 <= w <= math.pi + 1e-12
        assert math.isclose(math.cos(w), math.cos(a), abs_tol=1e-9)
        assert math.isclose(math.sin(w), math.sin(a), abs_tol=1e-9)


def test_yaw_from_quaternion():
    for yaw in (-3.0, -1.0, 0.0, 0.5, 2.5):
        q = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
        assert angles.yaw_from_quaternion(*q) == pytest.approx(yaw)


def test_phases():
    assert set(phases.DOCK_PHASES) <= set(phases.MOTION_PHASES)
    assert set(phases.DOCKED_MOTION_PHASES) <= set(phases.MOTION_PHASES)
    assert 'IDLE' not in phases.MOTION_PHASES and 'CHARGING' not in phases.MOTION_PHASES
