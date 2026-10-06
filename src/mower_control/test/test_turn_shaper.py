"""cmd_vel_slew both-wheel turn shaper: no one-wheel turns (pivots and arcs are legal)."""

import pytest

from mower_control.cmd_vel_slew import (TRACK_WIDTH_M, TurnShaper, shape_for_both_wheels)

DT = 0.05
HALF = TRACK_WIDTH_M / 2.0


def wheels(v, w):
    return v - w * HALF, v + w * HALF     # (right/inner for w>0 ... left) signed wheel speeds


def test_track_width_is_urdf_value():
    assert TRACK_WIDTH_M == pytest.approx(0.48)


def test_pivot_passes_as_pivot():
    assert shape_for_both_wheels(0.0, 0.3) == (0.0, 0.3, None)
    assert shape_for_both_wheels(0.0, -0.25) == (0.0, -0.25, None)


def test_slow_pivot_bumped_to_proper_pivot():
    v, w, info = shape_for_both_wheels(0.0, 0.1)
    assert v == 0.0 and w == pytest.approx(0.25) and info['kind'] == 'pivot'
    assert info['inner'] == pytest.approx(-0.06)
    v, w, _ = shape_for_both_wheels(0.0, -0.05)
    assert v == 0.0 and w == pytest.approx(-0.25)


def test_one_wheel_turn_snapped_to_nearer_regime():
    # inner = 0.06 - 0.25*0.24 = 0.0 -> exactly one-wheel: snapped to a pivot
    v, w, info = shape_for_both_wheels(0.06, 0.25)
    assert info['kind'] == 'pivot' and v == 0.0 and abs(w) >= 0.25
    # inner = 0.03 - 0.3*0.24 = -0.042 -> pivot (nearer)
    v, w, info = shape_for_both_wheels(0.03, 0.3)
    assert info['kind'] == 'pivot' and v == 0.0 and w == pytest.approx(0.3)
    # inner = 0.1 - 0.25*0.24 = +0.04 -> arc (nearer)
    v, w, info = shape_for_both_wheels(0.1, 0.25)
    a, b = wheels(v, w)
    assert info['kind'] == 'arc' and v > 0 and min(a, b) == pytest.approx(0.06)


@pytest.mark.parametrize('v,w', [(0.1, 0.3), (0.15, 0.53), (0.08, 0.25), (0.25, 1.0)])
def test_tight_arcs_raised_and_outer_capped(v, w):
    sv, sw, info = shape_for_both_wheels(v, w)
    a, b = wheels(sv, sw)
    assert info is not None and sv > 0
    assert min(a, b) >= 0.06 - 1e-9 and max(a, b) <= 0.3 + 1e-9


def test_rpp_tight_arc_lowers_w_when_outer_would_exceed_max():
    sv, sw, info = shape_for_both_wheels(0.15, 0.53)
    assert sv == pytest.approx(0.18) and sw == pytest.approx(0.5)
    assert info['inner'] == pytest.approx(0.06) and info['outer'] == pytest.approx(0.3)


@pytest.mark.parametrize('v,w', [(0.3, 0.0), (0.3, 0.2), (0.2, -0.5), (-0.1, 0.0), (0.0, 0.0), (0.0, 0.3), (0.01, 0.3)])
def test_cruise_and_straight_untouched(v, w):
    assert shape_for_both_wheels(v, w) == (v, w, None)


def test_reverse_docking_leg_keeps_both_wheels_backward():
    sv, sw, info = shape_for_both_wheels(-0.035, 0.1)    # inner +0.011: reverse arc
    a, b = wheels(sv, sw)
    assert sv < 0 and sw == pytest.approx(0.1)
    assert max(a, b) == pytest.approx(-0.06) and min(a, b) < 0


def test_docking_commands_in_docking_phase():
    ts = TurnShaper()
    v, w = ts.apply(0.0, 0.25, 'RETURNING_HOME', DT)       # ALIGNING about-turn pivot
    assert (v, w) == (0.0, 0.25) and ts.status == 'pass'
    v, w = ts.apply(0.0, 0.15, 'RETURNING_HOME', DT)       # slow pivot -> bumped
    assert (v, w) == (0.0, pytest.approx(0.25)) and ts.status.startswith('pivot w=')
    v, w = ts.apply(0.08, 0.2, 'RETURNING_HOME', DT)       # inner +0.032: arc
    a, b = wheels(v, w)
    assert v > 0 and min(a, b) == pytest.approx(0.06) and ts.status.startswith('arc r=')
    v, w = ts.apply(-0.05, -0.05, 'LOW_BATTERY_DOCKING', DT)  # reverse with correction
    a, b = wheels(v, w)
    assert v < 0 and max(a, b) <= -0.06 + 1e-9
    assert ts.apply(-0.05, 0.0, 'RETURNING_HOME', DT) == (-0.05, 0.0)  # straight reverse


@pytest.mark.parametrize('phase', ['MANUAL_MOWING', 'RECORDING', 'IDLE', None])
def test_manual_and_idle_untouched(phase):
    ts = TurnShaper()
    assert ts.apply(0.0, 0.3, phase, DT) == (0.0, 0.3) and ts.status == 'pass'


def test_shape_manual_opt_in():
    ts = TurnShaper(shape_manual=True)
    assert ts.apply(0.0, 0.1, 'MANUAL_MOWING', DT)[1] == pytest.approx(0.25)


def test_long_pivot_never_converted_to_arc():
    ts = TurnShaper()
    outs = [ts.apply(0.0, 0.3, 'TRANSIT', DT) for _ in range(int(10 / DT))]
    assert all(o == (0.0, 0.3) for o in outs)


def test_reverse_arcs():
    v, w, info = shape_for_both_wheels(-0.08, -0.2)       # inner -0.032 backward -> arc
    a, b = wheels(v, w)
    assert info['kind'] == 'arc' and v < 0 and max(a, b) == pytest.approx(-0.06)
    assert shape_for_both_wheels(-0.2, 0.3) == (-0.2, 0.3, None)


def test_disabled_is_neutral():
    ts = TurnShaper(enabled=False)
    assert ts.apply(0.0, 0.1, 'TRANSIT', DT) == (0.0, 0.1)
