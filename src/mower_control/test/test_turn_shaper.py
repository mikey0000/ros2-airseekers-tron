"""cmd_vel_slew both-wheel turn shaper: no one-wheel turns, no counter-rotating pivots."""

import pytest

from mower_control.cmd_vel_slew import (TRACK_WIDTH_M, TurnShaper, shape_for_both_wheels)

DT = 0.05
HALF = TRACK_WIDTH_M / 2.0


def wheels(v, w):
    return v - w * HALF, v + w * HALF     # (right/inner for w>0 ... left) signed wheel speeds


def test_track_width_is_urdf_value():
    assert TRACK_WIDTH_M == pytest.approx(0.48)


def test_pivot_becomes_forward_arc_with_inner_at_min():
    v, w, info = shape_for_both_wheels(0.0, 0.3)
    a, b = wheels(v, w)
    assert v > 0 and w == pytest.approx(0.3)
    assert min(a, b) == pytest.approx(0.06) and max(a, b) <= 0.3 + 1e-9
    assert info['r'] == pytest.approx(v / 0.3)
    v2, w2, _ = shape_for_both_wheels(0.0, -0.3)
    assert v2 == pytest.approx(v) and w2 == pytest.approx(-0.3)


@pytest.mark.parametrize('v,w', [(0.05, 0.3), (0.15, 0.53), (0.03, 0.25), (0.0, 1.0)])
def test_tight_arcs_raised_and_outer_capped(v, w):
    sv, sw, info = shape_for_both_wheels(v, w)
    a, b = wheels(sv, sw)
    assert info is not None and sv > 0
    assert min(a, b) >= 0.06 - 1e-9 and max(a, b) <= 0.3 + 1e-9


def test_rpp_tight_arc_lowers_w_when_outer_would_exceed_max():
    sv, sw, info = shape_for_both_wheels(0.15, 0.53)
    assert sv == pytest.approx(0.18) and sw == pytest.approx(0.5)
    assert info['inner'] == pytest.approx(0.06) and info['outer'] == pytest.approx(0.3)


@pytest.mark.parametrize('v,w', [(0.3, 0.0), (0.3, 0.2), (0.2, -0.5), (-0.1, 0.0), (0.0, 0.0)])
def test_cruise_and_straight_untouched(v, w):
    assert shape_for_both_wheels(v, w) == (v, w, None)


def test_reverse_docking_leg_keeps_both_wheels_backward():
    sv, sw, info = shape_for_both_wheels(-0.035, 0.1)
    a, b = wheels(sv, sw)
    assert sv < 0 and sw == pytest.approx(0.1)
    assert max(a, b) == pytest.approx(-0.06) and min(a, b) < 0


def test_docking_commands_in_docking_phase():
    ts = TurnShaper()
    v, w = ts.apply(0.03, 0.25, 'RETURNING_HOME', DT)      # ALIGNING creep
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
    assert ts.apply(0.0, 0.3, 'MANUAL_MOWING', DT)[0] > 0


def test_pivot_cap_then_true_pivot_fallback_then_arc_again():
    ts = TurnShaper()
    outs = [ts.apply(0.0, 0.3, 'TRANSIT', DT) for _ in range(int(10 / DT))]
    dist = 0.0
    for i, (v, _w) in enumerate(outs):
        if v == 0.0:
            break
        dist += v * DT
    assert dist == pytest.approx(0.3, abs=0.01)
    # ~2 s of true pivot follows ...
    fb = 0
    while i + fb < len(outs) and outs[i + fb][0] == 0.0:
        fb += 1
    assert fb == pytest.approx(2.0 / DT, abs=2)
    # ... then arcing resumes with a fresh budget.
    assert outs[i + fb][0] > 0


def test_non_pivot_command_resets_the_cap():
    ts = TurnShaper()
    for _ in range(200):
        ts.apply(0.0, 0.3, 'MOWING', DT)
    ts.apply(0.3, 0.0, 'MOWING', DT)
    assert ts.apply(0.0, 0.3, 'MOWING', DT)[0] > 0


def test_disabled_is_neutral():
    ts = TurnShaper(enabled=False)
    assert ts.apply(0.0, 0.3, 'TRANSIT', DT) == (0.0, 0.3)
