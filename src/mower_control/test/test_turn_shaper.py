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


def test_slow_pivot_passes_unchanged_with_pure_pivot_off():
    # The 2026-10-07 rule, now behind turn_shaper_pure_pivot: false.
    p = {'pure_pivot': False}
    assert shape_for_both_wheels(0.0, 0.1, p) == (0.0, 0.1, None)
    assert shape_for_both_wheels(0.0, -0.02, p) == (0.0, -0.02, None)


@pytest.mark.parametrize('w', [0.1, -0.05, 0.03, 0.24])
def test_slow_pure_rotation_snaps_to_pivot_floor(w):
    """Fix 8: v = 0 with 0 < |w| < 0.25 left both wheels under the MCU deadband."""
    v, sw, info = shape_for_both_wheels(0.0, w)
    assert v == 0.0 and sw == pytest.approx(0.25 if w > 0 else -0.25)
    assert info['kind'] == 'pivot' and info['inner'] == pytest.approx(-0.06)


@pytest.mark.parametrize('w', [0.029, -0.01, 1e-5])
def test_pure_rotation_below_deadband_is_a_stop(w):
    v, sw, info = shape_for_both_wheels(0.0, w)
    assert (v, sw) == (0.0, 0.0) and info['kind'] == 'pivot' and info['deadband']
    ts = TurnShaper()
    assert ts.apply(0.0, w, 'TRANSIT', DT) == (0.0, 0.0)
    assert ts.status.startswith('pivot deadband') and '-> 0' in ts.status


@pytest.mark.parametrize('w', [0.25, -0.25, 0.3, 0.5])
def test_pure_rotation_at_or_above_floor_passes(w):
    assert shape_for_both_wheels(0.0, w) == (0.0, w, None)


def test_pure_pivot_param_off_restores_pass_through():
    from mower_control.cmd_vel_slew import TURN_DEFAULTS, set_turn_param
    assert TURN_DEFAULTS['turn_shaper_pure_pivot'] is True
    assert TURN_DEFAULTS['turn_shaper_pivot_deadband_rps'] == pytest.approx(0.03)
    ts = TurnShaper()
    set_turn_param(ts, 'turn_shaper_pure_pivot', False)
    for w in (0.1, -0.02, 0.2):
        assert ts.apply(0.0, w, 'TRANSIT', DT) == (0.0, w) and ts.status == 'pass'
    ts = TurnShaper()                    # old latch rule: a pure pivot releases an arc latch
    set_turn_param(ts, 'turn_shaper_pure_pivot', False)
    ts.apply(0.1, 0.08, 'TRANSIT', DT)
    assert ts.apply(0.0, 0.08, 'TRANSIT', DT) == (0.0, 0.08) and ts.regime == 'pivot'
    set_turn_param(ts, 'turn_shaper_pivot_deadband_rps', 0.05)
    assert ts.params['pivot_deadband_rps'] == pytest.approx(0.05)


def test_both_wheels_legal_target_rule_for_pure_rotations():
    from mower_control.cmd_vel_slew import both_wheels_legal
    for w in (0.25, 0.3, 0.02, 0.0):
        assert both_wheels_legal(0.0, w, target=True)
    for w in (0.03, 0.1, 0.24):
        assert not both_wheels_legal(0.0, w, target=True)
        assert both_wheels_legal(0.0, w)          # slew ramps pass through slow rotations
    assert both_wheels_legal(0.0, 0.1, {'track_width_m': 0.48, 'min_inner_wheel_mps': 0.06,
                                        'pure_pivot': False}, target=True)


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
    # The widening geometry (task 0.4 b turns this r_req 0.35 m request into a pivot by
    # default; the docking lane is never shaped at all, see test_docking_phases_are_exempt).
    sv, sw, info = shape_for_both_wheels(-0.035, 0.1, {'pivot_instead': False})
    a, b = wheels(sv, sw)
    assert sv < 0 and sw == pytest.approx(0.1)
    assert max(a, b) == pytest.approx(-0.06) and min(a, b) < 0


@pytest.mark.parametrize('lane', [None, 'docking'])
@pytest.mark.parametrize('phase', ['RETURNING_HOME', 'LOW_BATTERY_DOCKING', 'UNDOCKING'])
def test_docking_phases_are_exempt(phase, lane):
    """The marker-guided reverse must reach the MCU as sent: shaping it into pivots
    left the robot pushing nowhere at the contacts (2026-10-07). Lane unknown = the
    pre-fix-9 phase rule; the docking lane is never shaped."""
    ts = TurnShaper()
    for v, w in ((0.0, 0.15), (0.08, 0.2), (-0.05, -0.05), (-0.035, 0.1), (-0.05, 0.0)):
        assert ts.apply(v, w, phase, DT, lane) == (v, w) and ts.status == 'pass'


def test_transit_arc_shaped():
    ts = TurnShaper()
    v, w = ts.apply(0.08, 0.2, 'TRANSIT', DT)       # inner +0.032: one-wheel zone -> arc
    a, b = wheels(v, w)
    assert v > 0 and min(a, b) == pytest.approx(0.06) and ts.status.startswith('arc r=')


@pytest.mark.parametrize('phase', ['MANUAL_MOWING', 'RECORDING', 'IDLE', None])
def test_manual_and_idle_untouched(phase):
    ts = TurnShaper()
    assert ts.apply(0.0, 0.3, phase, DT) == (0.0, 0.3) and ts.status == 'pass'


def test_shape_manual_opt_in():
    ts = TurnShaper(shape_manual=True)
    assert ts.apply(0.05, 0.3, "MANUAL_MOWING", DT) == (0.0, 0.3)  # one-wheel turn -> proper pivot, rate kept


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


# ------------------------------------------------------------- regime latch (2026-10-14)
def _regime(v):
    return 'pivot' if v == 0.0 else 'arc'


def test_stateless_band_flips_between_pivot_and_arc():
    """The bug the latch fixes: at |w| = 0.3, 0.070 -> pivot and 0.075 -> arc."""
    assert shape_for_both_wheels(0.070, 0.3)[2]['kind'] == 'pivot'
    assert shape_for_both_wheels(0.075, 0.3)[2]['kind'] == 'arc'
    ts = TurnShaper(hysteresis_mps=0.0)                 # latch off = old behaviour
    outs = [ts.apply(v, 0.3, 'TRANSIT', DT)[0] for v in (0.070, 0.075) * 10]
    assert len({_regime(v) for v in outs}) == 2


@pytest.mark.parametrize('start', [0.070, 0.075])
def test_oscillating_band_gives_a_stable_regime(start):
    ts = TurnShaper()
    other = 0.145 - start
    seq = [start, other] * 40 + [start, other, start + 0.001, other - 0.001] * 10
    outs = [ts.apply(v, 0.3, 'MOWING', DT) for v in seq]
    regimes = {_regime(v) for v, _w in outs}
    assert regimes == {_regime(shape_for_both_wheels(start, 0.3)[0])}
    for v, w in outs:                                    # always a legal both-wheel command
        a, b = wheels(v, w)
        assert v == 0.0 or min(abs(a), abs(b)) >= 0.06 - 1e-9


def test_latch_releases_past_the_hysteresis():
    ts = TurnShaper(hysteresis_mps=0.02, latch_ticks=0)
    assert ts.apply(0.070, 0.3, 'TRANSIT', DT)[0] == 0.0        # pivot (inner -0.002)
    assert ts.apply(0.085, 0.3, 'TRANSIT', DT)[0] == 0.0        # inner +0.013 < h: held
    assert 'held' in ts.status
    v, _w = ts.apply(0.095, 0.3, 'TRANSIT', DT)                  # inner +0.023 >= h: arc
    assert v == pytest.approx(0.132) and ts.regime == 'arc'
    v, _w = ts.apply(0.060, 0.3, 'TRANSIT', DT)                  # inner -0.012 > -h: held
    assert v == pytest.approx(0.132) and 'held' in ts.status
    assert ts.apply(0.050, 0.3, 'TRANSIT', DT)[0] == 0.0         # inner -0.022: pivot


def test_latch_releases_after_n_ticks_of_the_other_regime():
    ts = TurnShaper(hysteresis_mps=0.02, latch_ticks=6)
    ts.apply(0.070, 0.3, 'TRANSIT', DT)
    outs = [ts.apply(0.075, 0.3, 'TRANSIT', DT)[0] for _ in range(8)]
    assert outs[:5] == [0.0] * 5 and outs[5] == pytest.approx(0.132)


def test_latched_legal_arc_held_against_a_slow_pivot_command():
    """A legal pass-through arc also latches the arc regime."""
    ts = TurnShaper()
    assert ts.apply(0.14, 0.3, 'TRANSIT', DT) == (0.14, 0.3)
    v, w = ts.apply(0.07, 0.3, 'TRANSIT', DT)
    assert v == pytest.approx(0.132) and w == pytest.approx(0.3)


@pytest.mark.parametrize('nxt', [(0.0, 0.3), (0.2, 0.0), (0.07, -0.3), (-0.07, 0.3)])
def test_pure_pivot_straight_or_sign_change_release_the_latch(nxt):
    ts = TurnShaper()
    ts.apply(0.14, 0.3, 'TRANSIT', DT)                           # arc latched
    v, w = ts.apply(*nxt, 'TRANSIT', DT)
    assert (v, w) == shape_for_both_wheels(*nxt)[:2]


def test_latch_params_are_node_params():
    from mower_control.cmd_vel_slew import TURN_DEFAULTS, TURN_PARAMS, set_turn_param
    assert TURN_DEFAULTS['turn_shaper_hysteresis_mps'] == pytest.approx(0.02)
    assert TURN_DEFAULTS['turn_shaper_latch_ticks'] == 6
    ts = TurnShaper()
    set_turn_param(ts, 'turn_shaper_hysteresis_mps', 0.0)
    set_turn_param(ts, 'turn_shaper_latch_ticks', 3)
    assert ts.hysteresis_mps == 0.0 and ts.latch_ticks == 3 and set(TURN_PARAMS) >= {
        'turn_shaper_hysteresis_mps', 'turn_shaper_latch_ticks'}


# ------------------------------------------------ pure rotations join the pivot regime (fix 8)
def _field_dither(ts, phase='MOWING', seconds=2.0):
    seq = [(0.010, -0.10), (0.000, -0.10)] * int(seconds / DT / 2)
    return [ts.apply(v, w, phase, DT) for v, w in seq], seq


def test_field_dither_is_one_constant_pivot():
    """2026-10-14 mow, t 51-64: MPPI alternated (0.010, -0.10) / (0.000, -0.10) and
    /cmd_vel flipped 0.25 / 0.10 every ~0.3 s. Now: a constant 0.25 rad/s pivot."""
    ts = TurnShaper()
    outs, _ = _field_dither(ts)
    assert len(outs) == 40
    assert all(o == (0.0, pytest.approx(-0.25)) for o in outs)
    assert ts.regime == 'pivot'
    regimes = []
    ts = TurnShaper()
    for v, w in [(0.010, -0.10), (0.000, -0.10)] * 20:
        ts.apply(v, w, 'MOWING', DT)
        regimes.append(ts.regime)
    assert set(regimes) == {'pivot'}


def test_field_dither_with_pure_pivot_off_hunts():
    ts = TurnShaper(pure_pivot=False)
    outs, _ = _field_dither(ts)
    assert {round(abs(w), 2) for _v, w in outs} == {0.25, 0.1}


def test_latched_arc_held_when_v_drops_to_zero_briefly():
    ts = TurnShaper()                                 # h 0.02, latch_ticks 6
    assert ts.apply(0.1, 0.08, 'TRANSIT', DT) == (0.1, 0.08)      # legal arc, latched
    held = [ts.apply(0.0, 0.08, 'TRANSIT', DT) for _ in range(5)]  # inner -0.019 > -h
    for v, w in held:
        assert v == pytest.approx(0.06 + 0.08 * HALF) and w == pytest.approx(0.08)
    assert ts.regime == 'arc' and 'held' in ts.status
    assert ts.apply(0.1, 0.08, 'TRANSIT', DT) == (0.1, 0.08)      # back to the arc
    for _ in range(5):
        ts.apply(0.0, 0.08, 'TRANSIT', DT)
    assert ts.apply(0.0, 0.08, 'TRANSIT', DT) == (0.0, pytest.approx(0.25))  # 6th: pivot
    assert ts.regime == 'pivot'


def test_reverse_arc_held_backwards_when_v_drops_to_zero():
    ts = TurnShaper()
    ts.apply(-0.1, 0.08, 'TRANSIT', DT)
    v, w = ts.apply(0.0, 0.08, 'TRANSIT', DT)
    assert v == pytest.approx(-(0.06 + 0.08 * HALF)) and w == pytest.approx(0.08)


def test_v_sign_flip_through_zero_still_releases():
    ts = TurnShaper()
    ts.apply(0.1, 0.08, 'TRANSIT', DT)                # forward arc
    ts.apply(0.0, 0.08, 'TRANSIT', DT)                # held, still forward
    assert ts.apply(-0.1, 0.08, 'TRANSIT', DT) == (-0.1, 0.08)    # reverse: released


def test_w_sign_change_releases_the_pivot_latch():
    ts = TurnShaper()
    for _ in range(4):
        assert ts.apply(0.0, -0.10, 'MOWING', DT) == (0.0, pytest.approx(-0.25))
    assert ts.apply(0.0, 0.10, 'MOWING', DT) == (0.0, pytest.approx(0.25))
    ts = TurnShaper()
    ts.apply(0.1, 0.08, 'TRANSIT', DT)                # arc latched
    assert ts.apply(0.0, -0.08, 'TRANSIT', DT) == (0.0, pytest.approx(-0.25))


def test_pivot_latch_released_by_a_real_arc():
    ts = TurnShaper()
    ts.apply(0.0, 0.1, 'MOWING', DT)
    assert ts.apply(0.15, 0.1, 'MOWING', DT) == (0.15, 0.1)        # inner +0.126 >= h


def test_deadband_releases_the_latch():
    ts = TurnShaper()
    ts.apply(0.0, 0.1, 'MOWING', DT)
    assert ts.apply(0.0, 0.01, 'MOWING', DT) == (0.0, 0.0) and ts.regime is None


# ------------------------------------------ return transit shaped on the nav lane (fix 9)
DOCK_PHASES = ['RETURNING_HOME', 'LOW_BATTERY_DOCKING', 'RAIN_DETECTED_DOCKING',
               'COVERAGE_FAILED_DOCKING']
ALL_PHASES = DOCK_PHASES + ['TRANSIT', 'MOWING', 'BOUNDARY_PAUSED', 'UNDOCKING',
                            'MANUAL_MOWING', 'RECORDING', 'IDLE', None]


@pytest.mark.parametrize('phase', DOCK_PHASES)
def test_return_transit_on_nav_lane_is_shaped(phase):
    """2026-10-14 mow t ~ 2424: (0.016, 0.09) on the nav lane in RETURNING_HOME drove
    151 one-wheel ticks unshaped."""
    ts = TurnShaper()
    assert ts.is_active(phase, 'navigation')
    assert ts.apply(0.016, 0.09, phase, DT, 'navigation') == (0.0, pytest.approx(0.25))
    assert ts.status.startswith('pivot')


@pytest.mark.parametrize('phase', DOCK_PHASES)
def test_same_command_on_docking_lane_is_not_shaped(phase):
    ts = TurnShaper()
    assert not ts.is_active(phase, 'docking')
    assert ts.apply(0.016, 0.09, phase, DT, 'docking') == (0.016, 0.09)
    assert ts.status == 'pass'


@pytest.mark.parametrize('phase', ['TRANSIT', 'MOWING'])
@pytest.mark.parametrize('lane', [None, 'navigation'])
def test_transit_and_mowing_unchanged(phase, lane):
    ts = TurnShaper()
    assert ts.apply(0.08, 0.2, phase, DT, lane) == TurnShaper().apply(0.08, 0.2, phase, DT)
    assert ts.status.startswith('arc r=')


@pytest.mark.parametrize('phase', ALL_PHASES)
def test_docking_lane_reverse_never_shaped(phase):
    for shape_manual in (False, True):
        ts = TurnShaper(shape_manual=shape_manual)
        for _ in range(20):
            assert ts.apply(-0.05, 0.1, phase, DT, 'docking') == (-0.05, 0.1)
        assert ts.status == 'pass'


@pytest.mark.parametrize('lane', ['bumper', 'emergency', 'teleop', 'docking'])
@pytest.mark.parametrize('phase', ['TRANSIT', 'MOWING', 'RETURNING_HOME'])
def test_other_lanes_never_shaped(lane, phase):
    ts = TurnShaper()
    assert ts.apply(0.05, 0.3, phase, DT, lane) == (0.05, 0.3)


def test_teleop_lane_shaped_only_with_manual_opt_in():
    assert TurnShaper(shape_manual=True).apply(0.05, 0.3, 'MANUAL_MOWING', DT,
                                               'teleop') == (0.0, 0.3)
    assert TurnShaper().apply(0.05, 0.3, 'MANUAL_MOWING', DT, 'teleop') == (0.05, 0.3)


def test_lane_switch_to_docking_drops_the_latch():
    ts = TurnShaper()
    ts.apply(0.0, 0.1, 'RETURNING_HOME', DT, 'navigation')
    assert ts.regime == 'pivot'
    assert ts.apply(-0.05, 0.1, 'RETURNING_HOME', DT, 'docking') == (-0.05, 0.1)
    assert ts.regime is None and ts.status == 'pass'


def test_dock_transit_param_off_restores_phase_exemption():
    from mower_control.cmd_vel_slew import TURN_DEFAULTS, set_turn_param
    assert TURN_DEFAULTS['turn_shaper_dock_transit'] is True
    ts = TurnShaper()
    set_turn_param(ts, 'turn_shaper_dock_transit', False)
    assert ts.apply(0.016, 0.09, 'RETURNING_HOME', DT, 'navigation') == (0.016, 0.09)
    assert ts.apply(0.08, 0.2, 'TRANSIT', DT, 'navigation') != (0.08, 0.2)


# ---------------------------------------------------------------------------------------
# Task 0.4 (b), 2026-10-14 run 3: pivot instead of widening a tight corner
# ---------------------------------------------------------------------------------------
def _widened_r(v, w):
    from mower_control.cmd_vel_slew import _tight_arc
    return _tight_arc(v, w, HALF, 0.06, 0.3)[2]['r']


@pytest.mark.parametrize('v, w', [(0.04, 0.2), (0.02, 0.1), (0.03, 0.1), (0.05, 0.2),
                                  (-0.03, 0.1)])
def test_tight_corner_becomes_pivot_not_wide_arc(v, w):
    """r_req 0.2-0.3 m (MPPI's slow corners in run 3) -> pivot on the spot, both wheels
    counter-rotating at >= the 0.25 rad/s floor, instead of a 0.5-1.0 m arc."""
    sv, sw, info = shape_for_both_wheels(v, w)
    assert sv == 0.0 and abs(sw) >= 0.25 - 1e-9 and sw * w > 0
    a, b = wheels(sv, sw)
    assert a * b < 0 and min(abs(a), abs(b)) >= 0.06 - 1e-9
    assert info['kind'] == 'pivot'


def test_r_req_0_2_corner_is_a_pivot_through_the_shaper():
    ts = TurnShaper()
    out = ts.apply(0.04, 0.2, 'TRANSIT', DT, lane='navigation')    # r_req 0.2 m
    assert out[0] == 0.0 and out[1] == pytest.approx(0.25)
    out = TurnShaper().apply(0.03, 0.1, 'TRANSIT', DT, lane='navigation')   # r_req 0.3 m
    assert out[0] == 0.0 and out[1] == pytest.approx(0.25)


def test_pivot_instead_marks_the_status():
    ts = TurnShaper()
    ts.apply(0.03, 0.1, 'TRANSIT', DT, lane='navigation')
    assert ts.status.startswith('pivot') and 'instead of an arc' in ts.status


@pytest.mark.parametrize('v, w', [(0.05, 0.1), (0.07, 0.2), (0.1, 0.3), (-0.05, 0.1)])
def test_arcs_at_or_above_the_threshold_are_unchanged(v, w):
    """r_req >= 0.5 x widened radius: the same tight arc as before task 0.4 (b)."""
    assert (abs(v) / abs(w)) >= 0.5 * _widened_r(v, w)
    old = shape_for_both_wheels(v, w, {'pivot_instead': False})
    new = shape_for_both_wheels(v, w)
    assert new[:2] == pytest.approx(old[:2]) and new[2]['kind'] == 'arc'


def test_pivot_instead_off_restores_widening():
    from mower_control.cmd_vel_slew import TURN_DEFAULTS, set_turn_param
    assert TURN_DEFAULTS['turn_shaper_pivot_instead'] is True
    assert TURN_DEFAULTS['turn_shaper_pivot_instead_ratio'] == pytest.approx(0.5)
    ts = TurnShaper()
    set_turn_param(ts, 'turn_shaper_pivot_instead', False)
    v, w = ts.apply(0.03, 0.1, 'TRANSIT', DT, lane='navigation')
    assert v > 0.0 and ts.status.startswith('arc r=')
    assert v / w == pytest.approx(_widened_r(0.03, 0.1))


def test_ratio_param_moves_the_threshold():
    from mower_control.cmd_vel_slew import set_turn_param
    ts = TurnShaper()
    set_turn_param(ts, 'turn_shaper_pivot_instead_ratio', 0.2)
    v, w = ts.apply(0.03, 0.1, 'TRANSIT', DT, lane='navigation')     # q = 0.3 / 0.84
    assert v > 0.0 and ts.status.startswith('arc r=')


def _q(v, w):
    return (abs(v) / abs(w)) / _widened_r(v, w)


def test_no_chatter_across_the_threshold():
    """A controller hovering just either side of the ratio (q 0.46 / 0.54) keeps one
    regime: the latch holds across ratio +- hysteresis."""
    w = 0.1
    below, above = 0.46 * _widened_r(0.03, w) * w, 0.54 * _widened_r(0.03, w) * w
    assert _q(below, w) < 0.5 < _q(above, w) < 0.6
    for first in (below, above):
        ts = TurnShaper(latch_ticks=0)
        kinds = set()
        for k in range(40):
            v = first if k % 2 == 0 else (above if first == below else below)
            out = ts.apply(v, w, 'TRANSIT', DT, lane='navigation')
            kinds.add('pivot' if out[0] == 0.0 else 'arc')
        assert len(kinds) == 1


def test_latch_leaves_the_pivot_once_clearly_past_the_band():
    w = 0.1
    ts = TurnShaper(latch_ticks=0)
    ts.apply(0.03, w, 'TRANSIT', DT, lane='navigation')               # q 0.36: pivot
    assert ts.regime == 'pivot'
    v_far = 0.65 * _widened_r(0.05, w) * w                            # q 0.65 > 0.6
    out = ts.apply(v_far, w, 'TRANSIT', DT, lane='navigation')
    assert out[0] > 0.0 and ts.regime == 'arc'
    v_low = 0.35 * _widened_r(0.03, w) * w                            # q 0.35 < 0.4
    out = ts.apply(v_low, w, 'TRANSIT', DT, lane='navigation')
    assert out[0] == 0.0 and ts.regime == 'pivot'


# --- 2026-10-15: transit pivot hold (off by default) ----------------------------------

def test_transit_pivot_hold_is_off_by_default():
    ts = TurnShaper()
    assert ts.apply(0.02, 0.1, 'TRANSIT', 0.05, lane='navigation') == (0.0, 0.25)


def test_transit_pivot_hold_stops_slow_turn_ins_in_transit_only():
    ts = TurnShaper(transit_pivot_hold_rps=0.2)
    assert ts.apply(0.02, 0.1, 'TRANSIT', 0.05, lane='navigation') == (0.0, 0.0)
    assert 'transit pivot hold' in ts.status
    assert ts.apply(0.0, -0.15, 'RETURNING_HOME', 0.05, lane='navigation') == (0.0, 0.0)
    # a real pivot request passes as before; never in MOWING
    assert ts.apply(0.0, 0.3, 'TRANSIT', 0.05, lane='navigation') == (0.0, 0.3)
    ts2 = TurnShaper(transit_pivot_hold_rps=0.2)
    assert ts2.apply(0.02, 0.1, 'MOWING', 0.05, lane='navigation') == (0.0, 0.25)
    # arcs are untouched
    v, w = ts.apply(0.2, 0.1, 'TRANSIT', 0.05, lane='navigation')
    assert (v, w) == (0.2, 0.1)
