"""cmd_vel_slew coupled slew (2026-10-14): in turn-shaper phases both axes ramp together,
so an arc keeps its curvature from rest and the inner wheel never crawls backwards in the
one-wheel zone."""

import random
import types

import pytest

from mower_control import cmd_vel_slew as cs

DT = 0.05
HALF = cs.TRACK_WIDTH_M / 2.0


def inner(v, w):
    return abs(v) - abs(w) * HALF


def backward_one_wheel(v, w):
    """Inner wheel in (-0.06, 0): the scrubbing one-wheel case (0 itself excluded)."""
    return abs(v) > 1e-9 and abs(w) > 1e-9 and -0.06 + 1e-9 < inner(v, w) < -1e-9


def _gate(state='TRANSIT'):
    g = cs.MotionGate()
    g.set_motion_enabled(True, 0.0)
    g.set_status(state, False)
    g.set_base(False, False, False, False)
    return g


def _node(always=True, state='TRANSIT'):
    log = types.SimpleNamespace()
    for level in ('debug', 'info', 'warn', 'warning', 'error'):
        setattr(log, level, lambda text: None)
    return types.SimpleNamespace(
        _max_linear_accel=0.5, _max_angular_accel=1.0, _max_linear_decel=0.8,
        _max_angular_decel=1.0, _hard_stop_lanes=frozenset(cs.HARD_STOP_LANES),
        _slew_both_wheels_always=always, _cmd_timeout=0.5, _log_period=1.0,
        _cur_lin=0.0, _cur_ang=0.0, _tgt=None, _last_cmd_time=None, _last_tick=None,
        _next_log=0.0, get_logger=lambda: log, _gate=_gate(state),
        _pivot=cs.PivotAssist(enabled=False), _turn=cs.TurnShaper(),
        _assist_gate=cs.PivotAssistGate())


def run(node, seq):
    t = 0.0
    cs.CmdVelSlewNode.step(node, t)
    out = []
    for lin, ang in seq:
        t = round(t + DT, 6)
        node._tgt, node._last_cmd_time = (lin, 0.0, 0.0, 0.0, 0.0, ang), t
        node._assist_gate.note_lane('navigation', t)
        node._gate.set_motion_enabled(True, t)
        o = cs.CmdVelSlewNode.step(node, t)
        out.append((o[0], o[5]))
    return out


@pytest.mark.parametrize('tgt', [(0.2, 0.3), (0.15, -0.3), (0.3, 0.2), (0.14, 0.3),
                                 (-0.2, 0.3), (0.25, 0.1)])
def test_arc_from_rest_keeps_curvature_and_inner_wheel(tgt):
    out = run(_node(), [tgt] * 30)
    k = tgt[1] / tgt[0]
    for v, w in out:
        assert not backward_one_wheel(v, w)
        assert inner(v, w) >= -1e-9                       # inner rolls with the outer
        assert w == pytest.approx(k * v, abs=1e-9)          # curvature of the target
    assert out[-1] == pytest.approx(tgt)


def test_per_axis_ramp_bent_the_arc():
    """What the default fixes: the old per-axis ramp (always=False) changes v/w."""
    out = run(_node(always=False), [(0.2, 0.3)] * 30)
    ratios = {round(w / v, 3) for v, w in out if v}
    assert len(ratios) > 1


def run_lane(node, seq, lane):
    t = 0.0
    cs.CmdVelSlewNode.step(node, t)
    out = []
    for lin, ang in seq:
        t = round(t + DT, 6)
        node._tgt, node._last_cmd_time = (lin, 0.0, 0.0, 0.0, 0.0, ang), t
        node._assist_gate.note_lane(lane, t)
        node._gate.set_motion_enabled(True, t)
        o = cs.CmdVelSlewNode.step(node, t)
        out.append((o[0], o[5]))
    return out


def test_disabled_outside_shaper_phases():
    """Manual / undocking phases keep the plain per-axis slew (docking stays exempt)."""
    for state in ('MANUAL_MOWING', 'UNDOCKING'):
        out = run(_node(state=state), [(0.2, 0.3)] * 3)
        assert out[0] == pytest.approx((0.025, 0.05))


def test_dock_phases_shape_the_nav_lane_only():
    """Fix 9: the return transit on the navigation lane gets the coupled slew; the same
    command on the docking lane keeps the plain per-axis slew of the raw command."""
    for state in ('RETURNING_HOME', 'LOW_BATTERY_DOCKING', 'RAIN_DETECTED_DOCKING'):
        out = run(_node(state=state), [(0.2, 0.3)] * 3)
        assert out[0] == pytest.approx((0.025, 0.0375))          # coupled, ratio 1.5
        out = run_lane(_node(state=state), [(0.2, 0.3)] * 3, 'docking')
        assert out[0] == pytest.approx((0.025, 0.05))
        out = run_lane(_node(state=state), [(-0.05, 0.1)] * 20, 'docking')
        assert out[-1] == pytest.approx((-0.05, 0.1))             # reverse reaches the MCU
        node = _node(state=state)
        out = run_lane(node, [(-0.05, 0.1)] * 20, 'docking')
        assert node._turn.status == 'pass'
    out = run(_node(state='RETURNING_HOME'), [(0.016, 0.09)] * 20)
    assert out[-1] == pytest.approx((0.0, 0.25))                  # snapped to a pivot


def test_pivot_to_arc_passes_through_a_stop():
    out = run(_node(), [(0.0, 0.3)] * 20 + [(0.2, 0.3)] * 40)
    for v, w in out:
        assert cs.both_wheels_legal(v, w) or inner(v, w) >= -1e-9
        assert not backward_one_wheel(v, w)
    assert out[-1] == pytest.approx((0.2, 0.3))


def test_decel_and_stop_keep_curvature():
    out = run(_node(), [(0.2, 0.3)] * 30 + [(0.0, 0.0)] * 20)
    stop = out[30:]
    # both axes shrink together; the angular decel (0.3 -> 0.25) is the binding axis here
    assert stop[0] == pytest.approx((0.2 * 5 / 6, 0.25))
    out = run(_node(), [(0.3, 0.0)] * 30 + [(0.0, 0.0)])
    assert out[-1][0] == pytest.approx(0.3 - 0.8 * DT)     # decel rate, not accel
    for v, w in stop:
        assert w == pytest.approx(1.5 * v, abs=1e-9)
    assert stop[-1] == (0.0, 0.0)


def test_random_shaped_targets_never_crawl_backwards_and_respect_limits():
    rng = random.Random(14)
    seq = []
    for _ in range(300):
        tgt = (rng.choice([0.0, rng.uniform(-0.1, 0.3)]), rng.choice([0.0, rng.uniform(-0.3, 0.3)]))
        seq += [tgt] * rng.randint(1, 15)
    node = _node()
    out = run(node, seq)
    prev = (0.0, 0.0)
    for v, w in out:
        assert not backward_one_wheel(v, w), (prev, (v, w))
        assert abs(v - prev[0]) <= 0.8 * DT + 1e-9 and abs(w - prev[1]) <= 1.0 * DT + 1e-9
        prev = (v, w)


def test_coupled_step_unit():
    assert cs.slew_coupled(0.0, 0.0, 0.2, 0.3, 0.5, 1.0, DT) == pytest.approx(
        (0.025, 0.0375))                                 # v-limited, ratio 1.5 kept
    assert cs.slew_coupled(0.2, 0.3, 0.2, 0.3, 0.5, 1.0, DT) == (0.2, 0.3)


def test_param_default_on():
    import inspect
    src = inspect.getsource(cs.CmdVelSlewNode.__init__)
    assert "declare_parameter('slew_both_wheels_always', True)" in src



# ---- docking lane: no one-wheel step (2026-10-16) -------------------------------------------
def test_no_one_wheel_step_caps_w_in_the_one_wheel_band():
    from mower_control.cmd_vel_slew import no_one_wheel_step
    v, w = no_one_wheel_step(-0.025, 0.2)              # inner -0.023: one-wheel
    assert v == -0.025 and 0.0 <= abs(v) - abs(w) * 0.24 < 1e-6
    assert no_one_wheel_step(-0.05, 0.05) == (-0.05, 0.05)    # floor zone: unchanged
    assert no_one_wheel_step(0.0, 0.3) == (0.0, 0.3)          # pivot
    assert no_one_wheel_step(-0.13, 0.06) == (-0.13, 0.06)    # proper arc
    assert no_one_wheel_step(-0.01, -0.3) == (-0.01, -0.3)    # inner -0.062: counter-rotating
