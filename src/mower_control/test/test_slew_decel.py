"""cmd_vel_slew stops (2026-10-14): decel faster than accel, hard zero on the emergency /
bumper lanes, nav-lane zero still ramps."""

import types

import pytest

from mower_control import cmd_vel_slew as cs

DT = 0.05


def test_slew_ad_uses_decel_toward_zero_and_accel_away():
    assert cs.slew_ad(0.2, 0.3, 0.5, 0.8, DT) == pytest.approx(0.225)   # growing: accel
    assert cs.slew_ad(0.2, 0.0, 0.5, 0.8, DT) == pytest.approx(0.16)    # shrinking: decel
    assert cs.slew_ad(-0.2, 0.0, 0.5, 0.8, DT) == pytest.approx(-0.16)
    assert cs.slew_ad(0.2, 0.1, 0.5, 0.8, DT) == pytest.approx(0.16)
    assert cs.slew_ad(0.0, -0.3, 0.5, 0.8, DT) == pytest.approx(-0.025)
    assert cs.slew_ad(0.2, 0.3, 0.5, None, DT) == cs.slew(0.2, 0.3, 0.5, DT)


def test_slew_ad_zero_crossing_is_decel_then_accel():
    # 0.02 m/s left: 0.025 s of decel to zero, the other 0.025 s accelerates the other way
    assert cs.slew_ad(0.02, -0.3, 0.5, 0.8, DT) == pytest.approx(-0.0125)
    assert cs.slew_ad(0.1, -0.3, 0.5, 0.8, DT) == pytest.approx(0.06)


def _gate(state='TRANSIT'):
    g = cs.MotionGate()
    g.set_motion_enabled(True, 0.0)
    g.set_status(state, False)
    g.set_base(False, False, False, False)
    return g


def _node(**kw):
    log = types.SimpleNamespace()
    for level in ('debug', 'info', 'warn', 'warning', 'error'):
        setattr(log, level, lambda text: None)
    node = types.SimpleNamespace(
        _max_linear_accel=0.5, _max_angular_accel=1.0, _max_linear_decel=0.8,
        _max_angular_decel=1.0, _hard_stop_lanes=frozenset(cs.HARD_STOP_LANES),
        _cmd_timeout=0.5, _log_period=1.0, _cur_lin=0.0, _cur_ang=0.0, _tgt=None,
        _last_cmd_time=None, _last_tick=None, _next_log=0.0, get_logger=lambda: log,
        _gate=_gate(), _assist_gate=cs.PivotAssistGate())
    node.__dict__.update(kw)
    return node


def _drive(node, t, lin, ang=0.0, lane='navigation'):
    node._tgt, node._last_cmd_time = (lin, 0.0, 0.0, 0.0, 0.0, ang), t
    node._assist_gate.note_lane(lane, t)
    node._gate.set_motion_enabled(True, t)
    return cs.CmdVelSlewNode.step(node, t)


def _cruise(node, v=0.3, seconds=1.0):
    t = 0.0
    cs.CmdVelSlewNode.step(node, t)
    for _ in range(int(round(seconds / DT))):
        t = round(t + DT, 6)
        out = _drive(node, t, v)
    assert out[0] == pytest.approx(v)
    return t


def test_node_decel_faster_than_accel():
    node = _node()
    t = _cruise(node)
    ups, downs = [], []
    while node._cur_lin > 0.0:
        t = round(t + DT, 6)
        downs.append(_drive(node, t, 0.0)[0])
    while node._cur_lin < 0.3:
        t = round(t + DT, 6)
        ups.append(_drive(node, t, 0.3)[0])
    assert downs[0] == pytest.approx(0.3 - 0.8 * DT)
    assert ups[0] == pytest.approx(0.5 * DT)
    assert len(downs) < len(ups)                    # 8 ticks down, 12 up


@pytest.mark.parametrize('lane', ['emergency', 'bumper'])
def test_hard_stop_lane_zero_is_immediate(lane):
    node = _node()
    t = _cruise(node)
    t = round(t + DT, 6)
    out = _drive(node, t, 0.0, lane=lane)
    assert (out[0], out[5]) == (0.0, 0.0)
    assert (node._cur_lin, node._cur_ang) == (0.0, 0.0)


@pytest.mark.parametrize('lane', ['navigation', 'teleop', 'docking'])
def test_other_lane_zero_still_ramps(lane):
    node = _node()
    t = _cruise(node)
    t = round(t + DT, 6)
    out = _drive(node, t, 0.0, lane=lane)
    assert out[0] == pytest.approx(0.3 - 0.8 * DT)


def test_hard_stop_lane_non_zero_command_is_slewed():
    """The bumper back-off itself (non-zero) is slewed as usual; only its zero is hard."""
    node = _node()
    t = _cruise(node)
    t = round(t + DT, 6)
    out = _drive(node, t, -0.1, lane='bumper')
    assert out[0] == pytest.approx(0.3 - 0.8 * DT)


def test_hard_stop_lanes_empty_disables_it():
    node = _node(_hard_stop_lanes=frozenset())
    t = _cruise(node)
    t = round(t + DT, 6)
    assert _drive(node, t, 0.0, lane='emergency')[0] == pytest.approx(0.3 - 0.8 * DT)


def test_hard_stop_lanes_are_twist_mux_lanes():
    names = {name for name, *_rest in cs.TWIST_MUX_LANES}
    assert set(cs.HARD_STOP_LANES) <= names


def test_watchdog_hard_zero_unchanged():
    node = _node()
    t = _cruise(node)
    out = cs.CmdVelSlewNode.step(node, t + 0.6)       # > cmd_timeout, no fresh command
    assert (out[0], out[5]) == (0.0, 0.0)


def test_slew_both_wheels_decel_rate():
    v, w = cs.slew_both_wheels(0.3, 0.0, 0.0, 0.0, 0.5, 1.0, DT,
                               max_lin_decel=0.8, max_ang_decel=1.0)
    assert (v, w) == pytest.approx((0.26, 0.0))
