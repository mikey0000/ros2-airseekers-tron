"""Motion gate in cmd_vel_slew: zero unless the mission allows motion, no stale release."""

import types

import pytest

from mower_control import cmd_vel_slew as cs


def _node(gate):
    log = types.SimpleNamespace()
    for level in ('debug', 'info', 'warn', 'warning', 'error'):
        setattr(log, level, lambda text: None)
    return types.SimpleNamespace(
        _max_linear_accel=100.0, _max_angular_accel=100.0, _cmd_timeout=0.5,
        _log_period=1.0, _cur_lin=0.0, _cur_ang=0.0, _tgt=None, _last_cmd_time=None,
        _last_tick=None, _next_log=0.0, get_logger=lambda: log, _gate=gate)


def _step(node, now):
    return cs.CmdVelSlewNode.step(node, now)


def _cmd(node, t, lin=0.3, ang=-0.5):
    node._tgt, node._last_cmd_time = (lin, 0.0, 0.0, 0.0, 0.0, ang), t


def _open_gate(t, state='MANUAL_MOWING'):
    g = cs.MotionGate()
    g.set_motion_enabled(True, t)
    g.set_status(state, False)
    g.set_base(False, False, False, False)
    return g


def test_zero_without_any_gate_inputs():
    node = _node(cs.MotionGate())
    _step(node, 0.0)
    _cmd(node, 0.04)
    out = _step(node, 0.05)
    assert out[0] == 0.0 and out[5] == 0.0


def test_passes_when_enabled():
    node = _node(_open_gate(0.0))
    _step(node, 0.0)
    _step(node, 0.02)                                     # gate opens here
    _cmd(node, 0.04)
    out = _step(node, 0.05)
    assert out[0] == pytest.approx(0.3) and out[5] == pytest.approx(-0.5)


@pytest.mark.parametrize('mutate', [
    lambda g: g.set_motion_enabled(False, 0.0),
    lambda g: g.set_status('EMERGENCY', True),
    lambda g: g.set_status('IDLE', False),
    lambda g: g.set_status('CHARGING', False),
    lambda g: g.set_status('MOWING_COMPLETE', False),
    lambda g: g.set_status('NAV_TO_DOCK_FAILED', False),
    lambda g: g.set_status('BOUNDARY_EMERGENCY_STOP', False),
    lambda g: g.set_base(True, True, False, False),       # docked, manual teleop
    lambda g: g.set_base(False, False, True, False),      # stop button
    lambda g: g.set_base(False, False, False, True),      # lifted
])
def test_zero_when_disabled(mutate):
    g = _open_gate(0.0)
    mutate(g)
    node = _node(g)
    _step(node, 0.0)
    for k in range(1, 20):
        _cmd(node, 0.05 * k - 0.01)                       # bumper lane hammering
        out = _step(node, 0.05 * k)
        assert out[0] == 0.0 and out[5] == 0.0


def test_undocking_allowed_while_docked():
    g = _open_gate(0.0, 'UNDOCKING')
    g.set_base(True, True, False, False)
    node = _node(g)
    _step(node, 0.0)
    _step(node, 0.02)
    _cmd(node, 0.04, lin=-0.2, ang=0.0)
    assert _step(node, 0.05)[0] == pytest.approx(-0.2)


def test_stale_motion_enabled_closes_gate():
    node = _node(_open_gate(0.0))
    _step(node, 0.0)
    _cmd(node, 3.9)
    assert _step(node, 4.0)[0] == 0.0                     # /motion_enabled 4 s old


def test_no_stale_release_on_enable():
    g = _open_gate(0.0, 'EMERGENCY')
    g.set_status('EMERGENCY', True)
    node = _node(g)
    _step(node, 0.0)
    _cmd(node, 0.95)                                      # command held during estop
    assert _step(node, 1.0)[5] == 0.0
    # emergency reset: mission back in a motion state, gate opens at t=1.05
    g.set_status('MANUAL_MOWING', False)
    g.set_motion_enabled(True, 1.04)
    _cmd(node, 1.04)                                      # arrived before the gate opened
    out = _step(node, 1.05)
    assert out[0] == 0.0 and out[5] == 0.0
    assert _step(node, 1.10)[5] == 0.0                    # still not released
    _cmd(node, 1.12, ang=0.4)                             # fresh command after reset
    assert _step(node, 1.15)[5] == pytest.approx(0.4)
