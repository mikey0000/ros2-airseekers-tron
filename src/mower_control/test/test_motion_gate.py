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
    lambda g: (g.set_status('MOWING', False), g.set_base(True, True, False, False)),  # docked, not undocking/manual
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


def test_live_undock_sequence_from_dock():
    """Live 2026-10-06 repro: docked+charging, CHARGING -> PREFLIGHT_CHECK -> UNDOCKING,
    /motion_enabled True, then docking-server backup commands at 20 Hz must pass."""
    logs = []
    g = cs.MotionGate()
    node = _node(g)
    node.get_logger().warning = logs.append
    g.set_base(True, True, False, False)
    g.set_status('CHARGING', False)
    g.set_motion_enabled(False, 0.0)
    t = 0.0
    _step(node, t)
    for _ in range(5):                                   # idle on the dock
        t += 0.05
        assert _step(node, t)[0] == 0.0
    g.set_status('PREFLIGHT_CHECK', False)
    g.set_status('UNDOCKING', False)
    g.set_base(True, False, False, False)                # CHARGER_OFF, still on contacts
    g.set_motion_enabled(True, t + 0.02)
    outs = []
    for i in range(40):                                  # BACKING_UP at 20 Hz
        t += 0.05
        _cmd(node, t - 0.01, lin=-0.15, ang=0.0)
        outs.append(_step(node, t)[0])
        if i % 10 == 9:
            g.set_motion_enabled(True, t)                 # 1 s re-assert
    assert outs[1] == pytest.approx(-0.15)
    assert all(o == pytest.approx(-0.15) for o in outs[1:])


def test_closed_gate_logs_why_when_dropping_nonzero():
    logs = []
    node = _node(cs.MotionGate())
    node.get_logger().warning = logs.append
    _step(node, 0.0)
    t = 0.0
    for _ in range(60):                                  # 3 s of dropped commands
        t += 0.05
        _cmd(node, t - 0.01)
        _step(node, t)
    drops = [m for m in logs if 'dropping cmd' in m]
    assert 2 <= len(drops) <= 2 and '/motion_enabled not received' in drops[0]


def test_receipt_callbacks_are_sampled():
    """with_receipt is only honoured on sampled entries; an event entry calls
    callback(msg) -> TypeError on every message (starved the gate's /motion_enabled)."""
    import ast
    import pathlib
    root = pathlib.Path(cs.__file__).resolve().parents[2]
    bad = []
    for path in root.rglob('*.py'):
        if '/test' in str(path):
            continue
        try:
            tree = ast.parse(path.read_text())
        except (SyntaxError, UnicodeDecodeError):
            continue
        for call in ast.walk(tree):
            if not isinstance(call, ast.Call):
                continue
            kw = {k.arg: k.value for k in call.keywords if k.arg}
            rec = kw.get('with_receipt')
            if isinstance(rec, ast.Constant) and rec.value is True:
                smp = kw.get('sampled')
                if not (isinstance(smp, ast.Constant) and smp.value is True):
                    bad.append('%s:%d' % (path, call.lineno))
    assert not bad, bad


def test_pump_rejects_receipt_without_sampled():
    pytest.importorskip('rclpy')
    from mower_control import sub_pump
    pump = sub_pump.SubscriptionPump.__new__(sub_pump.SubscriptionPump)
    pump._thread = None
    with pytest.raises(ValueError):
        sub_pump.SubscriptionPump.subscribe(pump, object, '/x', lambda m, r: None, 1,
                                            with_receipt=True)
