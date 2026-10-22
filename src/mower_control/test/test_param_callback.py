"""cmd_vel_slew parameter callback (code review 2026-10-23): a request is validated as a whole
and applied at once under the lock step() runs under; a rejected request changes nothing."""

import threading
import types

from mower_control import cmd_vel_slew as cs


def _node():
    log = types.SimpleNamespace(info=lambda *_a, **_k: None, warn=lambda *_a, **_k: None)
    return types.SimpleNamespace(
        _turn=cs.TurnShaper(), _pivot=cs.PivotAssist(), _assist_gate=cs.PivotAssistGate(),
        _shaper=cs.DriveShaper(), _cfg_lock=threading.Lock(), get_logger=lambda: log)


def P(name, value):
    return types.SimpleNamespace(name=name, value=value)


def _snapshot(n):
    return (dict(n._turn.params), n._turn.latch_ticks, n._pivot.enabled,
            {k: getattr(n._shaper, k) for k in cs.SHAPER_PARAMS})


def test_rejected_request_applies_nothing():
    n = _node()
    before = _snapshot(n)
    r = cs.CmdVelSlewNode._on_set_parameters(n, [
        P('turn_shaper_latch_ticks', 3), P('pivot_assist', False),
        P('turn_shaper_pivot_deadband_rps', 0.05), P('heading_hold_kp', 'fast')])
    assert not r.successful and 'heading_hold_kp' in r.reason
    assert _snapshot(n) == before


def test_accepted_request_applies_everything():
    n = _node()
    params_obj = n._turn.params
    r = cs.CmdVelSlewNode._on_set_parameters(n, [
        P('turn_shaper_latch_ticks', 3), P('pivot_assist', False),
        P('turn_shaper_pivot_deadband_rps', 0.05), P('heading_hold_kp', 2)])
    assert r.successful
    assert n._turn.latch_ticks == 3 and n._pivot.enabled is False
    assert n._turn.params['pivot_deadband_rps'] == 0.05
    assert params_obj is not n._turn.params        # swapped, not mutated under a reader
    assert n._shaper.heading_hold_kp == 2.0


def test_apply_waits_for_the_lock():
    n = _node()
    n._cfg_lock.acquire()
    done = []
    t = threading.Thread(target=lambda: done.append(cs.CmdVelSlewNode._on_set_parameters(
        n, [P('turn_shaper_latch_ticks', 9)])))
    t.start()
    t.join(0.2)
    assert not done and n._turn.latch_ticks != 9      # blocked while a tick holds the lock
    n._cfg_lock.release()
    t.join(2.0)
    assert done[0].successful and n._turn.latch_ticks == 9
