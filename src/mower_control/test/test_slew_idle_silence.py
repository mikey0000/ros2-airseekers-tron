"""cmd_vel_slew idle = silence: zeros only for idle_publish_s after motion / gate change."""

import types

from mower_control import cmd_vel_slew

Z = (0.0,) * 6


def _node(hold=0.5):
    return types.SimpleNamespace(_idle_publish_s=hold, _gate_status='OPEN: ok')


def _pub(node, out, now):
    return cmd_vel_slew.CmdVelSlewNode.should_publish(node, out, now)


def test_zero_tail_then_silence_then_resume():
    n = _node()
    assert _pub(n, (0.2, 0, 0, 0, 0, 0.0), 10.0)          # moving: always published
    assert _pub(n, Z, 10.05) and _pub(n, Z, 10.5)          # stop tail (0.5 s of zeros)
    assert not _pub(n, Z, 10.55) and not _pub(n, Z, 60.0)  # then silence
    assert _pub(n, (0.0, 0, 0, 0, 0, 0.1), 61.0)           # new motion: first frame goes out
    assert _pub(n, Z, 61.05)                               # and a fresh stop tail follows


def test_gate_change_publishes_a_new_tail():
    n = _node()
    assert _pub(n, Z, 0.0)
    assert not _pub(n, Z, 5.0)
    n._gate_status = 'CLOSED: stuck'
    assert _pub(n, Z, 6.0) and _pub(n, Z, 6.4)
    assert not _pub(n, Z, 6.6)


def test_startup_tail_and_disable():
    n = _node()
    assert _pub(n, Z, 100.0) and not _pub(n, Z, 100.6)
    n = _node(hold=-1.0)
    assert _pub(n, Z, 0.0) and _pub(n, Z, 1e6)
