"""bag_recorder low-power rework (2026-10-13 review B5): a non-persisted low-power profile
that never touches the user's switch, a mow start that wins over a late /mission/activity,
a short ring while docked (incident pins keep data), and non-blocking stops."""

import os
import signal
import subprocess
import sys

from mower_control import bag_recorder as rec
from test_bag_recorder_process import WALL, FakePopen


def _setup(tmp_path, default=True, low_topics=True, state=None):
    popen = FakePopen()
    r = rec.RecorderProcess(str(tmp_path / 'bags'), str(tmp_path / 'recorder.log'),
                            popen=popen)
    sf = str(tmp_path / 'state.json')
    if state is not None:
        (tmp_path / 'state.json').write_text('{"enabled": %s}' % ('true' if state else 'false'))
    sw = rec.RecorderSwitch(r, sf, default)
    gate = rec.LowPowerGate(True)
    ring = rec.RingController(r, sw, gate, low_topics)
    return ring, r, sw, gate, popen


def _run(ring, now):
    """One node tick: start what the controller asks for (as BagRecorder._tick does)."""
    prof, _stopped = ring.step(now)
    if prof:
        ring.rec.start(lambda out, _p=prof: ['ros2', 'bag', 'record', '-o', out, _p],
                       now, WALL + now, prof)
    return prof


def _exit_stopping(r):
    if r.stopping is not None:
        r.stopping.returncode = -2


def test_hl_before_activity_starts_the_mow_ring(tmp_path):
    """Mow from the dock: activity docked_idle (low ring), then high_level_status leaves
    IDLE before /mission/activity says mowing -> full ring, mow pin can open."""
    ring, r, sw, gate, popen = _setup(tmp_path)
    gate.on_activity('docked_idle')
    gate.on_state('IDLE_DOCKED')
    assert _run(ring, 0.0) == rec.LOW
    assert not ring.mow_ready()
    gate.on_state('UNDOCKING')
    gate.set_mow(True)                       # HL first; activity still docked_idle
    assert _run(ring, 1.0) is None           # low child stopping (non-blocking)
    assert popen.procs[0].signals == [signal.SIGINT] and r.busy()
    _exit_stopping(r)
    assert _run(ring, 2.0) == rec.FULL
    assert ring.mow_ready() and r.restarts == 0
    gate.on_activity('mowing')               # late activity: no change
    assert _run(ring, 3.0) is None and r.running() and len(popen.procs) == 2


def test_user_switch_never_overwritten_by_low_power(tmp_path):
    ring, r, sw, gate, popen = _setup(tmp_path)
    _run(ring, 0.0)
    gate.on_activity('docked_idle')
    _run(ring, 1.0)
    assert sw.enabled is True
    assert not os.path.exists(sw.state_file)            # nothing persisted by the gate
    sw.set(False, 2.0)                                  # user: off
    _exit_stopping(r)
    gate.on_activity('mowing')
    gate.set_mow(True)
    for t in (3.0, 4.0, 5.0):
        assert _run(ring, t) is None
    assert sw.enabled is False and not r.running()
    assert rec.RecorderSwitch(r, sw.state_file, True).enabled is False


def test_persisted_off_survives_activity_changes(tmp_path):
    ring, r, sw, gate, popen = _setup(tmp_path, state=False)
    for act in ('docked_idle', 'mowing', 'idle', 'mowing'):
        gate.on_activity(act)
        assert _run(ring, 1.0) is None
    assert popen.calls == []


def test_short_ring_while_docked_for_incident_pins(tmp_path):
    ring, r, sw, gate, popen = _setup(tmp_path)
    gate.on_activity('docked_idle')
    assert _run(ring, 0.0) == rec.LOW and r.running() and r.profile == rec.LOW
    # empty low_power_topics: paused while docked (the switch still says enabled)
    ring2, r2, sw2, gate2, _ = _setup(tmp_path / 'b', low_topics=False)
    gate2.on_activity('docked_idle')
    assert _run(ring2, 0.0) is None and sw2.enabled
    gate2.on_activity('mowing')
    assert _run(ring2, 1.0) == rec.FULL


def test_emergency_keeps_the_full_ring(tmp_path):
    """activity.py maps EMERGENCY to idle; its aftermath is what a pin needs."""
    ring, r, sw, gate, popen = _setup(tmp_path)
    gate.on_state('EMERGENCY')
    gate.on_activity('idle')
    assert _run(ring, 0.0) == rec.FULL
    gate.on_state('IDLE')
    _run(ring, 1.0)
    assert r.busy()                          # back to the low ring once out of EMERGENCY


def test_charging_is_not_an_activity():
    assert 'charging' not in rec.LOW_POWER_ACTIVITIES
    gate = rec.LowPowerGate(True)
    assert not gate.low_power                # no activity yet: active
    gate.on_activity('docking')
    assert not gate.low_power
    off = rec.LowPowerGate(False)
    off.on_activity('docked_idle')
    assert not off.low_power


def test_stop_async_does_not_block_and_kills_late(tmp_path):
    ring, r, sw, gate, popen = _setup(tmp_path)
    _run(ring, 0.0)
    p = popen.procs[0]
    sw.set(False, 1.0)
    assert p.wait_calls == []                # no wait in the executor
    r.poll_stopping(10.0)
    assert not p.killed and r.busy()
    r.poll_stopping(16.5)
    assert p.killed
    p.returncode = -9
    r.poll_stopping(17.0)
    assert not r.busy()


def test_module_imports_without_rclpy():
    code = ("import sys; sys.modules['rclpy'] = None; "
            "import mower_control.bag_recorder as m; print(m.FULL)")
    out = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True,
                         env=dict(os.environ, PYTHONPATH=os.pathsep.join(sys.path)))
    assert out.returncode == 0, out.stderr
    assert out.stdout.strip() == 'full'
