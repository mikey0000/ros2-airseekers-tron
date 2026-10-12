#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Pure decision logic of mower_navigation/activity_gated_lifecycle.py; no ROS needed.

2026-10-13 incident: lifecycle_manager_navigation hung for 5.5 h in "Configuring
controller_server" (lost change_state reply), the helper's single is_active query never
completed and it never retried; its parameter sets were re-sent + logged every 2 s.
"""
import importlib.util
import os
import signal

HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location(
    "activity_gated_lifecycle",
    os.path.join(HERE, "..", "mower_navigation", "activity_gated_lifecycle.py"))
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)
ParamSync = _mod.ParamSync
BringupWatchdog = _mod.BringupWatchdog
nodes_to_restart = _mod.nodes_to_restart
find_node_pids = _mod.find_node_pids
kill_pids = _mod.kill_pids


# --- ParamSync -----------------------------------------------------------------------------

def test_param_sync_sends_once_per_change():
    p = ParamSync(ack_timeout_s=5.0, resync_s=0.0)
    assert p.should_send(True, 0.0)
    p.sent(True, 0.0)
    assert not p.should_send(True, 1.0)          # outstanding
    assert p.acked(True, 1.0) is True            # changed -> one INFO line
    for t in range(2, 100, 2):                    # 2 s ticks: nothing re-sent
        assert not p.should_send(True, float(t))
    assert p.should_send(False, 100.0)           # activity changed
    p.sent(False, 100.0)
    assert p.acked(True, 100.5) is True
    assert not p.should_send(False, 102.0)


def test_param_sync_retries_after_failure():
    p = ParamSync(resync_s=0.0)
    p.sent(True, 0.0)
    assert p.acked(False, 0.5) is False
    assert p.should_send(True, 2.0)


def test_param_sync_unanswered_expires():
    p = ParamSync(ack_timeout_s=5.0, resync_s=0.0)
    p.sent(True, 0.0)
    assert not p.expired(4.9)
    assert not p.should_send(True, 4.9)
    assert p.expired(5.0)
    assert p.should_send(True, 5.0)
    assert p.acked(True, 6.0) is False           # a late reply after expiry is ignored


def test_param_sync_resend_after_node_respawn_and_resync():
    p = ParamSync(resync_s=60.0)
    p.sent(True, 0.0)
    p.acked(True, 0.0)
    assert not p.should_send(True, 30.0)
    assert p.should_send(True, 60.0)             # quiet periodic resync
    p.sent(True, 60.0)
    assert p.acked(True, 60.0) is False          # unchanged -> no INFO line
    p.on_unavailable()                           # service gone (respawn)
    assert p.should_send(True, 61.0)


# --- BringupWatchdog -----------------------------------------------------------------------

def test_watchdog_fires_after_unanswered_window():
    w = BringupWatchdog(timeout_s=90.0, now=0.0)
    for t in (10.0, 22.0, 34.0, 46.0, 58.0, 70.0, 82.0):
        assert not w.due(t, we_paused=False)
    assert w.due(94.0, we_paused=False)


def test_watchdog_any_answer_resets_slow_autostart_never_fires():
    w = BringupWatchdog(timeout_s=90.0, now=0.0)
    # autostart blocks the manager's services for 80 s, then it answers "not active"
    assert not w.due(80.0, False)
    w.on_answer(80.0)
    assert not w.due(160.0, False)
    assert w.due(170.0, False)


def test_watchdog_disabled():
    w = BringupWatchdog(timeout_s=0.0, now=0.0)
    assert not w.enabled
    assert not w.due(1e6, False)


def test_watchdog_not_while_we_paused():
    w = BringupWatchdog(timeout_s=90.0, now=0.0)
    assert not w.due(100.0, we_paused=True)
    # window restarted by our pause: no immediate fire once the pause is gone
    assert not w.due(101.0, we_paused=False)
    assert w.due(190.0, we_paused=False)


def test_watchdog_unavailable_and_manager_restart_restart_window():
    w = BringupWatchdog(timeout_s=90.0, now=0.0)
    w.on_unavailable(50.0)                       # manager not discovered: not a hang
    assert not w.due(100.0, False)
    w.on_manager_pid(1234, 100.0)                # first sighting: no reset
    assert w.due(140.0, False)
    w.on_manager_pid(1234, 141.0)
    w.on_manager_pid(None, 141.0)                # lookup failed: ignored
    assert w.due(141.0, False)
    w.on_manager_pid(5678, 142.0)                # respawned manager: full window again
    assert not w.due(200.0, False)
    assert w.due(232.0, False)


def test_watchdog_attempt_cap_and_backoff():
    w = BringupWatchdog(timeout_s=90.0, max_attempts=3, backoff_s=60.0, now=0.0)
    t = 90.0
    assert w.due(t, False)
    w.record_attempt(t)                          # next allowed >= 150 and window to 180
    assert not w.due(t + 89.0, False)
    assert w.due(t + 90.0, False)
    t += 90.0
    w.record_attempt(t)                          # backoff 120 s > window 90 s
    assert not w.due(t + 90.0, False)
    assert w.due(t + 120.0, False)
    t += 120.0
    w.record_attempt(t)
    assert w.exhausted()
    assert not w.due(t + 1e5, False)


def test_watchdog_defer_when_all_active():
    w = BringupWatchdog(timeout_s=90.0, now=0.0)
    assert w.due(90.0, False)
    w.defer(90.0)
    assert w.attempts == 0
    assert not w.due(100.0, False)
    assert w.due(180.0, False)


# --- nodes_to_restart ----------------------------------------------------------------------

def test_restart_incident_states():
    states = {'controller_server': 'inactive', 'planner_server': 'unconfigured',
              'behavior_server': 'unconfigured', 'bt_navigator': 'unconfigured',
              'velocity_smoother': 'unconfigured', 'collision_monitor': 'unconfigured'}
    assert nodes_to_restart(states) == ['controller_server']


def test_restart_unanswered_and_transitional_nodes():
    states = {'a': 'active', 'b': None, 'c': 'configuring', 'd': 'unconfigured'}
    assert nodes_to_restart(states) == ['a', 'b', 'c']


def test_restart_none_when_all_active():
    assert nodes_to_restart({'a': 'active', 'b': 'active'}) is None


def test_restart_all_unconfigured_only_manager():
    assert nodes_to_restart({'a': 'unconfigured', 'b': 'unconfigured'}) == []


# --- pid lookup / kill ---------------------------------------------------------------------

def _proc(root, pid, *args):
    d = root / str(pid)
    d.mkdir()
    (d / 'cmdline').write_bytes(b'\0'.join(a.encode() for a in args) + b'\0')


def test_find_node_pids_exact_token(tmp_path):
    _proc(tmp_path, 10, '/opt/ros/jazzy/lib/nav2_controller/controller_server', '--ros-args',
          '-r', '__node:=controller_server', '--params-file', '/tmp/x.yaml')
    _proc(tmp_path, 11, 'python3', 'x.py', '--ros-args', '-r', '__node:=controller_server_2')
    _proc(tmp_path, 12, '/lib/lifecycle_manager', '--ros-args', '-r',
          '__node:=lifecycle_manager_navigation')
    _proc(tmp_path, 13, 'bash', '-c', 'echo __node:=controller_server')   # not a token
    (tmp_path / 'self').mkdir()
    (tmp_path / '99').mkdir()                                             # no cmdline
    assert find_node_pids('controller_server', str(tmp_path)) == [10]
    assert find_node_pids('lifecycle_manager_navigation', str(tmp_path)) == [12]
    assert find_node_pids('lifecycle_manager_navigation', str(tmp_path), exclude=(12,)) == []
    assert find_node_pids('x', str(tmp_path / 'missing')) == []


def test_kill_pids_skips_gone_processes():
    sent = []

    def fake_kill(pid, sig):
        if pid == 2:
            raise ProcessLookupError
        sent.append((pid, sig))

    assert kill_pids([1, 2, 3], kill=fake_kill) == [1, 3]
    assert sent == [(1, signal.SIGTERM), (3, signal.SIGTERM)]
