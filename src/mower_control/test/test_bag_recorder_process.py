"""bag_recorder.RecorderProcess: supervised child, backoff, stop (fake popen + one real child)."""

import os
import signal
import subprocess
import time

import pytest

from mower_control import bag_recorder as rec

WALL = time.mktime((2026, 10, 9, 10, 15, 0, 0, 0, -1))


class FakeProc:
    def __init__(self, wait_raises=False):
        self.returncode = None
        self.signals = []
        self.killed = False
        self.wait_calls = []
        self.wait_raises = wait_raises

    def poll(self):
        return self.returncode

    def send_signal(self, sig):
        self.signals.append(sig)

    def wait(self, timeout=None):
        self.wait_calls.append(timeout)
        if self.wait_raises and not self.killed:
            raise subprocess.TimeoutExpired('x', timeout)
        if self.returncode is None:
            self.returncode = -2
        return self.returncode

    def kill(self):
        self.killed = True


class FakePopen:
    def __init__(self, **proc_kw):
        self.calls = []
        self.procs = []
        self.proc_kw = proc_kw

    def __call__(self, argv, **kw):
        self.calls.append((argv, kw))
        p = FakeProc(**self.proc_kw)
        self.procs.append(p)
        return p


def make(tmp_path, **kw):
    popen = FakePopen(**kw)
    r = rec.RecorderProcess(str(tmp_path / 'bags'), str(tmp_path / 'recorder.log'), popen=popen)
    return r, popen


def cmd_for(path):
    return ['ros2', 'bag', 'record', '-o', path]


def test_start_creates_session_and_log_header(tmp_path):
    r, popen = make(tmp_path)
    path = r.start(cmd_for, 10.0, WALL)
    assert path == str(tmp_path / 'bags' / '20261009-101500')
    assert r.session == '20261009-101500' and r.started == 10.0
    assert r.running()
    argv, kw = popen.calls[0]
    assert argv == cmd_for(path)
    assert kw['stdin'] == subprocess.DEVNULL and kw['stderr'] == subprocess.STDOUT
    assert callable(kw['preexec_fn'])
    log = (tmp_path / 'recorder.log').read_text()
    assert '=== ' in log and 'start ' + path in log


def test_second_start_same_second_gets_suffix(tmp_path):
    r, _ = make(tmp_path)
    p1 = r.start(cmd_for, 0.0, WALL)
    os.makedirs(p1)                      # rosbag2 creates the dir
    p2 = r.start(cmd_for, 1.0, WALL)
    assert p2 == p1 + '.1' and r.session == '20261009-101500.1'
    os.makedirs(p2)
    assert r.start(cmd_for, 2.0, WALL) == p1 + '.2'


def test_log_appended_and_truncated_when_huge(tmp_path):
    r, _ = make(tmp_path)
    r.start(cmd_for, 0.0, WALL)
    r.start(cmd_for, 1.0, WALL + 10)
    assert (tmp_path / 'recorder.log').read_text().count('=== ') == 2
    (tmp_path / 'recorder.log').write_bytes(b'x' * (5 * 1024 * 1024 + 1))
    r.start(cmd_for, 2.0, WALL + 20)
    assert (tmp_path / 'recorder.log').stat().st_size < 1000


def test_poll_exit_none_while_running(tmp_path):
    r, popen = make(tmp_path)
    assert r.poll_exit(0.0) is None
    r.start(cmd_for, 0.0, WALL)
    assert r.poll_exit(1.0) is None and r.running()


def test_poll_exit_returns_rc_once_and_backs_off(tmp_path):
    r, popen = make(tmp_path)
    r.start(cmd_for, 100.0, WALL)
    popen.procs[-1].returncode = 3
    assert r.poll_exit(101.0) == 3
    assert r.poll_exit(101.0) is None
    assert r.proc is None and not r.running()
    assert r.restarts == 1 and r.backoff_s == 4.0 and r.next_start == 105.0


def test_backoff_doubles_and_caps_at_60(tmp_path):
    r, popen = make(tmp_path)
    seen = []
    t = 0.0
    for i in range(8):
        r.start(cmd_for, t, WALL + i)
        popen.procs[-1].returncode = 1
        t += 1.0
        r.poll_exit(t)
        seen.append(r.backoff_s)
    assert seen == [4.0, 8.0, 16.0, 32.0, 60.0, 60.0, 60.0, 60.0]
    assert r.next_start == t + 60.0


def test_backoff_reset_after_long_run(tmp_path):
    r, popen = make(tmp_path)
    r.backoff_s = 60.0
    r.start(cmd_for, 1.0, WALL)
    popen.procs[-1].returncode = 0
    r.poll_exit(122.0)
    assert r.backoff_s == 2.0 and r.next_start == 124.0
    r.start(cmd_for, 200.0, WALL + 1)
    popen.procs[-1].returncode = 0
    r.poll_exit(320.0)                   # exactly 120 s is not "long"
    assert r.backoff_s == 4.0


def test_backoff_reset_when_started_at_time_zero(tmp_path):
    r, popen = make(tmp_path)
    r.backoff_s = 60.0
    r.start(cmd_for, 0.0, WALL)
    popen.procs[-1].returncode = 0
    r.poll_exit(500.0)
    assert r.backoff_s == 2.0


def test_stop_sends_sigint_and_waits(tmp_path):
    r, popen = make(tmp_path)
    r.start(cmd_for, 0.0, WALL)
    p = popen.procs[-1]
    r.stop(timeout_s=7.0)
    assert p.signals == [signal.SIGINT] and p.wait_calls == [7.0] and not p.killed
    assert r.proc is None
    r.stop()                             # idempotent
    assert p.signals == [signal.SIGINT]


def test_stop_kills_on_timeout(tmp_path):
    r, popen = make(tmp_path, wait_raises=True)
    r.start(cmd_for, 0.0, WALL)
    p = popen.procs[-1]
    r.stop(timeout_s=1.0)
    assert p.signals == [signal.SIGINT] and p.killed
    assert p.wait_calls == [1.0, 5.0]
    assert r.proc is None


def test_stop_skips_already_dead_child(tmp_path):
    r, popen = make(tmp_path)
    r.start(cmd_for, 0.0, WALL)
    popen.procs[-1].returncode = 1
    r.stop()
    assert popen.procs[-1].signals == []


def test_real_subprocess_stop_terminates_sleep(tmp_path):
    r = rec.RecorderProcess(str(tmp_path / 'bags'), str(tmp_path / 'rec.log'))
    r.start(lambda _p: ['sleep', '30'], time.monotonic(), WALL)
    proc = r.proc
    assert r.running()
    t0 = time.monotonic()
    r.stop(timeout_s=5.0)
    assert time.monotonic() - t0 < 5.0
    assert proc.poll() is not None
    assert proc.returncode == -signal.SIGINT
    assert not r.running()


# --------------------------------------------------------------------------- ~/enable switch
def make_switch(tmp_path, default=True):
    r, popen = make(tmp_path)
    sw = rec.RecorderSwitch(r, str(tmp_path / 'state' / 'bag_recorder.json'), default)
    return sw, r, popen


def test_switch_default_when_no_state_file(tmp_path):
    assert make_switch(tmp_path, True)[0].enabled is True
    assert make_switch(tmp_path / 'b', False)[0].enabled is False


def test_switch_disable_stops_child_and_persists(tmp_path):
    sw, r, popen = make_switch(tmp_path)
    r.start(cmd_for, 0.0, WALL)
    assert sw.due_start(False, 1.0) is False            # running
    changed, saved = sw.set(False, 5.0)
    assert changed and saved
    assert popen.procs[0].signals == [signal.SIGINT]     # clean rosbag2 close
    assert not r.running() and r.restarts == 0           # a stop is not a crash
    assert sw.due_start(False, 100.0) is False           # stays off
    # persisted: a new node (stack restart) comes up disabled even with default on
    sw2 = rec.RecorderSwitch(r, sw.state_file, True)
    assert sw2.enabled is False


def test_switch_enable_starts_at_next_tick_with_fresh_backoff(tmp_path):
    sw, r, popen = make_switch(tmp_path, default=False)
    assert sw.due_start(False, 0.0) is False
    r.backoff_s, r.next_start = 60.0, 500.0              # was crash-looping before
    changed, saved = sw.set(True, 10.0)
    assert changed and saved and sw.enabled
    assert r.backoff_s == 2.0 and sw.due_start(False, 10.0) is True
    assert sw.due_start(True, 10.0) is False             # low-disk pause still wins
    assert rec.RecorderSwitch(r, sw.state_file, False).enabled is True


def test_switch_set_same_value_is_idempotent(tmp_path):
    sw, r, popen = make_switch(tmp_path)
    r.start(cmd_for, 0.0, WALL)
    changed, saved = sw.set(True, 1.0)
    assert not changed and saved
    assert r.running() and popen.procs[0].signals == []


def test_switch_unwritable_state_still_switches(tmp_path):
    r, _ = make(tmp_path)
    (tmp_path / 'ro').write_text('x')                    # a file where the dir should be
    sw = rec.RecorderSwitch(r, str(tmp_path / 'ro' / 'state.json'), True)
    changed, saved = sw.set(False, 0.0)
    assert changed and not saved and sw.enabled is False
