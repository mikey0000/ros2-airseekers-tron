"""Python crash records (mower_control.crash_record)."""

import datetime
import os
import sys
import threading

from mower_control import crash_record as cr


def _boom():
    raise ValueError('kaput')


def _exc_info():
    try:
        _boom()
    except ValueError:
        return sys.exc_info()


def test_write_record_names_and_dedups(tmp_path):
    now = datetime.datetime(2026, 10, 7, 12, 0, 0)
    a = cr.write_record('obstacle_guard', 'x', str(tmp_path), now)
    b = cr.write_record('obstacle_guard', 'y', str(tmp_path), now)
    assert os.path.basename(a) == 'obstacle_guard.20261007-120000.txt'
    assert os.path.basename(b) == 'obstacle_guard.20261007-120000.1.txt'


def test_write_record_never_raises():
    assert cr.write_record('n', 'x', '/proc/definitely/not/writable') is None


def test_excepthook_writes_traceback_and_chains(tmp_path):
    seen = []
    hook = cr.make_excepthook('det_range', lambda *a: seen.append(a), str(tmp_path))
    hook(*_exc_info())
    files = os.listdir(tmp_path)
    assert len(files) == 1 and files[0].startswith('det_range.')
    text = (tmp_path / files[0]).read_text()
    assert 'node: det_range' in text and 'ValueError: kaput' in text and '_boom' in text
    assert 'revision:' in text and 'pid: %d' % os.getpid() in text
    assert len(seen) == 1


def test_excepthook_ignores_keyboard_interrupt(tmp_path):
    hook = cr.make_excepthook('n', None, str(tmp_path))
    hook(KeyboardInterrupt, KeyboardInterrupt(), None)
    assert os.listdir(tmp_path) == []


def test_thread_excepthook(tmp_path):
    hook = cr.make_thread_excepthook('n', None, str(tmp_path))
    et, ev, tb = _exc_info()
    hook(threading.ExceptHookArgs((et, ev, tb, threading.current_thread())))
    (f,) = os.listdir(tmp_path)
    assert f.startswith('n.thread.')


def test_git_revision_reads_head(tmp_path):
    g = tmp_path / '.git'
    (g / 'refs' / 'heads').mkdir(parents=True)
    (g / 'HEAD').write_text('ref: refs/heads/main\n')
    (g / 'refs' / 'heads' / 'main').write_text('abc123\n')
    sub = tmp_path / 'src' / 'pkg'
    sub.mkdir(parents=True)
    assert cr.git_revision(str(sub)) == 'abc123'


def test_git_revision_falls_back_to_deployed_rev(tmp_path, monkeypatch):
    (tmp_path / 'DEPLOYED_REV').write_text('nogit-20261007 sha256:ff\n')
    monkeypatch.setenv('MOWER_STACK_DIR', str(tmp_path))
    assert cr.git_revision('/') == 'nogit-20261007 sha256:ff'
