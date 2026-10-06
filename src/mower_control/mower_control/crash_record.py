# SPDX-License-Identifier: GPL-3.0-or-later
"""Crash recording for the Python nodes (pure Python, no ROS imports).

``install(node_name)`` is called first thing in a node's ``main()``:

* ``faulthandler`` is enabled into ``<crash_dir>/<node>.faulthandler.log`` (append), so a
  hard crash of the interpreter (SIGSEGV/SIGABRT/SIGBUS/SIGFPE/SIGILL in an extension
  module such as rclpy, cv2, rknnlite) still leaves every thread's Python stack;
* ``sys.excepthook`` and ``threading.excepthook`` write an uncaught exception (the one that
  makes ``rclpy.spin`` return and the process exit) to ``<crash_dir>/<node>.<ts>.txt``
  with the traceback, pid, argv and the deployed git revision, then chain to the previous
  hook so it still reaches stderr / the launch log.

Callback exceptions that a node already catches and logs ("callback raised", sub_pump) are
not crashes and are not recorded here; the supervisor black box captures them via /rosout.

The crash directory is ``$MOWER_CRASH_DIR`` or ``/userdata/ros2/crashes``. Everything here
is best effort: a read-only or full disk must never stop a node from starting.
"""

import datetime
import faulthandler
import os
import sys
import threading
import traceback

DEFAULT_CRASH_DIR = '/userdata/ros2/crashes'
_state = {'installed': False, 'fh_file': None}


def crash_dir():
    return os.environ.get('MOWER_CRASH_DIR', DEFAULT_CRASH_DIR)


def timestamp(now=None):
    now = now or datetime.datetime.now()
    return now.strftime('%Y%m%d-%H%M%S')


def git_revision(start=None):
    """HEAD of the deployed tree (reads .git directly; no git binary needed)."""
    path = os.path.abspath(start or os.environ.get('MOWER_STACK_DIR', '/work'))
    for _ in range(8):
        head = os.path.join(path, '.git', 'HEAD')
        if os.path.isfile(head):
            try:
                with open(head, encoding='utf-8') as f:
                    ref = f.read().strip()
                if ref.startswith('ref: '):
                    ref_path = os.path.join(path, '.git', ref[5:])
                    if os.path.isfile(ref_path):
                        with open(ref_path, encoding='utf-8') as f:
                            return f.read().strip()
                    packed = os.path.join(path, '.git', 'packed-refs')
                    if os.path.isfile(packed):
                        with open(packed, encoding='utf-8') as f:
                            for line in f:
                                if line.rstrip().endswith(' ' + ref[5:]):
                                    return line.split()[0]
                    return ref
                return ref
            except OSError:
                return 'unknown'
        parent = os.path.dirname(path)
        if parent == path:
            break
        path = parent
    rev_file = os.path.join(os.environ.get('MOWER_STACK_DIR', '/work'), 'DEPLOYED_REV')
    try:
        with open(rev_file, encoding='utf-8') as f:
            return f.read().strip() or 'unknown'
    except OSError:
        return 'unknown'


def format_record(node_name, exc_type, exc, tb, thread_name=None, now=None):
    lines = [
        'node: %s' % node_name,
        'time: %s' % (now or datetime.datetime.now()).isoformat(timespec='seconds'),
        'pid: %d' % os.getpid(),
        'thread: %s' % (thread_name or threading.current_thread().name),
        'argv: %s' % ' '.join(sys.argv),
        'revision: %s' % git_revision(),
        'exception: %s: %s' % (getattr(exc_type, '__name__', exc_type), exc),
        '',
    ]
    lines.extend(l.rstrip('\n') for l in traceback.format_exception(exc_type, exc, tb))
    return '\n'.join(lines) + '\n'


def write_record(node_name, text, directory=None, now=None):
    """Write ``<dir>/<node>.<ts>.txt``; returns the path or None. Never raises."""
    directory = directory or crash_dir()
    try:
        os.makedirs(directory, exist_ok=True)
        base = os.path.join(directory, '%s.%s' % (node_name, timestamp(now)))
        path, n = base + '.txt', 1
        while os.path.exists(path):
            path, n = '%s.%d.txt' % (base, n), n + 1
        with open(path, 'w', encoding='utf-8') as f:
            f.write(text)
        return path
    except OSError:
        return None


def make_excepthook(node_name, previous, directory=None):
    def hook(exc_type, exc, tb):
        if not issubclass(exc_type, (KeyboardInterrupt, SystemExit)):
            path = write_record(node_name, format_record(node_name, exc_type, exc, tb),
                                directory)
            if path:
                sys.stderr.write('[%s] crash record written to %s\n' % (node_name, path))
        if previous is not None:
            previous(exc_type, exc, tb)
    return hook


def make_thread_excepthook(node_name, previous, directory=None):
    def hook(args):
        if not issubclass(args.exc_type, SystemExit):
            text = format_record(node_name, args.exc_type, args.exc_value, args.exc_traceback,
                                 thread_name=getattr(args.thread, 'name', None))
            write_record(node_name + '.thread', text, directory)
        if previous is not None:
            previous(args)
    return hook


def install(node_name, directory=None):
    """Enable faulthandler + exception hooks for this process (idempotent)."""
    if _state['installed']:
        return
    _state['installed'] = True
    directory = directory or crash_dir()
    try:
        os.makedirs(directory, exist_ok=True)
        fh = open(os.path.join(directory, '%s.faulthandler.log' % node_name), 'a',
                  encoding='utf-8')
        fh.write('--- %s pid %d started %s\n' % (node_name, os.getpid(),
                                                 datetime.datetime.now().isoformat()))
        fh.flush()
        faulthandler.enable(file=fh, all_threads=True)
        _state['fh_file'] = fh            # must stay open for the process lifetime
    except OSError:
        faulthandler.enable(all_threads=True)          # stderr -> launch log
    sys.excepthook = make_excepthook(node_name, sys.excepthook, directory)
    threading.excepthook = make_thread_excepthook(node_name, threading.excepthook, directory)


def install_soft(node_name):
    """For nodes in other packages: ``from mower_control.crash_record import install_soft``
    is wrapped by callers in try/except ImportError, so a missing mower_control never
    breaks them."""
    install(node_name)
