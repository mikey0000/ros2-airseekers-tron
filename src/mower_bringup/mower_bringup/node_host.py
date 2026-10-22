"""node_host: run several rclpy nodes in ONE process (one rclpy context, one DDS participant).

Memory diet M1/M2/M6 (docs/analysis/2026-10-22_memory_footprint.md): every Python node process
costs ~38 MB (dev) / ~54 MB (mower) before it does any work, 25 MB of it the Fast-DDS
participant. A host runs N nodes for one participant + one interpreter.

    node_host <spec.json>

The spec is written by ``mower_bringup.node_host_launch`` (``compose_nodes:=true``)::

    {"group": "control",
     "nodes": [{"name": "cmd_vel_slew", "package": "mower_control",
                "executable": "cmd_vel_slew", "path": "/.../lib/mower_control/cmd_vel_slew",
                "ros_args": ["--ros-args", "-r", "__node:=cmd_vel_slew", "--params-file", ...]},
               ...]}

Each node runs its own UNMODIFIED ``main()`` (console_scripts entry point, or the ``main`` of
an installed Python script) in its own thread, with:

* the exact ``--ros-args`` launch would have given its standalone process, applied as that
  node's ``cli_args`` with ``use_global_arguments=False`` (rclpy.node.Node.__init__ shim):
  node name, namespace, params files, ``-p`` overrides and remaps are per node, so topics,
  services and parameters are identical to the standalone layout;
* its own executor: ``rclpy.spin(node)`` uses a per-thread SingleThreadedExecutor
  (``rclpy.get_global_executor`` shim), a node that builds its own MultiThreadedExecutor keeps
  it. One node's slow callback therefore never waits behind another node's executor; the only
  shared resource is the GIL (and the participant's DDS threads);
* ``rclpy.init`` / ``rclpy.shutdown`` / ``rclpy.try_shutdown`` made no-ops for node threads
  (the host owns the context), ``mower_control.crash_record.install`` a no-op (the host
  installs it once as ``node_host_<group>``).

Crash semantics (fail fast, same as the standalone processes): when any node's ``main()``
ends while the host is running -- a callback raised, ``sys.exit``, or it returned -- the host
logs which node and why (crash record ``<crash_dir>/<node>-*.txt`` with the traceback), shuts
the context down so every other node runs its own ``finally`` clean-up (lights off, child
processes stopped, ...), waits up to ``JOIN_S`` and exits 1. launch then respawns the WHOLE
group (``respawn`` of the host = any member's). A node never dies silently while its peers
keep running. SIGINT/SIGTERM (launch shutdown) shut the context down and exit 0.
"""
import importlib.util
import json
import os
import re
import sys
import threading
import time
import traceback

JOIN_S = 5.0
_tls = threading.local()


def log(group, msg):
    sys.stderr.write('[node_host_%s] %s\n' % (group, msg))
    sys.stderr.flush()


# ------------------------------------------------------------------ entry point resolution
_ENTRY_RE = re.compile(r"#\s*EASY-INSTALL-ENTRY-SCRIPT:\s*'([^']+)','([^']+)','([^']+)'")


def load_main(path, module_name):
    """``main`` callable of an installed node executable at ``path``.

    setuptools console script (``# EASY-INSTALL-ENTRY-SCRIPT: 'dist','console_scripts','x'``):
    its entry point; any other Python script: its module-level ``main`` (loaded with
    ``__name__ != '__main__'``, its own directory on sys.path as when run directly)."""
    real = os.path.realpath(path)
    with open(real, encoding='utf-8', errors='replace') as f:
        head = f.read(4096)
    m = _ENTRY_RE.search(head)
    if m:
        from importlib.metadata import distribution
        dist, group, name = m.groups()
        for ep in distribution(dist).entry_points:
            if ep.group == group and ep.name == name:
                return ep.load()
        raise RuntimeError('%s: entry point %s/%s not found in %s' % (path, group, name, dist))
    d = os.path.dirname(real)
    if d not in sys.path:
        sys.path.insert(0, d)
    spec = importlib.util.spec_from_file_location(module_name, real)
    if spec is None or spec.loader is None:
        raise RuntimeError('%s is not a loadable Python file' % path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = mod
    spec.loader.exec_module(mod)
    main = getattr(mod, 'main', None)
    if not callable(main):
        raise RuntimeError('%s has no main()' % path)
    return main


def split_log_level(ros_args):
    """(ros_args without ``--log-level X``, [level, ...]): logging is per process in rcl, so
    a node's own ``--log-level`` is applied to its logger instead."""
    out, levels, i = [], [], 0
    while i < len(ros_args):
        if ros_args[i] == '--log-level' and i + 1 < len(ros_args):
            levels.append(ros_args[i + 1])
            i += 2
            continue
        out.append(ros_args[i])
        i += 1
    return out, levels


# ------------------------------------------------------------------------------- rclpy shims
def install_shims():
    import rclpy
    import rclpy.node
    from rclpy.executors import SingleThreadedExecutor

    real_shutdown, real_try_shutdown = rclpy.shutdown, rclpy.try_shutdown

    def hosted():
        return getattr(_tls, 'node', None) is not None

    def init(*_a, **_kw):          # the host owns the one context
        return None

    def shutdown(*a, **kw):
        return None if hosted() else real_shutdown(*a, **kw)

    def try_shutdown(*a, **kw):
        return None if hosted() else real_try_shutdown(*a, **kw)

    def get_global_executor():     # one executor per thread: nodes never share a wait set
        ex = getattr(_tls, 'executor', None)
        if ex is None:
            ex = _tls.executor = SingleThreadedExecutor()
        return ex

    orig_init = rclpy.node.Node.__init__

    def node_init(self, node_name, *args, **kwargs):
        ros_args = getattr(_tls, 'ros_args', None)
        if ros_args is not None and kwargs.get('cli_args') is None:
            kwargs['cli_args'] = list(ros_args)
            kwargs.setdefault('use_global_arguments', False)
        orig_init(self, node_name, *args, **kwargs)

    rclpy.init = init
    rclpy.shutdown = shutdown
    rclpy.try_shutdown = try_shutdown
    rclpy.get_global_executor = get_global_executor
    rclpy.node.Node.__init__ = node_init
    try:
        import mower_control.crash_record as cr
        cr.install = lambda *_a, **_kw: None
    except ImportError:
        pass
    return real_shutdown


# ------------------------------------------------------------------------------------- host
class Host:
    def __init__(self, spec, write_record=None):
        self.group = spec['group']
        self.nodes = spec['nodes']
        self.write_record = write_record
        self.wake = threading.Event()
        self.lock = threading.Lock()
        self.ended = []            # (name, why, traceback text)
        self.threads = []
        self.stopping = False

    def _run(self, entry, main):
        name = entry['name']
        _tls.node = name
        ros_args, levels = split_log_level(entry.get('ros_args') or [])
        _tls.ros_args = ros_args
        why, tb = 'main() returned', ''
        try:
            if levels:
                import rclpy.logging
                rclpy.logging.set_logger_level(
                    name, getattr(rclpy.logging.LoggingSeverity, levels[-1].upper()))
            main()
        except SystemExit as e:
            why = 'sys.exit(%r)' % (e.code,)
        except BaseException as e:  # noqa: B902 -- report everything, the host decides
            why = '%s: %s' % (type(e).__name__, e)
            tb = traceback.format_exc()
        with self.lock:
            self.ended.append((name, why, tb))
        self.wake.set()

    def start(self, mains):
        for entry, main in mains:
            t = threading.Thread(target=self._run, args=(entry, main),
                                 name='node:%s' % entry['name'], daemon=True)
            self.threads.append(t)
            t.start()

    def wait(self, ok):
        """Block until the context shuts down (-> 0) or a node ends early (-> 1)."""
        while True:
            if not ok():
                return 0
            with self.lock:
                if self.ended:
                    return 1
            self.wake.wait(0.5)
            self.wake.clear()

    def report_failure(self):
        with self.lock:
            ended = list(self.ended)
        for name, why, tb in ended:
            log(self.group, 'node %s ENDED while the host was running: %s; stopping the whole '
                'group (%s) so launch respawns it' % (name, why,
                                                      ', '.join(n['name'] for n in self.nodes)))
            if tb:
                sys.stderr.write(tb)
            if self.write_record is not None:
                try:
                    self.write_record(name, 'node_host_%s: node %s ended: %s\n%s'
                                      % (self.group, name, why, tb))
                except Exception:  # noqa: B902 -- never mask the exit
                    pass

    def join(self, timeout=JOIN_S):
        end = time.monotonic() + timeout
        for t in self.threads:
            t.join(max(0.0, end - time.monotonic()))
        return [t.name for t in self.threads if t.is_alive()]


def run(spec, rclpy_args=None):
    """Run a host to completion; returns the process exit code."""
    group = spec['group']
    write_record = None
    try:
        from mower_control import crash_record
        crash_record.install('node_host_%s' % group)

        def write_record(name, text):  # noqa: E306
            crash_record.write_record(name, text)
    except ImportError:
        pass
    import rclpy
    rclpy.init(args=rclpy_args or [sys.argv[0]])
    real_shutdown = install_shims()
    host = Host(spec, write_record)
    mains = []
    for i, entry in enumerate(spec['nodes']):
        mains.append((entry, load_main(entry['path'], '_node_host_%d_%s' % (
            i, re.sub(r'\W', '_', entry['name'])))))
    log(group, 'hosting %d nodes in pid %d: %s' % (
        len(mains), os.getpid(), ', '.join(e['name'] for e, _ in mains)))
    host.start(mains)
    try:
        code = host.wait(rclpy.ok)
    except KeyboardInterrupt:      # rclpy's SIGINT handler chains to the default one
        code = 0
    if code:
        host.report_failure()
    host.stopping = True
    try:
        real_shutdown()
    except Exception:  # noqa: B902 -- already shut down by the signal handler
        pass
    stuck = host.join()
    if stuck:
        log(group, 'threads still running after %.0f s: %s' % (JOIN_S, ', '.join(stuck)))
    return code


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        sys.stderr.write('usage: node_host <spec.json>\n')
        return 2
    with open(argv[0], encoding='utf-8') as f:
        spec = json.load(f)
    code = run(spec)
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)   # node threads (daemon or not) must not keep a failed host alive


if __name__ == '__main__':
    main()
