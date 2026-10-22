"""node_host (mower_bringup/node_host.py) and its launch side (node_host_launch.py).

The host runs fake node scripts in a subprocess: each writes what it ended up with (name,
namespace, parameter, resolved topic, thread) to a JSON file, and a ``.cleanup`` marker from
its own ``finally``.
"""
import json
import os
import signal
import subprocess
import sys
import time

import pytest

from mower_bringup import node_host

FAKE = '''#!/usr/bin/env python3
import json, os, threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def main(args=None):
    rclpy.init(args=args)
    node = Node('fake')
    p = node.declare_parameter('p', 0).value
    crash_after = node.declare_parameter('crash_after', 0.0).value
    pub = node.create_publisher(String, 'out', 1)
    out = os.path.join(os.environ['NH_OUT'], node.get_name())
    with open(out + '.json', 'w') as f:
        json.dump({'name': node.get_name(), 'ns': node.get_namespace(), 'p': p,
                   'topic': pub.topic_name, 'thread': threading.current_thread().name,
                   'pid': os.getpid()}, f)
    if crash_after > 0:
        def boom():
            raise RuntimeError('fake node failure')
        node.create_timer(crash_after, boom)
    try:
        rclpy.spin(node)
    finally:
        open(out + '.cleanup', 'w').close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
'''


def _spec(tmp_path, nodes):
    script = tmp_path / 'fake_node.py'
    script.write_text(FAKE)
    entries = []
    for name, ns, p, extra in nodes:
        args = ['--ros-args', '-r', '__node:=%s' % name, '-p', 'p:=%d' % p,
                '-r', 'out:=%s_out' % name] + extra
        if ns:
            args += ['-r', '__ns:=%s' % ns]
        entries.append({'name': name, 'package': 'test', 'executable': 'fake_node.py',
                        'path': str(script), 'ros_args': args})
    spec = tmp_path / 'spec.json'
    spec.write_text(json.dumps({'group': 'test', 'nodes': entries}))
    return spec


def _start(tmp_path, spec):
    env = dict(os.environ, NH_OUT=str(tmp_path), MOWER_CRASH_DIR=str(tmp_path / 'crashes'))
    return subprocess.Popen([sys.executable, '-m', 'mower_bringup.node_host', str(spec)],
                            env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)


def _wait_for(paths, timeout=20.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if all(p.exists() for p in paths):
            return True
        time.sleep(0.1)
    return False


def test_hosts_n_nodes_with_their_own_names_params_and_remaps(tmp_path):
    spec = _spec(tmp_path, [('alpha', '', 1, []), ('beta', '/ns', 2, []),
                            ('gamma', '', 3, ['--log-level', 'debug'])])
    proc = _start(tmp_path, spec)
    try:
        files = [tmp_path / ('%s.json' % n) for n in ('alpha', 'beta', 'gamma')]
        assert _wait_for(files), proc.stdout.read1().decode() if proc.poll() else 'timeout'
        got = {f.stem: json.loads(f.read_text()) for f in files}
        assert got['alpha']['p'] == 1 and got['beta']['p'] == 2 and got['gamma']['p'] == 3
        assert got['beta']['ns'] == '/ns' and got['alpha']['ns'] == '/'
        assert got['alpha']['topic'] == '/alpha_out'
        assert got['beta']['topic'] == '/ns/beta_out'
        assert len({g['pid'] for g in got.values()}) == 1          # one process
        assert len({g['thread'] for g in got.values()}) == 3       # one thread per node
        proc.send_signal(signal.SIGINT)
        assert proc.wait(15) == 0
        for n in ('alpha', 'beta', 'gamma'):
            assert (tmp_path / ('%s.cleanup' % n)).exists()
    finally:
        if proc.poll() is None:
            proc.kill()


def test_one_node_failing_stops_the_whole_host_for_respawn(tmp_path):
    spec = _spec(tmp_path, [('ok_node', '', 1, []),
                            ('bad_node', '', 2, ['-p', 'crash_after:=1.0'])])
    proc = _start(tmp_path, spec)
    try:
        rc = proc.wait(30)
        out = proc.stdout.read().decode()
    finally:
        if proc.poll() is None:
            proc.kill()
    assert rc == 1, out
    assert 'node bad_node ENDED' in out and 'fake node failure' in out
    # the healthy peer was shut down cleanly (its own finally ran), not left orphaned
    assert (tmp_path / 'ok_node.cleanup').exists()
    assert (tmp_path / 'bad_node.cleanup').exists()


def test_split_log_level():
    args, levels = node_host.split_log_level(
        ['--ros-args', '-r', '__node:=x', '--log-level', 'warn', '-p', 'a:=1'])
    assert args == ['--ros-args', '-r', '__node:=x', '-p', 'a:=1'] and levels == ['warn']


def test_load_main_of_a_plain_script(tmp_path):
    script = tmp_path / 'plain.py'
    script.write_text('import sys\nRAN = __name__\ndef main():\n    return 42\n')
    main = node_host.load_main(str(script), '_nh_test_plain')
    assert main() == 42 and sys.modules['_nh_test_plain'].RAN == '_nh_test_plain'


def test_load_main_rejects_a_script_without_main(tmp_path):
    script = tmp_path / 'nomain.py'
    script.write_text('X = 1\n')
    with pytest.raises(RuntimeError):
        node_host.load_main(str(script), '_nh_test_nomain')


# ------------------------------------------------------------------------------ launch side
def _context(compose='true', armed=True):
    from launch import LaunchContext
    from mower_bringup import node_host_launch as nhl
    ctx = LaunchContext()
    ctx.launch_configurations['compose_nodes'] = compose
    if armed:
        nhl._arm(ctx)
    return ctx


def _hosted(tmp_path, name, group='g', **kw):
    from mower_bringup.node_host_launch import hosted_node
    script = tmp_path / 'fake_node.py'
    script.write_text(FAKE)
    return hosted_node(group, executable=str(script), name=name,
                       parameters=[{'p': 7}], remappings=[('out', '/x/out')], **kw)


def test_launch_registers_hosted_nodes_with_their_standalone_argv(tmp_path):
    from launch_ros.actions import Node
    from mower_bringup import node_host_launch as nhl
    ctx = _context()
    assert _hosted(tmp_path, 'n1', respawn=True, respawn_delay=2.0).execute(ctx) is None
    assert _hosted(tmp_path, 'n2').execute(ctx) is None
    _hosted(tmp_path, 'solo', group='h').execute(ctx)
    members = nhl._REGISTRY[ctx]['g']
    argv = members[0]['ros_args']
    assert argv[0] == '--ros-args' and '__node:=n1' in argv
    assert 'out:=/x/out' in argv
    pfile = argv[argv.index('--params-file') + 1]
    assert 'p: 7' in open(pfile).read()
    actions = nhl._start(ctx)
    from launch.utilities import perform_substitutions
    procs = {perform_substitutions(ctx, a.name): a for a in actions
             if a.__class__.__name__ == 'ExecuteProcess'}
    assert set(procs) == {'node_host_g', 'solo'}      # single member: its own argv, standalone
    assert procs['node_host_g']._ExecuteLocal__respawn
    assert ctx not in nhl._REGISTRY                          # late hosted nodes -> standalone


@pytest.mark.parametrize('compose,armed', [('false', True), ('true', False)])
def test_launch_without_composition_runs_the_node_itself(tmp_path, compose, armed):
    from launch_ros.actions import Node
    ctx = _context(compose, armed)
    node = _hosted(tmp_path, 'n1')
    assert isinstance(node, Node)
    import launch.actions
    called = []
    orig = launch.actions.ExecuteProcess.execute
    launch.actions.ExecuteProcess.execute = lambda self, c: called.append(self) or None
    try:
        node.execute(ctx)
    finally:
        launch.actions.ExecuteProcess.execute = orig
    assert called == [node]
