"""bag_recorder node end to end (real rclpy + a real `ros2 bag record` child): the ~/enable
switch stops/starts the recorder and persists, ~/get_state and /bag_recorder/status report it,
and a mission (high_level_status leaving IDLE and coming back) produces a per-mow pin.

Skipped on the host rclpy shim and when mowgli_interfaces / `ros2 bag` are not available
(run inside the dev image: ./scripts/dev_build.sh test --packages-select mower_control).
"""

import json
import os
import shutil
import subprocess
import sys
import time

import pytest

import conftest

pytestmark = pytest.mark.skipif(not conftest.HAS_RCLPY or shutil.which('ros2') is None,
                                reason='needs real rclpy and the ros2 CLI')

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _spin_until(node, pred, timeout):
    import rclpy
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if pred():
            return True
    return False


def _call(node, client, req, timeout=30.0):
    import rclpy
    assert client.wait_for_service(timeout_sec=20.0), client.srv_name
    fut = client.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    assert fut.done(), 'service call timed out'
    return fut.result()


@pytest.fixture
def recorder(tmp_path):
    pytest.importorskip('mowgli_interfaces.msg')
    import rclpy
    ns = 'bagrec_test_%d' % os.getpid()
    args = [sys.executable, '-m', 'mower_control.bag_recorder', '--ros-args',
            '-r', '__node:=bag_recorder', '-r', '__ns:=/' + ns,
            '-p', 'root:=%s' % (tmp_path / 'bags'),
            '-p', 'incidents_dir:=%s' % (tmp_path / 'incidents'),
            '-p', 'crash_dir:=%s' % (tmp_path / 'crashes'),
            '-p', 'mows_dir:=%s' % (tmp_path / 'mows'),
            '-p', 'state_file:=%s' % (tmp_path / 'state.json'),
            '-p', 'split_s:=2', '-p', 'stop_after_idle_s:=0.5',
            '-p', 'min_free_gb:=0.0', '-p', 'polling_ms:=500',
            '-p', "topics:=['/bagrec_test_chatter']"]
    env = dict(os.environ, PYTHONPATH=PKG + os.pathsep + os.environ.get('PYTHONPATH', ''))
    proc = subprocess.Popen(args, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node('bagrec_test_client')
    try:
        yield ns, node, tmp_path, proc
    finally:
        node.destroy_node()
        proc.send_signal(2)
        try:
            proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            proc.kill()
        if rclpy.ok():
            rclpy.shutdown()


def _children(pid):
    out = subprocess.run(['pgrep', '-P', str(pid), '-f', 'bag record'],
                         stdout=subprocess.PIPE, text=True).stdout.split()
    return [int(p) for p in out]


def test_enable_switch_persists_and_mow_pin(recorder):
    from std_msgs.msg import String
    from std_srvs.srv import SetBool, Trigger
    from mowgli_interfaces.msg import HighLevelStatus
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
    ns, node, tmp, proc = recorder

    statuses = []
    latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/%s/bag_recorder/status' % ns,
                             lambda m: statuses.append(json.loads(m.data)), latched)
    get_state = node.create_client(Trigger, '/%s/bag_recorder/get_state' % ns)
    enable = node.create_client(SetBool, '/%s/bag_recorder/enable' % ns)
    chatter = node.create_publisher(String, '/bagrec_test_chatter', 10)
    hl = node.create_publisher(HighLevelStatus, '/behavior_tree_node/high_level_status', 10)

    st = json.loads(_call(node, get_state, Trigger.Request()).message)
    assert st['enabled'] is True and st['topics'] == 1 and st['mow_pins'] is True
    assert _spin_until(node, lambda: _children(proc.pid), 15.0), 'recorder child not started'

    # ---- a mow: status leaves IDLE, data flows over a few 2 s splits, back to IDLE
    def publish_for(seconds, state):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            chatter.publish(String(data='x' * 100))
            hl.publish(HighLevelStatus(state=2, state_name=state))
            _spin_until(node, lambda: False, 0.1)

    publish_for(1.0, 'IDLE')
    publish_for(6.0, 'MOWING')
    st = json.loads(_call(node, get_state, Trigger.Request()).message)
    assert st['mow'] is not None
    mow_dir = tmp / 'mows' / st['mow']
    publish_for(8.0, 'IDLE')            # stop after 0.5 s idle, the last split closes
    assert _spin_until(node, lambda: json.loads(
        _call(node, get_state, Trigger.Request()).message)['mow'] is None, 20.0)
    info = json.loads((mow_dir / 'incident.json').read_text())
    assert info['reason'] == 'mow' and info['end'] is not None
    files = [f for f in os.listdir(mow_dir) if f.endswith('.zstd')]
    assert len(files) >= 3, files                # split 0 + several mow splits
    assert sorted(files) == sorted(info['files'])

    # ---- off: the child is stopped cleanly, state persisted, status says so
    res = _call(node, enable, SetBool.Request(data=False))
    assert res.success and json.loads(res.message)['enabled'] is False
    assert json.loads((tmp / 'state.json').read_text())['enabled'] is False
    assert _spin_until(node, lambda: not _children(proc.pid), 20.0)
    assert _spin_until(node, lambda: statuses and statuses[-1]['enabled'] is False, 10.0)
    st = json.loads(_call(node, get_state, Trigger.Request()).message)
    assert st['recording'] is False and st['restarts'] == 0
    sessions_off = sorted(os.listdir(tmp / 'bags'))
    _spin_until(node, lambda: False, 3.0)
    assert not _children(proc.pid), 'disabled recorder restarted its child'

    # ---- on again: a new session starts at once
    res = _call(node, enable, SetBool.Request(data=True))
    assert res.success and json.loads(res.message)['enabled'] is True
    assert _spin_until(node, lambda: _children(proc.pid), 15.0)
    assert json.loads((tmp / 'state.json').read_text())['enabled'] is True
    assert _spin_until(node, lambda: sorted(os.listdir(tmp / 'bags')) != sessions_off, 10.0)


def test_disabled_state_file_survives_restart(tmp_path):
    """A node started with a persisted 'off' never spawns the recorder."""
    pytest.importorskip('mowgli_interfaces.msg')
    (tmp_path / 'state.json').write_text('{"enabled": false}')
    env = dict(os.environ, PYTHONPATH=PKG + os.pathsep + os.environ.get('PYTHONPATH', ''))
    proc = subprocess.Popen(
        [sys.executable, '-m', 'mower_control.bag_recorder', '--ros-args',
         '-r', '__node:=bag_recorder', '-r', '__ns:=/bagrec_off_%d' % os.getpid(),
         '-p', 'root:=%s' % (tmp_path / 'bags'), '-p', 'min_free_gb:=0.0',
         '-p', 'incidents_dir:=%s' % (tmp_path / 'inc'),
         '-p', 'crash_dir:=%s' % (tmp_path / 'crash'),
         '-p', 'mows_dir:=%s' % (tmp_path / 'mows'),
         '-p', 'state_file:=%s' % (tmp_path / 'state.json'),
         '-p', "topics:=['/bagrec_test_chatter']"],
        env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        time.sleep(6.0)
        assert proc.poll() is None
        assert not _children(proc.pid)
        assert not os.path.exists(tmp_path / 'bags') or not [
            d for d in os.listdir(tmp_path / 'bags') if d[0].isdigit()]
    finally:
        proc.send_signal(2)
        out, _ = proc.communicate(timeout=20)
    assert 'DISABLED' in out
