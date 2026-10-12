# SPDX-License-Identifier: Apache-2.0
"""B6: scenario --stub-state publishes nothing (/motion_enabled, high_level_status) until
/sim/status has been received; and main() refuses a non-isolated graph. Real rclpy on the
conftest's private localhost-only domain."""

import json
import time

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('nav2_msgs')
HighLevelStatus = pytest.importorskip('mowgli_interfaces.msg').HighLevelStatus


@pytest.fixture
def ros(monkeypatch):
    # the unit test may run in a checkout on the robot (marker paths exist there)
    monkeypatch.setenv('MOWER_SIM_ALLOW_ON_ROBOT', '1')
    rclpy.init()
    yield
    rclpy.shutdown()


def _spin(executor, seconds, until=lambda: False):
    end = time.monotonic() + seconds
    while time.monotonic() < end and not until():
        executor.spin_once(timeout_sec=0.05)
    return until()


def test_stub_silent_until_sim_status(ros):
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, String

    from mower_sim.scenario import ScenarioClient

    client = ScenarioClient('test_stub_client', stub_state='TRANSIT')
    probe = rclpy.create_node('test_stub_probe')
    got_motion, got_hl = [], []
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    probe.create_subscription(Bool, '/motion_enabled', got_motion.append, latched)
    probe.create_subscription(HighLevelStatus, '/behavior_tree_node/high_level_status',
                              got_hl.append, 10)
    sim_pub = probe.create_publisher(String, '/sim/status', 10)
    ex = SingleThreadedExecutor()
    ex.add_node(client)
    ex.add_node(probe)
    try:
        _spin(ex, 1.5)
        assert got_motion == [] and got_hl == []
        assert probe.count_publishers('/motion_enabled') == 0
        assert probe.count_publishers('/behavior_tree_node/high_level_status') == 0
        assert client.status is None

        status = json.dumps({'x': 0.0, 'y': 0.0, 'collisions': 0})

        def published():
            sim_pub.publish(String(data=status))
            return bool(got_motion) and bool(got_hl)
        assert _spin(ex, 10.0, published), 'stub never started after /sim/status'
        assert got_motion[-1].data is True
        assert got_hl[-1].state == HighLevelStatus.HIGH_LEVEL_STATE_AUTONOMOUS
        assert got_hl[-1].state_name == 'TRANSIT'
    finally:
        ex.shutdown()
        client.destroy_node()
        probe.destroy_node()


def test_no_stub_without_stub_state(ros):
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String

    from mower_sim.scenario import ScenarioClient

    client = ScenarioClient('test_nostub_client', stub_state='')
    probe = rclpy.create_node('test_nostub_probe')
    sim_pub = probe.create_publisher(String, '/sim/status', 10)
    ex = SingleThreadedExecutor()
    ex.add_node(client)
    ex.add_node(probe)
    try:
        def got():
            sim_pub.publish(String(data='{"x": 1.0}'))
            return client.status is not None
        assert _spin(ex, 10.0, got)
        _spin(ex, 0.5)
        assert probe.count_publishers('/motion_enabled') == 0
    finally:
        ex.shutdown()
        client.destroy_node()
        probe.destroy_node()


def test_main_refuses_domain_0(monkeypatch, capsys):
    from mower_sim import scenario
    monkeypatch.setenv('ROS_DOMAIN_ID', '0')
    monkeypatch.delenv('MOWER_SIM_ALLOW_LAN', raising=False)
    monkeypatch.setattr(scenario.rclpy, 'init',
                        lambda *a, **k: pytest.fail('rclpy.init before the guard'))
    assert scenario.main(['navigate', '--x', '1', '--y', '0']) == 2
    assert 'refuses' in capsys.readouterr().err


def test_sim_node_main_refuses_without_localhost(monkeypatch):
    from mower_sim import sim_node
    monkeypatch.setenv('ROS_DOMAIN_ID', '7')
    monkeypatch.delenv('ROS_LOCALHOST_ONLY', raising=False)
    monkeypatch.delenv('ROS_AUTOMATIC_DISCOVERY_RANGE', raising=False)
    monkeypatch.delenv('MOWER_SIM_ALLOW_LAN', raising=False)
    monkeypatch.setattr(sim_node.rclpy, 'init',
                        lambda *a, **k: pytest.fail('rclpy.init before the guard'))
    with pytest.raises(SystemExit) as e:
        sim_node.main()
    assert e.value.code == 2
