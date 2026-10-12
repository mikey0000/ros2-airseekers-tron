"""B6: launch/sim.launch.py runs the isolation guard as its FIRST action, refuses a
non-isolated environment before any node is visited, and pins localhost-only discovery
(ROS_AUTOMATIC_DISCOVERY_RANGE + ROS_LOCALHOST_ONLY) for its children unless
MOWER_SIM_ALLOW_LAN=1."""

import importlib.util
import os
import types

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
STACK = os.path.realpath(os.path.join(HERE, '..', '..', '..'))


def _load():
    pytest.importorskip('launch')
    pytest.importorskip('launch_ros')
    pytest.importorskip('mower_sim.isolation')
    spec = importlib.util.spec_from_file_location(
        'sim_launch', os.path.join(STACK, 'launch', 'sim.launch.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _ctx(**env):
    from mower_sim.isolation import on_robot_markers
    if on_robot_markers():          # a checkout on the robot itself
        env.setdefault('MOWER_SIM_ALLOW_ON_ROBOT', '1')
    return types.SimpleNamespace(environment=env)


def _env_actions(actions):
    from launch.actions import SetEnvironmentVariable
    out = {}
    for a in actions:
        if isinstance(a, SetEnvironmentVariable):
            name = ''.join(s.perform(None) for s in a.name)
            out[name] = ''.join(s.perform(None) for s in a.value)
    return out


def test_guard_is_first_action():
    from launch.actions import OpaqueFunction
    sim = _load()
    first = sim.generate_launch_description().entities[0]
    assert isinstance(first, OpaqueFunction)
    assert first._OpaqueFunction__function is sim._isolation_guard


@pytest.mark.parametrize('env', [{}, {'ROS_DOMAIN_ID': '0', 'ROS_LOCALHOST_ONLY': '1'},
                                 {'ROS_DOMAIN_ID': '7'}])
def test_guard_refuses(env):
    from mower_sim.isolation import IsolationError
    sim = _load()
    with pytest.raises(IsolationError):
        sim._isolation_guard(_ctx(**env))


def test_guard_pins_localhost_discovery():
    sim = _load()
    actions = sim._isolation_guard(_ctx(ROS_DOMAIN_ID='77', ROS_LOCALHOST_ONLY='1'))
    assert _env_actions(actions) == {'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
                                     'ROS_LOCALHOST_ONLY': '1'}


def test_guard_lan_override_leaves_discovery():
    from launch.actions import LogInfo
    sim = _load()
    actions = sim._isolation_guard(_ctx(ROS_DOMAIN_ID='55', MOWER_SIM_ALLOW_LAN='1'))
    assert _env_actions(actions) == {}
    assert any(isinstance(a, LogInfo) for a in actions)
