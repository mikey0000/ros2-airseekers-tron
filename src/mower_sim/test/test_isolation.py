# SPDX-License-Identifier: Apache-2.0
"""B6 isolation guard (mower_sim/isolation.py): pure env / marker checks, no rclpy."""

import pytest

from mower_sim.isolation import (IsolationError, check_isolation, isolation_problems,
                                 lan_allowed, on_robot_markers)



def _check(env, exists=lambda path: False, ismount=lambda path: False):
    warned = []
    out = check_isolation('test', env=env, exists=exists, ismount=ismount,
                          warn=warned.append)
    assert out == warned
    return warned


def test_rejects_unset_domain():
    with pytest.raises(IsolationError, match='ROS_DOMAIN_ID is not set'):
        _check({'ROS_LOCALHOST_ONLY': '1'})


def test_rejects_domain_0_even_localhost_only():
    with pytest.raises(IsolationError, match='ROS_DOMAIN_ID=0'):
        _check({'ROS_DOMAIN_ID': '0', 'ROS_LOCALHOST_ONLY': '1',
                'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'})


def test_rejects_non_integer_domain():
    with pytest.raises(IsolationError, match='not an integer'):
        _check({'ROS_DOMAIN_ID': 'abc', 'ROS_LOCALHOST_ONLY': '1'})


@pytest.mark.parametrize('extra', [{}, {'ROS_LOCALHOST_ONLY': '0'},
                                   {'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET'},
                                   {'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SYSTEM_DEFAULT'}])
def test_rejects_domain_7_without_localhost_discovery(extra):
    with pytest.raises(IsolationError, match='not localhost-only'):
        _check(dict({'ROS_DOMAIN_ID': '7'}, **extra))


def test_accepts_77_localhost_only():
    assert _check({'ROS_DOMAIN_ID': '77', 'ROS_LOCALHOST_ONLY': '1'}) == []


@pytest.mark.parametrize('rng', ['LOCALHOST', 'localhost', 'OFF'])
def test_accepts_discovery_range_localhost(rng):
    assert _check({'ROS_DOMAIN_ID': '42', 'ROS_AUTOMATIC_DISCOVERY_RANGE': rng}) == []


def test_rejects_static_peers():
    with pytest.raises(IsolationError, match='ROS_STATIC_PEERS'):
        _check({'ROS_DOMAIN_ID': '42', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
                'ROS_STATIC_PEERS': '192.168.1.105'})


def test_lan_override_allows_and_warns():
    env = {'ROS_DOMAIN_ID': '0', 'MOWER_SIM_ALLOW_LAN': '1'}
    warned = _check(env)
    assert len(warned) == 1
    assert 'MOWER_SIM_ALLOW_LAN=1' in warned[0] and 'WITHOUT DDS isolation' in warned[0]
    assert 'ROS_DOMAIN_ID=0' in warned[0]
    assert lan_allowed(env)
    assert not lan_allowed({'MOWER_SIM_ALLOW_LAN': '0'})


def test_lan_override_does_not_cover_the_robot():
    with pytest.raises(IsolationError, match='on the mower itself'):
        _check({'ROS_DOMAIN_ID': '7', 'MOWER_SIM_ALLOW_LAN': '1'},
               ismount=lambda p: p == '/userdata/ros2')


@pytest.mark.parametrize('marker', ['/userdata/ros2_stack', '/userdata/mower'])
def test_refuses_on_robot_paths(marker):
    env = {'ROS_DOMAIN_ID': '77', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'}
    with pytest.raises(IsolationError, match='MOWER_SIM_ALLOW_ON_ROBOT'):
        _check(env, exists=lambda p: p == marker)
    assert on_robot_markers(exists=lambda p: p == marker,
                            ismount=lambda p: False) == [marker]


@pytest.mark.parametrize('mount', ['/userdata', '/userdata/ros2'])
def test_refuses_on_robot_mounts(mount):
    env = {'ROS_DOMAIN_ID': '77', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'}
    with pytest.raises(IsolationError, match='on the mower itself'):
        _check(env, ismount=lambda p: p == mount)


def test_plain_userdata_ros2_dir_is_not_the_robot():
    # nodes and the robot image create /userdata/ros2/... in any container (dev image too)
    env = {'ROS_DOMAIN_ID': '77', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'}
    assert _check(env, exists=lambda p: p.startswith('/userdata/ros2/')
                  or p in ('/userdata', '/userdata/ros2')) == []


def test_on_robot_override_warns():
    env = {'ROS_DOMAIN_ID': '77', 'ROS_LOCALHOST_ONLY': '1', 'MOWER_SIM_ALLOW_ON_ROBOT': '1'}
    warned = _check(env, exists=lambda p: True, ismount=lambda p: True)
    assert len(warned) == 1 and 'ON THE MOWER' in warned[0]


def test_default_warn_goes_to_stderr(capsys):
    check_isolation('t', env={'ROS_DOMAIN_ID': '0', 'MOWER_SIM_ALLOW_LAN': 'true'},
                    exists=lambda p: False, ismount=lambda p: False)
    assert 'MOWER_SIM_ALLOW_LAN=1' in capsys.readouterr().err


def test_problems_empty_when_isolated():
    assert isolation_problems({'ROS_DOMAIN_ID': '150', 'ROS_LOCALHOST_ONLY': '1',
                               'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'}) == []
