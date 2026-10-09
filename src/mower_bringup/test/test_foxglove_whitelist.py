"""launch/mower.launch.py FOXGLOVE_GUI_TOPICS vs the GUI's topicMap (gui/pkg/providers/ros.go):
the GUI reads gui_bridge's low-rate /gui/* copies and the busy originals are not streamed."""

import importlib.util
import os
import re

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
STACK = os.path.realpath(os.path.join(HERE, '..', '..', '..'))
ROS_GO = os.path.join(STACK, 'third_party', 'mowglinext', 'gui', 'pkg', 'providers', 'ros.go')


@pytest.fixture(scope='module')
def whitelist():
    pytest.importorskip('launch')
    pytest.importorskip('launch_ros')
    spec = importlib.util.spec_from_file_location(
        'mower_launch_fox', os.path.join(STACK, 'launch', 'mower.launch.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return [re.compile(p) for p in module.FOXGLOVE_GUI_TOPICS]


def _allowed(whitelist, topic):
    return any(p.search(topic) for p in whitelist)


def _topic_map():
    if not os.path.exists(ROS_GO):
        pytest.skip('GUI sources not present')
    with open(ROS_GO, encoding='utf-8') as f:
        src = f.read()
    return dict(re.findall(r'^\s*"(\w+)":\s*\{"(/[^"]+)",\s*"[^"]+"\}', src, re.M))


@pytest.mark.parametrize('topic', ['/gui/obstacle_close', '/gui/diagnostics', '/gui/pose',
                                   '/gui/status', '/gui/emergency', '/gui/detections',
                                   '/gui/gnss_status', '/wheel_odom'])
def test_gui_copies_are_streamed(whitelist, topic):
    assert _allowed(whitelist, topic)


@pytest.mark.parametrize('topic', ['/vision/obstacle_close', '/diagnostics', '/odom',
                                   '/imu/data_aligned', '/mower_base/status'])
def test_busy_originals_are_not_streamed(whitelist, topic):
    assert not _allowed(whitelist, topic)


def test_gui_topic_map_uses_the_copies(whitelist):
    tm = _topic_map()
    assert tm['visionObstacleClose'] == '/gui/obstacle_close'
    assert tm['diagnostics'] == '/gui/diagnostics'
    assert tm['detections'] == '/gui/detections'
    for key in ('visionObstacleClose', 'diagnostics', 'wheelOdom', 'highLevelStatus'):
        assert _allowed(whitelist, tm[key]), key
