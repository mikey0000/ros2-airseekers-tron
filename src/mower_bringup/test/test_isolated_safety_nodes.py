"""Review 2026-11-02 H2: cmd_vel_slew, the supervisor and gui_bridge run in their own process
in mower.launch.py and sim.launch.py (never in a shared node_host group). Skipped without ROS
launch."""

import importlib.util
import os

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
LAUNCH_DIR = os.path.realpath(os.path.join(HERE, '..', '..', '..', 'launch'))
ISOLATED = {'cmd_vel_slew', 'supervisor', 'gui_bridge'}


def _flatten(entities):
    for e in entities:
        yield e
        sub = getattr(e, 'actions', None) or getattr(e, '_GroupAction__actions', None)
        if sub:
            yield from _flatten(sub)


def _nodes(launch_file):
    pytest.importorskip('launch')
    pytest.importorskip('launch_ros')
    spec = importlib.util.spec_from_file_location('isolated_' + launch_file.split('.')[0],
                                                  os.path.join(LAUNCH_DIR, launch_file))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    from launch_ros.actions import Node
    found = {}
    for e in _flatten(module.generate_launch_description().entities):
        if isinstance(e, Node):
            exe = getattr(e, '_Node__node_executable', None)
            if isinstance(exe, str) and exe in ISOLATED:
                found[exe] = e
    return found


@pytest.mark.parametrize('launch_file,expected', [
    ('mower.launch.py', ISOLATED),
    ('sim.launch.py', {'cmd_vel_slew', 'gui_bridge'}),      # the sim has no supervisor
])
def test_safety_nodes_are_standalone(launch_file, expected):
    from mower_bringup.node_host_launch import HostedNode
    found = _nodes(launch_file)
    assert set(found) == expected
    for exe, node in found.items():
        assert not isinstance(node, HostedNode), '%s is hosted in %s' % (exe, launch_file)
