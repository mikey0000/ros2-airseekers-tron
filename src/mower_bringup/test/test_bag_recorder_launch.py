"""launch/mower.launch.py wiring of the always-on bag_recorder node (skipped without ROS launch)."""

import importlib.util
import os

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
STACK = os.path.realpath(os.path.join(HERE, '..', '..', '..'))
LAUNCH_DIR = os.path.join(STACK, 'launch')
CONTROL_SETUP = os.path.join(STACK, 'src', 'mower_control', 'setup.py')


def _load_mower_launch():
    pytest.importorskip('launch')
    pytest.importorskip('launch_ros')
    spec = importlib.util.spec_from_file_location('mower_launch_bag',
                                                  os.path.join(LAUNCH_DIR, 'mower.launch.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _flatten(entities):
    for e in entities:
        yield e
        sub = getattr(e, 'actions', None) or getattr(e, '_GroupAction__actions', None)
        if sub:
            yield from _flatten(sub)


@pytest.fixture(scope='module')
def description():
    module = _load_mower_launch()
    return list(_flatten(module.generate_launch_description().entities))


def _arg(description, name):
    from launch.actions import DeclareLaunchArgument
    found = [a for a in description if isinstance(a, DeclareLaunchArgument) and a.name == name]
    assert found, 'no DeclareLaunchArgument %s' % name
    return found[0]


def _default(arg):
    from launch.utilities import perform_substitutions
    from launch import LaunchContext
    return perform_substitutions(LaunchContext(), arg.default_value)


def _bag_node(description):
    from launch_ros.actions import Node
    nodes = []
    for a in description:
        if not isinstance(a, Node):
            continue
        if getattr(a, '_Node__package', None) == 'mower_control' and \
                _text(getattr(a, '_Node__node_executable', None)) == 'bag_recorder':
            nodes.append(a)
    assert len(nodes) == 1
    return nodes[0]


def _text(subs):
    from launch import LaunchContext
    from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions
    if subs is None:
        return None
    if isinstance(subs, str):
        return subs
    return perform_substitutions(LaunchContext(), normalize_to_list_of_substitutions(subs))


def test_declared_arguments(description):
    assert _default(_arg(description, 'bag_recorder')) == 'true'
    assert _default(_arg(description, 'bag_max_gb')) == '3.0'
    assert _default(_arg(description, 'bag_min_free_gb')) == '3.0'
    # 2026-10-09: mow_recorder folded into bag_recorder (mow pins), off by default
    assert _default(_arg(description, 'mow_recorder')) == 'false'


def test_node_respawns(description):
    node = _bag_node(description)
    respawn = None
    for attr in ('_ExecuteProcess__respawn', '_ExecuteLocal__respawn'):
        if hasattr(node, attr):
            respawn = getattr(node, attr)
            break
    else:
        pytest.fail('no respawn attribute on %s' % type(node).__mro__)
    assert respawn in (True, 'true') or _text(respawn).lower() == 'true'


def test_node_condition_follows_bag_recorder_argument(description):
    from launch import LaunchContext
    from launch.conditions import IfCondition
    node = _bag_node(description)
    cond = node.condition
    assert isinstance(cond, IfCondition)
    for value, expected in (('false', False), ('true', True)):
        ctx = LaunchContext()
        ctx.launch_configurations['bag_recorder'] = value
        assert cond.evaluate(ctx) is expected


def test_node_parameters(description):
    node = _bag_node(description)
    params = node._Node__parameters
    flat = {}
    for p in params:
        if isinstance(p, dict):
            flat.update(p)
    keys = {}
    for k, v in flat.items():
        keys[_text(k)] = v
    # launch_ros normalises parameter values to substitutions holding a YAML scalar
    import yaml
    assert yaml.safe_load(_text(keys['root'])) == '/userdata/ros2/bags'
    assert yaml.safe_load(_text(keys['incidents_dir'])) == '/userdata/ros2/incidents'
    assert 'max_total_gb' in keys and 'min_free_gb' in keys
    assert yaml.safe_load(_text(keys['state_file'])) == '/userdata/ros2/bag_recorder.json'
    assert yaml.safe_load(_text(keys['mows_dir'])) == '/userdata/ros2/mows'


def test_mow_pins_follow_mow_recorder_argument(description):
    from launch import LaunchContext
    node = _bag_node(description)
    flat = {}
    for p in node._Node__parameters:
        if isinstance(p, dict):
            flat.update(p)
    pv = {_text(k): v for k, v in flat.items()}['mow_pins']
    for value, expected in (('false', 'True'), ('true', 'False'), ('True', 'False')):
        ctx = LaunchContext()
        ctx.launch_configurations['mow_recorder'] = value
        subs = pv.value if hasattr(pv, 'value') else pv
        if not isinstance(subs, (list, tuple)):
            subs = [subs]
        from launch.utilities import perform_substitutions
        assert perform_substitutions(ctx, list(subs)) == expected, value


def test_setup_py_entry_point():
    with open(CONTROL_SETUP, encoding='utf-8') as f:
        assert "'bag_recorder = mower_control.bag_recorder:main'" in f.read()
