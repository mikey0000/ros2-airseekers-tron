"""localization_mode (2026-10-09 REP-105 dual EKF) in launch/nav2.launch.py + mower.launch.py.

Renders the launch files' localization pieces against a LaunchContext (no processes):
dual (default) = ekf_node (odom, ekf_dual.yaml) + ekf_map (remapped to
/odometry/filtered_map), navsat anchored on ekf_map, no static map -> odom, gui_bridge not
relaying; single = the old ekf.yaml node + static identity + relay. Needs launch/launch_ros
(skipped otherwise) and, for the FindPackageShare paths, the built workspace sourced.
"""

import importlib.util
import os

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
STACK = os.path.realpath(os.path.join(HERE, '..', '..', '..'))
LAUNCH_DIR = os.path.join(STACK, 'launch')


def _installed(package):
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory(package)
        return True
    except Exception:  # noqa: BLE001 - not sourced / not built
        return False


pytestmark = pytest.mark.skipif(not _installed('mower_localization'),
                                reason='needs launch_ros and the built workspace sourced')


def _load(name):
    pytest.importorskip('launch')
    pytest.importorskip('launch_ros')
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'),
                                                  os.path.join(LAUNCH_DIR, name))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _context(ld, **overrides):
    """LaunchContext with every declared argument at its default, then ``overrides``."""
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    context = LaunchContext()
    for entity in ld.entities:
        if isinstance(entity, DeclareLaunchArgument) and entity.default_value is not None:
            context.launch_configurations[entity.name] = ''.join(
                s.perform(context) for s in entity.default_value)
    context.launch_configurations.update(overrides)
    return context


def _text(context, subs):
    from launch.utilities import perform_substitutions
    if isinstance(subs, str):
        return subs
    return perform_substitutions(context, list(subs))


def _nodes(ld, context):
    """Node actions of ``ld`` (OpaqueFunctions expanded), as dicts."""
    from launch.actions import OpaqueFunction
    from launch_ros.actions import Node
    out = []
    todo = list(ld.entities)
    while todo:
        e = todo.pop(0)
        if isinstance(e, OpaqueFunction):
            todo[0:0] = list(e.execute(context) or [])
            continue
        if not isinstance(e, Node):
            continue
        cond = e.condition
        if cond is not None and not cond.evaluate(context):
            continue
        name = e._Node__node_name
        remaps = {_text(context, a): _text(context, b) for a, b in (e._Node__remappings or [])}
        out.append(_NodeView(context, e, name if isinstance(name, str) else _text(context, name),
                             remaps))
    return out


class _NodeView(dict):
    """name/remaps eagerly; files/params evaluated on access (robot_state_publisher's
    xacro Command must not run here)."""

    def __init__(self, context, node, name, remaps):
        super().__init__(name=name, remaps=remaps)
        self._context, self._node = context, node

    def __missing__(self, key):
        if key in ('files', 'params'):
            self['files'], self['params'] = _params(self._context, self._node)
            return self[key]
        raise KeyError(key)


def _params(context, node):
    """(parameter file basenames, merged evaluated dict) of a launch_ros Node."""
    from launch_ros.utilities import evaluate_parameters
    files, params = [], {}
    for p in evaluate_parameters(context, node._Node__parameters or []):
        if isinstance(p, dict):
            params.update(p)
        else:
            files.append(os.path.basename(str(p)))
    return files, params


def _by_name(nodes, name):
    found = [n for n in nodes if n['name'] == name]
    assert len(found) <= 1, name
    return found[0] if found else None


def _nav2(**overrides):
    module = _load('nav2.launch.py')
    ld = module.generate_launch_description()
    context = _context(ld, **overrides)
    return _nodes(ld, context), context


def test_dual_is_the_default_and_runs_two_filters():
    nodes, _ = _nav2()
    odom, ekf_map = _by_name(nodes, 'ekf_node'), _by_name(nodes, 'ekf_map')
    assert odom is not None and ekf_map is not None
    assert odom['files'] == ['ekf_dual.yaml'] and ekf_map['files'] == ['ekf_dual.yaml']
    assert odom['remaps'] == {}, 'ekf_node keeps /odometry/filtered (Nav2 odom_topic)'
    assert ekf_map['remaps'] == {'odometry/filtered': '/odometry/filtered_map'}
    navsat = _by_name(nodes, 'navsat_transform_node')
    assert navsat['remaps']['odometry/filtered'] == '/odometry/filtered_map'
    assert navsat['remaps']['gps/fix'] == '/fix_gated'
    assert _by_name(nodes, 'localization_monitor') is not None


def test_dual_vio_adds_the_dual_overlay_to_both_filters():
    nodes, _ = _nav2(vio='true')
    for name in ('ekf_node', 'ekf_map'):
        assert _by_name(nodes, name)['files'] == ['ekf_dual.yaml', 'ekf_dual_vio.yaml']
    assert _by_name(nodes, 'vio_gate') is not None


def test_single_is_the_pre_dual_node():
    nodes, _ = _nav2(localization_mode='single')
    assert _by_name(nodes, 'ekf_map') is None
    assert _by_name(nodes, 'ekf_node')['files'] == ['ekf.yaml']
    assert 'odometry/filtered' not in _by_name(nodes, 'navsat_transform_node')['remaps']
    nodes, _ = _nav2(localization_mode='single', vio='true')
    assert _by_name(nodes, 'ekf_node')['files'] == ['ekf.yaml', 'ekf_vio.yaml']


def test_unknown_mode_is_refused():
    with pytest.raises(ValueError):
        _nav2(localization_mode='triple')


def test_gps_gate_gets_quality_covariance():
    nodes, _ = _nav2()
    gate = _by_name(nodes, 'gps_gate')
    assert 'quality_covariance' in gate['params']


def test_datum_is_kept_in_both_modes():
    for mode in ('dual', 'single'):
        nodes, context = _nav2(localization_mode=mode, datum_lat='48.1', datum_lon='11.5',
                               datum_yaw='0.3')
        params = _by_name(nodes, 'navsat_transform_node')['params']
        assert params['wait_for_datum'] is True
        assert list(params['datum']) == [48.1, 11.5, 0.3]


def _mower(**overrides):
    module = _load('mower.launch.py')
    ld = module.generate_launch_description()
    # datum_lat/lon default to '' (filled by an OpaqueFunction from datum.env at launch)
    context = _context(ld, **{'datum_lat': '0.0', 'datum_lon': '0.0', **overrides})
    from launch_ros.actions import Node
    found = {}
    for e in ld.entities:
        if isinstance(e, Node) and e._Node__node_name in ('static_map_odom', 'gui_bridge'):
            found[e._Node__node_name] = e
    return found, context


@pytest.mark.parametrize('mode,localization,static,relay', [
    ('dual', 'true', False, False),
    ('single', 'true', True, True),
    ('dual', 'false', True, True),     # bench without localization: identity + relay as before
])
def test_mower_launch_map_odom_owner(mode, localization, static, relay):
    found, context = _mower(localization_mode=mode, localization=localization)
    assert found['static_map_odom'].condition.evaluate(context) is static
    topic = _params(context, found['gui_bridge'])[1]['filtered_odom_topic']
    assert topic == ('/odometry/filtered' if relay else '/odometry/filtered_map')


def test_static_map_odom_can_still_be_switched_off():
    found, context = _mower(localization_mode='single', publish_static_map_odom='false')
    assert found['static_map_odom'].condition.evaluate(context) is False
