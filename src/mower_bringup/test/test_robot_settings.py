"""GUI settings -> node parameters at launch (launch/robot_settings.py + mower.launch.py).

The pure helper is tested anywhere with PyYAML. The launch-file part renders
_apply_robot_settings against a LaunchContext and needs a sourced ROS 2 Humble
environment with the workspace built (mower_mission, mower_docking,
mower_navigation installed); it is skipped otherwise.
"""

import importlib.util
import os
import re
import sys

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
STACK = os.path.realpath(os.path.join(HERE, '..', '..', '..'))
LAUNCH_DIR = os.path.join(STACK, 'launch')
GO_TABLE = os.path.join(STACK, 'third_party', 'mowglinext', 'gui', 'pkg', 'api', 'param_bindings.go')
sys.path.insert(0, LAUNCH_DIR)

import robot_settings as rs  # noqa: E402

SAMPLE = """\
# Mowgli Robot Configuration - managed by mowglinext-gui
mowgli:
  ros__parameters:
    mower_model: AirseekersTron
    datum_lat: 48.1
    tool_width: 0.24
    mowing_speed: 0.3
    transit_speed: 0.4
    battery_low_percent: 30
    rain_mode: 2
    undock_distance: 1.2
    dock_approach_distance: 1.0
    dock_max_retries: 5
    ntrip_enabled: true
    ntrip_host: caster.example
    ntrip_port: 2102
    ntrip_mountpoint: MOUNT
    ntrip_user: u
    ntrip_password: secret
    wheel_track: 0.5
"""


@pytest.fixture
def sample(tmp_path):
    path = tmp_path / 'mowgli_robot.yaml'
    path.write_text(SAMPLE)
    return str(path)


def test_missing_or_broken_file_means_package_defaults(tmp_path):
    assert rs.load_settings('') == {}
    assert rs.load_settings(str(tmp_path / 'nope.yaml')) == {}
    bad = tmp_path / 'bad.yaml'
    bad.write_text('mowgli: [unclosed')
    assert rs.load_settings(str(bad)) == {}
    assert rs.bound_values({}) == ({}, [])


def test_bound_values_map_and_coerce(sample):
    values, warnings = rs.bound_values(rs.load_settings(sample))
    assert warnings == []
    assert values['launch'] == {'cut_width_m': 0.24}
    assert values['controller_server'] == {
        'FollowCoveragePath.desired_linear_vel': 0.3, 'FollowPath.desired_linear_vel': 0.4}
    mission = values['behavior_tree_node']
    assert mission['battery_low_percent'] == 30.0 and isinstance(mission['battery_low_percent'], float)
    assert mission['rain_mode'] == 1 and isinstance(mission['rain_mode'], int)
    assert mission['undock_distance_m'] == 1.2
    assert 'rain_delay_minutes' not in mission, 'absent keys keep the package default'
    assert values['mower_docking'] == {'approach_distance': 1.0, 'max_retries': 5}
    assert values['um960_gps_driver'] == {
        'correction_source': 'ntrip', 'ntrip_host': 'caster.example', 'ntrip_port': 2102,
        'ntrip_mountpoint': 'MOUNT', 'ntrip_user': 'u', 'ntrip_password': 'secret'}


def test_ntrip_disabled_means_lora():
    values, _ = rs.bound_values({'ntrip_enabled': False})
    assert values == {'um960_gps_driver': {'correction_source': 'lora'}}


def test_bad_values_are_skipped_with_a_warning():
    values, warnings = rs.bound_values({'rain_mode': 7, 'mowing_speed': 'fast', 'dock_max_retries': 2.5})
    assert values == {}
    assert len(warnings) == 3 and all('package default' in w for w in warnings)


def test_merged_document_keeps_the_package_file(tmp_path):
    base = tmp_path / 'nav2.yaml'
    base.write_text(yaml.safe_dump({
        'controller_server': {'ros__parameters': {
            'controller_frequency': 20.0,
            'FollowPath': {'plugin': 'rpp', 'desired_linear_vel': 0.3, 'lookahead_dist': 0.6}}},
        'planner_server': {'ros__parameters': {'expected_planner_frequency': 1.0}},
    }))
    doc = rs.merged_params_document(str(base), 'controller_server',
                                    {'FollowPath.desired_linear_vel': 0.5})
    cs = doc['controller_server']['ros__parameters']
    assert cs['FollowPath'] == {'plugin': 'rpp', 'desired_linear_vel': 0.5, 'lookahead_dist': 0.6}
    assert cs['controller_frequency'] == 20.0
    assert doc['planner_server'] == {'ros__parameters': {'expected_planner_frequency': 1.0}}
    path = rs.write_params_file(doc, str(tmp_path), 'out.yaml')
    assert yaml.safe_load(open(path)) == doc


def test_overlay_document():
    assert rs.overlay_document('um960_gps_driver', {'ntrip_port': 2101}) == {
        'um960_gps_driver': {'ros__parameters': {'ntrip_port': 2101}}}


@pytest.mark.skipif(not os.path.isfile(GO_TABLE), reason='GUI sources not present')
def test_matches_the_gui_binding_table():
    """Same keys and targets as the AirseekersTron table the GUI pushes live."""
    src = open(GO_TABLE, encoding='utf-8').read()
    table = src[src.index('"AirseekersTron": {'):src.index('GuiKeys:')]
    go = {m.group(1): (m.group(2).lstrip('/'), m.group(3)) for m in re.finditer(
        r'\{Key: "([^"]+)", Node: "([^"]+)", Param: "([^"]+)"', table)}
    py = {k: (target, param) for k, (target, param, _t, _m) in rs.BINDINGS.items()}
    assert set(go) == set(py)
    for key in go:
        if key == 'tool_width':
            # Live: operation_width = tool_width - overlap; boot: cut_width_m = tool_width.
            assert go[key] == ('coverage_server', 'operation_width')
            assert py[key] == ('launch', 'cut_width_m')
        else:
            assert go[key] == py[key], key


def _load_mower_launch():
    pytest.importorskip('launch')
    pytest.importorskip('launch_ros')
    spec = importlib.util.spec_from_file_location('mower_launch', os.path.join(LAUNCH_DIR, 'mower.launch.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _render(module, settings_file, cut_width=''):
    from launch import LaunchContext
    from launch.actions import SetLaunchConfiguration
    context = LaunchContext()
    context.launch_configurations['robot_settings_file'] = settings_file
    context.launch_configurations['cut_width_m'] = cut_width
    for action in module._apply_robot_settings(context):
        if isinstance(action, SetLaunchConfiguration):
            action.execute(context)
    return context.launch_configurations


def _installed(package):
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory(package)
        return True
    except Exception:  # noqa: BLE001 - not sourced / not built
        return False


@pytest.mark.skipif(not all(_installed(p) for p in ('mower_mission', 'mower_docking', 'mower_navigation')),
                    reason='needs the built workspace sourced')
def test_launch_renders_settings_into_params_files(sample):
    cfg = _render(_load_mower_launch(), sample)
    assert cfg['cut_width_m'] == '0.24'
    mission = yaml.safe_load(open(cfg['mission_params_file']))['behavior_tree_node']['ros__parameters']
    assert mission['battery_low_percent'] == 30.0 and mission['rain_mode'] == 1
    assert mission['undock_distance_m'] == 1.2
    assert mission['rtk_timeout_s'] == 120.0, 'unbound mission params keep the package value'
    nav = yaml.safe_load(open(cfg['nav2_params_file']))['controller_server']['ros__parameters']
    assert nav['FollowCoveragePath']['desired_linear_vel'] == 0.3
    assert nav['FollowPath']['desired_linear_vel'] == 0.4
    # 2026-10-09: transit is MPPI; navigation.launch.py maps desired_linear_vel onto vx_max
    assert nav['FollowPath']['plugin'].endswith('MPPIController')
    dock = yaml.safe_load(open(cfg['docking_params_file']))['mower_docking']['ros__parameters']
    assert dock['approach_distance'] == 1.0 and dock['max_retries'] == 5
    assert dock['marker_size'] == 0.04
    gnss = yaml.safe_load(open(cfg['um960_params_file']))['um960_gps_driver']['ros__parameters']
    assert gnss['correction_source'] == 'ntrip' and gnss['ntrip_port'] == 2102


@pytest.mark.skipif(not all(_installed(p) for p in ('mower_mission', 'mower_docking', 'mower_navigation')),
                    reason='needs the built workspace sourced')
def test_launch_without_settings_uses_package_files(tmp_path):
    cfg = _render(_load_mower_launch(), str(tmp_path / 'missing.yaml'))
    assert cfg['cut_width_m'] == '0.20'
    assert cfg['mission_params_file'].endswith(os.path.join('mower_mission', 'config', 'mission.yaml'))
    assert cfg['um960_params_file'] == ''
    # An explicit cut_width_m launch argument wins over the GUI value.
    assert _render(_load_mower_launch(), str(tmp_path / 'missing.yaml'), cut_width='0.3')['cut_width_m'] == '0.3'
