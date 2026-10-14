"""launch/sim.launch.py: the mission's parameter file is a launch argument
(mission_params_file, "" = mower_mission/config/mission.yaml), passed to the mission include
explicitly, like docking_params_file (sim runs with a changed mission.yaml copy)."""

import importlib.util
import os

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


def _text(subs):
    from launch.substitutions import LaunchConfiguration
    if isinstance(subs, str):
        return subs
    if not isinstance(subs, (list, tuple)):
        subs = [subs]
    out = []
    for s in subs:
        if isinstance(s, LaunchConfiguration):
            out.append('$(%s)' % ''.join(p.perform(None) for p in s.variable_name))
        else:
            out.append(s if isinstance(s, str) else s.perform(None))
    return ''.join(out)


def test_mission_params_file_argument_declared():
    from launch.actions import DeclareLaunchArgument
    ld = _load().generate_launch_description()
    args = {a.name: a for a in ld.entities if isinstance(a, DeclareLaunchArgument)}
    assert 'mission_params_file' in args
    assert _text(args['mission_params_file'].default_value) == ''


def test_mission_include_gets_the_params_file():
    from launch.actions import IncludeLaunchDescription
    ld = _load().generate_launch_description()
    incs = [e for e in ld.entities if isinstance(e, IncludeLaunchDescription)]
    mission = [i for i in incs if "pkg='mower_mission'" in str(
        i.launch_description_source.location)]
    assert len(mission) == 1
    args = {_text(k): _text(v) for k, v in mission[0].launch_arguments}
    assert args.get('params_file') == '$(mission_params_file)'
