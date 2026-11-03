"""Review 2026-11-02 H5: config/mission.yaml is the single source of every parameter value.
The fsm Params / mission_node defaults must equal it, so the unit tests (which run on the
defaults) exercise the field configuration. Fails on any future drift."""

import dataclasses

import pytest

from mower_common.shipped_params import drift, shipped_params
from mower_mission.fsm.common import Params, SAFETY_GUARDS, disabled_safety_guards

# Host paths: the yaml carries the container mount (/ros2_ws/maps), the code a per-user
# default so tests and dev runs never write into a mower path.
EXEMPT = ('coverage_resume_path', 'alternate_state_path', 'recording_quality_dir')


def shipped():
    return shipped_params('mower_mission', 'mission.yaml', 'behavior_tree_node')


def test_fsm_params_match_the_shipped_yaml():
    p = Params()
    defaults = {f.name: getattr(p, f.name) for f in dataclasses.fields(p)}
    assert drift(defaults, shipped(), EXEMPT) == {}


def test_node_params_match_the_shipped_yaml():
    mn = pytest.importorskip('mower_mission.mission_node')   # needs the built interfaces
    assert drift({**mn.TOPIC_DEFAULTS, **mn.NODE_DEFAULTS}, shipped(), EXEMPT) == {}


def test_every_yaml_key_is_a_parameter():
    p = Params()
    known = {f.name for f in dataclasses.fields(p)} | {'use_sim_time'}
    try:
        from mower_mission import mission_node as mn
        known |= set(mn.TOPIC_DEFAULTS) | set(mn.NODE_DEFAULTS)
    except ImportError:
        pytest.skip('mission_node needs the built interfaces')
    assert sorted(set(shipped()) - known) == []


def test_field_values_review_2026_11_02():
    p = Params()
    assert (p.dock_timeout_s, p.dock_action_timeout_s, p.manual_blade_requires_enable) == \
        (480.0, 540.0, True)


def test_disabled_safety_guards_are_listed_for_the_startup_warning():
    assert disabled_safety_guards(Params()) == []          # shipped: every guard on
    p = Params.from_dict({'stuck_guard': False, 'manual_blade_requires_enable': False})
    assert disabled_safety_guards(p) == ['stuck_guard', 'manual_blade_requires_enable']
    assert set(SAFETY_GUARDS) == {'hard_boundary', 'stuck_guard', 'tilt_guard', 'loc_hold',
                                  'manual_blade_requires_enable'}
