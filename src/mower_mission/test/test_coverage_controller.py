"""Coverage controller selection (2026-10-15 MPPI study): coverage_controller mppi | rpp.

``mppi`` drives the coverage sub-paths with the feasibility-aware MPPI (nav2_params.yaml
FollowCoveragePathMPPI / FollowCoveragePathReverseMPPI); ``rpp`` (the shipped default until MPPI
passes the sim gate, see docs/analysis/2026-10-15_mppi_study.md) is the RPP pair as before.
The cut speed follows the controller: MPPI has no desired_linear_vel, its vx_max is set
(capped at the MCU clamp)."""

import math
import os
import sys

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from mower_mission import mission_fsm as f   # noqa: E402
from test_mission_fsm import (Harness, line, mowing, param_calls, plan,  # noqa: E402
                              start_until_planning)

HERE = os.path.dirname(os.path.abspath(__file__))
MISSION_YAML = os.path.join(HERE, '..', 'config', 'mission.yaml')


def _k_turn(h):
    """swath -> reverse leg (poses face against the travel) -> forward leg."""
    start_until_planning(h)
    fwd = [(0.2, 0.0, 0.0)] + [(1.0 + 0.1 * k, 0.0, 0.0) for k in range(11)]
    ryaw = math.atan2(-0.35, 0.1)
    rev = [(2.0, 0.0, ryaw), (1.95, 0.175, ryaw), (1.9, 0.35, ryaw)]
    fwd2 = [(1.9, 0.35, math.pi), (0.0, 0.36, math.pi)]
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([fwd, rev, fwd2]))
    h.fsm.inputs.is_cutting = True
    h.tick()
    return fwd, rev, fwd2


def _drive_legs(h):
    fwd, rev, fwd2 = _k_turn(h)
    ids = [h.goal(f.ACT_FOLLOW)['controller_id']]
    h.fsm.inputs.pose = (2.0, 0.0, 0.0)
    h.finish(f.ACT_FOLLOW)
    h.tick()
    g = h.goal(f.ACT_FOLLOW)
    assert g['poses'] == rev
    ids.append(g['controller_id'])
    h.finish(f.ACT_FOLLOW)              # arrives at the cusp facing +x (yaw 0)
    h.tick()
    ids.append(h.goal(f.ACT_FOLLOW)['controller_id'])
    return ids


def test_default_is_mppi_with_path_conditioning():
    assert f.Params().coverage_controller == 'mppi' and f.Params().path_conditioning
    h = Harness(coverage_controller=f.Params().coverage_controller)
    assert h.fsm.coverage_controller_id(False) == 'FollowCoveragePathMPPI'
    assert h.fsm.coverage_controller_id(True) == 'FollowCoveragePathReverseMPPI'


def test_mppi_forward_and_reverse():
    h = Harness(coverage_controller='mppi')
    assert _drive_legs(h) == ['FollowCoveragePathMPPI', 'FollowCoveragePathReverseMPPI',
                              'FollowCoveragePathMPPI']
    assert h.name == 'MOWING' and h.blade


def test_rpp_revert_uses_the_rpp_pair():
    h = Harness(coverage_controller='rpp')
    assert _drive_legs(h) == ['FollowCoveragePath', 'FollowCoveragePathReverse',
                              'FollowCoveragePath']


def test_selection_is_case_and_space_tolerant():
    h = Harness(coverage_controller=' MPPI ')
    assert h.fsm.coverage_controller_id(False) == 'FollowCoveragePathMPPI'
    assert h.fsm.coverage_controller_id(True) == 'FollowCoveragePathReverseMPPI'
    h = Harness(coverage_controller='RPP')
    assert h.fsm.coverage_controller_id(True) == 'FollowCoveragePathReverse'


def test_custom_mppi_ids():
    h = Harness(coverage_controller='mppi', mppi_follow_controller_id='A',
                mppi_reverse_controller_id='B')
    assert (h.fsm.coverage_controller_id(False), h.fsm.coverage_controller_id(True)) == \
        ('A', 'B')


def test_cut_speed_goes_to_mppi_vx_max_capped_at_the_mcu_clamp():
    h = Harness(coverage_controller='mppi')
    h.area_settings = {0: {'cut_speed_mps': 0.45}}
    m = h.mark()
    start_until_planning(h)
    assert param_calls(h, f.PARAM_NODE_CONTROLLER, m) == [{'FollowCoveragePathMPPI.vx_max': 0.3}]
    h = Harness(coverage_controller='mppi')
    h.area_settings = {0: {'cut_speed_mps': 0.2}}
    m = h.mark()
    start_until_planning(h)
    assert param_calls(h, f.PARAM_NODE_CONTROLLER, m) == [{'FollowCoveragePathMPPI.vx_max': 0.2}]


def test_cut_speed_rpp_revert_keeps_desired_linear_vel():
    h = Harness(coverage_controller='rpp')
    h.area_settings = {0: {'cut_speed_mps': 0.45}}
    m = h.mark()
    start_until_planning(h)
    assert param_calls(h, f.PARAM_NODE_CONTROLLER, m) == \
        [{'FollowCoveragePath.desired_linear_vel': 0.45}]


def test_slope_caution_caps_the_mppi_coverage_speed():
    h = Harness(coverage_controller='mppi')
    mowing(h)
    m = h.mark()
    for _ in range(5):
        h.fsm.inputs.tilt_stamp = h.t + 0.1
        h.fsm.inputs.tilt_band = 'caution'
        h.fsm.inputs.tilt_roll_deg = 17
        h.fsm.inputs.tilt_axis = 'roll'
        h.tick(dt=0.1)
    assert param_calls(h, f.PARAM_NODE_CONTROLLER, m) == \
        [{'FollowCoveragePathMPPI.vx_max': 0.15, 'FollowPath.vx_max': 0.15}]


def test_transit_route_controller_is_unchanged():
    """Route legs stay on FollowPath (MPPI transit) whichever coverage controller is used."""
    assert f.Params().route_controller_id == 'FollowPath'


def test_shipped_mission_yaml_has_both_controllers():
    with open(MISSION_YAML) as fh:
        doc = yaml.safe_load(fh)
    p = doc['behavior_tree_node']['ros__parameters']
    assert p['coverage_controller'] in ('mppi', 'rpp')
    assert p['coverage_controller'] == f.Params().coverage_controller
    assert p['mppi_follow_controller_id'] == 'FollowCoveragePathMPPI'
    assert p['mppi_reverse_controller_id'] == 'FollowCoveragePathReverseMPPI'
    assert p['mppi_controller_speed_param'] == 'FollowCoveragePathMPPI.vx_max'
    assert 0.0 < p['mppi_speed_max_mps'] <= 0.3
    # the revert ids are still configured
    assert p['follow_controller_id'] == 'FollowCoveragePath'
    assert p['reverse_controller_id'] == 'FollowCoveragePathReverse'
    q = f.Params.from_dict({k: v for k, v in p.items() if k in f.Params.__dataclass_fields__})
    assert q.coverage_controller == p['coverage_controller']


def test_line_helper_still_forward():
    # guard: the helper the K-turn test builds on gives forward (non-reverse) poses
    from mower_mission import geometry as geo
    assert not geo.is_reverse_subpath(line(0, 0, 1, 0))
