# SPDX-License-Identifier: GPL-3.0-or-later
"""Terrain-memory hooks of the mission FSM: incidents + slope-aware swath angle."""
from mower_mission import mission_fsm as f

from test_mission_fsm import (Harness, line, mowing_two_swaths, plan, start_until_planning,
                              static_abort_and_detour)


def incidents(h, mark=0):
    return [e for e in h.since(mark, f.RecordIncident)]


def test_follow_abort_and_subpath_failed_incidents_at_pose():
    h = Harness(follow_retries=0)
    start_until_planning(h)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(0, 0, 5, 0), line(5, 0.3, 0, 0.3)]))
    h.fsm.inputs.is_cutting = True
    h.tick()
    h.fsm.inputs.pose = (2.1, 0.1, 0.0)
    m = h.mark()
    h.finish(f.ACT_FOLLOW, f.ABORTED)
    inc = incidents(h, m)
    assert [i.kind for i in inc] == ['follow_abort', 'subpath_failed']
    assert (inc[0].x, inc[0].y) == (2.1, 0.1)
    assert 'aborted' in inc[0].detail


def test_transit_abort_incident():
    h = Harness(transit_retries=0)
    start_until_planning(h)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(3, 3, 4, 3), line(0, 0, 4, 0)]))
    m = h.mark()
    h.finish(f.ACT_NAV, f.ABORTED)
    assert [i.kind for i in incidents(h, m)] == ['transit_abort', 'subpath_failed']


def test_detour_incident_once_per_detour():
    h = Harness()
    sp0, _ = mowing_two_swaths(h)
    m = h.mark()
    static_abort_and_detour(h, sp0)
    kinds = [i.kind for i in incidents(h, m)]
    assert kinds.count('detour') == 1
    assert 'follow_abort' not in kinds        # a detour is not a traction failure


def test_no_incident_without_pose():
    h = Harness()
    h.fsm.inputs.pose = None
    h.fsm._fx = []
    h.fsm._incident('follow_abort', 'x')
    assert h.fsm._fx == []


def _plan_angle(h):
    return h.goal(f.ACT_PLAN)['mow_angle_deg']


def test_slope_angle_used_when_auto_and_mode_on():
    h = Harness()
    h.area_settings = {0: {'slope_mode': 'contour', 'mow_angle_deg': -1.0,
                           'slope_mow_angle_deg': 120.0, 'slope_angle_why': 'contour: test'}}
    start_until_planning(h)
    assert _plan_angle(h) == 120.0
    assert h.fsm.mission.settings['slope_mode'] == 'contour'


def test_slope_angle_ignored_when_mode_off_or_angle_fixed():
    h = Harness()
    h.area_settings = {0: {'slope_mode': 'off', 'slope_mow_angle_deg': 120.0}}
    start_until_planning(h)
    assert _plan_angle(h) == -1.0
    h2 = Harness()
    h2.area_settings = {0: {'slope_mode': 'auto', 'mow_angle_deg': 30.0,
                            'slope_mow_angle_deg': 120.0}}
    start_until_planning(h2)
    assert _plan_angle(h2) == 30.0


def test_slope_angle_null_keeps_planner_choice():
    h = Harness()
    h.area_settings = {0: {'slope_mode': 'auto', 'slope_mow_angle_deg': None,
                           'slope_angle_why': 'not enough slope data yet'}}
    m = h.mark()
    start_until_planning(h)
    assert _plan_angle(h) == -1.0
    assert any('planner angle kept (not enough slope data yet)' in e.text
               for e in h.since(m, f.Log))
