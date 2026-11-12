# SPDX-License-Identifier: GPL-3.0-or-later
"""Narrow-strip plans in the mission (docs/analysis/2026-11-13_narrow_strip.md): driven as
planned, an obstacle no detour passes stops the mower, and the finish text leads with the
mowed raster's cut share."""

import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from mower_mission import mission_fsm as f  # noqa: E402
from mower_mission import mowed_raster as mr  # noqa: E402
from mower_mission import path_conditioning as pcond  # noqa: E402
from mower_mission.fsm.common import STRIP_PLAN_TAG, condition_plan, is_strip_plan  # noqa: E402
from test_mission_fsm import (Harness, line, plan, square,  # noqa: E402
                              start_until_planning, tick_seen)


def strip_plan(subpaths):
    res = plan(subpaths)
    res['message'] = '%s 2 pass(es) at 17.7 deg | ...' % STRIP_PLAN_TAG
    return res


def reverse(sp):
    """Reverse leg: the poses face against the travel (the bridge's convention)."""
    return [(x, y, 3.141592653589793) for x, y, _ in reversed(sp)]


def test_is_strip_plan_reads_the_bridge_tag_also_in_a_merged_message():
    assert is_strip_plan({'message': STRIP_PLAN_TAG + ' 3 pass(es)'})
    assert is_strip_plan({'message': 'run 1: %s 3 pass(es)' % STRIP_PLAN_TAG})
    assert not is_strip_plan({'message': '3 ring(s) + 20 swath(s)'})
    assert not is_strip_plan({})


def test_strip_plan_is_not_conditioned_and_its_retrace_is_not_joined():
    fwd = line(0, 0, 1.0, 0, n=11)                # a short pass: join_turn_chains would
    back = reverse(fwd)                           # merge pass / retrace / next pass
    nxt = line(0, 0, 1.0, 0.18, n=11)
    job = {'subpaths': [fwd, back, nxt], 'region': [[(-1, -1), (2, -1), (2, 1), (-1, 1)]],
           'prm': pcond.Params(body_limit_m=0.26), 'leg_max_m': 1.2, 'area': 0,
           'strip': True}
    out = condition_plan(job)
    assert out['subpaths'] == [list(fwd), list(back), list(nxt)]
    assert out['cond'] == {} and out['cond_errors'] == {}
    assert any('strip plan' in t for _lvl, t in out['logs'])


def mowing_strip(h, tagged=True, size=12, end=10.0):
    h.areas = [square(-1, -1, size)]
    start_until_planning(h)
    sp0 = line(0, 0, end, 0, n=int(round(2 * end)) + 1)
    res = (strip_plan if tagged else plan)([sp0, reverse(sp0)])
    h.finish(f.ACT_PLAN, f.SUCCEEDED, res)
    h.fsm.inputs.is_cutting = True
    h.tick()
    assert h.fsm.mission.strip is tagged
    return sp0


def cat_stays(h, x):
    h.fsm.inputs.pose = (x, 0.0, 0.0)
    h.tick()
    tick_seen(h, 'dynamic', 'cat', 30.5)


# the area ends at x = 4: the detour target 2 m past the cat at 3 m lies outside it
NO_ROOM = dict(dynamic_wait_s=30.0, path_conditioning=False, hard_boundary_body=False)


def test_strip_obstacle_no_detour_stops_the_mower_needs_help():
    h = Harness(**NO_ROOM)
    mowing_strip(h, size=5)
    cat_stays(h, 3.0)
    assert h.name == 'STUCK_NEEDS_HELP', h.name
    assert 'no detour fits' in h.fsm.sub_state or 'STUCK' in h.name
    # nothing of the pass was skipped: START resumes it where it stopped
    assert all(not ac.skipped for ac in h.fsm.cursor.areas.values())


def test_same_obstacle_on_a_normal_plan_gives_the_subpath_up():
    h = Harness(**NO_ROOM)
    mowing_strip(h, tagged=False, size=5)
    cat_stays(h, 3.0)
    assert h.name != 'STUCK_NEEDS_HELP'
    assert any(x[1] == 0 for x in h.fsm.mission.failed)


def test_strip_obstacle_near_the_pass_end_is_not_skipped():
    # the detour target lies past the pass end: a normal plan skips the rest of the
    # sub-path there, a strip plan never cuts a pass short
    h = Harness(dynamic_wait_s=30.0)
    mowing_strip(h, end=4.0)
    cat_stays(h, 3.0)
    assert h.name == 'STUCK_NEEDS_HELP', h.name
    n = Harness(dynamic_wait_s=30.0)
    mowing_strip(n, tagged=False, end=4.0)
    cat_stays(n, 3.0)
    assert n.name != 'STUCK_NEEDS_HELP' and n.fsm.cursor.areas[0].skipped


def test_finish_text_leads_with_the_cut_share():
    h = Harness()
    mowing_strip(h)
    m = h.fsm.mission
    m.planned_m, m.missed_m = 100.0, 1.2
    m.cut_m2 = {0: (1.73, 3.76)}
    txt = h.fsm._mowed_text()
    assert txt.startswith('cut 46.0 % of the area (mowed raster)'), txt
    assert '98.8 % of the planned path mowed' in txt


def test_raster_cut_m2_matches_percent():
    r = mr.MowedRaster.for_area([(0, 0), (2, 0), (2, 1), (0, 1)])
    r.stamp_segment(0.0, 0.5, 1.0, 0.5, 0.5)
    cut, total = r.cut_m2([(0, 0), (2, 0), (2, 1), (0, 1)])
    assert abs(total - 2.0) < 0.05
    assert abs(100.0 * cut / total - r.percent([(0, 0), (2, 0), (2, 1), (0, 1)])) < 1e-9
