"""Mowed raster (A3 coverage map, 2026-11-05): stamping, %, persistence / clear rules, resume
by position, missed-patch report. No ROS."""

import json
import math

import pytest

from mower_mission import mission_fsm as f
from mower_mission import mow_progress
from mower_mission import mowed_raster as mr
from mower_mission.resume import AreaCursor, ResumeCursor
from test_mission_fsm import (Harness, follow_current, line, plan, square,
                              start_until_planning)

SQUARE = [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0), (0.0, 5.0)]


def logs(h, mk=0):
    return [e.text for e in h.since(mk, f.Log)]


def raster(h, idx=0):
    ac = h.fsm.cursor.areas.get(idx)
    return ac.raster if ac is not None else None


def drive(h, x0, y0, x1, y1, step=0.05):
    """Move base_link from (x0, y0) to (x1, y1), heading along, one tick per step."""
    n = max(1, int(round(math.hypot(x1 - x0, y1 - y0) / step)))
    yaw = math.atan2(y1 - y0, x1 - x0)
    for k in range(n + 1):
        h.fsm.inputs.pose = (x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n, yaw)
        h.tick()


def mowing(h, sps):
    h.fsm.inputs.pose = tuple(sps[0][0][:2]) + (0.0,)   # at the start: no transit
    start_until_planning(h)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan(sps))
    h.fsm.inputs.is_cutting = True
    h.tick()
    assert h.pending_action(f.ACT_FOLLOW)


# ---------------------------------------------------------------------------
# the raster itself
# ---------------------------------------------------------------------------
def test_stamp_segment_cuts_the_swept_disc_only():
    r = mr.MowedRaster.for_area(SQUARE)
    assert r.res == pytest.approx(0.05)
    r.stamp_segment(1.0, 2.0, 3.0, 2.0, 0.11)
    # (sample points off the cell edges: a 0.05 m grid places the disc edge to +-2.5 cm)
    across = r.cut_at([2.02] * 5, [2.0 + d for d in (-0.16, -0.08, 0.01, 0.08, 0.16)])
    assert across.tolist() == [False, True, True, True, False]
    assert r.cut_at([0.85, 3.15], [2.0, 2.0]).tolist() == [False, False]   # disc ends
    assert r.cut_at([0.92, 3.08], [2.0, 2.0]).tolist() == [True, True]
    # 2 m x 0.22 m band + the two half discs
    area = r.data.sum() * r.res ** 2
    assert area == pytest.approx(2.0 * 0.22 + math.pi * 0.11 ** 2, rel=0.12)
    assert r.stamp_segment(1.0, 2.0, 3.0, 2.0, 0.11) == 0          # nothing new


def test_percent_of_the_mowable_polygon_excludes_accepted_holes():
    r = mr.MowedRaster.for_area(SQUARE)
    for k in range(13):                                  # the lower half, 0.2 m rows
        y = 0.1 + 0.2 * k
        r.stamp_segment(0.0, y, 5.0, y, 0.11)
    assert r.percent(SQUARE) == pytest.approx(52.0, abs=1.5)
    hole = [(0.0, 2.6), (5.0, 2.6), (5.0, 5.0), (0.0, 5.0)]     # the uncut half is a no-go
    assert r.percent(SQUARE, [hole]) == pytest.approx(100.0, abs=0.5)


def test_raster_round_trips_through_the_resume_cursor_file():
    c = ResumeCursor()
    c.current_command = 1
    r = mr.MowedRaster.for_area(SQUARE)
    r.stamp_segment(0.5, 0.5, 4.5, 4.0, 0.11)
    c.areas[1] = AreaCursor(30, 7, 12, {0}, raster=r)
    text = c.dumps()
    row = [ln for ln in text.splitlines() if ln.startswith('area_raster 1 ')]
    assert len(row) == 1 and len(row[0]) < 2000             # zlib: a 5 m square stays small
    back = ResumeCursor.loads(text)
    assert back == c and back.areas[1].raster.key == mr.area_key(SQUARE)
    # a torn row is skipped like any malformed row, the rest of the cursor survives
    bad = ResumeCursor.loads(text.replace(row[0], row[0][:-40]))
    assert bad.areas[1].raster is None and bad.areas[1].completed == {0}


def test_blade_offset_from_the_static_tree_chains_frames():
    st = {'blade_link': ('base_link', 0.23, 0.0, 0.0)}
    assert mr.blade_offset_from(st) == pytest.approx((0.23, 0.0))
    st = {'blade_link': ('deck', 0.1, 0.0, 0.0), 'deck': ('base_link', 0.2, 0.05, math.pi / 2)}
    assert mr.blade_offset_from(st) == pytest.approx((0.2, 0.15))
    assert mr.blade_offset_from({'blade_link': ('odom', 0, 0, 0)}) is None


def test_plan_progress_uses_the_blade_point_of_each_pose():
    """base_link 0.23 m behind the blade: a pose is mowed where the BLADE was at it."""
    r = mr.MowedRaster.for_area(SQUARE)
    r.stamp_segment(1.23, 1.0, 3.23, 1.0, 0.11)       # base_link drove x 1 -> 3 at yaw 0
    sp = [(1.0 + 0.1 * k, 1.0, 0.0) for k in range(31)]     # x 1 -> 4
    cov = r.poses_mowed(sp, 0.09, (0.23, 0.0))
    last = max(k for k, c in enumerate(cov) if c)
    assert 1.0 + 0.1 * last == pytest.approx(3.0, abs=0.06)
    assert all(cov[:last + 1])
    done, prefix = r.plan_progress([sp], 0.09, (0.23, 0.0))
    assert done == set() and prefix == {0: last}
    # without the offset the stamped band reads 0.23 m further on
    cov0 = r.poses_mowed(sp, 0.09)
    assert 1.0 + 0.1 * max(k for k, c in enumerate(cov0) if c) == pytest.approx(3.2, abs=0.06)


def test_missed_patches_skip_the_boundary_strip_and_find_a_real_gap():
    r = mr.MowedRaster.for_area(SQUARE)
    for k in range(25):                               # 0.2 m rows from 0.1 to 4.9 ...
        y = 0.1 + 0.2 * k
        if 2.0 < y < 3.0:
            r.stamp_segment(0.1, y, 2.0, y, 0.11)     # ... with a 1 x 3 m gap at the right
            r.stamp_segment(3.0, y, 4.9, y, 0.11)
        else:
            r.stamp_segment(0.1, y, 4.9, y, 0.11)
    # the strip the disc leaves along the boundary (~0.01 m) is not a patch
    assert r.percent(SQUARE) < 100.0
    pts = r.missed_patches(SQUARE, [], 0.22, 0.25)
    assert len(pts) == 1
    a, x, y = pts[0]
    assert a == pytest.approx(0.8 * 0.8, rel=0.35) and x == pytest.approx(2.5, abs=0.1) \
        and y == pytest.approx(2.5, abs=0.15)
    assert r.missed_patches(SQUARE, [], 0.22, 2.0) == []


# ---------------------------------------------------------------------------
# stamping in the FSM
# ---------------------------------------------------------------------------
def test_mowing_stamps_the_blade_footprint_at_the_blade_point():
    h = Harness()
    h.fsm.inputs.blade_offset = (0.23, 0.0)
    mowing(h, [line(0.5, 1.0, 4.5, 1.0, 41), line(4.5, 2.0, 0.5, 2.0, 41)])
    drive(h, 0.5, 1.0, 3.0, 1.0)
    r = raster(h)
    assert r is not None and r.key == mr.area_key(h.areas[0]['outer'])
    assert r.cut_at([3.21], [1.0])[0] and not r.cut_at([3.41], [1.0])[0]   # blade at 3.23
    assert r.cut_at([2.02, 2.02], [0.92, 1.08]).all() and not r.cut_at([2.02], [1.16])[0]
    assert 0.0 < h.fsm.mowed_area_percent() < 15.0


def test_blade_commanded_but_not_cutting_is_not_stamped():
    """The cutter status counts, not the command (blade spin-up / an MCU refusal)."""
    h = Harness()
    mowing(h, [line(0.5, 1.0, 4.5, 1.0, 41)])
    assert h.blade                                      # BladeOn was sent
    h.fsm.inputs.is_cutting = False
    drive(h, 0.5, 1.0, 3.0, 1.0)
    assert not raster(h).cut_at([1.02, 2.02, 2.92], [1.0] * 3).any()
    h.fsm.inputs.is_cutting = True
    drive(h, 3.0, 1.0, 4.0, 1.0)
    r = raster(h)
    assert r.cut_at([3.52], [1.0])[0] and not r.cut_at([2.02], [1.0])[0]


@pytest.mark.parametrize('state', ['degraded', 'lost'])
def test_untrusted_localisation_is_not_stamped(state):
    h = Harness(loc_hold=False)                         # keep driving through it
    mowing(h, [line(0.5, 1.0, 4.5, 1.0, 41)])
    i = h.fsm.inputs
    i.loc_state, i.loc_stamp = 'ok', h.t
    drive(h, 0.5, 1.0, 2.0, 1.0)
    i.loc_state = state
    for k in range(21):
        i.loc_stamp = h.t
        i.pose = (2.0 + 0.05 * k, 1.0, 0.0)
        h.tick()
    i.loc_state = 'ok'
    i.loc_stamp = h.t
    drive(h, 3.0, 1.0, 4.0, 1.0)
    r = raster(h)
    assert r.cut_at([1.5, 3.5], [1.0, 1.0]).all()
    assert not r.cut_at([2.5], [1.0])[0]
    # a status that stopped arriving is not trusted either
    h.tick(dt=float(h.fsm.p.loc_status_stale_s) + 1.0)
    drive(h, 4.0, 2.0, 1.0, 2.0)
    assert not raster(h).cut_at([2.5], [2.0])[0]


def test_no_blade_transform_stamps_nothing_and_says_so_once():
    h = Harness()
    h.fsm.inputs.blade_offset = None
    mk = h.mark()
    mowing(h, [line(0.5, 1.0, 4.5, 1.0, 41)])
    drive(h, 0.5, 1.0, 3.0, 1.0)
    assert not raster(h).data.any()
    assert sum('no base_link -> blade_link' in t for t in logs(h, mk)) == 1


def test_raster_is_saved_with_the_cursor_while_cutting():
    h = Harness()
    mowing(h, [line(0.5, 1.0, 4.5, 1.0, 41)])
    h.saved = None
    for k in range(40):                                   # 4 m in 0.8 s each: 32 s
        h.fsm.inputs.pose = (0.5 + 0.1 * k, 1.0, 0.0)
        h.tick(dt=0.8)
    assert h.saved is not None
    assert ResumeCursor.loads(h.saved).areas[0].raster.cut_at([2.0], [1.0])[0]


# ---------------------------------------------------------------------------
# clear rules
# ---------------------------------------------------------------------------
def stop_after_cutting_row0(h):
    mowing(h, [line(0.5, 1.0, 4.5, 1.0, 41), line(4.5, 2.0, 0.5, 2.0, 41),
               line(0.5, 3.0, 4.5, 3.0, 41)])
    drive(h, 0.5, 1.0, 3.0, 1.0)
    assert h.cmd(f.CMD_STOP)
    assert ResumeCursor.loads(h.saved).areas[0].raster.data.any()


def test_resume_keeps_the_raster_a_new_full_mow_clears_it():
    h = Harness()
    stop_after_cutting_row0(h)
    h = Harness(cursor=ResumeCursor.loads(h.saved))           # across a restart
    start_until_planning(h)
    assert h.fsm.mission.resume
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(0.5, 1.0, 4.5, 1.0, 41)]))
    assert raster(h).cut_at([2.0], [1.0])[0]
    h.cmd(f.CMD_STOP)
    ok, _msg, fx = h.fsm.clear_resume(h.t)                    # GUI "start over"
    h._apply(fx)
    start_until_planning(h)
    assert not h.fsm.mission.resume
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(0.5, 1.0, 4.5, 1.0, 41)]))
    assert not raster(h).data.any()


def test_edited_area_boundary_clears_the_raster():
    h = Harness()
    stop_after_cutting_row0(h)
    a = square(0, 0, 5)
    a['outer'] = [(0, 0), (5, 0), (5, 5.5), (0, 5)]            # boundary moved
    h.areas = [a]
    start_until_planning(h)
    mk = h.mark()
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(0.5, 1.0, 4.5, 1.0, 41)]))
    assert not raster(h).data.any()
    assert raster(h).key == mr.area_key(a['outer'])
    assert any('boundary changed since its mowed raster' in t for t in logs(h, mk))


def test_a_repeat_run_starts_with_an_empty_raster():
    h = Harness()
    h.area_settings = {0: {'repeat': 2}}
    mowing(h, [line(0.5, 1.0, 4.5, 1.0, 41)])
    drive(h, 0.5, 1.0, 4.5, 1.0)
    assert raster(h).data.any()
    h.finish(f.ACT_FOLLOW)
    assert h.fsm.mission.run_i == 1
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(0.5, 1.0, 4.5, 1.0, 41)]))
    assert not raster(h).data.any()            # run 2 mows the area again, nothing skipped
    assert h.fsm.mission.sub_i == 0 and h.fsm.mission.start_local == 0


# ---------------------------------------------------------------------------
# resume by position
# ---------------------------------------------------------------------------
def test_resume_skips_a_sub_path_the_raster_shows_mowed():
    """A sub-path the cursor does not have as completed (it failed near its end, a detour
    finished it, ...) but the blade cut is not mowed again on resume."""
    h = Harness()
    stop_after_cutting_row0(h)
    cur = ResumeCursor.loads(h.saved)
    ac = cur.areas[0]
    # row 0 cut up to x 3.0 only; cut the rest and drop the progress from the cursor
    ac.raster.stamp_segment(2.9, 1.0, 4.5, 1.0, 0.11)
    ac.completed, ac.done_fps, ac.resume_index = set(), set(), 0
    h = Harness(cursor=cur)
    start_until_planning(h)
    mk = h.mark()
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(0.5, 1.0, 4.5, 1.0, 41),
                                            line(4.5, 2.0, 0.5, 2.0, 41),
                                            line(0.5, 3.0, 4.5, 3.0, 41)]))
    m = h.fsm.mission
    assert h.fsm.cursor.areas[0].completed == {0}
    assert (m.sub_i, m.start_local) == (1, 0)
    assert any('1 of them by the mowed raster' in t for t in logs(h, mk))


def test_resume_starts_after_the_mowed_lead_in_never_before_the_cursor():
    h = Harness()
    h.fsm.inputs.blade_offset = (0.23, 0.0)
    sps = [line(0.5, 1.0, 4.5, 1.0, 41), line(4.5, 2.0, 0.5, 2.0, 41)]
    mowing(h, sps)
    drive(h, 0.5, 1.0, 2.0, 1.0)
    assert h.cmd(f.CMD_STOP)
    cur = ResumeCursor.loads(h.saved)
    stored = cur.areas[0].resume_index
    # the cursor lags (e.g. the progress was taken before the last ticks): the raster wins
    cur.areas[0].resume_index = 3
    h2 = Harness(cursor=ResumeCursor.loads(cur.dumps()))
    h2.fsm.inputs.blade_offset = (0.23, 0.0)
    start_until_planning(h2)
    h2.finish(f.ACT_PLAN, f.SUCCEEDED, plan(sps))
    m = h2.fsm.mission
    x = m.subpaths[0][m.start_local][0]
    assert m.sub_i == 0 and x == pytest.approx(2.0, abs=0.12), (m.start_local, x, stored)
    # ... but a cursor ahead of the raster is kept (an uncut gap is the patch report's)
    cur.areas[0].resume_index = stored + 10
    h3 = Harness(cursor=ResumeCursor.loads(cur.dumps()))
    start_until_planning(h3)
    h3.finish(f.ACT_PLAN, f.SUCCEEDED, plan(sps))
    assert h3.fsm.mission.start_local == stored + 10


def test_no_go_replan_keeps_what_the_raster_shows_cut():
    """The mid-mow re-plan maps the new plan onto the raster (was: plan polylines)."""
    h = Harness()
    mowing(h, [line(0.5, 1.0, 4.5, 1.0, 41), line(4.5, 2.0, 0.5, 2.0, 41)])
    drive(h, 0.5, 1.0, 4.5, 1.0)
    h.finish(f.ACT_FOLLOW)
    if h.fsm._action is not None and h.fsm._action.name == f.ACT_NAV:
        h.finish(f.ACT_NAV)
    h.tick()
    m = h.fsm.mission
    assert m.sub_i == 1
    drive(h, 4.5, 2.0, 2.5, 2.0)
    m.nogo_replan = {'sub': 1, 'why': 'test', 'progress': m.progress_local, 'phase': 'MOWING'}
    h.fsm._action = None
    h.fsm._start_plan(purpose='nogo_replan')
    new = [line(0.5, 1.0, 4.5, 1.0, 41), line(4.5, 2.0, 2.0, 2.0, 26),
           line(1.0, 2.0, 0.5, 2.0, 6)]
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan(new))
    assert h.fsm.cursor.areas[0].completed == {0}
    assert m.sub_i == 1
    assert m.subpaths[1][m.start_local][0] == pytest.approx(2.5, abs=0.12)
    assert raster(h).data.any()


# ---------------------------------------------------------------------------
# % for the GUI and the end-of-area report
# ---------------------------------------------------------------------------
def test_mow_progress_carries_the_raster_percent_next_to_the_plan_percent():
    h = Harness()
    mowing(h, [line(0.5, 1.0, 4.5, 1.0, 41), line(4.5, 2.0, 0.5, 2.0, 41)])
    drive(h, 0.5, 1.0, 4.5, 1.0)
    pid, _area, _subs = mow_progress.plan_geometry(h.fsm)
    st = mow_progress.progress(h.fsm, pid, True, 10.0, 4.0)
    json.dumps(st)
    assert st['percent'] > 0.0                         # plan progress, unchanged meaning
    cut = st['mowed_area_percent']
    assert cut == pytest.approx(100.0 * 4.0 * 0.22 / 25.0, abs=0.8)
    h.cmd(f.CMD_STOP)
    assert mow_progress.progress(h.fsm, pid, False, 10.0, 4.0)['mowed_area_percent'] is None


def test_end_of_area_reports_the_cut_share_and_patches_only_when_enabled():
    for min_m2, want in ((0.0, 0), (1.0, 1)):
        h = Harness(missed_patch_min_m2=min_m2)
        mowing(h, [line(0.5, 1.0, 4.5, 1.0, 41)])
        mk = h.mark()
        h.finish(f.ACT_FOLLOW)               # drives the path: the rest of the square uncut
        txt = logs(h, mk)
        assert any('% of the mowable area cut (mowed raster)' in t for t in txt), txt
        patch = [t for t in txt if 'uncut patch(es)' in t]
        assert len(patch) == want, txt
        inc = [e for e in h.since(mk, f.RecordIncident) if e.kind == 'missed_patches']
        assert len(inc) == want
