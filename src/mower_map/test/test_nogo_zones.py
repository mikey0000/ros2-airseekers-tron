# SPDX-License-Identifier: GPL-3.0-or-later
"""Automatic obstacle no-go zones, option A (2026-10-23 trunk incident;
docs/analysis/2026-10-23_obstacle_nogo_options.md): class-voted, sized, on-path PENDING
zones in obstacle_tracker_core, the swept-footprint geometry (mower_map.nogo_zones) and the
map_server accept / re-size paths. ROS-free."""

import math
import types
from unittest import mock

import numpy as np
import pytest

from mower_map import areas as core
from mower_map.nogo_zones import (FOOTPRINT_BOX, PathCursor, Zone, ZoneSet, densify,
                                  path_hit, pivot_clear_direction, pivot_sweep_hits,
                                  pivot_vertices, poses_hit_disc, sweep_yaws)
from mower_map.obstacle_tracker_core import TrackerCore, TrackerParams

from test_dock_yaw_autocorrect import msn  # noqa: F401 (fixture)

SQUARE = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
LAWN = [(0, 'lawn', SQUARE)]
ROW = np.array([[1.0, 5.0], [9.0, 5.0]])                 # planned row along y = 5
CORNER = np.array([[1.0, 5.0], [5.0, 5.0], [5.0, 9.0]])  # 90 deg left turn at (5, 5)
TRUNK = (5.0, 5.3)                                        # 0.3 m beside the row centre line


def _core(**kw):
    return TrackerCore(TrackerParams(**kw))


def _feed(c, n, xy=TRUNK, label='trunk', score=0.7, width=0.25, t0=0.0, dt=0.5,
          plans=(ROW,), allow=True, jitter=0.0, **step_kw):
    """n detections of one object, one step per detection; returns all promotions."""
    out = []
    rng = np.random.default_rng(1)
    for k in range(n):
        t = t0 + k * dt
        x, y = xy
        if jitter:
            x, y = x + rng.normal(0, jitter), y + rng.normal(0, jitter)
        c.add_detections([(x, y, label, score, width)], t)
        out += c.step(np.empty((0, 2)), t, areas=LAWN, plans=plans, allow_promote=allow,
                      **step_kw)['promote']
    return out


# ------------------------------------------------------------------ geometry

def test_poses_hit_disc_rectangle_distance():
    pose = np.array([[0.0, 0.0, 0.0]])
    assert poses_hit_disc(pose, 0.65, 0.0, 0.06)[0]          # 0.05 m past the nose
    assert not poses_hit_disc(pose, 0.70, 0.0, 0.05)[0]
    assert poses_hit_disc(pose, 0.3, 0.3, 0.07)[0]           # 0.065 m beside the side
    assert not poses_hit_disc(pose, 0.3, 0.3, 0.06)[0]
    # a rotated pose: nose points along +y
    assert poses_hit_disc(np.array([[0.0, 0.0, math.pi / 2]]), 0.0, 0.65, 0.06)[0]


def test_densify_and_pivot_vertices():
    poses = densify(CORNER, 0.1)
    assert len(poses) >= 80 and np.allclose(poses[0, 2], 0.0)
    assert pivot_vertices(CORNER) == [(5.0, 5.0, 0.0, pytest.approx(math.pi / 2))]
    assert pivot_vertices(ROW) == []
    # a 20 deg bend is driven, not pivoted
    assert pivot_vertices([[0, 0], [1, 0], [2, 0.36]], min_turn_deg=30.0) == []


def test_sweep_yaws_direction():
    ccw = sweep_yaws(0.0, math.pi / 2, +1)
    cw = sweep_yaws(0.0, math.pi / 2, -1)
    assert ccw[-1] == pytest.approx(math.pi / 2) and np.all(np.diff(ccw) > 0)
    assert cw[-1] == pytest.approx(-3 * math.pi / 2) and np.all(np.diff(cw) < 0)
    assert sweep_yaws(0.0, -0.5, 0)[-1] == pytest.approx(-0.5)   # shorter way


def test_on_path_corridor_pivot_and_off_path():
    r = 0.225                                   # trunk prior 0.25 / 2 + 0.10 margin
    assert path_hit(ROW, 5.0, 5.3, r) == 'corridor'
    assert path_hit(ROW, 5.0, 5.0 + 0.235 + r + 0.05, r) == ''
    assert path_hit(ROW, 5.0, 6.5, r) == ''
    # (5.5, 5.5): clear of both straight stretches, but the nose corners (0.645 m from the
    # rear axle) sweep it while pivoting at the corner
    assert path_hit(CORNER, 5.5, 5.5, r) == 'pivot'
    assert path_hit(CORNER, 5.5, 5.5, r, include_pivots=False) == ''
    assert path_hit(np.empty((0, 2)), 5.0, 5.0, r) == ''


def test_pivot_sweep_query_direction_and_clear_direction():
    """The mission's pre-pivot query (option C uses it): left turn at (5, 5) from yaw 0 to
    90 deg; a trunk front-left is in the CCW sweep, not in the CW (270 deg) one ... unless
    the CW sweep passes it too."""
    zones = [Zone(5.5, 5.5, 0.225, 7, 'trunk')]
    hits = pivot_sweep_hits(5.0, 5.0, 0.0, math.pi / 2, +1, zones)
    assert [h.zone.id for h in hits] == [7]
    assert 0.0 < hits[0].yaw < math.pi / 2
    # turning the long way round (CW) passes the rear and the right side: clear
    assert pivot_sweep_hits(5.0, 5.0, 0.0, math.pi / 2, -1, zones) == []
    assert pivot_clear_direction(5.0, 5.0, 0.0, math.pi / 2, zones) == -1
    # a mark behind-right as well blocks the CW way: no clear direction
    both = zones + [Zone(4.6, 4.4, 0.2, state='mark')]
    assert pivot_sweep_hits(5.0, 5.0, 0.0, math.pi / 2, -1, both)
    assert pivot_clear_direction(5.0, 5.0, 0.0, math.pi / 2, both) is None
    assert pivot_clear_direction(5.0, 5.0, 1.0, 1.0, both) == 0
    # far away: no hit
    assert pivot_sweep_hits(0.0, 0.0, 0.0, math.pi, +1, zones) == []


def test_zone_set_json_roundtrip():
    zs = ZoneSet(12.5, [Zone(1.0, 2.0, 0.3, 4, 'trunk', 'pending', 0.81),
                        Zone(3.0, 4.0, 0.2, 5, 'stone', 'candidate', 0.6)],
                 [Zone(5.0, 6.0, 0.22, state='mark')])
    back = ZoneSet.from_json(zs.to_json())
    assert back.stamp == 12.5 and back.zones[0].label == 'trunk'
    assert back.zones[0].state == 'pending' and back.marks[0].r == pytest.approx(0.22)
    assert len(back.discs()) == 3
    assert [z.id for z in back.discs(include_candidates=False, include_marks=False)] == [4]
    assert ZoneSet.from_json('').zones == []


def test_path_cursor_counts_ahead_only():
    plan = np.array([[x, 5.0] for x in np.arange(0.0, 10.01, 0.5)]
                    + [[x, 5.2] for x in np.arange(10.0, -0.01, -0.5)])   # out and back
    c = PathCursor(back_m=1.0, near_m=0.3)
    c.set_path(plan)
    c.update(6.0, 5.0)
    ahead = c.ahead()
    assert ahead[0][0] == pytest.approx(5.0)       # 1 m behind the robot
    # on the way back the neighbouring row 0.2 m away does not pull the cursor back
    c.update(6.0, 5.15)
    assert c.idx >= 12


# ------------------------------------------------------------------ voting / gates

def test_trunk_on_path_becomes_zone_after_five_frames_in_ten_seconds():
    c = _core()
    got = _feed(c, 4)
    assert got == []                                # 4 frames: not yet
    got = _feed(c, 1, t0=2.0)
    assert len(got) == 1
    t = c.find_track(got[0])
    assert t.kind == 'det' and t.label() == 'trunk' and t.on_path == 'corridor'


def test_low_score_and_slow_frames_do_not_promote():
    c = _core()
    assert _feed(c, 20, score=0.55) == []           # below det_min_score 0.6
    c = _core()
    assert _feed(c, 8, dt=3.0) == []                # never 5 within 10 s


@pytest.mark.parametrize('label', ['person', 'dog', 'cat', 'hedgehog', 'rabbit'])
def test_living_classes_never_become_zones(label):
    c = _core()
    assert _feed(c, 30, label=label, score=0.95) == []
    assert c.tracks == []


def test_person_votes_at_a_trunk_veto_it():
    c = _core()
    _feed(c, 3)
    c.add_detections([(TRUNK[0], TRUNK[1], 'person', 0.9, 0.5)], 1.6)
    assert _feed(c, 10, t0=2.0) == []


def test_not_whitelisted_class_never_promotes_and_vote_decides():
    c = _core()
    assert _feed(c, 20, label='sports ball') == []
    # a trunk mostly voted 'wood' (both static) still promotes, named by the vote
    c = _core()
    for k in range(6):
        c.add_detections([(5.0, 5.3, 'wood', 0.8, 0.2)], k * 0.5)
    got = _feed(c, 1, label='trunk', score=0.7, t0=3.0)
    assert len(got) == 1 and c.find_track(got[0]).label() == 'wood'


def test_off_path_object_never_promotes_unless_bumped():
    c = _core()
    assert _feed(c, 10, xy=(5.0, 7.0)) == []
    c.add_bumper_hits([(5.0, 6.7)], 5.0, LAWN)
    got = _feed(c, 6, xy=(5.0, 7.0), t0=5.5)
    assert len(got) == 1 and c.find_track(got[0]).on_path == 'bumper'
    # require_on_path off: any whitelisted static object promotes
    c = _core(require_on_path=False)
    assert len(_feed(c, 6, xy=(5.0, 7.0), plans=())) == 1


def test_pivot_sweep_makes_a_zone():
    c = _core()
    got = _feed(c, 6, xy=(5.5, 5.5), plans=(CORNER,))
    assert len(got) == 1 and c.find_track(got[0]).on_path == 'pivot'


def test_not_mowing_blocked_area_and_outside_area():
    c = _core()
    assert _feed(c, 10, allow=False) == []          # only while mowing
    c = _core()
    assert _feed(c, 10, area_blocked=[0]) == []     # obstacle_detection: none
    c = _core()
    assert _feed(c, 10, xy=(12.0, 5.3), plans=(np.array([[11, 5], [14, 5]]),)) == []
    assert c.tracks == []


# ------------------------------------------------------------------ size

def test_size_from_median_width_with_class_prior_floor():
    c = _core()
    got = _feed(c, 6, width=0.40, jitter=0.02)
    t = c.find_track(got[0])
    assert t.footprint_radius(c.params) == pytest.approx(0.40 / 2 + 0.10)
    # narrower than the trunk prior (0.25 m): the prior wins
    c = _core()
    t = c.find_track(_feed(c, 6, width=0.10)[0])
    assert t.footprint_radius(c.params) == pytest.approx(0.25 / 2 + 0.10)
    # unknown width (0): the prior
    c = _core()
    t = c.find_track(_feed(c, 6, width=0.0)[0])
    assert t.footprint_radius(c.params) == pytest.approx(0.225)
    # a huge bogus width is capped
    c = _core(require_on_path=False)
    t = c.find_track(_feed(c, 6, width=5.0, plans=())[0])
    assert t.footprint_radius(c.params) == pytest.approx(1.0)
    # the robust median ignores a few outliers
    c = _core()
    for k, w in enumerate([0.3, 0.3, 0.3, 1.4, 0.3, 0.3]):
        c.add_detections([(5.0, 5.3, 'trunk', 0.8, w)], k * 0.5)
    assert c.tracks[0].sized_radius(c.params) == pytest.approx(0.25)


def test_pending_zone_is_resized_as_observations_arrive():
    c = _core()
    tid = _feed(c, 5, width=0.25)[0]
    c.confirm_promoted(tid, 3.0)
    t = c.find_track(tid)
    assert t.pending and t.promoted_radius == pytest.approx(0.225)
    out = []
    for k in range(20):
        c.add_detections([(5.0, 5.3, 'trunk', 0.8, 0.45)], 3.5 + k * 0.5)
        out += c.step(np.empty((0, 2)), 3.5 + k * 0.5, areas=LAWN, plans=(ROW,))['resize']
    assert out == [tid]                    # once, while the re-send is in flight
    c.resize_sent(tid, True)
    assert t.promoted_radius == pytest.approx(t.footprint_radius(c.params))
    assert t.promoted_radius > 0.26


# ------------------------------------------------------------------ lifecycle

def test_pending_expires_after_unseen_mowing_time_in_its_area():
    c = _core(pending_expire_s=30.0)
    tid = _feed(c, 5)[0]
    c.confirm_promoted(tid, 2.0)
    removed = []
    for k in range(1, 60):
        # mowing, robot elsewhere in the same area, not looking at it
        r = c.step(np.empty((0, 2)), 2.0 + k, areas=LAWN, robot_area=0,
                   robot_pose=(1.0, 1.0, math.pi))
        removed += r['remove']
        if k == 20:
            c.add_detections([(5.0, 5.3, 'trunk', 0.8, 0.25)], 2.0 + k)   # re-seen: reset
    assert removed == [tid]
    assert c.find_track(tid).unseen_s >= 30.0
    # not mowing / robot in another area: the clock stands
    c = _core(pending_expire_s=30.0)
    tid = _feed(c, 5)[0]
    c.confirm_promoted(tid, 2.0)
    for k in range(1, 100):
        assert c.step(np.empty((0, 2)), 2.0 + k, areas=LAWN, robot_area=-1)['remove'] == []
        assert c.step(np.empty((0, 2)), 2.5 + k, areas=LAWN, robot_area=0,
                      allow_promote=False)['remove'] == []


def test_det_zone_removed_only_when_looked_at_inside_camera_view():
    c = _core(pending_expire_s=0.0)
    tid = _feed(c, 5)[0]
    c.confirm_promoted(tid, 2.0)
    beside = (5.0, 4.6, 0.0)          # 0.7 m away, beside: the camera cannot see it
    for k in range(30):
        assert c.step(np.empty((0, 2)), 10.0 + k, robot_pose=beside, areas=LAWN)['remove'] == []
    looking = (3.5, 5.3, 0.0)         # 1.5 m away, straight ahead
    out = []
    for k in range(8):
        out += c.step(np.empty((0, 2)), 50.0 + k, robot_pose=looking, areas=LAWN)['remove']
    assert out == [tid]


def test_detection_keeps_zone_alive():
    c = _core(pending_expire_s=0.0)
    tid = _feed(c, 5)[0]
    c.confirm_promoted(tid, 2.0)
    looking = (3.5, 5.3, 0.0)
    for k in range(20):
        c.add_detections([(5.0, 5.3, 'trunk', 0.7, 0.25)], 10.0 + k)
        assert c.step(np.empty((0, 2)), 10.0 + k, robot_pose=looking, areas=LAWN)['remove'] == []


def test_duplicates_merge_into_one_zone():
    c = _core()
    c.add_detections([(5.0, 5.3, 'trunk', 0.8, 0.25)], 0.0)
    c.tracks[0].cx += 0.0
    # a second track 0.25 m away (outside the 0.3 m association of the moving centre)
    c.tracks.append(type(c.tracks[0])(id=99, cx=5.25, cy=5.3, radius=0.0, first_seen=1.0,
                                      last_seen=1.0, kind='det', votes={'trunk': 0.8}))
    c.step(np.empty((0, 2)), 1.0, areas=LAWN, plans=(ROW,))
    assert [t.id for t in c.tracks] == [1]
    assert c.tracks[0].votes['trunk'] == pytest.approx(1.6)


def test_promoted_zone_absorbs_later_detections_and_stereo_cluster():
    c = _core()
    tid = _feed(c, 5)[0]
    c.confirm_promoted(tid, 2.0)
    c.add_detections([(5.2, 5.45, 'trunk', 0.8, 0.25)], 3.0)   # inside the 0.225 m zone
    c.step(np.array([[5.05, 5.2], [5.1, 5.25]]), 3.0, areas=LAWN, plans=(ROW,))
    assert len(c.tracks) == 1 and c.tracks[0].cx == pytest.approx(5.0, abs=0.05)


def test_long_object_seen_in_parts_grows_one_zone_instead_of_stacking():
    # field 2026-10-28: a 1.5 m ladder seen in parts made 8+ stacked r 0.85 zones; the
    # promoted zone associated only within det_assoc_m (0.3 m) of its centre
    c = _core()
    tid = _feed(c, 5, xy=(5.0, 5.3), width=0.8)[0]
    c.confirm_promoted(tid, 3.0)
    t = c.find_track(tid)
    r0 = t.promoted_radius
    assert r0 == pytest.approx(0.5)
    out = []
    for k in range(30):                    # the other end of the object, 0.55 m away
        tk = 3.5 + 0.5 * k
        c.add_detections([(5.55, 5.3, 'trunk', 0.8, 0.8)], tk)
        r = c.step(np.empty((0, 2)), tk, areas=LAWN, plans=(ROW,))
        out += r['resize']
        assert r['promote'] == []
    assert [q.id for q in c.tracks] == [tid]          # no second zone
    assert out == [tid]                                # grown through the resize protocol
    c.resize_sent(tid, True)
    assert t.promoted_radius > r0 + 0.1 and t.promoted_radius <= 1.0
    assert (t.cx, t.cy) == (pytest.approx(5.0, abs=0.05), pytest.approx(5.3, abs=0.05))
    # beyond the footprint + 0.10 m: a separate object
    c.add_detections([(5.0 + t.footprint_radius(c.params) + 0.3, 5.3, 'trunk', 0.8, 0.2)], 30.0)
    assert len(c.tracks) == 2


def test_zone_growth_is_capped():
    c = _core()
    tid = _feed(c, 5, xy=(5.0, 5.3), width=1.5)[0]
    c.confirm_promoted(tid, 3.0)
    t = c.find_track(tid)
    for k in range(60):
        c.add_detections([(5.0 + 0.9 * math.cos(k), 5.3 + 0.9 * math.sin(k), 'trunk', 0.8,
                           1.5)], 3.5 + k)
    assert t.footprint_radius(c.params) == pytest.approx(1.0)
    assert len(c.tracks) == 1


def test_ignore_suppresses_the_spot():
    c = _core()
    tid = _feed(c, 5)[0]
    c.confirm_promoted(tid, 2.0)
    assert c.ignore(tid).id == tid and c.tracks == []
    assert _feed(c, 20, t0=10.0) == []           # same trunk again: suppressed
    assert len(_feed(c, 6, xy=(8.0, 5.2), t0=30.0)) == 1   # elsewhere: fine
    assert c.ignore(12345) is None


def test_session_end_drops_pending_zones_not_accepted_ones():
    # option C (2026-10-23): pending zones are hard obstacles for the mow that found them;
    # at the end of the session (MOWING_COMPLETE / _INCOMPLETE) they are dropped unless
    # accepted, without suppressing the spot (a next mow may propose them again)
    c = _core()
    tid = _feed(c, 5)[0]
    c.confirm_promoted(tid, 2.0)
    assert c.find_track(tid).pending
    acc = _feed(c, 6, xy=(8.0, 5.2), t0=30.0)[0]
    c.confirm_promoted(acc, 31.0)
    c.set_pending(acc, False)                    # accepted by the operator
    dropped = c.drop_pending()
    assert [t.id for t in dropped] == [tid]
    assert c.find_track(tid) is None and c.find_track(acc) is not None
    assert c.drop_pending() == []
    assert len(_feed(c, 6, t0=60.0)) == 1        # not suppressed: proposed again


def test_bumper_only_zone_is_opt_in():
    c = _core()
    for k in range(5):
        c.add_bumper_hits([(5.0, 5.4)], float(k), LAWN)
        assert c.step(np.empty((0, 2)), float(k), areas=LAWN)['promote'] == []
    c = _core(bumper_zone_hits=3)
    out = []
    for k in range(4):
        c.add_bumper_hits([(5.0, 5.4)], float(k), LAWN)
        out += c.step(np.empty((0, 2)), float(k), areas=LAWN)['promote']
    assert len(out) == 1
    assert c.find_track(out[0]).footprint_radius(c.params) == pytest.approx(0.25)


def test_adopt_keeps_pending_flag_and_set_pending():
    c = _core()
    c.adopt_keepouts([(0, 5.0, 5.3, 0.3, 17, True)], 0.0)
    t = c.find_track(17)
    assert t.promoted and t.pending
    c.set_pending(17, False)            # accepted in the GUI
    assert not t.pending
    c.adopt_keepouts([(0, 2.0, 2.0, 0.3, 18)], 0.0)   # old 5-tuple form: not pending
    assert not c.find_track(18).pending


# ------------------------------------------------------------------ map_server

def _map_server(msn, snapshot):
    pt = types.SimpleNamespace
    n = msn.MapServerNode.__new__(msn.MapServerNode)
    n.store = core.MapStore()
    n.store.add_area('lawn', SQUARE, False, [])
    n.tracker_snapshot = snapshot
    n._pump = mock.MagicMock()
    n._tracker_sub = None
    n.rebuild = mock.MagicMock()
    n.persist_best_effort = mock.MagicMock()
    n.get_logger = lambda: mock.MagicMock()
    return n, pt


def _snap(pt, tid, poly):
    return [pt(id=tid, polygon=pt(points=[pt(x=x, y=y) for x, y in poly]))]


def test_map_server_pending_auto_zone_accept_from_gui_keeps_source(msn):
    c = _core()
    tid = _feed(c, 5)[0]
    t = c.find_track(tid)
    pt = types.SimpleNamespace
    n, pt = _map_server(msn, _snap(pt, tid, t.footprint(c.params)))
    req = pt(pending_id=0, polygon=pt(points=[]), obstacle_id=tid, area_index=0,
             name='auto: trunk #%d' % tid)
    assert n.srv_promote_obstacle(req, pt(success=False, message='')).success
    obs = n.store.areas[0].obstacles
    assert obs[0].pending and obs[0].source == core.SOURCE_TRACKER
    # not saved while pending
    assert core.parse_areas_dat(core.format_areas_dat(n.store.areas))[0][0].obstacles == []
    # the GUI's Accept sends the tracker polygon + id: accept the proposal (was a no-op)
    gui = pt(pending_id=0, obstacle_id=tid, area_index=0, name='',
             polygon=pt(points=[pt(x=x, y=y) for x, y in t.footprint(c.params)]))
    res = n.srv_promote_obstacle(gui, pt(success=False, message=''))
    assert res.success and 'accepted' in res.message
    assert len(obs) == 1 and not obs[0].pending and obs[0].source == core.SOURCE_TRACKER
    saved = core.parse_areas_dat(core.format_areas_dat(n.store.areas))[0][0].obstacles
    assert len(saved) == 1 and saved[0].source == core.SOURCE_TRACKER     # source = auto
    assert saved[0].name == 'auto: trunk #%d' % tid
    n.persist_best_effort.assert_called_once()


def test_map_server_resize_replaces_pending_polygon(msn):
    from mower_map.obstacle_tracker_core import ring_polygon
    pt = types.SimpleNamespace
    n, pt = _map_server(msn, _snap(pt, 3, ring_polygon(5.0, 5.3, 0.225, 12)))
    req = pt(pending_id=0, polygon=pt(points=[]), obstacle_id=3, area_index=0,
             name='auto: trunk #3')
    assert n.srv_promote_obstacle(req, pt(success=False, message='')).success
    n.tracker_snapshot = _snap(pt, 3, ring_polygon(5.0, 5.3, 0.32, 12))
    res = n.srv_promote_obstacle(req, pt(success=False, message=''))
    assert res.success and 're-sized' in res.message
    obs = n.store.areas[0].obstacles
    assert len(obs) == 1 and obs[0].pending
    assert max(math.hypot(x - 5.0, y - 5.3) for x, y in obs[0].polygon) == pytest.approx(
        0.32, abs=1e-3)
    n.persist_best_effort.assert_not_called()
    # a user polygon for an unrelated tracker id is still a permanent user keepout
    user = pt(pending_id=0, obstacle_id=44, area_index=0, name='',
              polygon=pt(points=[pt(x=x, y=y) for x, y in ring_polygon(2.0, 2.0, 0.3, 8)]))
    assert n.srv_promote_obstacle(user, pt(success=False, message='')).success
    assert obs[-1].source == core.SOURCE_USER and not obs[-1].pending


def test_store_find_pending_tracker_matches_id_tag_only():
    s = core.MapStore()
    s.add_area('lawn', SQUARE, False, [])
    s.add_obstacle(0, [(1, 1), (2, 1), (2, 2)], 'auto: trunk #17', core.SOURCE_TRACKER, True)
    s.add_obstacle(0, [(5, 5), (6, 5), (6, 6)], 'auto: trunk #171', core.SOURCE_TRACKER, True)
    s.add_obstacle(0, [(8, 8), (9, 8), (9, 9)], 'tree #17', core.SOURCE_USER, False)
    assert s.find_pending_tracker(17).name == 'auto: trunk #17'
    assert s.find_pending_tracker(171).name == 'auto: trunk #171'
    assert s.find_pending_tracker(5) is None and s.find_pending_tracker(0) is None


# ------------------------------------------------------------------ node helpers

def test_blocked_areas_from_settings_snapshot():
    import importlib
    import sys
    try:
        import rclpy  # noqa: F401
    except ImportError:
        pytest.skip('rclpy not installed')
    mod = importlib.import_module('mower_map.obstacle_tracker_node')
    areas = [(0, 'Area 5', SQUARE), (1, 'Area 7', SQUARE)]
    js = ('{"defaults": {"obstacle_detection": "standard"}, '
          '"areas": {"Area 5": {"obstacle_detection": "none"}, "Area 7": {}}}')
    assert mod.blocked_areas(js, areas) == {0}
    assert mod.blocked_areas('{"defaults": {"obstacle_detection": "none"}}', areas) == {0, 1}
    assert mod.blocked_areas('garbage', areas) == set()
    p = mod.core_params(lambda k: mod.PARAMS[k])
    assert p.static_classes[0] == 'trunk' and 'person' in p.never_classes
    assert dict(p.class_prior_m)['trunk'] == 0.25
    # det_range ranges the front face: the zone point moves half a width along the ray
    ns = types.SimpleNamespace
    det = ns(results=[ns(hypothesis=ns(class_id='trunk', score=0.8),
                         pose=ns(pose=ns(position=ns(x=2.466, y=0.0)),
                                 covariance=[0.01] + [0.0] * 34 + [0.3]))])
    x, y, label, score, w = mod.det_observation(det, (1.0, 2.0, math.pi / 2), mod.DET_CLASSES)
    assert (label, w) == ('trunk', 0.3)
    assert x == pytest.approx(1.0) and y == pytest.approx(2.0 + 2.466 + 0.15)
    sys.modules.pop('mower_map.obstacle_tracker_node', None)


def test_footprint_box_matches_nav2():
    assert FOOTPRINT_BOX == (-0.10, 0.60, -0.235, 0.235)
    assert math.hypot(0.60, 0.235) == pytest.approx(0.644, abs=1e-3)
