# SPDX-License-Identifier: GPL-3.0-or-later
"""ROS-free tests for the obstacle auto-marker (run: python3 -m pytest src/mower_map/test).

Covers mower_map.obstacle_tracker_core (clustering, tracking, the promote /
remove gates) and the MapStore removal helpers the tracker's ~/remove_obstacle
path uses.
"""

import numpy as np

from mower_map import areas as core
from mower_map.obstacle_tracker_core import (TrackerCore, TrackerParams, cell_clusters,
                                             cluster_points, ring_polygon)

SQUARE = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
LAWN = [(0, 'lawn', SQUARE)]
P0 = np.array([[5.0, 5.0]])


def _core(**kw):
    return TrackerCore(TrackerParams(**kw))


# --------------------------------------------------------------- clustering

def test_cluster_points_merges_neighbours_only():
    pts = np.array([[0.0, 0.0], [0.05, 0.0],        # one blob (same cell)
                    [3.0, 3.0], [3.1, 3.05]])       # another blob
    clusters = cluster_points(pts, resolution=0.1, merge_cells=2)
    assert len(clusters) == 2
    assert sorted(len(c) for c in clusters) == [2, 2]


def test_cluster_points_separates_distant_blobs():
    pts = np.array([[0.0, 0.0], [5.0, 5.0]])
    assert len(cluster_points(pts, 0.1, 2)) == 2


def test_cell_clusters_joins_across_one_cell_gap():
    # (0,0) and (2,0) are <= merge_cells apart -> one group.
    assert len(cell_clusters([(0, 0), (2, 0)], merge_cells=2)) == 1
    assert len(cell_clusters([(0, 0), (3, 0)], merge_cells=2)) == 2


def test_ring_polygon_is_closed_and_sized():
    poly = ring_polygon(1.0, 2.0, 0.5, 8)
    assert len(poly) == 8
    for x, y in poly:
        assert abs(np.hypot(x - 1.0, y - 2.0) - 0.5) < 1e-9


# --------------------------------------------------------------- promotion

def test_track_promotes_after_age_and_observations():
    c = _core(persistence_age_s=3.0, min_observations=3)
    assert c.step(P0, now=100.0, areas=LAWN)['promote'] == []
    assert c.step(P0, now=101.0, areas=LAWN)['promote'] == []
    assert c.step(P0, now=102.5, areas=LAWN)['promote'] == []   # age 2.5 < 3
    res = c.step(P0, now=103.5, areas=LAWN)                      # obs 4, age 3.5
    assert len(res['promote']) == 1
    tid = res['promote'][0]
    c.confirm_promoted(tid, now=103.5)
    assert c.find_track(tid).promoted


def test_no_promotion_before_min_observations():
    c = _core(persistence_age_s=0.0, min_observations=5)
    for k in range(4):
        assert c.step(P0, now=float(k), areas=LAWN)['promote'] == []
    assert c.step(P0, now=4.0, areas=LAWN)['promote'] != []


def test_no_track_outside_mowing_area():
    c = _core(persistence_age_s=0.0, min_observations=1)
    outside = np.array([[20.0, 20.0]])
    assert c.step(outside, now=1.0, areas=LAWN)['promote'] == []
    assert c.tracks == []


def test_tracks_outside_area_when_required_off_but_never_promote():
    c = _core(persistence_age_s=0.0, min_observations=1, require_inside_mowing_area=False)
    outside = np.array([[20.0, 20.0]])
    c.step(outside, now=1.0, areas=LAWN)
    assert len(c.tracks) == 1
    assert c.tracks[0].area_index == -1
    assert c.step(outside, now=2.0, areas=LAWN)['promote'] == []   # no area -> no promote


def test_oversized_and_tiny_clusters_rejected():
    c = _core(persistence_age_s=0.0, min_observations=1,
              min_obstacle_radius_m=0.05, max_obstacle_radius_m=1.0, det_radius_m=0.0)
    # A tight line < min radius -> the (single) cluster is dropped (radius 0).
    tiny = np.array([[5.0, 5.0]])
    c.step(tiny, now=0.0, areas=LAWN)
    assert c.tracks == []


def test_promotion_disabled():
    c = _core(auto_promote=False, persistence_age_s=0.0, min_observations=1)
    for k in range(5):
        assert c.step(P0, now=float(k), areas=LAWN)['promote'] == []
    assert c.tracks and not c.tracks[0].promoted


def test_allow_promote_gate_blocks_promotion_but_keeps_tracking():
    c = _core(persistence_age_s=0.0, min_observations=1)
    for k in range(5):
        res = c.step(P0, now=float(k), areas=LAWN, allow_promote=False)
        assert res['promote'] == []
    assert len(c.tracks) == 1 and not c.tracks[0].promoted
    # once mowing (gate open), the already-tracked cluster promotes immediately
    assert c.step(P0, now=6.0, areas=LAWN, allow_promote=True)['promote'] == [c.tracks[0].id]


def test_drifting_track_is_not_promoted():
    # A cluster that keeps sliding (slow moving object / swaying bush) never
    # meets the drift gate even though each step associates.
    c = _core(persistence_age_s=0.0, min_observations=3, max_track_drift_m=0.2,
              association_dist_m=0.5)
    for k in range(5):
        pts = np.array([[3.0 + 0.25 * k, 5.0]])
        assert c.step(pts, now=float(k), areas=LAWN)['promote'] == []
    assert c.tracks


def test_transient_timeout_drops_unpromoted_track():
    c = _core(transient_timeout_s=5.0, persistence_age_s=1000.0)
    c.step(P0, now=100.0, areas=LAWN)
    assert len(c.tracks) == 1
    c.step(np.empty((0, 2)), now=104.0, areas=LAWN)     # unseen 4 s < 5
    assert len(c.tracks) == 1
    c.step(np.empty((0, 2)), now=106.0, areas=LAWN)     # unseen 6 s > 5
    assert c.tracks == []


# --------------------------------------------------------------- removal

def _promoted_core(**kw):
    c = _core(persistence_age_s=0.0, min_observations=1, **kw)
    res = c.step(P0, now=100.0, areas=LAWN)
    tid = res['promote'][0]
    c.confirm_promoted(tid, now=100.0)
    return c, tid


def test_present_point_resets_and_prevents_removal():
    c, tid = _promoted_core(min_lifetime_s=0.0, gone_confirmations=2)
    for k in range(4):
        res = c.step(P0, now=101.0 + k, robot_pose=(4.0, 5.0, 0.0), areas=LAWN)
        assert res['remove'] == []
    assert c.find_track(tid) is not None


def test_removal_requires_confirmation():
    c, tid = _promoted_core(min_lifetime_s=0.0, gone_confirmations=3)
    for k in range(2):
        res = c.step(np.empty((0, 2)), now=101.0 + k, robot_pose=(4.0, 5.0, 0.0), areas=LAWN)
        assert res['remove'] == []
    res = c.step(np.empty((0, 2)), now=103.0, robot_pose=(4.0, 5.0, 0.0), areas=LAWN)
    assert res['remove'] == [tid]
    c.confirm_removed(tid)
    assert c.find_track(tid) is None


def test_removal_blocked_until_min_lifetime():
    c, tid = _promoted_core(min_lifetime_s=10.0, gone_confirmations=1)
    for k in range(5):
        res = c.step(np.empty((0, 2)), now=105.0 + k, robot_pose=(4.0, 5.0, 0.0), areas=LAWN)
        assert res['remove'] == []          # now - promoted_at(=100) < 10
    res = c.step(np.empty((0, 2)), now=111.0, robot_pose=(4.0, 5.0, 0.0), areas=LAWN)
    assert res['remove'] == [tid]


def test_absence_requires_robot_near_and_facing():
    c, tid = _promoted_core(min_lifetime_s=0.0, gone_confirmations=2)
    # Far away: no votes accumulate.
    c.step(np.empty((0, 2)), now=101.0, robot_pose=(0.0, 0.0, 0.0), areas=LAWN)
    c.step(np.empty((0, 2)), now=102.0, robot_pose=(0.0, 0.0, 0.0), areas=LAWN)
    assert c.find_track(tid).gone_votes == 0
    # Near but facing away (yaw = pi = away from +x towards the obstacle): no vote.
    c.step(np.empty((0, 2)), now=103.0, robot_pose=(4.0, 5.0, 3.14159), areas=LAWN)
    assert c.find_track(tid).gone_votes == 0
    # Near and facing: votes.
    r1 = c.step(np.empty((0, 2)), now=104.0, robot_pose=(4.0, 5.0, 0.0), areas=LAWN)
    assert r1['remove'] == []
    r2 = c.step(np.empty((0, 2)), now=105.0, robot_pose=(4.0, 5.0, 0.0), areas=LAWN)
    assert r2['remove'] == [tid]


def test_clear_ground_point_counts_even_far_away():
    c, tid = _promoted_core(min_lifetime_s=0.0, gone_confirmations=2)
    clear = np.array([[5.0, 5.0]])
    r1 = c.step(np.empty((0, 2)), now=101.0, clear_points=clear, areas=LAWN)
    assert r1['remove'] == []
    r2 = c.step(np.empty((0, 2)), now=102.0, clear_points=clear, areas=LAWN)
    assert r2['remove'] == [tid]


def test_auto_remove_disabled_keeps_promoted_track():
    c, tid = _promoted_core(auto_remove=False, min_lifetime_s=0.0, gone_confirmations=1)
    for k in range(5):
        assert c.step(np.empty((0, 2)), now=101.0 + k,
                      robot_pose=(4.0, 5.0, 0.0), areas=LAWN)['remove'] == []
    assert c.find_track(tid) is not None


def test_stale_sensors_do_not_vote_for_removal():
    # A dead/stale cloud must never be read as "the object is gone".
    c, tid = _promoted_core(min_lifetime_s=0.0, gone_confirmations=1)
    for k in range(5):
        res = c.step(np.empty((0, 2)), now=101.0 + k, robot_pose=(4.0, 5.0, 0.0),
                     areas=LAWN, obstacle_fresh=False, clear_fresh=False)
        assert res['remove'] == []
    assert c.find_track(tid).gone_votes == 0


def test_removal_not_re_emitted_while_in_flight():
    c, tid = _promoted_core(min_lifetime_s=0.0, gone_confirmations=1)
    assert c.step(np.empty((0, 2)), now=101.0, robot_pose=(4.0, 5.0, 0.0), areas=LAWN)['remove'] == [tid]
    # a second step before the service answers must not fire again...
    assert c.step(np.empty((0, 2)), now=102.0, robot_pose=(4.0, 5.0, 0.0), areas=LAWN)['remove'] == []
    # ...but a failed call re-arms it.
    c.reject_removal(tid)
    assert c.step(np.empty((0, 2)), now=103.0, robot_pose=(4.0, 5.0, 0.0), areas=LAWN)['remove'] == [tid]


# ------------------------------------------------------- store removal (map_server)

def _store_with_tracker_obstacle():
    s = core.MapStore()
    s.add_area('lawn', SQUARE, False, [])
    s.add_obstacle(0, ring_polygon(5.0, 5.0, 0.2, 8), 'auto', core.SOURCE_TRACKER)
    s.add_area('other', SQUARE, False,
               [(ring_polygon(2.0, 2.0, 0.2, 8), 'manual', core.SOURCE_USER)])
    return s


def test_remove_obstacle_by_centroid_tracker_only():
    s = _store_with_tracker_obstacle()
    ok, _ = s.remove_obstacle(centroid=(5.05, 5.05), max_dist=0.5,
                              sources=(core.SOURCE_TRACKER,))
    assert ok
    assert all(o.source != core.SOURCE_TRACKER for a in s.areas for o in a.obstacles)
    # the user keepout is untouched
    assert any(o.source == core.SOURCE_USER for a in s.areas for o in a.obstacles)


def test_remove_obstacle_never_touches_user_keepout():
    s = _store_with_tracker_obstacle()
    ok, _ = s.remove_obstacle(centroid=(2.0, 2.0), max_dist=0.5,
                              sources=(core.SOURCE_TRACKER,))
    assert not ok
    assert any(o.source == core.SOURCE_USER for a in s.areas for o in a.obstacles)


def test_remove_obstacle_by_id():
    s = _store_with_tracker_obstacle()
    obs = [o for a in s.areas for o in a.obstacles if o.source == core.SOURCE_TRACKER][0]
    ok, _ = s.remove_obstacle(obstacle_id=obs.id, sources=(core.SOURCE_TRACKER,))
    assert ok
    assert s.find_obstacle_by_centroid(5.0, 5.0, 1.0, (core.SOURCE_TRACKER,)) is None


def test_remove_obstacle_no_match_is_clean_failure():
    s = _store_with_tracker_obstacle()
    ok, msg = s.remove_obstacle(centroid=(50.0, 50.0), max_dist=0.5,
                                sources=(core.SOURCE_TRACKER,))
    assert not ok and 'no matching' in msg
