# SPDX-License-Identifier: GPL-3.0-or-later
"""ROS-free tests for the obstacle auto-marker (run: python3 -m pytest src/mower_map/test).

Covers mower_map.obstacle_tracker_core (clustering, tracking, the promote /
remove gates) and the MapStore removal helpers the tracker's ~/remove_obstacle
path uses.
"""

import numpy as np
import pytest

from mower_map import areas as core
from mower_map.obstacle_tracker_core import (TrackerCore, TrackerParams, cell_clusters,
                                             cluster_points, ring_polygon)

SQUARE = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
LAWN = [(0, 'lawn', SQUARE)]
P0 = np.array([[5.0, 5.0]])


def _core(**kw):
    # promotion gate tests switch promotion on; the shipped default is off (B2)
    kw.setdefault('auto_promote', True)
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


# ------------------------------------------ B2 (review 2026-10-13): safe defaults

import ast  # noqa: E402
import os  # noqa: E402
import types  # noqa: E402
from unittest import mock  # noqa: E402

import yaml  # noqa: E402

from mower_map import obstacle_snapshot as snap  # noqa: E402
from mower_map.obstacle_tracker_core import keepout_entry, keepout_id  # noqa: E402

from test_dock_yaw_autocorrect import msn  # noqa: E402,F401 (fixture)

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _node_params():
    """obstacle_tracker_node.PARAMS without importing rclpy (literal dict in the source)."""
    tree = ast.parse(open(os.path.join(PKG, 'mower_map', 'obstacle_tracker_node.py')).read())
    for node in tree.body:
        if isinstance(node, ast.Assign) and getattr(node.targets[0], 'id', '') == 'PARAMS':
            out = {}
            for k, v in zip(node.value.keys, node.value.values):
                try:
                    out[k.value] = ast.literal_eval(v)
                except ValueError:
                    out[k.value] = None        # list(...) of a module constant
            return out
    raise AssertionError('PARAMS not found')


def _launch_args():
    tree = ast.parse(open(os.path.join(PKG, 'launch', 'obstacle_tracker.launch.py')).read())
    for node in tree.body:
        if isinstance(node, ast.Assign) and getattr(node.targets[0], 'id', '') == 'ARGS':
            return {e.elts[0].value: e.elts[1].value for e in node.value.elts}
    raise AssertionError('ARGS not found')


def test_defaults_are_safe_everywhere():
    """auto marking off, mowing only, minutes of persistence, <= 1 m: in the core, the node
    parameters, the params yaml and the launch args (which win over the yaml)."""
    core_p = TrackerParams()
    node_p = _node_params()
    with open(os.path.join(PKG, 'config', 'obstacle_tracker.yaml')) as fh:
        yaml_p = yaml.safe_load(fh)['obstacle_tracker']['ros__parameters']
    launch = _launch_args()
    assert core_p.auto_promote is False
    for src in (node_p, yaml_p):
        assert src['auto_promote'] is False
        assert src['only_while_mowing'] is True
        assert src['persistence_age_s'] >= 60.0
        assert src['max_obstacle_radius_m'] <= 1.0
    assert core_p.persistence_age_s >= 60.0 and core_p.max_obstacle_radius_m <= 1.0
    assert launch['auto_promote'] == 'false' and launch['only_while_mowing'] == 'true'
    for c in ('person', 'dog', 'cat'):
        assert c in yaml_p['dynamic_classes']


def _shipped(**kw):
    """The shipped tunables with auto_promote switched on (GUI toggle)."""
    kw.setdefault('auto_promote', True)
    return TrackerCore(TrackerParams(**kw))


PERSON = np.array([[5.0, 5.0], [5.1, 5.05], [5.05, 5.15]])


@pytest.mark.parametrize('stand_s', [3, 5, 10])
def test_person_standing_still_is_never_promoted(stand_s):
    """A person standing 3-10 s (seen by the stereo cloud every second, ranged by YOLO)
    then walking off: no promotion, neither while standing nor minutes later."""
    c = _shipped()
    t0 = 1000.0
    for k in range(stand_s + 1):
        res = c.step(PERSON, now=t0 + k, areas=LAWN, dynamic_points=np.array([[5.05, 5.1]]))
        assert res['promote'] == []
    assert all(t.dynamic for t in c.tracks)
    for k in range(stand_s + 1, 400):
        assert c.step(np.empty((0, 2)), now=t0 + k, areas=LAWN)['promote'] == []
    assert c.tracks == []                                    # timed out, never promoted


@pytest.mark.parametrize('stand_s', [3, 10])
def test_undetected_person_standing_briefly_is_never_promoted(stand_s):
    """Even without a YOLO detection (only the depth cloud), 3-10 s of standing still is far
    below persistence_age_s, and a track that left view never promotes later."""
    c = _shipped()
    for k in range(stand_s + 1):
        assert c.step(PERSON, now=float(k), areas=LAWN)['promote'] == []
    for k in range(stand_s + 1, 400):
        assert c.step(np.empty((0, 2)), now=float(k), areas=LAWN)['promote'] == []


def test_dynamic_veto_holds_even_with_a_short_persistence():
    c = _shipped(persistence_age_s=3.0, min_observations=3)
    dyn = np.array([[5.6, 5.0]])                             # ranged person 0.6 m away
    for k in range(60):
        res = c.step(PERSON, now=float(k), areas=LAWN,
                     dynamic_points=dyn if k == 2 else None)  # seen once is enough
        assert res['promote'] == []
    far = _shipped(persistence_age_s=3.0, min_observations=3)
    got = []
    for k in range(5):
        got += far.step(PERSON, now=float(k), areas=LAWN,
                        dynamic_points=np.array([[9.0, 9.0]]))['promote']
    assert got                                               # a person 5 m away: no veto


def test_static_blob_promotes_after_minutes_as_pending_only(msn):
    """A static blob seen every second for > persistence_age_s promotes once; map_server
    turns the tracker's id-only request into a PENDING keepout that is not written to
    areas.dat until the operator accepts it."""
    c = _shipped()
    blob = np.array([[5.0, 5.0], [5.1, 5.0], [5.0, 5.1]])
    promoted = []
    for k in range(0, 200):
        promoted += c.step(blob, now=float(k), areas=LAWN)['promote']
        if promoted:
            break
    assert len(promoted) == 1 and k >= TrackerParams().persistence_age_s
    t = c.find_track(promoted[0])

    n = msn.MapServerNode.__new__(msn.MapServerNode)
    n.store = core.MapStore()
    n.store.add_area('lawn', SQUARE, False, [])
    pt = types.SimpleNamespace
    n.tracker_snapshot = [pt(id=t.id, polygon=pt(points=[pt(x=x, y=y)
                                                        for x, y in t.footprint(c.params)]))]
    n._pump = mock.MagicMock()
    n._tracker_sub = None
    n.rebuild = mock.MagicMock()
    n.persist_best_effort = mock.MagicMock()
    n.get_logger = lambda: mock.MagicMock()
    req = pt(pending_id=0, polygon=pt(points=[]), obstacle_id=t.id, area_index=t.area_index,
             name='auto: obstacle #%d' % t.id)
    res = n.srv_promote_obstacle(req, pt(success=False, message=''))
    assert res.success and 'pending' in res.message
    obs = n.store.areas[0].obstacles
    assert len(obs) == 1 and obs[0].pending and obs[0].source == core.SOURCE_TRACKER
    n.persist_best_effort.assert_not_called()
    assert core.parse_areas_dat(core.format_areas_dat(n.store.areas))[0][0].obstacles == []
    # operator accepts it -> persisted
    acc = n.srv_promote_obstacle(pt(pending_id=obs[0].id, name='', polygon=pt(points=[]),
                                    obstacle_id=0, area_index=0),
                                 pt(success=False, message=''))
    assert acc.success and not obs[0].pending
    n.persist_best_effort.assert_called_once()


def test_user_polygon_promotion_is_still_permanent(msn):
    """The GUI's promote (polygon supplied) stays an immediate, persisted keepout."""
    n = msn.MapServerNode.__new__(msn.MapServerNode)
    n.store = core.MapStore()
    n.store.add_area('lawn', SQUARE, False, [])
    n.rebuild = mock.MagicMock()
    n.persist_best_effort = mock.MagicMock()
    n.get_logger = lambda: mock.MagicMock()
    pt = types.SimpleNamespace
    poly = pt(points=[pt(x=x, y=y) for x, y in ring_polygon(3.0, 3.0, 0.3, 8)])
    res = n.srv_promote_obstacle(pt(pending_id=0, polygon=poly, obstacle_id=4, area_index=0,
                                    name=''), pt(success=False, message=''))
    assert res.success
    o = n.store.areas[0].obstacles[0]
    assert not o.pending and o.source == core.SOURCE_USER
    n.persist_best_effort.assert_called_once()


def test_cluster_over_one_metre_is_rejected():
    c = _shipped(persistence_age_s=0.0, min_observations=1)
    g = np.mgrid[-1.2:1.21:0.1, -1.2:1.21:0.1].reshape(2, -1).T
    disc = g[np.hypot(g[:, 0], g[:, 1]) <= 1.2] + 5.0      # one blob, 1.2 m radius
    assert len(cluster_points(disc, 0.1, 2)) == 1
    for k in range(5):
        assert c.step(disc, now=float(k), areas=LAWN)['promote'] == []
    assert c.tracks == []
    # a track that grows past the cap after it formed never promotes either
    c = _shipped(persistence_age_s=0.0, min_observations=3)
    c.step(P0, now=0.0, areas=LAWN)
    c.tracks[0].radius = 1.5
    for k in range(1, 5):
        assert c.step(P0, now=float(k), areas=LAWN)['promote'] == []


def test_restart_re_adopts_tracker_keepouts():
    """A keepout promoted (and accepted) before a restart is re-adopted from areas.dat
    under its tracker id, so the new tracker still removes it once the object is gone;
    user keepouts are not adopted."""
    store = core.MapStore()
    store.add_area('lawn', SQUARE, False, [])
    store.add_obstacle(0, ring_polygon(5.0, 5.0, 0.25, 12), 'auto: obstacle #17',
                       core.SOURCE_TRACKER)
    store.add_obstacle(0, ring_polygon(2.0, 2.0, 0.25, 12), 'tree', core.SOURCE_USER)
    areas, _ = core.parse_areas_dat(core.format_areas_dat(store.areas))   # restart
    keepouts = [keepout_entry(i, o.polygon, o.name) for i, a in enumerate(areas)
                for o in a.obstacles if o.source == core.SOURCE_TRACKER]
    assert len(keepouts) == 1 and keepouts[0][4] == 17

    c = _shipped(min_lifetime_s=0.0, gone_confirmations=2)
    c.reserve_ids(30)                      # snapshot files 1..29 kept on disk
    assert c.adopt_keepouts(keepouts, now=10.0) == [17]
    assert c.adopt_keepouts(keepouts, now=11.0) == []      # idempotent per refresh
    t = c.find_track(17)
    assert t.promoted and t.adopted and abs(t.cx - 5.0) < 1e-6
    assert abs(t.footprint_radius(c.params) - 0.25) < 0.01
    clear = np.array([[5.0, 5.0]])
    assert c.step(np.empty((0, 2)), now=12.0, clear_points=clear, areas=LAWN)['remove'] == []
    assert c.step(np.empty((0, 2)), now=13.0, clear_points=clear, areas=LAWN)['remove'] == [17]
    restored = core.MapStore()
    restored.load(areas)
    ok, _ = restored.remove_obstacle(centroid=(t.cx, t.cy),
                                     max_dist=t.footprint_radius(c.params),
                                     sources=(core.SOURCE_TRACKER,))
    assert ok and [o.name for o in restored.areas[0].obstacles] == ['tree']
    # a new obstacle after the restart gets an id above the kept snapshots
    c.step(np.array([[8.0, 8.0]]), now=14.0, areas=LAWN)
    assert max(tr.id for tr in c.tracks) >= 30


def test_adopt_takes_over_a_live_track_at_the_spot():
    c = _shipped()
    c.step(P0, now=0.0, areas=LAWN)
    tid = c.tracks[0].id
    assert c.adopt_keepouts([(0, 5.0, 5.0, 0.2, 0)], now=1.0) == [tid]
    assert len(c.tracks) == 1 and c.tracks[0].promoted


def test_keepout_id_parsing():
    assert keepout_id('auto: obstacle #17') == 17
    assert keepout_id('apple tree') == 0 and keepout_id(None) == 0


def test_snapshot_ids_and_prune_keep_live(tmp_path):
    for i in (3, 7, 12):
        (tmp_path / snap.snapshot_filename(i)).write_bytes(b'x')
    (tmp_path / 'notes.txt').write_text('keep')
    assert snap.snapshot_ids(str(tmp_path)) == {3, 7, 12}
    assert snap.prune_snapshots(str(tmp_path), {7}) == 2
    assert sorted(os.listdir(tmp_path)) == ['7.jpg', 'notes.txt']
