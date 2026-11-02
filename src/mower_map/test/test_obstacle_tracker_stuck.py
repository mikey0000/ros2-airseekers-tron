"""Repeated stuck spots -> pending no-go proposal (stock parity 2026-11-03)."""

import numpy as np

from mower_map.obstacle_tracker_core import TrackerCore, TrackerParams
from mower_map.obstacle_tracker_stuck import StuckSpotStore

SQUARE = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
LAWN = [(0, 'lawn', SQUARE)]
NONE = np.empty((0, 2))


def test_second_stuck_within_radius_proposes_and_persists(tmp_path):
    path = str(tmp_path / 'stuck_spots.json')
    s = StuckSpotStore(path, radius_m=1.0, hits=2)
    assert s.record(3.0, 3.0, 1000.0, 'STUCK_NEEDS_HELP') is None
    # a restart (new session) keeps the first event
    s2 = StuckSpotStore(path, radius_m=1.0, hits=2)
    assert len(s2.events) == 1
    assert s2.record(9.0, 9.0, 2000.0, 'stuck') is None          # elsewhere
    spot = s2.record(3.6, 3.0, 90000.0, 'stuck')
    assert spot is not None and abs(spot[0] - 3.3) < 1e-9
    # consumed: the next event at the spot starts a new count
    assert [e['x'] for e in StuckSpotStore(path).events] == [9.0]


def test_duplicate_reports_of_one_incident_count_once(tmp_path):
    s = StuckSpotStore(str(tmp_path / 's.json'), radius_m=1.0, hits=2, min_gap_s=60.0)
    assert s.record(3.0, 3.0, 100.0, 'stuck') is None
    assert s.record(3.1, 3.0, 110.0, 'STUCK_NEEDS_HELP') is None   # same incident
    assert len(s.events) == 1
    assert s.record(3.1, 3.0, 400.0, 'stuck') is not None


def test_old_events_expire_and_bad_file_is_ignored(tmp_path):
    p = tmp_path / 's.json'
    p.write_text('not json')
    s = StuckSpotStore(str(p), max_age_days=1.0)
    assert s.events == []
    s.record(3.0, 3.0, 0.0, 'stuck')
    assert s.record(3.0, 3.0, 2 * 86400.0, 'stuck') is None        # first one expired


def test_stuck_zone_promotes_pending_outside_mowing_and_survives_session_end():
    core = TrackerCore(TrackerParams(stuck_keepout_radius_m=1.0, class_zones=False))
    t = core.add_stuck_zone(5.0, 5.0, 0.0, LAWN)
    assert t is not None and t.footprint_radius(core.params) == 1.0
    assert core.add_stuck_zone(5.5, 5.0, 0.0, LAWN) is None        # already proposed
    out = core.step(NONE, 1.0, areas=LAWN, allow_promote=False)
    assert out['promote'] == [t.id]
    core.confirm_promoted(t.id, 1.0)
    assert t.pending and t.promoted
    for k in range(2, 40):                                         # never expires / removed
        out = core.step(NONE, k * 200.0, areas=LAWN, robot_area=0)
        assert not out['remove']
    assert core.drop_pending() == [] and core.find_track(t.id) is not None


def test_stuck_zone_respects_ignore_and_needs_an_area():
    core = TrackerCore(TrackerParams())
    assert core.add_stuck_zone(50.0, 50.0, 0.0, LAWN) is not None   # outside every area
    assert core.step(NONE, 1.0, areas=LAWN)['promote'] == []
    t = core.add_stuck_zone(5.0, 5.0, 0.0, LAWN)
    core.step(NONE, 1.0, areas=LAWN)
    core.confirm_promoted(t.id, 1.0)
    core.ignore(t.id)
    assert core.add_stuck_zone(5.1, 5.0, 2.0, LAWN) is None
