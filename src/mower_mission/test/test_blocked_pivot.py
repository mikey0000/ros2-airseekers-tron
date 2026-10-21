"""2026-10-21 owner rule (stock firmware): a blocked coverage pivot is skipped, not re-sent."""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from mower_mission import mission_fsm as f             # noqa: E402
from test_mission_fsm import Harness, start_until_planning   # noqa: E402
from test_path_conditioning import ring, ring_h        # noqa: E402


def _blocked(**params):
    r = ring()
    y0 = math.atan2(r[1][1] - r[0][1], r[1][0] - r[0][0])
    h = ring_h(yaw=y0 + math.radians(140), **params)
    assert h.pending_action(f.ACT_SPIN).purpose == 'cov_pivot'
    return h


def _logs(h, mk=0):
    return [e.text for e in h.since(mk, f.Log)]


def test_blocked_coverage_pivot_is_skipped_not_resent():
    h = _blocked()
    m = h.fsm.mission
    assert h.fsm.p.blocked_pivot_policy == 'skip'
    mk = h.mark()
    h.finish(f.ACT_SPIN, f.ABORTED)                     # "Collision Ahead": yaw unchanged
    starts = h.since(mk, f.StartAction)
    assert not any(e.name == f.ACT_SPIN for e in starts)  # never the same Spin again
    assert any('blocked pivot: skip' in t and 'at (' in t for t in _logs(h, mk))
    assert m.pivot_skips == 1
    ac = h.fsm.cursor.area(m.area_idx)
    assert ac.skipped and ac.skipped[0][0] == 0         # recorded for the end-of-area pass
    assert h.name != 'STUCK_NEEDS_HELP'
    piv = [c for c in m.cond[0]['pivots'] if c > 0]
    if piv:                                             # next leg of the same sub-path
        assert m.sub_i == 0 and m.start_local == piv[0]
        assert h.pending_action(f.ACT_NAV) and m.step == 'transit'
    else:                                               # next sub-path
        assert m.sub_i == 1


def test_blocked_pivot_without_requeue_is_reported_uncut():
    h = _blocked(blocked_pivot_requeue=False)
    m = h.fsm.mission
    h.finish(f.ACT_SPIN, f.TIMEOUT)
    assert m.blocked and m.blocked[0][0] == m.area_idx and m.blocked[0][1] == 0
    assert not h.fsm.cursor.area(m.area_idx).skipped
    assert 'blocked' in h.fsm._not_mowed_text()


def test_blocked_pivot_cap_falls_back_to_resend_then_stuck():
    h = _blocked(blocked_pivot_max_skips=0)
    for _ in range(2):
        h.finish(f.ACT_SPIN, f.ABORTED)
        assert h.pending_action(f.ACT_SPIN).purpose == 'cov_pivot'
    h.finish(f.ACT_SPIN, f.ABORTED)
    assert h.name == 'STUCK_NEEDS_HELP'


def test_blocked_pivot_within_threshold_follows_the_leg():
    h = _blocked()
    x, y = h.fsm.inputs.pose[:2]
    h.fsm.inputs.pose = (x, y, h.fsm.mission.pivot_target[0] + math.radians(5))
    h.finish(f.ACT_SPIN, f.ABORTED)
    assert h.pending_action(f.ACT_FOLLOW) and h.fsm.mission.pivot_skips == 0


def test_blocked_pivot_in_end_of_area_retry_is_reported_blocked():
    h = _blocked()
    m = h.fsm.mission
    h.finish(f.ACT_SPIN, f.ABORTED)
    stretch = h.fsm.cursor.area(m.area_idx).skipped[0]
    # the end-of-area pass re-tries the piece once; blocked again -> reported, no re-send
    m.requeue = [stretch]
    h.fsm._cancel_all('test')
    h.fsm._next_stretch()
    h.fsm.inputs.is_cutting = True
    for _ in range(50):
        if h.fsm._action is not None:
            break
        h.tick()
    assert h.pending_action(f.ACT_SPIN).purpose == 'cov_pivot'
    mk = h.mark()
    h.finish(f.ACT_SPIN, f.ABORTED)
    assert (m.area_idx,) + tuple(stretch) in m.blocked
    assert any('blocked pivot: skip' in t for t in _logs(h, mk))


def test_start_resets_the_pivot_retry_counter():
    h = Harness(path_conditioning=True)
    h.fsm._pivot_tries = {('cov', 0, 0, 0): 2}
    start_until_planning(h)
    assert h.fsm._pivot_tries == {}
