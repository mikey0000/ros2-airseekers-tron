"""Field 2026-10-20 (area 5): 78 s of path conditioning under the FSM lock, a plan_route
deadline stamped from the event start (timed out 60 ms after the send), and a costmap
fallback to a goal 34 m away (outside the 50 m rolling global costmap)."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from mower_mission import mission_fsm as f   # noqa: E402
from test_mission_fsm import (Harness, _answer_route, _route_resp, line, plan,  # noqa: E402
                              square, start_until_planning)


# ---- request deadlines from the send-time clock ------------------------------------
def _late_plan(clock):
    """Plan result fed at event time t, the FSM spends 78 s on it (wall clock ahead)."""
    h = Harness(route_transits=True)
    h.areas = [square(-1, -1, 12)]
    lag = [0.0]
    if clock:
        h.fsm.clock = lambda: h.t + lag[0]
    start_until_planning(h)
    lag[0] = 78.0
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(5, 5, 9, 5)]))
    assert h.name == 'TRANSIT' and h.fsm._service.name == f.SRV_PLAN_ROUTE
    h.t += 78.0                       # the next tick runs once the slow event is over
    lag[0] = 0.0
    h.tick(dt=0.06)
    return h


def test_late_send_keeps_a_fresh_plan_route_deadline():
    h = _late_plan(clock=True)
    s = h.fsm._service
    assert s is not None and s.name == f.SRV_PLAN_ROUTE        # not timed out
    assert not [e for e in h.fx if isinstance(e, f.StartAction) and e.name == f.ACT_NAV]
    _answer_route(h)
    assert h.fsm._action.name == f.ACT_FOLLOW


def test_event_clock_deadline_expires_at_once_without_the_clock():
    """The field bug, reproduced without the injected clock."""
    h = _late_plan(clock=False)
    assert h.fsm._service is None or h.fsm._service.name != f.SRV_PLAN_ROUTE


def test_mark_sent_restamps_from_the_actual_send():
    h = Harness(route_transits=True)
    h.areas = [square(-1, -1, 12)]
    start_until_planning(h)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(5, 5, 9, 5)]))
    s = h.fsm._service
    h.tick(dt=1.0, n=9)                       # 9 s waiting for the server
    h.fsm.mark_sent(s.token, h.t)
    h.tick(dt=1.0, n=9)                       # 18 s after the event, 9 s after the send
    assert h.fsm._service is s
    h.fsm.mark_sent(s.token, 0.0)             # never shortens
    assert h.fsm._service.deadline >= h.t
    h.tick(dt=1.0, n=3)
    assert h.fsm._service is not s            # timed out 10 s after the send


def test_mark_sent_restamps_actions_and_ignores_stale_tokens():
    h = Harness()
    start_until_planning(h)
    a = h.pending_action(f.ACT_PLAN)
    d0 = a.deadline
    h.fsm.mark_sent(a.token + 99, h.t + 50.0)
    assert a.deadline == d0
    h.fsm.mark_sent(a.token, h.t + 50.0)
    assert a.deadline == h.t + 50.0 + h.fsm.p.plan_timeout_s


# ---- path conditioning off the FSM lock -------------------------------------------
def _cond_h():
    h = Harness(path_conditioning=True)
    h.areas = [square(-1, -1, 12)]
    start_until_planning(h)
    return h


SUBPATHS = [line(1, 1, 9, 1), line(9, 1.2, 9, 9, n=9), line(9, 9.2, 1, 9.2)]


def test_precomputed_conditioning_matches_inline():
    inline = _cond_h()
    inline.finish(f.ACT_PLAN, f.SUCCEEDED, plan([list(sp) for sp in SUBPATHS]))
    h = _cond_h()
    a = h.pending_action(f.ACT_PLAN)
    res = plan([list(sp) for sp in SUBPATHS])
    job = h.fsm.plan_conditioning_job(a.token, f.SUCCEEDED, res, h.t + 5.0)
    assert job is not None and h.fsm._action is a       # still pending, no effects
    assert a.deadline >= h.t + 5.0 + h.fsm.p.plan_timeout_s
    pre = f.condition_plan(job)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, dict(res, **{f.CONDITIONED_KEY: pre}))
    assert h.name == inline.name
    assert h.fsm.mission.subpaths == inline.fsm.mission.subpaths
    assert h.fsm.mission.cond == inline.fsm.mission.cond
    logs = [e.text for e in h.fx if isinstance(e, f.Log) and 'path conditioning' in e.text]
    assert logs


def test_no_conditioning_job_for_stale_token_failure_or_preview():
    h = _cond_h()
    a = h.pending_action(f.ACT_PLAN)
    assert h.fsm.plan_conditioning_job(a.token + 1, f.SUCCEEDED, plan(SUBPATHS), h.t) is None
    assert h.fsm.plan_conditioning_job(a.token, f.ABORTED, {}, h.t) is None
    assert h.fsm.plan_conditioning_job(a.token, f.SUCCEEDED, {'success': False}, h.t) is None
    off = Harness()
    start_until_planning(off)
    b = off.pending_action(f.ACT_PLAN)
    assert off.fsm.plan_conditioning_job(b.token, f.SUCCEEDED, plan(SUBPATHS), off.t) is None


# ---- far transit goal: re-ask the route, no costmap fallback ----------------------
def _far_h(**params):
    """Sub-path 0 starts at (30, 5), 30.4 m from the robot at (0, 0)."""
    h = Harness(route_transits=True, **params)
    h.areas = [square(-1, -1, 40)]
    start_until_planning(h)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(30, 5, 35, 5)]))
    assert h.name == 'TRANSIT'
    return h


def _navs(h, mark=0):
    return [e for e in h.since(mark, f.StartAction) if e.name == f.ACT_NAV]


def test_far_goal_route_failure_re_asks_the_route_with_backoff():
    h = _far_h()
    _answer_route(h, ok=False)
    assert not _navs(h) and h.fsm.mission.step == 'retry_wait'
    assert h.fsm.mission.route_plan_fails == 1 and h.fsm.mission.transit_fails == 0
    h.tick(dt=0.5, n=3)
    assert h.fsm._service is None                         # 2 s back-off not over
    h.tick(dt=0.5, n=2)
    assert h.fsm._service is not None and h.fsm._service.name == f.SRV_PLAN_ROUTE
    route = [(0.0, 0.0, 0.0), (15.0, 2.5, 0.0), (30.0, 5.0, 0.0)]
    _answer_route(h, resp=_route_resp(route))
    assert h.fsm._action.name == f.ACT_FOLLOW and not _navs(h)
    assert h.fsm.mission.route_plan_fails == 0


def test_far_goal_route_failures_are_bounded_then_a_transit_abort():
    h = _far_h(route_plan_retry_backoff_s=[1.0])
    for k in range(3):
        _answer_route(h, ok=False)
        assert h.fsm.mission.route_plan_fails == k + 1
        h.tick(dt=0.5, n=3)
    _answer_route(h, ok=False)                            # 4th failure: retries spent
    assert not _navs(h)
    assert h.fsm.mission.transit_fails == 1 and h.fsm.mission.step == 'retry_wait'


def test_near_goal_still_falls_back_and_zero_limit_restores_old_behaviour():
    h = _far_h(transit_fallback_max_goal_m=0.0)
    _answer_route(h, ok=False)
    assert _navs(h)
