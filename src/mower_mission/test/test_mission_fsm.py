"""Mission state machine tests (no ROS).

Every effect list produced by the FSM goes through :meth:`Harness._apply`,
which simulates the blade and asserts the blade invariants on every step:

* the blade is never commanded on while a transit / dock / undock / backup
  action is started;
* whenever a status is published while the blade is on, the state is
  MOWING (2, while following) or MANUAL_MOWING (4).
"""

import json
import math
import random

import pytest

from mower_mission import mission_fsm as f
from mower_mission.resume import ResumeCursor

MOVING_ACTIONS = (f.ACT_NAV, f.ACT_DOCK, f.ACT_UNDOCK, f.ACT_BACKUP)


def square(x0, y0, size):
    return {'name': 'a', 'outer': [(x0, y0), (x0 + size, y0), (x0 + size, y0 + size),
                                   (x0, y0 + size)], 'obstacles': [],
            'is_navigation_area': False}


def line(x0, y0, x1, y1, n=11):
    return [(x0 + (x1 - x0) * k / (n - 1), y0 + (y1 - y0) * k / (n - 1), 0.0) for k in range(n)]


class Harness:
    def __init__(self, cursor=None, **params):
        self.t = 0.0
        self.fsm = f.MissionFSM(f.Params.from_dict(params), cursor=cursor, now=0.0)
        i = self.fsm.inputs
        i.emergency_stamp = 0.0
        i.battery_percent = 80.0
        i.fix_type = 3
        i.pose = (0.0, 0.0, 0.0)
        self.heading('cog')
        self.blade = False
        self.fx = []
        self.silent = False
        self.areas = [square(0, 0, 5)]
        self.saved = None
        # get_area_settings / coverage set_parameters are answered automatically
        # (area index -> stored settings; None = service unavailable).
        self.auto_settings = True
        self.area_settings = {}
        self.coverage_params_ok = True
        self._answering = False
        self._apply(self.fsm.initial_effects(0.0))
        self.tick()

    # ---- effect sink with invariant checks ---------------------------
    def _apply(self, fx):
        for e in fx:
            self.fx.append(e)
            if isinstance(e, f.BladeOn):
                self.blade = True
                assert f.blade_allowed(self.fsm) or self.fsm.phase in ('MOWING', 'MANUAL_MOWING')
            elif isinstance(e, f.BladeOff):
                self.blade = False
            elif isinstance(e, f.StartAction) and e.name in MOVING_ACTIONS:
                assert not self.blade, 'blade on while starting %s' % e.name
            elif isinstance(e, f.PublishStatus) and self.blade:
                assert e.status['state_name'] in ('MOWING', 'MANUAL_MOWING'), e.status
            elif isinstance(e, f.SaveResume):
                self.saved = e.text
            elif isinstance(e, f.Log):
                assert not e.text.startswith('safety:'), e.text
        if self.blade:
            assert f.blade_allowed(self.fsm), (self.fsm.phase, self.fsm.mission)
        if self.auto_settings and not self._answering:
            self._answering = True
            try:
                self.answer_settings()
            finally:
                self._answering = False
        return fx

    def _request_of(self, token):
        for e in reversed(self.fx):
            if isinstance(e, f.CallService) and e.token == token:
                return e
        raise AssertionError('request not found')

    def answer_settings(self):
        """Answer a pending get_area_settings / coverage set_parameters call."""
        for _ in range(10):
            s = self.fsm._service
            if s is None or s.name not in (f.SRV_GET_AREA_SETTINGS, f.SRV_SET_PARAMS):
                return
            req = self._request_of(s.token)
            if req.name == f.SRV_GET_AREA_SETTINGS:
                st = self.area_settings.get(req.request['index'], {})
                resp = (False, {}) if st is None else \
                    (True, {'success': True, 'settings_json': json.dumps(st)})
            else:
                resp = (self.coverage_params_ok, {})
            self._apply(self.fsm.on_service_result(s.token, *resp, now=self.t))

    def heading(self, source, fresh=True):
        """heading_aligner status: source none|file|dock|cog; kept fresh by tick()."""
        i = self.fsm.inputs
        i.heading_source = source
        i.heading_aligned = source in ('dock', 'cog', 'file_verified')
        self.heading_fresh = fresh
        i.heading_stamp = self.t if fresh else None

    # ---- drivers -------------------------------------------------------
    def tick(self, dt=0.1, n=1):
        for _ in range(n):
            self.t += dt
            if self.heading_fresh:
                self.fsm.inputs.heading_stamp = self.t
            if not self.silent:
                self.fsm.inputs.emergency_stamp = self.t
            self._apply(self.fsm.tick(self.t))

    def cmd(self, c):
        ok, fx = self.fsm.command(c, self.t)
        self._apply(fx)
        return ok

    def mark(self):
        return len(self.fx)

    def since(self, mark, kind=None):
        return [e for e in self.fx[mark:] if kind is None or isinstance(e, kind)]

    def pending_action(self, name=None):
        a = self.fsm._action
        assert a is not None, 'no pending action'
        if name is not None:
            assert a.name == name, a.name
        return a

    def goal(self, name):
        a = self.pending_action(name)
        for e in reversed(self.fx):
            if isinstance(e, f.StartAction) and e.token == a.token:
                return e.goal
        raise AssertionError('goal not found')

    def finish(self, name, outcome=f.SUCCEEDED, result=None, arrive=True):
        a = self.pending_action(name)
        if arrive and outcome == f.SUCCEEDED and name == f.ACT_FOLLOW:
            for p in self.goal(name)['poses']:       # drive the path to its end
                self.fsm.inputs.pose = (p[0], p[1], 0.0)
                self.tick(dt=0.01)
        self._apply(self.fsm.on_action_result(a.token, outcome, result or {}, self.t))

    def answer_services(self):
        """Answer pending get_mowing_area / add_area calls from ``self.areas``."""
        for _ in range(50):
            s = self.fsm._service
            if s is None:
                return
            req = None
            for e in reversed(self.fx):
                if isinstance(e, f.CallService) and e.token == s.token:
                    req = e
                    break
            if req.name in (f.SRV_GET_AREA_SETTINGS, f.SRV_SET_PARAMS):
                self.answer_settings()
                continue
            if req.name == f.SRV_GET_AREA:
                idx = req.request['index']
                if idx < len(self.areas):
                    resp = (True, {'success': True, 'area': self.areas[idx]})
                else:
                    resp = (True, {'success': False})
            else:
                self.add_area_req = req.request
                resp = (True, {'success': True})
            self._apply(self.fsm.on_service_result(s.token, *resp, now=self.t))

    @property
    def name(self):
        return self.fsm.phase

    def statuses(self, mark=0):
        return [e.status['state_name'] for e in self.since(mark, f.PublishStatus)]


def plan(subpaths):
    return {'success': True, 'drivable_subpaths': subpaths,
            'full_path': [p for sp in subpaths for p in sp]}


def start_until_planning(h):
    assert h.cmd(f.CMD_START)
    h.answer_services()
    assert h.name == 'WAITING_FOR_RTK'
    h.tick()
    assert h.name == 'PLANNING'


def follow_current(h, cutting=True):
    """Blade spin-up confirmed, then follow_path succeeds."""
    h.fsm.inputs.is_cutting = cutting
    h.tick()
    h.finish(f.ACT_FOLLOW)


# =====================================================================
# basics / emergency
# =====================================================================
def test_initial_idle():
    h = Harness()
    assert (h.fsm.state, h.name) == (1, 'IDLE')


def test_idle_docked_and_charging_names():
    h = Harness()
    h.fsm.inputs.docked = True
    h.tick()
    assert h.name == 'IDLE_DOCKED'
    h.fsm.inputs.is_charging = True
    h.tick()
    assert h.name == 'CHARGING'


def test_emergency_silence_then_recovers_to_idle():
    h = Harness()
    h.silent = True
    h.tick(n=19)
    assert h.name == 'IDLE'
    m = h.mark()
    h.tick(n=3)
    assert h.name == 'EMERGENCY' and h.fsm.state == 0
    assert h.since(m, f.ZeroBurst) and h.since(m, f.BladeOff) and h.since(m, f.CancelActions)
    h.silent = False
    h.tick()
    assert h.name == 'IDLE'


def test_emergency_refuses_commands_except_stop_and_reset():
    h = Harness()
    h.fsm.inputs.emergency_active = True
    h.tick()
    assert h.name == 'EMERGENCY'
    for c in (f.CMD_START, f.CMD_HOME, f.CMD_RECORD_AREA, f.CMD_MANUAL_MOW):
        assert not h.cmd(c)
    m = h.mark()
    assert h.cmd(f.CMD_STOP)
    assert h.name == 'EMERGENCY'
    assert h.cmd(f.CMD_RESET_EMERGENCY)
    calls = h.since(m, f.CallService)
    assert [c.name for c in calls] == [f.SRV_CLEAR_ESTOP]


def test_lift_from_mower_base_is_an_emergency():
    h = Harness()
    h.fsm.inputs.lift = True
    h.tick()
    assert h.name == 'EMERGENCY' and 'lift' in h.fsm.sub_state


def test_unsupported_commands():
    h = Harness()
    assert not h.cmd(f.CMD_DELETE_MAPS)
    assert not h.cmd(f.CMD_S2)


# =====================================================================
# manual mowing
# =====================================================================
def test_manual_mow_publishes_state_4_before_blade_on():
    h = Harness()
    m = h.mark()
    assert h.cmd(f.CMD_MANUAL_MOW)
    seq = [type(e) for e in h.since(m) if isinstance(e, (f.PublishStatus, f.BladeOn))]
    assert seq == [f.PublishStatus, f.BladeOn]
    assert h.since(m, f.PublishStatus)[0].status['state'] == 4
    assert h.blade


def test_manual_mow_stop_turns_blade_off_first():
    h = Harness()
    h.cmd(f.CMD_MANUAL_MOW)
    m = h.mark()
    assert h.cmd(f.CMD_STOP)
    kinds = [type(e) for e in h.since(m) if isinstance(e, (f.BladeOff, f.PublishStatus))]
    assert kinds[0] is f.BladeOff
    assert h.since(m, f.ZeroBurst)
    assert h.name == 'IDLE' and not h.blade


def test_manual_mow_then_record_blade_off_first():
    h = Harness()
    h.cmd(f.CMD_MANUAL_MOW)
    m = h.mark()
    assert h.cmd(f.CMD_RECORD_AREA)
    assert isinstance(h.since(m, (f.BladeOff, f.PublishStatus))[0], f.BladeOff)
    assert h.name == 'RECORDING' and not h.blade


def test_manual_mow_emergency_blade_off_and_no_return_to_manual():
    h = Harness()
    h.cmd(f.CMD_MANUAL_MOW)
    h.fsm.inputs.emergency_active = True
    h.tick()
    assert h.name == 'EMERGENCY' and not h.blade
    h.fsm.inputs.emergency_active = False
    h.tick()
    assert h.name == 'IDLE' and not h.blade


def test_manual_mow_refused_while_autonomous():
    h = Harness()
    start_until_planning(h)
    assert not h.cmd(f.CMD_MANUAL_MOW)
    assert not h.cmd(f.CMD_RECORD_AREA)


# =====================================================================
# recording
# =====================================================================
def drive_rectangle(h, w=4.0, d=3.0, step=0.1):
    pts = []
    for x in range(int(w / step)):
        pts.append((x * step, 0.0))
    for y in range(int(d / step)):
        pts.append((w, y * step))
    for x in range(int(w / step), 0, -1):
        pts.append((x * step, d))
    for y in range(int(d / step), 0, -1):
        pts.append((0.0, y * step))
    for p in pts:
        h.fsm.inputs.pose = (p[0], p[1], 0.0)
        h.tick()


def test_recording_samples_and_drops_close_points():
    h = Harness()
    assert h.cmd(f.CMD_RECORD_AREA)
    assert h.name == 'RECORDING' and h.fsm.state == 3
    for k in range(10):                     # 4 mm steps, < 5 cm in total: dropped
        h.fsm.inputs.pose = (0.004 * k, 0.0, 0.0)
        h.tick()
    assert len(h.fsm._track) == 1
    h.fsm.inputs.pose = (0.2, 0.0, 0.0)
    m = h.mark()
    h.tick()
    assert len(h.fsm._track) == 2
    assert h.since(m, f.PublishTrajectory)[-1].points == [(0.0, 0.0), (0.2, 0.0)]


def test_record_finish_saves_simplified_area():
    h = Harness()
    h.areas = [square(0, 0, 5), dict(square(10, 0, 2), is_navigation_area=True)]
    h.cmd(f.CMD_RECORD_AREA)
    drive_rectangle(h)
    assert h.cmd(f.CMD_RECORD_FINISH)
    h.answer_services()
    req = h.add_area_req
    assert req['name'] == 'Area 2' and req['is_navigation_area'] is False
    assert len(req['polygon']) == 4
    assert h.name == 'RECORDING_COMPLETE' and h.fsm.state == 1
    h.tick(dt=1.0, n=6)
    assert h.name == 'IDLE'


def test_record_finish_too_small_is_rejected():
    h = Harness()
    h.cmd(f.CMD_RECORD_AREA)
    drive_rectangle(h, w=0.5, d=0.5)
    assert not h.cmd(f.CMD_RECORD_FINISH)
    assert h.name == 'IDLE' and 'rejected' in h.fsm.sub_state


def test_record_cancel_discards():
    h = Harness()
    h.cmd(f.CMD_RECORD_AREA)
    drive_rectangle(h)
    m = h.mark()
    assert h.cmd(f.CMD_RECORD_CANCEL)
    assert h.name == 'IDLE' and not h.since(m, f.CallService)
    assert h.fsm._track == []


def test_record_add_area_failure_keeps_polygon_in_fallback_file():
    h = Harness()
    h.cmd(f.CMD_RECORD_AREA)
    drive_rectangle(h)
    h.cmd(f.CMD_RECORD_FINISH)
    for _ in range(3):                      # get_mowing_area + add_area both unavailable
        s = h.fsm._service
        if s is None:
            break
        h._apply(h.fsm.on_service_result(s.token, False, {}, h.t))
    fb = h.since(0, f.SaveRecordingFallback)
    assert fb and fb[0].name == 'Area 1' and len(fb[0].points) == 4
    assert h.name == 'IDLE'


def test_record_finish_or_cancel_without_recording_refused():
    h = Harness()
    assert not h.cmd(f.CMD_RECORD_FINISH)
    assert not h.cmd(f.CMD_RECORD_CANCEL)


# =====================================================================
# start / preflight
# =====================================================================
@pytest.mark.parametrize('setup, reason', [
    (dict(battery_percent=15.0), 'battery'),
    (dict(battery_percent=None), 'battery level unknown'),
    (dict(docked=True, fix_type=0), 'no GNSS fix'),
    (dict(rain=True), 'rain'),
])
def test_preflight_failures(setup, reason):
    h = Harness()
    for k, v in setup.items():
        setattr(h.fsm.inputs, k, v)
    m = h.mark()
    assert not h.cmd(f.CMD_START)
    assert 'PREFLIGHT_CHECK' in h.statuses(m)
    assert h.name in ('IDLE', 'IDLE_DOCKED') and reason in h.fsm.sub_state
    assert h.fsm.mission is None


def test_start_fails_cleanly_without_map_server():
    h = Harness()
    assert h.cmd(f.CMD_START)
    s = h.fsm._service
    h._apply(h.fsm.on_service_result(s.token, False, {}, h.t))
    assert h.name == 'IDLE' and 'get_mowing_area unavailable' in h.fsm.sub_state
    assert h.fsm.mission is None and not h.since(0, f.StartAction)


def test_start_with_no_areas_fails():
    h = Harness()
    h.areas = []
    h.cmd(f.CMD_START)
    h.answer_services()
    assert h.name == 'IDLE' and 'no mowing areas' in h.fsm.sub_state


def test_start_docked_undocks_and_fails_cleanly_without_server():
    h = Harness()
    h.fsm.inputs.docked = True
    h.tick()
    h.cmd(f.CMD_START)
    h.answer_services()
    assert h.name == 'UNDOCKING'
    g = h.goal(f.ACT_UNDOCK)
    assert g['distance_m'] == 0.8 and g['wait_for_rtk'] is False
    h.finish(f.ACT_UNDOCK, f.UNAVAILABLE)
    assert h.name == 'UNDOCK_FAILED' and 'undock action server unavailable' in h.fsm.sub_state
    h.tick(dt=1.0, n=6)
    assert h.name == 'IDLE_DOCKED' and h.fsm.mission is None


def test_undock_then_rtk_then_planning_skips_navigation_areas():
    h = Harness()
    h.areas = [dict(square(20, 0, 2), is_navigation_area=True), square(0, 0, 5)]
    h.fsm.inputs.docked = True
    h.fsm.inputs.fix_type = 2
    h.tick()
    h.cmd(f.CMD_START)
    h.answer_services()
    h.fsm.inputs.docked = False
    h.finish(f.ACT_UNDOCK)
    assert h.name == 'WAITING_FOR_RTK'
    h.tick(n=5)
    assert h.name == 'WAITING_FOR_RTK'
    h.fsm.inputs.fix_type = 3
    h.tick()
    assert h.name == 'PLANNING' and h.fsm.mission.area_idx == 1
    assert h.goal(f.ACT_PLAN)['outer_boundary'] == square(0, 0, 5)['outer']


def test_rtk_timeout_aborts_and_docks():
    h = Harness(rtk_timeout_s=5.0)
    h.fsm.inputs.fix_type = 2
    h.cmd(f.CMD_START)
    h.answer_services()
    h.tick(dt=1.0, n=6)
    assert h.name == 'COVERAGE_FAILED_DOCKING'
    assert h.goal(f.ACT_DOCK)['use_vision'] is True


def test_plan_server_absent_aborts_then_dock_absent():
    h = Harness()
    start_until_planning(h)
    h.finish(f.ACT_PLAN, f.UNAVAILABLE)
    assert h.name == 'COVERAGE_FAILED_DOCKING' and 'plan_coverage' in h.fsm.sub_state
    h.finish(f.ACT_DOCK, f.UNAVAILABLE)
    assert h.name == 'NAV_TO_DOCK_FAILED'
    h.tick(dt=1.0, n=6)
    assert h.name == 'IDLE' and h.fsm.mission is None


def test_planning_failure_skips_area():
    h = Harness()
    h.areas = [square(0, 0, 5), square(10, 0, 5)]
    start_until_planning(h)
    m = h.mark()
    h.finish(f.ACT_PLAN, f.SUCCEEDED, {'success': False, 'message': 'degenerate polygon'})
    assert 'AREA_UNREACHABLE' in h.statuses(m)
    assert h.name == 'PLANNING' and h.fsm.mission.area_idx == 1


# =====================================================================
# mowing
# =====================================================================
def test_full_mow_happy_path_with_transit():
    h = Harness()
    h.areas = [square(0, 0, 5), square(10, 0, 5)]
    start_until_planning(h)
    sp0 = line(0.2, 0, 4, 0)                 # starts 0.2 m away: no transit
    sp1 = line(4, 2, 0, 2)                   # 2 m away: transit
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([sp0, sp1]))
    assert h.since(0, f.PublishPlan)[-1].poses[0] == sp0[0]
    assert h.name == 'MOWING' and h.blade
    h.fsm.inputs.is_cutting = True
    h.tick()
    g = h.goal(f.ACT_FOLLOW)
    assert g['controller_id'] == 'FollowCoveragePath'
    assert g['goal_checker_id'] == 'coverage_goal_checker'
    assert g['poses'] == sp0
    h.fsm.inputs.pose = (4, 0, 0)
    m = h.mark()
    h.finish(f.ACT_FOLLOW)
    assert h.name == 'TRANSIT' and not h.blade
    assert isinstance(h.since(m, (f.BladeOff, f.StartAction))[0], f.BladeOff)
    st = h.fsm.status()
    assert st['completed_swaths'] == 1 and st['total_swaths'] == 2
    assert 40 < st['coverage_percent'] < 60
    assert h.goal(f.ACT_NAV)['pose'][:2] == (4, 2)
    h.fsm.inputs.pose = (4, 2, 0)
    h.finish(f.ACT_NAV)
    assert h.name == 'MOWING'
    follow_current(h)
    # second area
    assert h.name == 'PLANNING' and h.fsm.mission.area_idx == 1
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(4, 2.3, 10, 2.3)]))
    assert h.name == 'TRANSIT' and not h.blade    # robot ended area 0 at (0, 2)
    h.finish(f.ACT_NAV)
    follow_current(h)
    m = h.mark()
    assert h.name == 'RETURNING_HOME'
    assert 'MOWING_COMPLETE' in h.statuses(m - 10)
    assert not h.blade and not h.fsm.cursor.available
    h.fsm.inputs.docked = True
    h.fsm.inputs.is_charging = True
    h.finish(f.ACT_DOCK)
    assert h.name == 'CHARGING' and h.fsm.state == 1


def test_close_subpaths_keep_blade_on_without_new_spinup():
    h = Harness()
    start_until_planning(h)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(0, 0, 4, 0), line(4, 0.3, 0, 0.3)]))
    follow_current(h)
    h.fsm.inputs.pose = (4, 0, 0)
    # first finish happened with pose (0,0): next start (4, 0.3) is far -> transit.
    assert h.name in ('TRANSIT', 'MOWING')
    if h.name == 'TRANSIT':
        h.finish(f.ACT_NAV)
        h.tick()
    assert h.fsm._action.name == f.ACT_FOLLOW


def test_adjacent_subpath_no_transit():
    h = Harness()
    start_until_planning(h)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(0, 0, 4, 0), line(4, 0.3, 0, 0.3)]))
    h.fsm.inputs.is_cutting = True
    h.tick()
    h.fsm.inputs.pose = (4.0, 0.0, 0.0)
    m = h.mark()
    h.finish(f.ACT_FOLLOW)
    assert not h.since(m, f.BladeOff) and not h.since(m, f.BladeOn)
    assert h.pending_action(f.ACT_FOLLOW) and h.blade


def test_blade_start_retries_then_aborts():
    h = Harness(blade_confirm_timeout_s=5.0, blade_retry_pause_s=1.0)
    start_until_planning(h)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(0, 0, 4, 0)]))
    m = h.mark()
    h.tick(dt=0.5, n=40)
    assert len(h.since(m, f.BladeOn)) == 2       # 1 at plan + 2 retries = 3 total
    assert len(h.since(0, f.BladeOn)) == 3
    assert h.name == 'COVERAGE_FAILED_DOCKING' and 'blade did not start' in h.fsm.sub_state
    assert not h.blade


def test_blade_confirmed_on_second_attempt():
    h = Harness(blade_confirm_timeout_s=5.0, blade_retry_pause_s=1.0)
    start_until_planning(h)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(0, 0, 4, 0)]))
    h.tick(dt=0.5, n=13)                     # first attempt failed, pause, second started
    assert len(h.since(0, f.BladeOn)) == 2 and h.fsm._action is None
    h.fsm.inputs.is_cutting = True
    h.tick()
    assert h.pending_action(f.ACT_FOLLOW)


def test_follow_abort_retries_from_nearest_then_fails_subpath():
    h = Harness(follow_retries=1, transit_retry_delay_s=3.0)
    start_until_planning(h)
    sp0, sp1 = line(0, 0, 5, 0, n=11), line(5, 0.3, 0, 0.3)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([sp0, sp1]))
    h.fsm.inputs.is_cutting = True
    h.tick()
    h.fsm.inputs.pose = (2.1, 0.1, 0.0)
    h.finish(f.ACT_FOLLOW, f.ABORTED)
    assert h.name == 'MOWING' and not h.blade and h.fsm._action is None
    assert 'retry 1/1' in h.fsm.sub_state and 'follow_path aborted' in h.fsm.sub_state
    h.tick(dt=1.0, n=3)
    h.tick()                                 # blade spin-up confirmed -> follow
    g = h.goal(f.ACT_FOLLOW)                 # within 0.6 m: no transit
    assert g['poses'][0] == sp0[4]
    h.finish(f.ACT_FOLLOW, f.ABORTED)
    assert h.fsm.mission.skipped == 1 and h.fsm.mission.sub_i == 1
    assert h.fsm.status()['skipped_swaths'] == 1
    assert 0 not in h.fsm.cursor.areas[0].completed


def test_progress_tracking_never_jumps_to_adjacent_lap():
    h = Harness()
    start_until_planning(h)
    lap1 = line(0, 0, 4, 0, n=41)
    lap2 = line(4, 0.15, 0, 0.15, n=41)
    sp = lap1 + lap2                          # one sub-path, two adjacent laps
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([sp, line(0, 3, 4, 3)]))
    h.fsm.inputs.is_cutting = True
    h.tick()
    for x in (0.5, 1.0, 1.5):                 # robot drives lap 1 slightly towards lap 2
        h.fsm.inputs.pose = (x, 0.1, 0.0)
        h.tick()
    assert h.fsm.mission.progress_local == 15
    st = h.fsm.status()
    assert st['current_path_index'] == 15 and 10 < st['coverage_percent'] < 20
    h.cmd(f.CMD_STOP)
    assert ResumeCursor.loads(h.saved).areas[0].resume_index == 15


def test_mowing_complete_reports_final_counts():
    h = Harness()
    start_until_planning(h)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(0, 0, 4, 0)]))
    m = h.mark()
    follow_current(h)
    done = [e.status for e in h.since(m, f.PublishStatus)
            if e.status['state_name'] == 'MOWING_COMPLETE'][0]
    assert done['completed_swaths'] == 1 and done['coverage_percent'] == 100.0


def test_premature_follow_success_continues_from_progress():
    h = Harness()
    start_until_planning(h)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(0, 0, 5, 0, n=11)]))
    h.fsm.inputs.is_cutting = True
    h.tick()
    h.fsm.inputs.pose = (1.5, 0.0, 0.0)
    h.tick()
    h.finish(f.ACT_FOLLOW, arrive=False)      # goal checker fired at 1.5 m of 5 m
    assert h.name == 'MOWING' and h.fsm.cursor.areas[0].completed == set()
    assert h.goal(f.ACT_FOLLOW)['poses'][0][0] == 1.5
    h.finish(f.ACT_FOLLOW)                    # now really at the end
    assert h.name == 'RETURNING_HOME'


def test_long_subpath_is_followed_in_chunks():
    h = Harness(follow_chunk_m=10.0)
    start_until_planning(h)
    sp = line(0, 0, 30, 0, n=301)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([sp]))
    h.fsm.inputs.is_cutting = True
    h.tick()
    goals = []
    for _ in range(5):
        if h.fsm._action is None or h.fsm._action.name != f.ACT_FOLLOW:
            break
        goals.append(h.goal(f.ACT_FOLLOW)['poses'])
        h.finish(f.ACT_FOLLOW)
    assert len(goals) == 3 and all(geo_len(g) <= 10.0 + 1e-6 for g in goals)
    assert goals[1][0] == goals[0][-1] and goals[-1][-1] == sp[-1]
    assert h.name == 'RETURNING_HOME' and h.fsm.cursor.available is False


def geo_len(poses):
    from mower_mission.geometry import path_length
    return path_length(poses)


def test_premature_success_retries_exhausted_skips_subpath():
    h = Harness(follow_premature_retries=2)
    start_until_planning(h)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(0, 0, 5, 0, n=11), line(0, 3, 5, 3)]))
    h.fsm.inputs.is_cutting = True
    h.tick()
    for _ in range(3):
        h.finish(f.ACT_FOLLOW, arrive=False)
    assert h.fsm.mission.skipped == 1 and h.fsm.mission.sub_i == 1
    assert h.fsm.cursor.areas[0].completed == set()


def test_transit_failure_without_retries_fails_subpath():
    h = Harness(transit_retries=0)
    start_until_planning(h)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(3, 3, 4, 3), line(0, 0, 4, 0)]))
    assert h.name == 'TRANSIT'
    h.finish(f.ACT_NAV, f.ABORTED)
    assert h.fsm.mission.sub_i == 1 and h.fsm.mission.skipped == 1
    assert h.name == 'MOWING'


def test_follow_server_absent_aborts_mission():
    h = Harness()
    start_until_planning(h)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(0, 0, 4, 0)]))
    h.fsm.inputs.is_cutting = True
    h.tick()
    h.finish(f.ACT_FOLLOW, f.UNAVAILABLE)
    assert h.name == 'COVERAGE_FAILED_DOCKING' and not h.blade


def test_action_watchdog_timeout_counts_as_failure():
    h = Harness(follow_timeout_s=10.0)
    start_until_planning(h)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(0, 0, 4, 0)]))
    h.fsm.inputs.is_cutting = True
    h.tick()
    m = h.mark()
    h.tick(dt=1.0, n=12)
    assert h.since(m, f.CancelActions)
    assert h.fsm.mission.follow_fails == 1 and h.fsm.mission.step == 'retry_wait'


def test_stale_results_are_ignored():
    h = Harness()
    start_until_planning(h)
    tok = h.pending_action(f.ACT_PLAN).token
    h.cmd(f.CMD_STOP)
    fx = h.fsm.on_action_result(tok, f.SUCCEEDED, plan([line(0, 0, 1, 0)]), h.t)
    assert fx == [] and h.name == 'IDLE'


# =====================================================================
# stop / resume / home
# =====================================================================
def mow_first_subpath_then_stop(h):
    start_until_planning(h)
    sps = [line(0, 0, 4, 0), line(4, 1, 0, 1), line(0, 2, 4, 2)]
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan(sps))
    follow_current(h)                        # sub-path 0 done
    assert h.name == 'TRANSIT'
    h.fsm.inputs.pose = (4, 1, 0)
    h.finish(f.ACT_NAV)
    h.tick()                                 # following sub-path 1
    h.fsm.inputs.pose = (2.05, 1.0, 0.0)
    m = h.mark()
    assert h.cmd(f.CMD_STOP)
    return sps, m


def test_stop_keeps_resume_cursor():
    h = Harness()
    sps, m = mow_first_subpath_then_stop(h)
    assert h.name == 'IDLE' and not h.blade
    assert h.since(m, f.CancelActions) and h.since(m, f.ZeroBurst)
    assert h.since(m, f.PublishResumeAvailable)[-1].available
    cur = ResumeCursor.loads(h.saved)
    assert cur.current_command == 1 and cur.current_area == 0
    assert cur.areas[0].completed == {0}
    assert cur.areas[0].resume_index == 11 + 5     # sub-path 1, nearest pose 5


def test_start_after_stop_resumes_from_cursor():
    h = Harness()
    sps, _ = mow_first_subpath_then_stop(h)
    start_until_planning(h)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan(sps))
    assert h.fsm.mission.sub_i == 1 and h.fsm.mission.start_local == 5
    h.fsm.inputs.is_cutting = True
    h.tick()
    assert h.goal(f.ACT_FOLLOW)['poses'][0] == sps[1][5]


def test_resume_cursor_survives_restart_via_file():
    h = Harness()
    sps, _ = mow_first_subpath_then_stop(h)
    h2 = Harness(cursor=ResumeCursor.loads(h.saved))
    assert h2.since(0, f.PublishStatus)
    start_until_planning(h2)
    h2.finish(f.ACT_PLAN, f.SUCCEEDED, plan(sps))
    assert (h2.fsm.mission.sub_i, h2.fsm.mission.start_local) == (1, 5)


def test_resume_discarded_when_plan_changes():
    h = Harness()
    sps, _ = mow_first_subpath_then_stop(h)
    start_until_planning(h)
    changed = [line(0, 0, 4.5, 0)] + sps[1:]
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan(changed))
    assert (h.fsm.mission.sub_i, h.fsm.mission.start_local) == (0, 0)
    assert h.fsm.cursor.areas[0].completed == set()


def test_clear_coverage_resume():
    h = Harness()
    mow_first_subpath_then_stop(h)
    ok, msg, fx = h.fsm.clear_resume(h.t)
    h._apply(fx)
    assert ok and any(isinstance(e, f.DeleteResume) for e in fx)
    assert not h.fsm.cursor.available
    start_until_planning(h)
    assert h.fsm.mission.resume is False


def test_clear_coverage_resume_refused_while_mowing():
    h = Harness()
    start_until_planning(h)
    ok, _msg, _fx = h.fsm.clear_resume(h.t)
    assert not ok


def test_home_mid_mission_docks_and_keeps_cursor():
    h = Harness()
    start_until_planning(h)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(0, 0, 4, 0), line(0, 2, 4, 2)]))
    h.fsm.inputs.is_cutting = True
    h.tick()
    m = h.mark()
    assert h.cmd(f.CMD_HOME)
    assert h.name == 'RETURNING_HOME' and not h.blade
    assert isinstance(h.since(m, (f.BladeOff, f.StartAction))[0], f.BladeOff)
    assert h.pending_action(f.ACT_DOCK)
    assert h.fsm.cursor.available
    h.fsm.inputs.docked = True
    h.finish(f.ACT_DOCK)
    assert h.name == 'IDLE_DOCKED'


def test_home_when_already_docked():
    h = Harness()
    h.fsm.inputs.docked = True
    h.tick()
    assert h.cmd(f.CMD_HOME)
    assert h.name == 'IDLE_DOCKED' and h.fsm._action is None


def test_emergency_mid_mission_not_auto_resumed():
    h = Harness()
    start_until_planning(h)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(0, 0, 4, 0)]))
    h.fsm.inputs.is_cutting = True
    h.tick()
    m = h.mark()
    h.fsm.inputs.emergency_active = True
    h.tick()
    assert h.name == 'EMERGENCY' and not h.blade
    assert h.since(m, f.CancelActions) and h.since(m, f.ZeroBurst)
    h.fsm.inputs.emergency_active = False
    h.tick(n=5)
    assert h.name == 'IDLE' and h.fsm.mission is None and h.fsm.cursor.available


def test_start_in_area_single_target():
    h = Harness()
    h.areas = [square(0, 0, 5), square(10, 0, 5), square(20, 0, 5)]
    ok, fx = h.fsm.start_in_area(2, h.t)
    h._apply(fx)
    assert ok
    h.answer_services()
    h.tick()
    assert h.fsm.mission.area_idx == 2 and h.fsm.mission.queue == []
    ok, fx = h.fsm.start_in_area(7, h.t)
    assert not ok


def test_start_in_area_missing_area_fails():
    h = Harness()
    ok, fx = h.fsm.start_in_area(5, h.t)
    h._apply(fx)
    h.answer_services()
    assert h.name == 'IDLE' and 'area 5 does not exist' in h.fsm.sub_state


# =====================================================================
# rain / battery / boundary guards
# =====================================================================
def mowing(h, **kw):
    start_until_planning(h)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(0, 0, 4, 0), line(0, 2, 4, 2)]))
    h.fsm.inputs.is_cutting = True
    h.tick()
    assert h.pending_action(f.ACT_FOLLOW)


def test_rain_docks_waits_and_resumes():
    h = Harness(rain_delay_minutes=1.0, rain_debounce_s=1.0)
    mowing(h)
    h.fsm.inputs.rain = True
    h.tick(n=5)
    assert h.name == 'MOWING'                # debounce
    h.tick(n=6)
    assert h.name == 'RAIN_DETECTED_DOCKING' and not h.blade
    h.fsm.inputs.docked = True
    h.finish(f.ACT_DOCK)
    assert h.name == 'RAIN_WAITING' and h.fsm.state == 1
    h.tick(dt=10.0, n=10)                    # still raining
    assert h.name == 'RAIN_WAITING'
    h.fsm.inputs.rain = False
    h.tick(dt=10.0, n=3)
    h.fsm.inputs.rain = True                 # rain again resets the timer
    h.tick()
    h.fsm.inputs.rain = False
    h.tick(dt=10.0, n=5)
    assert h.name == 'RAIN_WAITING'
    h.tick(dt=10.0, n=3)
    assert h.name == 'PREFLIGHT_CHECK'       # resumed: enumerating areas
    h.answer_services()
    assert h.name == 'UNDOCKING'


def test_rain_mode_0_ignores_rain():
    h = Harness(rain_mode=0)
    mowing(h)
    h.fsm.inputs.rain = True
    h.tick(n=50)
    assert h.name == 'MOWING'


def test_low_battery_docks_charges_and_resumes():
    h = Harness(battery_low_action='dock')
    mowing(h)
    h.fsm.inputs.battery_percent = 19.0
    h.tick()
    assert h.name == 'LOW_BATTERY_DOCKING' and not h.blade
    h.fsm.inputs.docked = True
    h.fsm.inputs.is_charging = True
    h.finish(f.ACT_DOCK)
    assert h.name == 'CHARGING'
    h.fsm.inputs.battery_percent = 90.0
    h.tick(n=10)
    assert h.name == 'CHARGING'
    h.fsm.inputs.battery_percent = 96.0
    h.tick()
    h.answer_services()
    assert h.name == 'UNDOCKING'


def test_stop_cancels_charge_resume():
    h = Harness(battery_low_action='dock')
    mowing(h)
    h.fsm.inputs.battery_percent = 10.0
    h.tick()
    h.fsm.inputs.docked = True
    h.finish(f.ACT_DOCK)
    h.cmd(f.CMD_STOP)
    h.fsm.inputs.battery_percent = 100.0
    h.tick(n=5)
    assert h.name == 'IDLE_DOCKED' and h.fsm.mission is None


def test_lethal_boundary_violation_latches():
    h = Harness()
    mowing(h)
    h.fsm.inputs.lethal_boundary_violation = True
    h.tick()
    assert h.name == 'BOUNDARY_EMERGENCY_STOP' and h.fsm.state == 0 and not h.blade
    h.fsm.inputs.lethal_boundary_violation = False
    h.tick(n=5)
    assert h.name == 'BOUNDARY_EMERGENCY_STOP'
    assert not h.cmd(f.CMD_START)
    assert h.cmd(f.CMD_STOP)
    assert h.name == 'IDLE'


def test_boundary_violation_pauses_and_resumes():
    h = Harness()
    mowing(h)
    h.fsm.inputs.boundary_violation = True
    m = h.mark()
    h.tick()
    assert h.name == 'BOUNDARY_PAUSED' and not h.blade
    assert h.since(m, f.CancelActions) and h.since(m, f.ZeroBurst)
    h.fsm.inputs.boundary_violation = False
    h.tick()
    assert h.name == 'MOWING' and h.blade      # spin-up again, then follow
    h.tick()
    assert h.pending_action(f.ACT_FOLLOW)


def test_boundary_violation_recovery_transits_then_latch():
    h = Harness(boundary_recover_after_s=3.0, boundary_max_recoveries=2)
    mowing(h)                                # sub-path 0: (0,0) -> (4,0)
    h.fsm.inputs.pose = (1.0, -0.2, 0.0)
    h.fsm.inputs.boundary_violation = True
    h.tick()
    assert h.name == 'BOUNDARY_PAUSED'
    h.tick(dt=1.0, n=3)
    assert h.name == 'TRANSIT' and not h.blade
    target = h.goal(f.ACT_NAV)['pose']
    assert target[0] >= 1.0 + 0.95 - 1e-9    # >= 1 m ahead of the nearest pose (1.2, 0)
    h.tick(n=5)
    assert h.name == 'TRANSIT'               # soft violation does not stop a blade-off transit
    h.finish(f.ACT_NAV)                      # arrived, but still flagged outside
    assert h.name == 'BOUNDARY_PAUSED' and not h.blade
    h.tick(dt=1.0, n=3)
    h.finish(f.ACT_NAV)
    assert h.name == 'BOUNDARY_PAUSED'
    h.tick(dt=1.0, n=3)
    assert h.name == 'BOUNDARY_EMERGENCY_STOP' and h.fsm.state == 0


def test_boundary_recovery_transit_success_resumes_mowing():
    h = Harness()
    mowing(h)
    h.fsm.inputs.pose = (1.0, -0.2, 0.0)
    h.fsm.inputs.boundary_violation = True
    h.tick()
    h.tick(dt=1.0, n=3)
    h.fsm.inputs.boundary_violation = False
    h.fsm.inputs.pose = (2.0, 0.0, 0.0)
    h.finish(f.ACT_NAV)
    assert h.name == 'MOWING' and h.blade
    h.tick()
    assert h.goal(f.ACT_FOLLOW)['poses'][0][0] >= 2.0 - 1e-9


# =====================================================================
# misc
# =====================================================================
def test_shutdown_turns_blade_off():
    h = Harness()
    h.cmd(f.CMD_MANUAL_MOW)
    fx = h._apply(h.fsm.shutdown(h.t))
    kinds = {type(e) for e in fx}
    assert {f.BladeOff, f.CancelActions, f.ZeroBurst} <= kinds and not h.blade


def test_without_docking_server_undock_uses_backup_and_charging():
    h = Harness(use_docking_server=False)
    h.fsm.inputs.docked = True
    h.tick()
    h.cmd(f.CMD_START)
    m = h.mark()
    h.answer_services()
    calls = [c for c in h.since(m, f.CallService) if c.name == f.SRV_CHARGING]
    assert calls and calls[0].request == {'enable': False}
    assert h.goal(f.ACT_BACKUP)['distance_m'] == 0.8


def test_start_is_idempotent_while_running():
    h = Harness()
    start_until_planning(h)
    tok = h.pending_action(f.ACT_PLAN).token
    assert h.cmd(f.CMD_START)
    assert h.pending_action(f.ACT_PLAN).token == tok


def test_random_sequences_keep_blade_invariant():
    rng = random.Random(1234)
    cmds = [f.CMD_START, f.CMD_HOME, f.CMD_RECORD_AREA, f.CMD_RECORD_FINISH,
            f.CMD_RECORD_CANCEL, f.CMD_MANUAL_MOW, f.CMD_STOP, f.CMD_RESET_EMERGENCY]
    outcomes = [f.SUCCEEDED] * 4 + [f.ABORTED, f.UNAVAILABLE, f.REJECTED]
    for _run in range(30):
        h = Harness(rain_debounce_s=0.0, rain_delay_minutes=0.01)
        h.areas = [square(0, 0, 5), square(10, 0, 5)]
        for _step in range(300):
            r = rng.random()
            i = h.fsm.inputs
            if r < 0.08:
                h.cmd(rng.choice(cmds))
            elif r < 0.12:
                i.emergency_active = rng.random() < 0.3
            elif r < 0.15:
                i.rain = rng.random() < 0.3
            elif r < 0.18:
                i.battery_percent = rng.choice([10.0, 50.0, 99.0])
            elif r < 0.21:
                i.boundary_violation = rng.random() < 0.3
                i.lethal_boundary_violation = rng.random() < 0.05
            elif r < 0.25:
                i.is_cutting = rng.random() < 0.8
                i.docked = rng.random() < 0.2
            elif r < 0.30:
                i.pose = (rng.uniform(0, 5), rng.uniform(0, 5), 0.0)
            elif r < 0.45 and h.fsm._action is not None:
                a = h.fsm._action
                res = {}
                if a.name == f.ACT_PLAN:
                    res = plan([line(0, 0, 4, 0), line(0, 2, 4, 2)])
                h._apply(h.fsm.on_action_result(a.token, rng.choice(outcomes), res, h.t))
            elif r < 0.55:
                h.answer_services()
            h.tick(dt=rng.choice([0.1, 1.0, 10.0]))


# =====================================================================
# per-area mowing settings
# =====================================================================
def calls(h, mark=0, name=None):
    return [e for e in h.since(mark, f.CallService) if name is None or e.name == name]


def param_calls(h, node, mark=0):
    return [e.request['params'] for e in calls(h, mark, f.SRV_SET_PARAMS)
            if e.request['node'] == node]


def mow_area_once(h, sp=None):
    """PLANNING -> plan with one sub-path from the robot pose -> mowed."""
    sp = sp or line(0, 0, 4, 0)
    h.fsm.inputs.pose = (sp[0][0], sp[0][1], 0.0)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([sp]))
    assert h.name == 'MOWING'
    follow_current(h)


def test_height_percent_table():
    fsm = f.MissionFSM()
    assert [fsm.height_percent(mm) for mm in (20, 30, 50, 60, 90, 120)] == [0, 0, 33, 50, 100, 100]
    fsm = f.MissionFSM({'cutter_height_mm_to_percent': [30.0, 10.0, 60.0, 40.0, 90.0, 100.0]})
    assert [fsm.height_percent(mm) for mm in (30, 45, 60, 75, 90)] == [10, 25, 40, 70, 100]


def test_area_settings_applied_before_planning():
    h = Harness()
    h.area_settings = {0: {'path_mode': 'spiral', 'perimeter_laps': 3, 'cut_speed_mps': 0.45,
                           'cutter_height_mm': 60, 'swath_overlap_m': 0.04,
                           'edge_first': False, 'mow_angle_deg': 30.0}}
    m = h.mark()
    start_until_planning(h)
    assert [c.request for c in calls(h, m, f.SRV_GET_AREA_SETTINGS)] == [{'index': 0}]
    assert [c.request for c in calls(h, m, f.SRV_CUTTER_HEIGHT)] == \
        [{'height_mm': 60, 'percent': 50}]
    assert param_calls(h, f.PARAM_NODE_CONTROLLER, m) == \
        [{'FollowCoveragePath.desired_linear_vel': 0.45}]
    cov = param_calls(h, f.PARAM_NODE_COVERAGE, m)
    assert cov == [{'operation_width': pytest.approx(0.16), 'headland_rings': 3,
                    'path_mode': 'spiral', 'mow_angle_deg': 30.0, 'edge_first': False}]
    goal = h.goal(f.ACT_PLAN)
    assert goal['mow_angle_deg'] == 30.0 and goal['perpendicular'] is False
    # order: settings -> height -> coverage params -> plan goal, all blade off
    seq = [type(e).__name__ + ':' + getattr(e, 'name', '') for e in h.since(m)
           if isinstance(e, (f.CallService, f.StartAction, f.BladeOn))
           and getattr(e, 'name', '') != f.SRV_GET_AREA]
    assert seq == ['CallService:get_area_settings', 'CallService:cutter_height',
                   'CallService:set_parameters', 'CallService:set_parameters',
                   'StartAction:plan_coverage']
    pub = h.since(m, f.PublishAreaSettings)[-1].settings
    assert pub['path_mode'] == 'spiral' and pub['area_index'] == 0
    assert pub['run'] == 1 and pub['runs'] == 1 and pub['cutter_height_percent'] == 50
    assert '[spiral, run 1/1]' in h.fsm.sub_state


def test_blade_height_command_precedes_blade_on():
    h = Harness()
    h.area_settings = {0: {'cutter_height_mm': 80}}
    m = h.mark()
    start_until_planning(h)
    mow_area_once(h)
    kinds = [e for e in h.since(m) if isinstance(e, f.BladeOn)
             or (isinstance(e, f.CallService) and e.name == f.SRV_CUTTER_HEIGHT)]
    assert isinstance(kinds[0], f.CallService) and kinds[0].request['percent'] == 83
    assert any(isinstance(e, f.BladeOn) for e in kinds[1:])


def test_cut_speed_is_capped():
    h = Harness(cut_speed_max_mps=0.4)
    h.area_settings = {0: {'cut_speed_mps': 0.5}}
    m = h.mark()
    start_until_planning(h)
    assert param_calls(h, f.PARAM_NODE_CONTROLLER, m) == \
        [{'FollowCoveragePath.desired_linear_vel': 0.4}]


def test_settings_service_unavailable_uses_defaults():
    h = Harness(mow_angle_deg=15.0)
    h.area_settings = {0: None}
    m = h.mark()
    start_until_planning(h)
    assert any('get_area_settings unavailable' in e.text for e in h.since(m, f.Log))
    assert [c.request['percent'] for c in calls(h, m, f.SRV_CUTTER_HEIGHT)] == [33]
    cov = param_calls(h, f.PARAM_NODE_COVERAGE, m)[0]
    assert cov['path_mode'] == 'zigzag' and cov['headland_rings'] == 2
    assert cov['edge_first'] is True and cov['operation_width'] == pytest.approx(0.18)
    # an auto-angle area falls back to the global mow_angle_deg parameter
    assert h.goal(f.ACT_PLAN)['mow_angle_deg'] == 15.0


def test_coverage_param_failure_still_plans():
    h = Harness()
    h.coverage_params_ok = False
    m = h.mark()
    start_until_planning(h)
    assert h.pending_action(f.ACT_PLAN)
    assert any('set_parameters failed' in e.text for e in h.since(m, f.Log))


def test_area_settings_disabled_plans_directly():
    h = Harness(use_area_settings=False, set_cut_speed=False, set_cutter_height=False)
    m = h.mark()
    start_until_planning(h)
    assert not calls(h, m, f.SRV_GET_AREA_SETTINGS)
    assert not calls(h, m, f.SRV_CUTTER_HEIGHT)
    assert not param_calls(h, f.PARAM_NODE_CONTROLLER, m)
    assert h.pending_action(f.ACT_PLAN)


def test_repeat_mows_the_area_n_times():
    h = Harness()
    h.area_settings = {0: {'repeat': 3, 'mow_angle_deg': 20.0}}
    start_until_planning(h)
    angles = []
    for run in range(3):
        assert h.name == 'PLANNING' and '[zigzag, run %d/3]' % (run + 1) in h.fsm.sub_state
        assert not h.blade
        angles.append(h.goal(f.ACT_PLAN)['mow_angle_deg'])
        mow_area_once(h)
    assert angles == [20.0, 20.0, 20.0]
    assert h.name == 'RETURNING_HOME'
    assert len(calls(h, 0, f.SRV_GET_AREA_SETTINGS)) == 1
    assert len(calls(h, 0, f.SRV_CUTTER_HEIGHT)) == 1
    assert len(param_calls(h, f.PARAM_NODE_COVERAGE)) == 3
    assert h.since(0, f.PublishAreaSettings)[-1].settings == {}


def test_cross_plans_twice_90_deg_apart():
    h = Harness()
    h.area_settings = {0: {'path_mode': 'cross', 'mow_angle_deg': 30.0}}
    h.areas = [square(0, 0, 5), square(10, 0, 5)]
    start_until_planning(h)
    g1 = h.goal(f.ACT_PLAN)
    assert param_calls(h, f.PARAM_NODE_COVERAGE)[-1]['path_mode'] == 'cross'
    mow_area_once(h)
    assert h.name == 'PLANNING' and h.fsm.mission.area_idx == 0
    g2 = h.goal(f.ACT_PLAN)
    assert (g1['mow_angle_deg'], g2['mow_angle_deg']) == (30.0, 120.0)
    assert param_calls(h, f.PARAM_NODE_COVERAGE)[-1]['mow_angle_deg'] == 120.0
    mow_area_once(h)
    assert h.name == 'PLANNING' and h.fsm.mission.area_idx == 1   # then the next area


def test_cross_on_auto_angle_uses_perpendicular_flag():
    h = Harness()
    h.area_settings = {0: {'path_mode': 'cross', 'repeat': 2}}
    start_until_planning(h)
    goals = []
    for _ in range(4):
        g = h.goal(f.ACT_PLAN)
        goals.append((g['mow_angle_deg'], g['perpendicular']))
        mow_area_once(h)
    assert goals == [(-1.0, False), (-1.0, True)] * 2


def test_alternate_rotates_each_run_and_across_sessions():
    h = Harness()
    h.area_settings = {0: {'path_mode': 'alternate', 'repeat': 2, 'mow_angle_deg': 10.0,
                           'alternate_angle_offset_deg': 45.0}}
    start_until_planning(h)
    a1 = h.goal(f.ACT_PLAN)['mow_angle_deg']
    mow_area_once(h)
    a2 = h.goal(f.ACT_PLAN)['mow_angle_deg']
    mow_area_once(h)
    assert (a1, a2) == (10.0, 55.0)
    saved = h.since(0, f.SaveAlternateCounts)
    assert saved and saved[-1].counts == {'a': 2}
    # next session continues the rotation
    h2 = Harness()
    h2.fsm.alternate_counts = dict(saved[-1].counts)
    h2.area_settings = {0: {'path_mode': 'alternate', 'mow_angle_deg': 10.0,
                            'alternate_angle_offset_deg': 45.0}}
    start_until_planning(h2)
    assert h2.goal(f.ACT_PLAN)['mow_angle_deg'] == 100.0


def test_alternate_on_auto_angle():
    h = Harness()
    h.area_settings = {0: {'path_mode': 'alternate', 'repeat': 3}}
    start_until_planning(h)
    goals = []
    for _ in range(3):
        g = h.goal(f.ACT_PLAN)
        goals.append((g['mow_angle_deg'], g['perpendicular']))
        mow_area_once(h)
    assert goals == [(-1.0, False), (-1.0, True), (-1.0, False)]


def test_resume_continues_the_interrupted_repeat_run():
    h = Harness()
    h.area_settings = {0: {'repeat': 2}}
    start_until_planning(h)
    mow_area_once(h)
    assert '[zigzag, run 2/2]' in h.fsm.sub_state
    sp = line(0, 0, 4, 0)
    h.fsm.inputs.pose = (0.0, 0.0, 0.0)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([sp, line(4, 0.2, 0, 0.2)]))
    follow_current(h)                     # first sub-path of run 2 done
    assert h.cmd(f.CMD_STOP)
    assert 'area_run 0 1' in h.saved
    h2 = Harness(cursor=ResumeCursor.loads(h.saved))
    h2.area_settings = {0: {'repeat': 2}}
    assert h2.fsm.cursor.available
    start_until_planning(h2)
    assert '[zigzag, run 2/2]' in h2.fsm.sub_state
    h2.finish(f.ACT_PLAN, f.SUCCEEDED, plan([sp, line(4, 0.2, 0, 0.2)]))
    assert h2.fsm.mission.sub_i == 1      # resumes inside run 2


def test_settings_reply_after_stop_is_ignored():
    h = Harness()
    h.auto_settings = False
    assert h.cmd(f.CMD_START)
    for _ in range(5):                       # answer get_mowing_area only
        s = h.fsm._service
        if s is None or s.name != f.SRV_GET_AREA:
            break
        req = h._request_of(s.token)
        idx = req.request['index']
        resp = (True, {'success': True, 'area': h.areas[idx]}) if idx < len(h.areas) \
            else (True, {'success': False})
        h._apply(h.fsm.on_service_result(s.token, *resp, now=h.t))
    h.tick()
    assert h.name == 'PLANNING' and h.fsm._service.name == f.SRV_GET_AREA_SETTINGS
    tok = h.fsm._service.token
    assert h.cmd(f.CMD_STOP)
    fx = h.fsm.on_service_result(tok, True, {'success': True, 'settings_json': '{}'}, h.t)
    h._apply(fx)
    assert not [e for e in fx if isinstance(e, (f.StartAction, f.CallService))]
    assert h.name == 'IDLE'


def test_resume_cursor_area_run_round_trip():
    cur = ResumeCursor()
    cur.current_command = 1
    cur.area_runs = {2: 1, 3: 0}
    text = cur.dumps()
    assert 'area_run 2 1' in text and 'area_run 3' not in text
    back = ResumeCursor.loads(text)
    assert back.area_runs == {2: 1} and back.available


def test_stop_clears_latched_plan(monkeypatch=None):
    """STOP after planning publishes an empty /coverage/full_plan so the GUI drops the route."""
    import inspect
    src = inspect.getsource(f.MissionFSM._go_idle)
    assert 'PublishPlan([])' in src


# =====================================================================
# two-step manual mowing (manual_blade_requires_enable)
# =====================================================================
def manual_two_step():
    h = Harness(manual_blade_requires_enable=True)
    assert h.cmd(f.CMD_MANUAL_MOW)
    return h


def blade_req(h, enable):
    ok, msg, fx = h.fsm.manual_blade(enable, h.t)
    h._apply(fx)
    return ok


def test_two_step_manual_enters_blade_off():
    h = manual_two_step()
    assert h.name == 'MANUAL_MOWING' and not h.blade
    assert h.fsm.sub_state == f.SUB_MANUAL_BLADE_OFF
    assert not h.since(0, f.BladeOn)


def test_two_step_manual_blade_on_off_stays_manual():
    h = manual_two_step()
    m = h.mark()
    assert blade_req(h, True)
    assert h.blade and h.fsm.sub_state == f.SUB_MANUAL_BLADE_ON
    assert h.since(m, f.PublishStatus)[-1].status['sub_state_name'] == f.SUB_MANUAL_BLADE_ON
    assert blade_req(h, False)
    assert not h.blade and h.name == 'MANUAL_MOWING'
    assert h.fsm.sub_state == f.SUB_MANUAL_BLADE_OFF
    h.tick(n=5)
    assert h.name == 'MANUAL_MOWING'


def test_manual_blade_refused_outside_manual():
    h = Harness(manual_blade_requires_enable=True)
    assert not blade_req(h, True)
    assert not h.blade and h.name == 'IDLE'


@pytest.mark.parametrize('veto', ['docked', 'is_charging', 'lift'])
def test_manual_blade_vetoes(veto):
    h = manual_two_step()
    setattr(h.fsm.inputs, veto, True)
    assert not blade_req(h, True)
    assert not h.blade


def test_manual_blade_off_when_docked_later():
    h = manual_two_step()
    assert blade_req(h, True)
    h.fsm.inputs.docked = True
    h.tick()
    assert not h.blade and h.name == 'MANUAL_MOWING'
    assert h.fsm.sub_state == f.SUB_MANUAL_BLADE_OFF


def test_two_step_stop_and_emergency_turn_blade_off():
    h = manual_two_step()
    blade_req(h, True)
    assert h.cmd(f.CMD_STOP)
    assert not h.blade and h.name == 'IDLE'
    h.cmd(f.CMD_MANUAL_MOW)
    assert not h.blade
    blade_req(h, True)
    h.fsm.inputs.emergency_active = True
    h.tick()
    assert h.name == 'EMERGENCY' and not h.blade


def test_manual_from_docked_drive_only():
    h = Harness(manual_blade_requires_enable=True)
    h.fsm.inputs.docked = True
    h.tick()
    assert h.cmd(f.CMD_MANUAL_MOW)
    assert h.name == 'MANUAL_MOWING' and not h.blade


def test_upstream_manual_blade_request_off_keeps_manual():
    h = Harness()
    h.cmd(f.CMD_MANUAL_MOW)
    assert h.blade
    assert blade_req(h, False)
    assert not h.blade and h.name == 'MANUAL_MOWING'


# =====================================================================
# heading alignment gating (mower_localization/heading_aligner)
# =====================================================================
@pytest.mark.parametrize('source', ['none', 'file'])
def test_preflight_refuses_unaligned_heading_away_from_dock(source):
    h = Harness()
    h.heading(source)
    h.tick()
    assert not h.cmd(f.CMD_START)
    assert h.name == 'IDLE' and h.fsm.sub_state == \
        'preflight failed: heading not aligned: drive straight 1 m in manual'
    assert h.fsm.mission is None and not h.since(0, f.StartAction)


def test_preflight_stale_heading_status_counts_as_unaligned():
    h = Harness()
    h.heading('cog', fresh=False)
    h.fsm.inputs.heading_stamp = h.t
    h.tick(dt=1.0, n=6)                 # > heading_status_timeout_s
    assert not h.cmd(f.CMD_START)
    assert f.HEADING_NOT_ALIGNED in h.fsm.sub_state


@pytest.mark.parametrize('source', ['dock', 'cog', 'file_verified'])
def test_aligned_heading_starts_away_from_dock(source):
    h = Harness()
    h.heading(source)
    start_until_planning(h)


def test_gate_can_be_disabled():
    h = Harness(require_heading_alignment=False)
    h.heading('none')
    start_until_planning(h)


@pytest.mark.parametrize('docked, charging', [(True, False), (False, True)])
def test_unaligned_at_dock_undocks_first_then_waits_for_cog(docked, charging):
    h = Harness()
    h.heading('none')
    h.fsm.inputs.docked = docked
    h.fsm.inputs.is_charging = charging
    h.tick()
    assert h.cmd(f.CMD_START)          # at the dock the undock seeds the heading
    h.answer_services()
    assert h.name == 'UNDOCKING'
    h.fsm.inputs.docked = h.fsm.inputs.is_charging = False
    h.finish(f.ACT_UNDOCK)
    assert h.name == 'WAITING_FOR_RTK'
    h.tick(n=3)
    assert h.name == 'WAITING_FOR_RTK' and 'heading' in h.fsm.sub_state
    assert not h.since(0, f.StartAction) or all(
        e.name == f.ACT_UNDOCK for e in h.since(0, f.StartAction))
    h.heading('cog')                     # the undock drive produced a COG measurement
    h.tick()
    assert h.name == 'PLANNING'


def test_charging_only_still_undocks_aligned():
    """is_charging without is_docking_done used to go straight to TRANSIT from the dock."""
    h = Harness()
    h.fsm.inputs.is_charging = True
    h.tick()
    h.cmd(f.CMD_START)
    h.answer_services()
    assert h.name == 'UNDOCKING' and h.goal(f.ACT_UNDOCK)['distance_m'] == 0.8


def test_no_cog_after_undock_stops_without_navigating():
    h = Harness(heading_wait_s=5.0)
    h.heading('none')
    h.fsm.inputs.docked = True
    h.tick()
    h.cmd(f.CMD_START)
    h.answer_services()
    h.fsm.inputs.docked = False
    h.finish(f.ACT_UNDOCK)
    m = h.mark()
    h.tick(dt=1.0, n=7)
    assert h.name == 'IDLE' and f.HEADING_NOT_ALIGNED in h.fsm.sub_state
    assert h.fsm.mission is None
    assert not h.since(m, f.StartAction)      # no dock / navigate with a wrong heading


def test_transit_goal_yaw_is_first_subpath_heading():
    h = Harness()
    start_until_planning(h)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(3, 3, 3, 6)]))
    assert h.name == 'TRANSIT'
    x, y, yaw = h.goal(f.ACT_NAV)['pose']
    assert (x, y) == (3, 3)
    assert abs(yaw - math.pi / 2) < 1e-6


def test_transit_abort_near_target_counts_as_arrived():
    h = Harness()
    start_until_planning(h)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(3, 3, 4, 3), line(0, 0, 4, 0)]))
    assert h.name == 'TRANSIT'
    h.fsm.inputs.pose = (3.15, 2.9, 0.0)          # ~0.18 m off, yaw unsettled
    m = h.mark()
    h.finish(f.ACT_NAV, f.ABORTED)
    assert h.fsm.mission.sub_i == 0 and h.fsm.mission.skipped == 0
    assert h.name == 'MOWING'
    assert any('treating as arrived' in str(e) for e in h.fx[m:])


def test_transit_timeout_near_target_counts_as_arrived():
    h = Harness()
    start_until_planning(h)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(3, 3, 4, 3), line(0, 0, 4, 0)]))
    h.fsm.inputs.pose = (3.0, 3.25, 0.0)
    h.tick(dt=h.fsm.p.transit_timeout_s + 1.0)
    assert h.fsm.mission.sub_i == 0 and h.fsm.mission.skipped == 0
    assert h.name == 'MOWING'


def test_transit_abort_beyond_radius_still_fails():
    h = Harness(transit_retries=0)
    start_until_planning(h)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(3, 3, 4, 3), line(0, 0, 4, 0)]))
    h.fsm.inputs.pose = (2.6, 3.0, 0.0)           # 0.4 m away
    h.finish(f.ACT_NAV, f.ABORTED)
    assert h.fsm.mission.sub_i == 1 and h.fsm.mission.skipped == 1


def test_low_battery_stops_in_place_by_default():
    h = Harness()
    mowing(h)
    h.fsm.inputs.battery_percent = 19.0
    h.tick()
    assert h.name == 'IDLE' and not h.blade
    assert any(isinstance(e, f.ZeroBurst) for e in h.fx)


def _charge_calls(h, mark):
    return [e.request['enable'] for e in h.fx[mark:]
            if isinstance(e, f.CallService) and e.name == f.SRV_CHARGING]


def test_charge_limit_disables_and_reenables_with_hysteresis():
    h = Harness(battery_max_charge_percent=80.0)
    h.fsm.inputs.docked = True
    h.fsm.inputs.battery_percent = 50.0
    m = len(h.fx)
    h.tick()
    assert _charge_calls(h, m) == [True]
    h.fsm.inputs.battery_percent = 80.0
    m = len(h.fx)
    h.tick()
    assert _charge_calls(h, m) == [False]
    h.fsm.inputs.battery_percent = 77.0
    m = len(h.fx)
    h.tick()
    assert _charge_calls(h, m) == []
    h.fsm.inputs.battery_percent = 75.0
    m = len(h.fx)
    h.tick()
    assert _charge_calls(h, m) == [True]


def test_charge_resume_waits_for_min_of_full_and_max():
    h = Harness(battery_low_action='dock', battery_max_charge_percent=80.0)
    mowing(h)
    h.fsm.inputs.battery_percent = 19.0
    h.tick()
    h.fsm.inputs.docked = True
    h.finish(f.ACT_DOCK)
    assert h.name == 'CHARGING'
    h.fsm.inputs.battery_percent = 80.0
    h.tick()
    h.answer_services()
    assert h.name == 'UNDOCKING'


def test_stop_while_returning_home_cancels_dock_goal():
    """GUI Stop (high_level_control 8) during RETURNING_HOME cancels /mower_docking/dock."""
    h = Harness()
    assert h.cmd(f.CMD_HOME)
    assert h.name == 'RETURNING_HOME'
    tok = h.pending_action(f.ACT_DOCK).token
    m = h.mark()
    assert h.cmd(f.CMD_STOP)
    assert h.since(m, f.CancelActions)
    assert h.since(m, f.ZeroBurst)
    assert h.name == 'IDLE' and h.fsm._action is None
    # the server's late CANCELED result must not turn into NAV_TO_DOCK_FAILED
    h._apply(h.fsm.on_action_result(tok, f.CANCELED,
                                    {'success': False, 'message': 'CANCELED'}, h.t))
    assert h.name == 'IDLE'


# =====================================================================
# retries / MOWING_INCOMPLETE
# =====================================================================
def _clears(h, mark=0):
    return [e for e in h.fx[mark:] if isinstance(e, f.CallService)
            and e.name == f.SRV_CLEAR_COSTMAPS]


def _two_subpaths_first_needs_transit(h):
    start_until_planning(h)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(3, 3, 4, 3), line(0, 0, 4, 0)]))
    assert h.name == 'TRANSIT'


def test_transit_abort_waits_clears_costmaps_and_resends():
    h = Harness(transit_retries=3, transit_retry_delay_s=3.0)
    _two_subpaths_first_needs_transit(h)
    first = h.pending_action(f.ACT_NAV).token
    m = h.mark()
    h.finish(f.ACT_NAV, f.ABORTED)
    assert h.name == 'TRANSIT' and h.fsm._action is None and not h.blade
    assert 'transit aborted, retry 1/3 in 3 s' in h.fsm.sub_state
    assert h.fsm.status()['sub_state_name'] == h.fsm.sub_state
    assert _clears(h, m)
    h.tick(dt=1.0, n=2)
    assert h.fsm._action is None                # still waiting
    h.fsm.inputs.pose = (1.0, 1.0, 0.0)          # robot moved meanwhile
    h.tick(dt=1.0, n=2)
    a = h.pending_action(f.ACT_NAV)
    assert a.token != first and h.goal(f.ACT_NAV)['pose'][:2] == (3, 3)   # same fixed target
    h.finish(f.ACT_NAV, f.ABORTED)
    assert 'retry 2/3' in h.fsm.sub_state
    h.tick(dt=1.0, n=4)
    h.fsm.inputs.pose = (3, 3, 0.0)
    h.finish(f.ACT_NAV)
    assert h.name == 'MOWING' and h.fsm.mission.skipped == 0 and h.fsm.mission.sub_i == 0


def test_transit_retry_counts_as_arrived_when_within_radius():
    h = Harness()
    _two_subpaths_first_needs_transit(h)
    h.finish(f.ACT_NAV, f.ABORTED)               # (0,0): far from (3,3)
    h.fsm.inputs.pose = (3.1, 2.9, 0.0)           # drifted in during the wait
    m = h.mark()
    h.tick(dt=1.0, n=4)
    assert h.name == 'MOWING'
    assert not [e for e in h.since(m, f.StartAction) if e.name == f.ACT_NAV]


def test_transit_retries_exhausted_ends_mowing_incomplete_in_place():
    h = Harness(transit_retries=3, transit_retry_delay_s=3.0)
    _two_subpaths_first_needs_transit(h)
    for k in range(4):
        h.finish(f.ACT_NAV, f.ABORTED)
        if k < 3:
            h.tick(dt=1.0, n=4)
    # sub-path 0 not mowed; sub-path 1 (starts at the robot) is mowed
    assert h.fsm.mission.sub_i == 1 and h.fsm.mission.skipped == 1
    assert any('NOT mowed' in str(e) for e in h.fx)
    m = h.mark()
    follow_current(h)
    assert h.name == 'MOWING_INCOMPLETE' and h.fsm.state == f.STATE_IDLE
    assert h.fsm.sub_state == '1 of 2 sub-paths not mowed: transit aborted 4 times'
    assert 'MOWING_COMPLETE' not in h.statuses(m)
    assert not [e for e in h.since(m, f.StartAction) if e.name == f.ACT_DOCK]
    assert h.since(m, f.ZeroBurst) and not h.blade
    assert h.fsm.cursor.available
    assert h.since(m, f.PublishResumeAvailable)[-1].available is True
    cur = ResumeCursor.loads(h.saved)
    assert cur.available and 0 not in cur.completed_areas
    assert cur.areas[0].completed == {1} and cur.areas[0].resume_index == 0
    h.tick(dt=1.0, n=60)                          # latched, not a timed display state
    assert h.name == 'MOWING_INCOMPLETE'
    # START resumes only the missing sub-path
    start_until_planning(h)
    assert h.fsm.mission.resume
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(3, 3, 4, 3), line(0, 0, 4, 0)]))
    assert h.name == 'TRANSIT' and h.goal(f.ACT_NAV)['pose'][:2] == (3, 3)
    h.fsm.inputs.pose = (3, 3, 0.0)
    h.finish(f.ACT_NAV)
    m = h.mark()
    follow_current(h)
    assert 'MOWING_COMPLETE' in h.statuses(m) and h.name == 'RETURNING_HOME'
    assert not h.fsm.cursor.available


def test_mowing_incomplete_commands():
    h = Harness(transit_retries=0)
    start_until_planning(h)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(3, 3, 4, 3)]))
    h.finish(f.ACT_NAV, f.ABORTED)
    assert h.name == 'MOWING_INCOMPLETE'
    assert h.fsm.sub_state.startswith('1 of 1 sub-paths not mowed: transit aborted')
    assert h.cmd(f.CMD_HOME) and h.name == 'RETURNING_HOME'
    assert h.cmd(f.CMD_STOP) and h.name == 'IDLE'
    assert h.fsm.cursor.available                 # STOP keeps the resume


def test_return_home_on_incomplete_docks_and_keeps_reason():
    h = Harness(transit_retries=0, return_home_on_incomplete=True)
    start_until_planning(h)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([line(3, 3, 4, 3)]))
    m = h.mark()
    h.finish(f.ACT_NAV, f.ABORTED)
    assert 'MOWING_INCOMPLETE' in h.statuses(m)
    assert h.name == 'RETURNING_HOME' and 'not mowed' in h.fsm.sub_state
    h.fsm.inputs.docked = True
    h.finish(f.ACT_DOCK)
    assert h.name == 'IDLE_DOCKED' and 'not mowed' in h.fsm.sub_state
    assert h.fsm.cursor.available


def test_follow_abort_resumes_from_last_pose_up_to_follow_retries():
    h = Harness(follow_retries=3, transit_retry_delay_s=3.0)
    start_until_planning(h)
    sp = line(0, 0, 10, 0, n=21)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, plan([sp]))
    h.fsm.inputs.is_cutting = True
    h.tick()
    starts = []
    for k, x in enumerate((2.0, 3.0, 4.0, 5.0)):
        h.fsm.inputs.pose = (x, 0.0, 0.0)
        h.tick()
        h.finish(f.ACT_FOLLOW, f.ABORTED)
        if k < 3:
            assert 'retry %d/3' % (k + 1) in h.fsm.sub_state and not h.blade
            h.tick(dt=1.0, n=4)
            starts.append(h.goal(f.ACT_FOLLOW)['poses'][0][0])
    assert starts == [2.0, 3.0, 4.0]
    assert h.name == 'MOWING_INCOMPLETE'
    assert 'follow_path aborted 4 times' in h.fsm.sub_state
    cur = ResumeCursor.loads(h.saved)
    assert cur.areas[0].resume_index == 10         # resume at the last reached pose (5 m)


def test_planning_failure_ends_incomplete():
    h = Harness()
    start_until_planning(h)
    h.finish(f.ACT_PLAN, f.SUCCEEDED, {'success': False, 'message': 'degenerate polygon'})
    assert h.name == 'MOWING_INCOMPLETE'
    assert 'area(s) not planned' in h.fsm.sub_state and 'degenerate' in h.fsm.sub_state


def _boundary_paused(h):
    mowing(h)                                    # sub-path 0: (0,0) -> (4,0)
    h.fsm.inputs.pose = (1.0, -0.2, 0.0)
    h.fsm.inputs.boundary_violation = True
    m = h.mark()
    h.tick()
    assert h.name == 'BOUNDARY_PAUSED'
    assert _clears(h, m)                         # costmaps refresh before the recovery
    h.tick(dt=1.0, n=3)
    assert h.name == 'TRANSIT' and h.fsm.mission.recovering


def test_boundary_recovery_transit_abort_is_retried():
    h = Harness(boundary_recovery_retries=3, transit_retry_delay_s=3.0)
    _boundary_paused(h)
    target = h.goal(f.ACT_NAV)['pose']
    h.finish(f.ACT_NAV, f.ABORTED)
    assert h.name == 'TRANSIT' and 'boundary recovery transit aborted, retry 1/3' \
        in h.fsm.sub_state
    h.tick(dt=1.0, n=4)
    assert h.goal(f.ACT_NAV)['pose'] == target
    h.fsm.inputs.boundary_violation = False
    h.fsm.inputs.pose = target
    h.finish(f.ACT_NAV)
    assert h.name == 'MOWING' and h.fsm.mission.skipped == 0


def test_boundary_recovery_failures_stop_in_place_incomplete():
    """Real-mower regression: the recovery transit aborted, the sub-path was
    skipped and the mission reported MOWING_COMPLETE and docked."""
    h = Harness(boundary_recovery_retries=2, transit_retry_delay_s=3.0)
    _boundary_paused(h)
    m = h.mark()
    for k in range(3):
        h.finish(f.ACT_NAV, f.ABORTED)
        if k < 2:
            h.tick(dt=1.0, n=4)
            assert h.pending_action(f.ACT_NAV)
    assert h.name == 'MOWING_INCOMPLETE' and not h.blade
    assert h.fsm.sub_state.startswith('2 of 2 sub-paths not mowed: boundary recovery transit '
                                      'aborted 3 times')
    names = h.statuses(m)
    assert 'MOWING_COMPLETE' not in names and 'RETURNING_HOME' not in names
    assert 'COVERAGE_FAILED_DOCKING' not in names
    assert not [e for e in h.since(m, f.StartAction) if e.name == f.ACT_DOCK]
    assert h.fsm.cursor.available and h.fsm.mission is None
    assert ResumeCursor.loads(h.saved).areas[0].completed == set()
    start_until_planning(h)
    assert h.fsm.mission.resume


# ---------------------------------------------------------------------------
# /motion_enabled (wheel motion latch)
# ---------------------------------------------------------------------------
NO_MOTION = ('IDLE', 'IDLE_DOCKED', 'CHARGING', 'EMERGENCY', 'BOUNDARY_EMERGENCY_STOP',
             'MOWING_COMPLETE', 'MOWING_INCOMPLETE', 'NAV_TO_DOCK_FAILED',
             'PREFLIGHT_CHECK', 'RAIN_WAITING', 'RECORDING_COMPLETE', 'UNDOCK_FAILED')
MOTION = ('MANUAL_MOWING', 'RECORDING', 'TRANSIT', 'MOWING', 'BOUNDARY_PAUSED',
          'RETURNING_HOME', 'LOW_BATTERY_DOCKING', 'UNDOCKING')


@pytest.mark.parametrize('phase', NO_MOTION)
def test_motion_disabled_in_non_motion_states(phase):
    h = Harness()
    h.fsm.phase = phase
    assert h.fsm.motion_enabled() is False


@pytest.mark.parametrize('phase', MOTION)
def test_motion_enabled_in_motion_states_off_dock(phase):
    h = Harness()
    h.fsm.phase = phase
    assert h.fsm.motion_enabled() is True


@pytest.mark.parametrize('phase', [p for p in MOTION
                                   if p not in ('UNDOCKING', 'MANUAL_MOWING', 'RETURNING_HOME',
                                                'LOW_BATTERY_DOCKING')])
def test_motion_disabled_when_docked_without_undock(phase):
    h = Harness()
    h.fsm.phase = phase
    h.fsm.inputs.docked = True
    assert h.fsm.motion_enabled() is False
    h.fsm.inputs.docked = False
    h.fsm.inputs.is_charging = True
    assert h.fsm.motion_enabled() is False


def test_motion_enabled_only_after_explicit_undock():
    h = Harness()
    h.fsm.inputs.docked = True
    h.fsm.inputs.is_charging = True
    h.tick()
    assert h.name == 'CHARGING' and not h.fsm.motion_enabled()
    h.cmd(f.CMD_MANUAL_MOW)            # manual while docked: the operator may drive off
    assert h.fsm.motion_enabled()
    h.cmd(f.CMD_STOP)
    h.cmd(f.CMD_START)
    h.answer_services()
    assert h.name == 'UNDOCKING' and h.fsm.motion_enabled()


def test_motion_disabled_on_emergency_and_not_restored_by_reset():
    h = Harness()
    h.cmd(f.CMD_MANUAL_MOW)
    assert h.fsm.motion_enabled()
    h.fsm.inputs.emergency_active = True
    h.tick()
    assert h.name == 'EMERGENCY' and not h.fsm.motion_enabled()
    h.fsm.inputs.emergency_active = False
    h.tick()
    # emergency cleared -> idle, not back to a motion state
    assert h.name == 'IDLE' and not h.fsm.motion_enabled()
