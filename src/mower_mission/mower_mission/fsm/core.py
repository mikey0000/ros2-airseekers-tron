# SPDX-License-Identifier: GPL-3.0-or-later
"""MissionFSM: state, effects, the public API (tick / status), composed of the mixins.

Moved verbatim from mission_fsm.py (2026-10-23 code review stage 1.6)."""

from mower_mission.fsm.common import (ACT_DOCK, ACT_PLAN, ACT_SPIN, BladeOff, BladeOn,
    CMD_START, CallService, CancelActions, DOCKED_MOTION_PHASES, DOCK_PHASES, DeleteResume,
    IDLE_NAMES, Inputs, Log, MOTION_PHASES, Params, PublishAreaSettings, PublishPlan,
    PublishResumeAvailable, PublishStatus, RecordIncident, ResumeCursor, SRV_CHARGING,
    STATE_CODES, STATE_IDLE, SUCCEEDED, SaveResume, StartAction, ZeroBurst, _Pending, geo,
    math)  # noqa: F401
from mower_mission.fsm.phases.preview import PreviewMixin
from mower_mission.fsm.guards.reactive import ReactiveGuardsMixin
from mower_mission.fsm.phases.dispatch import PhaseDispatchMixin
from mower_mission.fsm.commands import CommandsMixin
from mower_mission.fsm.phases.start import StartMixin
from mower_mission.fsm.phases.areas import AreasMixin
from mower_mission.fsm.phases.mowing import MowingMixin
from mower_mission.fsm.phases.pivots import PivotsMixin
from mower_mission.fsm.phases.outcome import OutcomeMixin
from mower_mission.fsm.retry import RetryMixin
from mower_mission.fsm.phases.hcal import HeadingCalMixin
from mower_mission.fsm.phases.docking import DockingMixin
from mower_mission.fsm.phases.recording import RecordingMixin
from mower_mission.fsm.phases.transit import TransitMixin
from mower_mission.fsm.holds import HoldsMixin
from mower_mission.fsm.guards.stuck import StuckGuardMixin
from mower_mission.fsm.guards.tilt import TiltGuardMixin
from mower_mission.fsm.guards.boundary import BoundaryGuardMixin
from mower_mission.fsm.guards.spin_drift import SpinDriftGuardMixin


class MissionFSM(PreviewMixin,
                 ReactiveGuardsMixin,
                 PhaseDispatchMixin,
                 CommandsMixin,
                 StartMixin,
                 AreasMixin,
                 MowingMixin,
                 PivotsMixin,
                 OutcomeMixin,
                 RetryMixin,
                 HeadingCalMixin,
                 DockingMixin,
                 RecordingMixin,
                 TransitMixin,
                 HoldsMixin,
                 StuckGuardMixin,
                 TiltGuardMixin,
                 BoundaryGuardMixin,
                 SpinDriftGuardMixin):
    """See module docstring."""

    def __init__(self, params=None, cursor=None, now=0.0, alternate_counts=None):
        self.p = params if isinstance(params, Params) else Params.from_dict(params)
        self.alternate_counts = dict(alternate_counts or {})
        self.inputs = Inputs()
        self.cursor = cursor if cursor is not None else ResumeCursor()
        self.phase = 'IDLE'
        self._charge_req = None   # last charge enable sent while docked (None = not sent)
        self._charge_req_t = 0.0
        self.sub_state = ''
        self.blade_on = False
        self._plan_shown = False
        self.mission = None
        self._now = now
        # optional wall clock (the node sets time.monotonic): request deadlines are stamped
        # from it, not from the event start, so a slow event (78 s of path conditioning on
        # the RK3588, field 2026-10-20) cannot hand a request an already-expired deadline
        self.clock = None
        self._created = now
        self._phase_since = now
        self._fx = []
        self._token = 0
        self._action = None
        self._service = None
        self._emergency = False
        self._dock_purpose = None
        self._dock_routed = None            # (phase, purpose) whose dock route already ran
        self._dock_route_tries = 0          # dock route re-sends of this dock (task 0.4 e)
        self._dock_route_end = None         # (x, y) end of the last dock route (approach pose)
        self._route_legs = None             # {'legs': [poses...], 'i': n} of a route follow
        self._route_watch = None            # route progress watch {'token', 's', 't'}
        self._pivot_tries = {}              # unfinished-pivot re-sends per pivot (key -> n)
        self._dock_contact_since = None
        self._manual_undocking = False           # standalone UNDOCK (CMD_UNDOCK) in flight
        self._resume_after = None           # None | 'rain' | 'charge'
        self._rain_since = None
        self._rain_clear_since = None
        self._boundary_since = None
        self._heading_wait_since = None
        self._hcal_wanted = False           # preflight let START through for a calibration drive
        self._hcal = None                   # heading calibration drive in progress
        self._hcal_rtk_since = None
        self._spin_attempt = 0
        self._spin_deadline = 0.0
        self._track = []
        self._last_sample = -1e9
        self._record_poly = None
        self._enum = None                   # {'purpose', 'index', 'areas', 'count'}
        self._region_areas = {}             # last area enumeration (path conditioning)
        self._record_name = ''
        self._record_kind = 'area'          # 'area' (closed boundary) | 'path' (open band)
        self._record_line = None            # simplified centreline of a PATH being saved
        self._record_index = -1             # area index the PATH gets in the map server
        self._last_status = None
        self._incomplete_text = ''
        self._dyn = None                    # dynamic-obstacle wait context
        self._path_eval = None              # last in-path judgement (_obstacle_on_path)
        self._loc = None                    # localization hold context (_loc_hold_tick)
        self._blk = None                    # path-blocked wait context (see _blocked)
        self.obstacle_decision = {'kind': 'none'}   # /mission/obstacle_decision (GUI)
        self._dyn_ignore = None             # post-timeout ignore window (see _dyn_ignored)
        self._last_static_t = -1e9
        self._pivot_since = None      # start of the current in-place pivot command
        self._preview = None                # running plan preview (see preview_plan)
        self._preview_shown = False         # a preview is drawn on /coverage/full_plan
        self._preview_id = 0
        self._stk = None                    # stuck escape in progress (see _stuck_tick)
        self._stuck_hist = []               # [[t, x, y, far]] stuck spots escaped from
        self._stuck_ignore_until = -1e9
        self._tob = None                    # tilt back-out in progress (see _tilt_tick)
        self._tilt_hist = []                # [[t, x, y]] steep spots backed out of
        self._tilt_trail = []               # [(x, y)] recent driven track, oldest first
        self._tilt_capped = False           # caution speed cap sent to the controller
        self._hb = None                     # hard-boundary retrace in progress (task 0.4 a)
        self._hb_count = 0                  # hard-boundary excursions of this mission
        self._hb_spots = []                 # [(x, y)] excursion origins of this mission
        self._hb_trail = []                 # [(x, y)] recent driven EKF track, oldest first
        self._hb_jump = None                # (last fresh boundary value, t, jump hold since)
        self._hb_entry = False              # first boundary reading after START / resume
        self._spin_from = None              # (token, x, y) where the running Spin started
        self._spin_drifts = {}              # coverage leg key -> Spin drifts
        self._spin_resend = None            # delayed pivot re-send (pivot_retry_pause_s)
        self._bump_seen = 0                 # inputs.bumper_hits already handled
        self._bump_log = []                 # [(t, x, y)] recent bumper hits

    # ==================================================================
    # presentation
    # ==================================================================
    @property
    def state(self):
        return STATE_CODES.get(self.phase, STATE_IDLE)

    def status(self):
        m = self.mission
        total = len(m.subpaths) if m and m.area_idx is not None else 0
        cov = 0.0
        if m and m.lengths and m.area_idx is not None:
            ac = self.cursor.areas.get(m.area_idx)
            tot_len = sum(m.lengths)
            if ac and tot_len > 0:
                done = sum(m.lengths[i] for i in ac.completed if i < len(m.lengths))
                if m.step == 'follow' and m.sub_i < len(m.cum) and m.sub_i not in ac.completed:
                    done += m.cum[m.sub_i][min(m.progress_local, len(m.cum[m.sub_i]) - 1)]
                cov = min(100.0, 100.0 * done / tot_len)
        ac = self.cursor.areas.get(m.area_idx) if m and m.area_idx is not None else None
        i = self.inputs
        return {
            'state': self.state,
            'state_name': self.phase,
            'sub_state_name': self.display_sub_state(),
            'current_area': m.area_idx if m and m.area_idx is not None else -1,
            'current_path': m.sub_i if m and m.area_idx is not None and m.subpaths else -1,
            'current_path_index': (m.progress_local if m.step == 'follow' else m.start_local)
            if m and m.subpaths else -1,
            'total_swaths': total,
            'completed_swaths': len(ac.completed) if ac else 0,
            'skipped_swaths': m.skipped if m else 0,
            'coverage_percent': cov,
            'gps_quality_percent': float(i.gps_quality),
            'battery_percent': float(i.battery_percent) if i.battery_percent is not None else 0.0,
            'is_charging': bool(i.is_charging),
            'emergency': self.phase == 'EMERGENCY' or self._emergency,
        }

    def motion_enabled(self):
        """True only when the wheels may move (see MOTION_PHASES)."""
        if self._emergency or self.phase not in MOTION_PHASES:
            return False
        i = self.inputs
        if (i.docked or i.is_charging) and self.phase not in DOCKED_MOTION_PHASES:
            return False
        return True

    # ==================================================================
    # effect helpers
    # ==================================================================
    def _begin(self, now):
        self._now = float(now)
        self._fx = []

    def _end(self):
        fx, self._fx = self._fx, []
        return fx

    def _log(self, level, text):
        self._fx.append(Log(level, text))

    def _incident(self, kind, detail=''):
        """Terrain-memory incident at the current pose (no-op without a pose)."""
        pose = self.inputs.pose
        if pose is None:
            return
        self._fx.append(RecordIncident(kind, float(pose[0]), float(pose[1]), str(detail)))

    def _go(self, name, sub=None):
        if self.blade_on and name not in ('MOWING', 'MANUAL_MOWING') and \
                not (name == 'TRANSIT' and self._blade_keep_ok()):
            # Safety net only: every path turns the blade off explicitly first.
            self._log('error', 'safety: blade forced off on entering %s' % name)
            self._blade_off('safety net (%s)' % name)
        if sub is not None:
            self.sub_state = sub
        if name != self.phase:
            self._log('info', 'state %s -> %s%s' % (
                self.phase, name, (' (%s)' % self.sub_state) if self.sub_state else ''))
            self.phase = name
            self._phase_since = self._now
            self._heading_wait_since = None
            self._hcal = None
            self._hcal_rtk_since = None
            if name not in ('PREFLIGHT_CHECK', 'WAITING_FOR_RTK'):
                self._hcal_wanted = False
        st = self.status()
        if st != self._last_status:
            self._last_status = st
            self._fx.append(PublishStatus(dict(st)))

    def _idle_name(self):
        if self.inputs.docked:
            return 'CHARGING' if self.inputs.is_charging else 'IDLE_DOCKED'
        return 'IDLE'

    def _resume_charge_percent(self):
        return min(float(self.p.battery_full_percent), float(self.p.battery_max_charge_percent))

    def _charge_limit_tick(self):
        """Keep the MCU charge enable in line with battery_max_charge_percent while
        docked (also re-enables charging after a stack restart on the dock)."""
        i = self.inputs
        if not i.docked:
            self._charge_req = None
            return
        b = i.battery_percent
        if b is None:
            return
        limit = float(self.p.battery_max_charge_percent)
        # Re-assert "off" every 60 s, and within 2 s whenever the MCU MEASURES charging
        # (is_charging = dock contacts + current into the battery, mcu_node 2026-10-09): the
        # docking server enables charging itself at the end of every dock (live 2026-10-09: 99 %,
        # limit 95 %, enabled 40 ms after our "off"), which the 60 s re-assert let run for a minute.
        if b >= limit and (self._charge_req is not False
                           or self._now - self._charge_req_t >= 60.0
                           or (i.is_charging and self._now - self._charge_req_t >= 2.0)):
            self._charge_req, self._charge_req_t = False, self._now
            self._log('info', 'battery %.0f%% >= max charge %.0f%%: charging disabled' % (b, limit))
            self._call(SRV_CHARGING, {'enable': False}, track=False)
        elif b <= limit - float(self.p.battery_charge_hysteresis_percent) \
                and (self._charge_req is not True
                     or (not i.is_charging and self.phase in IDLE_NAMES
                         and self._now - self._charge_req_t >= 10.0)):
            # Re-assert "on" when the MCU reports not charging 10 s after our
            # request: an undock (stopped on the dock) or the docking server
            # turns the charger off behind our back.
            self._charge_req, self._charge_req_t = True, self._now
            self._log('info', 'docked at %.0f%% (max charge %.0f%%): charging enabled' % (b, limit))
            self._call(SRV_CHARGING, {'enable': True}, track=False)

    def _go_idle(self, sub=None):
        self._resume_after = None
        if self._plan_shown:
            # clear the latched /coverage/full_plan so the GUI stops drawing the route
            self._plan_shown = False
            self._fx.append(PublishPlan([]))
        self._go(self._idle_name(), sub)

    def _blade_off(self, reason):
        if self.mission is not None:
            self.mission.blade_transit = False
        self.blade_on = False
        self._fx.append(BladeOff(reason))

    # ---- blade policy (2026-10-07 owner: "run blades unless e-stop / lift") ----
    def _continuous(self):
        m = self.mission
        return m is not None and m.settings is not None and \
            m.settings.get('blade_policy', 'continuous') == 'continuous'

    def _blade_keep_ok(self):
        """May a running blade stay on in the current mission step (continuous policy,
        inside the area)?  Used by the _go safety net and blade_allowed()."""
        m = self.mission
        keep_obs = bool(self.p.blade_keep_on_obstacles) and self._hb is None and \
            self._body_inside(self.inputs.pose)
        if not (self._continuous() or keep_obs) or self.inputs.boundary_violation or \
                self.phase not in ('MOWING', 'TRANSIT') or m.recovering:
            return False
        if m.step in ('dyn_wait', 'retry_wait', 'blocked_wait'):
            return True
        if m.step == 'pivot_backout' and keep_obs:
            return True
        return m.step == 'transit' and m.blade_transit

    def _soft_blade_off(self, reason):
        """Blade off unless the continuous policy keeps it running here, or (owner 2026-10-23,
        blade_keep_on_obstacles) the robot is inside the area by the body rules."""
        if self.blade_on and self._continuous() and not self.inputs.boundary_violation \
                and self.mission is not None and not self.mission.recovering \
                and self.phase in ('MOWING', 'TRANSIT'):
            self._log('info', 'blade kept on (%s, blade_policy continuous)' % reason)
            return
        if self._keep_blade_inside(reason):
            return
        self._blade_off(reason)

    def _body_inside(self, pose):
        """The robot at ``pose`` is inside the allowed area by the boundary guard's own rules
        (body points within their soft / hard limits; base_link check on drawn paths / the
        dock corridor counts as outside the mowing area here)."""
        if pose is None or len(pose) < 3:
            return False
        mode = self._hb_body_mode(pose)
        if mode is None:
            return False
        ev = self._hb_body_eval(pose, mode)
        return ev is not None and ev['excess'] <= 0.0

    def _keep_blade_inside(self, reason, target=None):
        """Owner 2026-10-23: obstacle avoidance / bumper / back-outs do not stop the blade
        while the robot stays inside the area by our body rules (and, for a move to
        ``target``, the straight path there does too). True = kept on (logged)."""
        m = self.mission
        if not self.p.blade_keep_on_obstacles or not self.blade_on or m is None or \
                self.phase not in ('MOWING', 'TRANSIT') or self.inputs.boundary_violation or \
                m.recovering or self._hb is not None or not self._body_inside(self.inputs.pose):
            return False
        if target is not None and not self._segment_in_area(self.inputs.pose, target,
                                                            body=True):
            return False
        self._log('info', 'blade kept on (%s): inside the area (blade_keep_on_obstacles)'
                  % reason)
        return True

    def _segment_in_area(self, a, b, body=False):
        m = self.mission
        area = m.areas.get(m.area_idx) if m is not None else None
        if area is None or a is None or b is None:
            return False
        d = geo.dist(a, b)
        n = max(1, int(math.ceil(d / max(0.05, float(self.p.blade_transit_sample_m)))))
        yaw = math.atan2(b[1] - a[1], b[0] - a[0]) if d > 1e-6 else \
            (float(a[2]) if len(a) >= 3 else 0.0)
        for k in range(n + 1):
            pt = (a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n)
            if not geo.inside_area(pt, area):
                return False
            if body and not self._body_inside((pt[0], pt[1], yaw)):
                return False
        return True

    def _transit_blade(self, target, reason, force_off=False):
        """Before a mission transit / detour: keep the blade on only for a continuous
        area whose straight robot->target segment stays inside the area polygon."""
        m = self.mission
        keep = (not force_off and self.blade_on and self._continuous()
                and not self.inputs.boundary_violation and not m.recovering
                and self._segment_in_area(self.inputs.pose, target))
        if keep:
            m.blade_transit = True
            self._log('info', 'blade kept on: %s stays inside area %d' % (reason, m.area_idx))
        elif self.phase in ('MOWING', 'TRANSIT') and self._keep_blade_inside(reason, target):
            m.blade_transit = True        # a detour / skip transit that stays inside the area
        else:
            self._blade_off(reason)

    def _blade_on(self):
        self.blade_on = True
        self._fx.append(BladeOn())

    def _next_token(self):
        self._token += 1
        return self._token

    def _fresh_now(self):
        """The clock a request deadline starts from: the injected clock when set (never
        earlier than the event time), else the event time."""
        c = self.clock
        if c is None:
            return self._now
        try:
            return max(self._now, float(c()))
        except Exception:  # noqa: BLE001
            return self._now

    def _start_action(self, name, goal, timeout, purpose=None):
        if self._action is not None:
            self._fx.append(CancelActions('superseded by %s' % name))
        tok = self._next_token()
        self._action = _Pending(name, tok, self._fresh_now() + float(timeout), purpose,
                                float(timeout))
        pose = self.inputs.pose
        self._spin_from = ((tok, float(pose[0]), float(pose[1]))
                           if name == ACT_SPIN and pose is not None else None)
        self._fx.append(StartAction(name, goal, tok))
        return tok

    def _call(self, name, request, track=True, purpose=None):
        tok = self._next_token()
        if track:
            self._service = _Pending(name, tok, self._fresh_now() + self.p.service_timeout_s,
                                     purpose, float(self.p.service_timeout_s))
        self._fx.append(CallService(name, request, tok))
        return tok

    def _cancel_all(self, reason):
        if self._preview is not None:
            self._preview_finish('failed', 'cancelled: %s' % reason)
        self._dyn = None
        self._blk = None
        self._dyn_ignore = None
        self._spin_resend = None
        self._action = None
        self._service = None
        self._enum = None
        self._manual_undocking = False
        self._dock_routed = None
        self._fx.append(CancelActions(reason))

    def _persist(self):
        self._fx.append(SaveResume(self.cursor.dumps()))
        self._fx.append(PublishResumeAvailable(self.cursor.available))

    # ==================================================================
    # public API
    # ==================================================================
    def tick(self, now):
        self._begin(now)
        self._charge_limit_tick()
        if not self._guards():
            self._run_phase()
        return self._end()

    def command(self, cmd, now):
        """HighLevelControl. Returns ``(success, effects)``."""
        self._begin(now)
        ok = self._command(int(cmd))
        return ok, self._end()

    def manual_blade(self, enable, now):
        """Blade on/off request while MANUAL_MOWING (``~/manual_blade``).
        Returns ``(success, message, effects)``."""
        self._begin(now)
        ok, msg = self._manual_blade(bool(enable))
        self._log('info' if ok else 'warn', 'manual_blade(%s): %s' % (bool(enable), msg))
        return ok, msg, self._end()

    def start_in_area(self, area, now):
        self._begin(now)
        if self._emergency or self.phase in ('EMERGENCY', 'BOUNDARY_EMERGENCY_STOP'):
            self._log('warn', 'start_in_area(%d) refused: emergency' % area)
            return False, self._end()
        if self.inputs.emergency_stamp is None:
            self._log('warn', 'start_in_area(%d) refused: no /hardware_bridge/emergency message '
                              'yet' % area)
            return False, self._end()
        self._stuck_operator(CMD_START)
        ok = self._start(single=int(area))
        return ok, self._end()

    def clear_resume(self, now):
        """``~/clear_coverage_resume``. Returns ``(success, message, effects)``."""
        self._begin(now)
        if self.mission is not None:
            return False, 'mowing session running; stop it first', self._end()
        self.cursor = ResumeCursor()
        self._fx.append(DeleteResume())
        self._fx.append(PublishResumeAvailable(False))
        self._log('info', 'coverage resume cleared: next START mows from scratch')
        return True, 'coverage resume cleared', self._end()

    def preview_plan(self, area, now):
        """``~/preview_plan``: plan area ``area`` (PREVIEW_ALL = every mowing area,
        PREVIEW_CLEAR = clear the drawn preview) with its current settings, exactly
        as PLANNING would, WITHOUT starting a mission or changing state. The result
        goes to /coverage/full_plan (latched) and ~/preview_summary.
        Returns ``(success, message, effects)``."""
        self._begin(now)
        ok, msg = self._preview_request(int(area))
        self._log('info' if ok else 'warn', 'preview_plan(%d): %s' % (int(area), msg))
        return ok, msg, self._end()

    def mark_sent(self, token, now):
        """The node actually sent request ``token`` at ``now`` (after waiting for the
        server / the Nav2 lifecycle): its watchdog runs from here. Never shortens a
        deadline. No effects."""
        for pend in (self._action, self._service):
            if pend is not None and pend.token == token and pend.timeout > 0:
                pend.deadline = max(pend.deadline, float(now) + pend.timeout)

    def plan_conditioning_job(self, token, outcome, result, now):
        """A /plan_coverage result ``token`` arrived: the inputs of its path conditioning
        (seconds to minutes of pure Python on the RK3588) so the node can run
        ``condition_plan(job)`` WITHOUT holding the FSM lock, then feed the result back with
        ``result[CONDITIONED_KEY]``. None when nothing is to be conditioned. Re-arms the
        plan watchdog for the conditioning time. No effects."""
        a, m = self._action, self.mission
        if (a is None or a.token != token or a.name != ACT_PLAN or a.purpose == 'preview'
                or m is None or not self.p.path_conditioning or outcome != SUCCEEDED):
            return None
        subpaths = self._plan_subpaths(outcome, result or {})
        if not subpaths:
            return None
        self._now = max(self._now, float(now))
        a.deadline = max(a.deadline, float(now) + max(a.timeout, float(self.p.plan_timeout_s)))
        return self._conditioning_job(subpaths)

    def on_action_result(self, token, outcome, result=None, now=None):
        self._begin(self._now if now is None else now)
        a = self._action
        if a is None or a.token != token:
            return self._end()          # stale (cancelled / superseded)
        self._action = None
        self._handle_action(a, outcome, result or {})
        return self._end()

    def on_action_feedback(self, token, detail, now=None):
        """Docking-server feedback detail (e.g. 'searching marker: seen 0.86 m, 3.5 deg')
        -> sub_state, so the GUI shows the marker visibility while docking."""
        self._begin(self._now if now is None else now)
        a = self._action
        if (a is not None and a.token == token and a.name == ACT_DOCK and detail
                and self.phase in DOCK_PHASES and self._dyn is None):
            self.sub_state = str(detail)
        return self._end()

    def on_cutter_height(self, mm, now=None):
        """Commanded deck height from the MCU driver (latched /cutter/height_mm).

        While a mow is running (any phase with a Mission), a height that differs from
        the current area's effective height is a live change (GUI /cutter/set_height):
        it overrides ``cutter_height_mm`` for the rest of THIS mow (later sub-path /
        area plans and blade re-requests) and is NOT persisted to the saved area
        settings. The echo of the mission's own per-area height equals the effective
        height and is ignored. Returns the effects (re-latched active settings).

        2026-10-13: the override is also stored in the resume cursor (``live_height``
        row), so a resumed mow (rain / charge / stop / stuck / e-stop interrupt, which
        rebuilds the Mission) keeps it instead of re-sending the area's saved height at
        the resumed area start. A fresh start (new cursor) drops it: the saved area
        setting is the default again. The GUI's "Save to area" is the only way the area
        setting itself changes.
        """
        self._begin(self._now if now is None else now)
        m = self.mission
        try:
            hmm = self.height_mm(int(mm))
        except (TypeError, ValueError):
            return self._end()
        if m is None:
            return self._end()
        cur = (m.settings or {}).get('cutter_height_mm')
        if m.settings is not None and cur is not None and self.height_mm(cur) == hmm:
            return self._end()
        m.height_override_mm = hmm
        self.cursor.live_height_mm = hmm
        self._persist()
        self._log('info', 'live blade height %d mm (this mow only)' % hmm)
        if m.settings is not None:
            m.settings = dict(m.settings, cutter_height_mm=hmm)
            if m.runs:
                self._fx.append(PublishAreaSettings(self.active_settings()))
        return self._end()

    def on_service_result(self, token, ok, response=None, now=None):
        self._begin(self._now if now is None else now)
        s = self._service
        if s is None or s.token != token:
            return self._end()
        self._service = None
        self._handle_service(s, bool(ok), response or {})
        return self._end()

    def shutdown(self, now):
        self._begin(now)
        if self.mission is not None:
            self._interrupt_mission('shutdown')
        self._cancel_all('shutdown')
        self._blade_off('shutdown')
        self._fx.append(ZeroBurst())
        return self._end()

    def initial_effects(self, now):
        """Effects to run once at node start (latched resume flag, status)."""
        self._begin(now)
        self._fx.append(PublishResumeAvailable(self.cursor.available))
        self._go(self.phase)
        return self._end()
