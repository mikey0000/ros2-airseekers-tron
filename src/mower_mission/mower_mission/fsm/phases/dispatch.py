# SPDX-License-Identifier: GPL-3.0-or-later
"""MissionFSM mixin: Per-phase tick logic.

Moved verbatim from mission_fsm.py (2026-10-23 code review stage 1.6)."""

from mower_mission.fsm.common import (ACT_BACKUP, ACT_DOCK, ACT_DRIVE, ACT_FOLLOW, ACT_NAV,
    ACT_PLAN, ACT_UNDOCK, CancelActions, DISPLAY_NAMES, DOCK_PHASES, IDLE_NAMES, TIMEOUT, geo)  # noqa: F401


class PhaseDispatchMixin:
    """Per-phase tick logic."""

    # ==================================================================
    # per-phase tick logic
    # ==================================================================
    def _run_phase(self):
        ph, now, i = self.phase, self._now, self.inputs
        a = self._action
        if a is not None and now > a.deadline:
            self._log('warn', '%s timed out after %.0fs: cancelling'
                      % (a.name, self._timeout_of(a.name)))
            self._fx.append(CancelActions('%s watchdog' % a.name))
            self._action = None
            self._handle_action(a, TIMEOUT, {})
            return
        s = self._service
        if s is not None and now > s.deadline:
            self._service = None
            self._log('warn', '%s timed out' % s.name)
            self._handle_service(s, False, {})
            return
        if self._ranged_static():
            self._last_static_t = now
        self._pivot_track(now)
        if self._edge_block_tick():
            return
        if self._dynamic_tick():
            return
        if self._loc_hold_tick():
            return
        if self._blocked_tick():
            return
        if self._route_progress_tick():
            return

        if ph == 'MANUAL_MOWING':
            self._manual_tick()
            return

        if ph in IDLE_NAMES:
            if self._resume_after == 'charge':
                b = i.battery_percent
                full = self._resume_charge_percent()
                if b is not None and b >= full:
                    self._log('info', 'battery %.0f%% >= %.0f%%: resuming mowing' % (b, full))
                    self._resume('charge')
                elif ph != 'CHARGING':
                    self._go('CHARGING')
            elif ph != self._idle_name():
                self._go(self._idle_name())
        elif ph in DISPLAY_NAMES:
            if now - self._phase_since >= self.p.result_display_s:
                self._go_idle()
        elif ph in DOCK_PHASES and a is not None and a.purpose == 'dock':
            if self._at_dock():
                if self._dock_contact_since is None:
                    self._dock_contact_since = now
                    self._log('info', 'dock contacts reported: waiting for the docking result')
                elif now - self._dock_contact_since >= self.p.dock_contact_grace_s:
                    self._log('warn', 'docked for %.0f s but the docking action is still '
                              'running: cancelling it' % self.p.dock_contact_grace_s)
                    self._fx.append(CancelActions('docked (contacts)'))
                    self._action = None
                    self._dock_contact_since = None
                    self._dock_done()
            else:
                self._dock_contact_since = None
        elif ph == 'RAIN_WAITING':
            if i.rain and self.p.rain_mode == 1:
                self._rain_clear_since = None
            elif self._rain_clear_since is None:
                self._rain_clear_since = now
            elif now - self._rain_clear_since >= self.p.rain_delay_minutes * 60.0:
                self._log('info', 'dry for %.0f min: resuming mowing' % self.p.rain_delay_minutes)
                self._resume('rain')
        elif ph == 'WAITING_FOR_RTK':
            if self._hcal is not None:
                self._hcal_tick()
                return
            if i.fix_type is not None and i.fix_type >= self.p.rtk_fix_type:
                # a START that came through preflight for a calibration drive plans only on a
                # fresh heading; the dock path (undock seeds 'dock') is unchanged
                if self._heading_plan_ok() if self._hcal_wanted else self.heading_ok():
                    self._log('info', 'RTK fixed, heading aligned (%s): planning'
                              % i.heading_source)
                    self._next_area()
                    return
                if self._hcal_wanted:
                    self._hcal_maybe_start()
                    return
                if self._heading_wait_since is None:
                    self._heading_wait_since = now
                    self._log('info', 'RTK fixed: waiting for heading alignment')
                    self._go('WAITING_FOR_RTK', 'waiting for heading alignment')
                elif now - self._heading_wait_since > self.p.heading_wait_s:
                    self._heading_failed()
            elif now - self._phase_since > self.p.rtk_timeout_s:
                if self._hcal_wanted and not self._heading_plan_ok():
                    # no dock / Nav2 transit with an unverified heading
                    self._hcal_fail('no RTK fixed within %.0fs' % self.p.rtk_timeout_s)
                else:
                    self._mission_failed('no RTK fixed within %.0fs' % self.p.rtk_timeout_s)
            else:
                self._hcal_rtk_since = None
        elif ph in ('MOWING', 'TRANSIT') and self.mission is not None \
                and self.mission.step == 'retry_wait':
            if now >= self.mission.retry_at:
                self._retry_fire()
        elif ph == 'MOWING' and self.mission is not None:
            if self.mission.step == 'follow':
                self._track_progress()
                if self._leg_watch():
                    return
            self._tick_blade()
        elif ph == 'TRANSIT' and self.mission is not None and self.mission.step == 'transit':
            if self._transit_nav_progress_tick():
                return
            self._fallback_corridor_tick()
        elif ph == 'BOUNDARY_PAUSED' and self.mission is not None:
            m = self.mission
            if not i.boundary_violation:
                self._log('info', 'boundary violation cleared: resuming the sub-path')
                m.start_local = self._track_progress()
                self._dispatch()
            elif now - self._boundary_since >= self.p.boundary_recover_after_s:
                if m.boundary_recoveries >= self.p.boundary_max_recoveries:
                    self._boundary_stop('still outside the boundary after %d recovery transits'
                                        % m.boundary_recoveries)
                    return
                m.boundary_recoveries += 1
                m.recovering, m.recovery_fails = True, 0
                sp = m.subpaths[m.sub_i]
                near = self._track_progress()
                m.start_local = geo.advance_index(sp, near, self.p.boundary_recovery_advance_m)
                self._log('warn', 'still outside the boundary: blade-off recovery transit %d/%d '
                                  'to sub-path pose %d' % (m.boundary_recoveries,
                                                           self.p.boundary_max_recoveries,
                                                           m.start_local))
                self._dispatch(force_transit=True)
        elif ph == 'RECORDING':
            self._tick_record()

    def _timeout_of(self, name):
        if self._hcal is not None and name in (ACT_DRIVE, ACT_BACKUP):
            return self.p.heading_calibration_timeout_s
        return {ACT_PLAN: self.p.plan_timeout_s, ACT_UNDOCK: self.p.undock_timeout_s,
                ACT_BACKUP: self.p.undock_timeout_s, ACT_NAV: self.p.transit_timeout_s,
                ACT_FOLLOW: self.p.follow_timeout_s,
                ACT_DOCK: self.p.dock_action_timeout_s}.get(name, 60.0)   # ACT_SPIN: per goal
