# SPDX-License-Identifier: GPL-3.0-or-later
"""MissionFSM mixin: Start / resume.

Moved verbatim from mission_fsm.py (2026-10-23 code review stage 1.6)."""

from mower_mission.fsm.common import (ACT_BACKUP, ACT_UNDOCK, CMD_START, DOCK_PHASES,
    HEADING_ALIGNED_SOURCES, HEADING_NOT_ALIGNED, Mission, ResumeCursor, SRV_CHARGING,
    SRV_GET_AREA)  # noqa: F401


class StartMixin:
    """Start / resume."""

    # ==================================================================
    # start / resume
    # ==================================================================
    def _preflight(self):
        i = self.inputs
        self._hcal_wanted = False
        if self._emergency_cause():
            return 'emergency active'
        if i.battery_percent is None:
            return 'battery level unknown'
        if i.battery_percent <= self.p.battery_low_percent:
            return 'battery %.0f%% <= %.0f%%' % (i.battery_percent, self.p.battery_low_percent)
        if self.p.rain_mode == 1 and i.rain:
            return 'rain detected'
        if i.docked and (i.fix_type is None or i.fix_type < self.p.preflight_min_fix_type_docked):
            return 'no GNSS fix (fix_type %s < %d)' % (
                i.fix_type, self.p.preflight_min_fix_type_docked)
        stale = '' if self._at_dock() else self._heading_cog_stale()
        if not self._at_dock() and (not self.heading_ok() or stale):
            # Undocked: Nav2 would steer with an arbitrary IMU heading (or a stale COG one,
            # 2026-10-14). At the dock the undock (straight drive out along the dock yaw)
            # seeds it instead.
            if self._hcal_prior_ok(('pending', 'yaw_ok')):
                # a usable saved heading: calibrate by a short straight drive once RTK is
                # fixed (WAITING_FOR_RTK, _hcal_maybe_start)
                if stale:
                    self._log('info', 'heading cog is %s: calibration drive first' % stale)
                self._hcal_wanted = True
                return ''
            if stale:
                self._log('warn', 'heading cog is %s and no calibration drive: not aligned'
                          % stale)
            return HEADING_NOT_ALIGNED
        return ''

    def _at_dock(self):
        return bool(self.inputs.docked or self.inputs.is_charging)

    def heading_ok(self):
        """Absolute heading available (heading_aligner source dock|cog, status fresh)."""
        if not self.p.require_heading_alignment:
            return True
        i = self.inputs
        if i.heading_stamp is None or self._now - i.heading_stamp > \
                self.p.heading_status_timeout_s:
            return False
        return bool(i.heading_aligned) and i.heading_source in HEADING_ALIGNED_SOURCES

    def _heading_cog_stale(self):
        """Why an aligned 'cog' heading is too old / too poor to plan a START off the dock on
        ('' = fine, or not a cog heading): last COG fix older than heading_max_cog_age_s, or
        status quality in heading_reject_quality (2026-10-14). Dock / rtk / file_verified
        headings are never stale here."""
        p, i = self.p, self.inputs
        if not p.require_heading_alignment or i.heading_source != 'cog' \
                or not i.heading_aligned:
            return ''
        why = []
        age, max_age = i.heading_cog_age, float(p.heading_max_cog_age_s)
        if max_age > 0.0 and age is not None and age > max_age:
            why.append('%.0f s old' % age)
        reject = [str(q) for q in (p.heading_reject_quality or []) if str(q)]
        if i.heading_quality and i.heading_quality in reject:
            why.append('quality %s' % i.heading_quality)
        return ' / '.join(why)

    def _heading_plan_ok(self):
        """heading_ok() and, for a cog heading, not stale (_heading_cog_stale)."""
        return self.heading_ok() and not self._heading_cog_stale()

    def _start(self, single=None, auto=False):
        if self.phase == 'RECORDING':
            self._log('warn', 'START refused: finish or cancel the recording first')
            return False
        if self.mission is not None:
            if single is None or single == self.mission.single_area_target:
                return True             # already mowing: idempotent
            self._log('warn', 'start_in_area(%d) refused: mowing session running' % single)
            return False
        if self.inputs.emergency_stamp is None:
            self._log('warn', 'START refused: no /hardware_bridge/emergency message yet')
            return False
        self._hb_entry = True               # outside at START / resume: inward first
        self._preview_drop('mission started')
        if not auto:
            self._hb_count = 0              # task 0.4 (a): excursions are counted per mission
            self._hb_spots = []
        if self.phase in DOCK_PHASES:
            self._cancel_all('start')
        self._leave_current_mode('start')
        self._go('PREFLIGHT_CHECK', '')
        reason = self._preflight()
        if reason:
            self._log('warn', 'START refused: preflight failed: %s' % reason)
            self._go_idle('preflight failed: %s' % reason)
            return False

        cur = self.cursor
        if single is not None:
            resume = cur.available and cur.single_area_target == single
            if not resume and cur.available:
                self._log('info', 'start_in_area(%d): discarding the previous resume cursor'
                          % single)
        else:
            resume = cur.available
            # an automatic resume (rain / charge) keeps a start_in_area target
            single = cur.single_area_target if (resume or auto) else None
        if not resume:
            self.cursor = ResumeCursor()
            self.cursor.single_area_target = single
        self.mission = Mission(single_area_target=single, resume=resume)
        self._pivot_tries = {}              # pivot re-send counters start fresh on resume
        # a resumed mow keeps its live GUI deck height (on_cutter_height)
        self.mission.height_override_mm = cur.live_height_mm if resume else None
        self._bump_transit_run()            # each drive varies its transit lines
        self._log('info', '%s%s: enumerating mowing areas' % (
            'resuming' if resume else 'starting',
            (' (area %d only)' % single) if single is not None else ''))
        self._go('PREFLIGHT_CHECK', 'loading areas')
        self._enum = {'purpose': 'start', 'index': 0, 'areas': {}, 'count': 0}
        self._call(SRV_GET_AREA, {'index': 0}, purpose='enum')
        return True

    def _resume(self, why):
        self._resume_after = None
        if not self._start(single=None, auto=True):
            self._log('warn', 'automatic resume after %s failed' % why)

    def _areas_loaded(self, areas, count):
        m = self.mission
        m.areas = areas
        m.all_area_count = count
        if m.single_area_target is not None:
            if m.single_area_target not in areas:
                self._abort_start('area %d does not exist or is a navigation area'
                                  % m.single_area_target)
                return
            queue = [m.single_area_target]
        else:
            queue = sorted(areas)
        cur = self.cursor
        queue = [a for a in queue if a not in cur.completed_areas]
        if cur.current_area in queue:   # resume the interrupted area first
            queue.remove(cur.current_area)
            queue.insert(0, cur.current_area)
        if not queue:
            if m.resume:
                self._log('info', 'resume cursor has no areas left: starting fresh')
                self.cursor = ResumeCursor()
                m.height_override_mm = None     # fresh mow: area heights again
                self.cursor.single_area_target = m.single_area_target
                queue = [m.single_area_target] if m.single_area_target is not None \
                    else sorted(areas)
            if not queue:
                self._abort_start('no mowing areas defined')
                return
        m.queue = queue
        self.cursor.current_command = CMD_START
        self._persist()
        self._log('info', 'areas to mow: %s' % queue)
        if self._at_dock():
            # charging but is_docking_done false still means "in the dock": always leave
            # it with the straight undock (it also yields the course-over-ground heading).
            self._undock()
        else:
            self._go('WAITING_FOR_RTK', '')

    def _abort_start(self, reason):
        """Failure before the robot moved: back to idle."""
        self._log('warn', 'START failed: %s' % reason)
        self.mission = None
        self._go_idle('start failed: %s' % reason)

    def _manual_undock(self):
        """UNDOCK command: drive straight off the dock, then idle off the dock."""
        if self.phase not in ('IDLE_DOCKED', 'CHARGING') or not self._at_dock() \
                or self.mission is not None:
            self._log('warn', 'UNDOCK refused: not docked (phase %s)' % self.phase)
            return False
        self._resume_after = None
        self._manual_undocking = True
        self._log('info', 'UNDOCK: driving %.2f m off the dock' % self.p.manual_undock_distance_m)
        self._undock(self.p.manual_undock_distance_m)
        return True

    def _undock(self, distance=None):
        distance = self.p.undock_distance_m if distance is None else float(distance)
        self._blade_off('undock')
        self._go('UNDOCKING', '')
        if self.p.use_docking_server:
            self._start_action(ACT_UNDOCK, {
                'distance_m': distance,
                'speed_mps': self.p.undock_speed_mps,
                'wait_for_rtk': False,
                'rtk_timeout_s': self.p.rtk_timeout_s}, self.p.undock_timeout_s)
        else:
            self._call(SRV_CHARGING, {'enable': False}, track=False)
            self._start_action(ACT_BACKUP, {'distance_m': distance,
                                            'speed_mps': self.p.undock_speed_mps},
                               self.p.undock_timeout_s)
