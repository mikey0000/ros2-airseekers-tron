# SPDX-License-Identifier: GPL-3.0-or-later
"""MissionFSM mixin: Guards (upstream ReactiveSequence), evaluated every tick.

Moved verbatim from mission_fsm.py (2026-10-23 code review stage 1.6)."""

from mower_mission.fsm.common import (MISSION_PHASES, ZeroBurst, geo)  # noqa: F401


class ReactiveGuardsMixin:
    """Guards (upstream ReactiveSequence), evaluated every tick."""

    # ==================================================================
    # guards (upstream ReactiveSequence), evaluated every tick
    # ==================================================================
    def _emergency_cause(self):
        i = self.inputs
        causes = []
        if i.emergency_active:
            causes.append(i.emergency_reason or 'emergency')
        if i.lift:
            causes.append('lift')
        if i.stop_button:
            causes.append('stop button')
        if i.critical_nodes_down:
            causes.append('node down: %s' % i.critical_nodes_down)
        if self._tilt_band() == 'critical':
            causes.append('tilt critical (%s)' % self._tilt_text())
        # Silence watchdog armed only after the first Emergency message (field 2026-10-21
        # 06:27:58: it fired at startup before the MCU driver's first message). Until then
        # motion commands are refused instead (_command / start_in_area: no emergency feed).
        stamp = i.emergency_stamp
        if stamp is not None and self._now - stamp > self.p.emergency_timeout_s:
            causes.append('/hardware_bridge/emergency silent > %.1fs' % self.p.emergency_timeout_s)
        return ', '.join(causes)

    def _guards(self):
        """Returns True when a guard owns this tick (phase logic skipped)."""
        self._stuck_watch()
        self._tilt_watch()
        self._hard_boundary_watch()
        cause = self._emergency_cause()
        if cause:
            if not self._emergency:
                self._emergency = True
                self._enter_emergency(cause)
            elif self.sub_state != cause and self.phase == 'EMERGENCY':
                self._go('EMERGENCY', cause)
            return True
        if self._emergency:
            self._emergency = False
            if self.phase == 'EMERGENCY':
                self._log('info', 'emergency cleared (mowing is not auto-resumed)')
                self._go_idle('emergency cleared')
            return True

        if self._stuck_tick():
            return True
        if self._tilt_tick():
            return True
        if self._hard_boundary_tick():
            return True
        if self._spin_drift_tick():
            return True
        if self._spin_resend_tick():
            return True
        if self._bumper_tick():
            return True

        i = self.inputs
        # Soft / lethal boundary rules (map_server flags, mowing areas only) apply only where
        # the blade may be on; the hard boundary above (task 0.4 a, areas U paths U dock
        # corridor at 0.10 m) covers every autonomous driving phase. PLANNING right
        # after an undock and blade-off TRANSITs (to/from the dock, along drawn
        # paths, the return corridor) are legitimately outside the mowing areas:
        # the Nav2 keepout mask bounds them (2026-10-07: a lethal stop fired in
        # PLANNING on the dock, 3.7 m from the area, before the robot moved).
        if self.phase in ('MOWING', 'BOUNDARY_PAUSED') and self.mission is not None:
            if self._lethal_agrees():          # flag + own-pose cross-check
                self._boundary_stop('lethal boundary violation')
                return True
            # A soft violation pauses only while the blade may be on (MOWING); a
            # blade-off TRANSIT is exactly how the robot gets back inside.
            if i.boundary_violation and self.phase == 'MOWING':
                self._boundary_pause()
                return True

        if self.p.rain_mode == 1 and self.phase in MISSION_PHASES and self.mission is not None:
            if i.rain:
                if self._rain_since is None:
                    self._rain_since = self._now
                if self._now - self._rain_since >= self.p.rain_debounce_s:
                    self._rain_since = None
                    self._log('warn', 'rain detected: docking, then waiting %.0f min'
                              % self.p.rain_delay_minutes)
                    self._interrupt_mission('rain')
                    self._dock('RAIN_DETECTED_DOCKING', 'rain')
                    return True
            else:
                self._rain_since = None

        b = i.battery_percent
        if b is not None and b < self.p.battery_low_percent and self.phase in MISSION_PHASES \
                and self.mission is not None:
            if str(self.p.battery_low_action).strip().lower() == 'dock':
                self._log('warn', 'battery %.0f%% < %.0f%%: docking to charge'
                          % (b, self.p.battery_low_percent))
                self._interrupt_mission('low battery')
                self._dock('LOW_BATTERY_DOCKING', 'battery')
                return True
            self._log('warn', 'battery %.0f%% < %.0f%%: stopping in place (battery_low_action=stop)'
                      % (b, self.p.battery_low_percent))
            self._interrupt_mission('low battery')
            self._cancel_all('low battery')
            self._blade_off('low battery')
            self._fx.append(ZeroBurst())
            self._dock_purpose = None
            self._go_idle('low battery %.0f%%: stopped' % b)
            return True
        return False

    def _boundary_pause(self):
        self._cancel_all('boundary violation')
        self._blade_off('boundary violation')
        self._fx.append(ZeroBurst())
        self._boundary_since = self._now
        # the robot is usually just outside the nav mask edge: let the costmaps
        # refresh during boundary_recover_after_s before any recovery transit.
        self._clear_costmaps()
        pose = self.inputs.pose
        self._log('warn', 'boundary violation at %s: mowing paused, blade off' % (
            '(%.2f, %.2f)' % (pose[0], pose[1]) if pose is not None else 'unknown pose'))
        self.mission.step = None
        self._go('BOUNDARY_PAUSED', 'boundary violation')

    def _enter_emergency(self, cause):
        self._log('error', 'EMERGENCY: %s -> cancel actions, blade off, zero velocity' % cause)
        if self.mission is not None:
            self._interrupt_mission('emergency')
        self._cancel_all('emergency')
        self._blade_off('emergency')
        self._fx.append(ZeroBurst())
        self._track = []
        self._record_poly = None
        self._resume_after = None
        self._dock_purpose = None
        self._go('EMERGENCY', cause)

    def _lethal_agrees(self):
        """map_server's lethal flag cross-checked with the mission's own pose: while that
        pose lies inside the current area's outer ring (+ lethal_agree_margin_m) the flag
        is held off, for at most lethal_disagree_max_s (an area edited smaller mid-mow must
        still stop the robot). No pose / no area: trust the flag (2026-10-07 incident: one
        injected (27, 0) pose latched BOUNDARY_EMERGENCY_STOP)."""
        if not self.inputs.lethal_boundary_violation:
            self._lethal_disagree_since = None
            return False
        m = self.mission
        area = m.areas.get(m.area_idx) if m is not None else None
        pose = self.inputs.pose
        if area is None or pose is None or not (area.get('outer') or []):
            return True
        outer_only = {'outer': area.get('outer')}
        if not geo.inside_area(pose[:2], outer_only, float(self.p.lethal_agree_margin_m)):
            self._lethal_disagree_since = None
            return True
        since = getattr(self, '_lethal_disagree_since', None)
        if since is None:
            self._lethal_disagree_since = since = self._now
            self._log('warn', 'lethal boundary flag ignored: mission pose (%.2f, %.2f) is inside '
                      'area %s' % (pose[0], pose[1], m.area_idx))
        return self._now - since >= float(self.p.lethal_disagree_max_s)

    def _boundary_stop(self, why):
        self._log('error', 'BOUNDARY_EMERGENCY_STOP: %s (STOP or RESET_EMERGENCY to clear)' % why)
        if self.mission is not None:
            self._interrupt_mission(why)
        self._cancel_all(why)
        self._blade_off(why)
        self._fx.append(ZeroBurst())
        self._go('BOUNDARY_EMERGENCY_STOP', why)
