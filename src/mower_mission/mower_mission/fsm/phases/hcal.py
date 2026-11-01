# SPDX-License-Identifier: GPL-3.0-or-later
"""MissionFSM mixin: Heading calibration drive.

Moved verbatim from mission_fsm.py (2026-10-23 code review stage 1.6)."""

from mower_mission.fsm.common import (ABORTED, ACT_BACKUP, ACT_DRIVE, CANCELED, CMD_START,
    CancelActions, HEADING_NOT_ALIGNED, PublishAreaSettings, REJECTED, SUCCEEDED, UNAVAILABLE,
    ZeroBurst, geo, math, subpath_offsets)  # noqa: F401


class HeadingCalMixin:
    """Heading calibration drive."""

    # ------------------------------------------------------------------
    # heading calibration drive (2026-10-13; Params.heading_calibration_*)
    # ------------------------------------------------------------------
    def _hcal_enabled(self):
        """Calibration drives are on and the aligner status is fresh."""
        p, i = self.p, self.inputs
        if not (p.heading_calibration_drive and p.require_heading_alignment):
            return False
        return i.heading_stamp is not None and \
            self._now - i.heading_stamp <= p.heading_status_timeout_s

    def _hcal_prior_ok(self, checks=('yaw_ok', 'pose_prior')):
        """A calibration drive may use the aligner's saved 'file' heading as its prior:
        enabled, status fresh, source file, continuity check state in ``checks`` (yaw_ok =
        the IMU kept integrating since the save, only the position is unproven; pose_prior =
        the IMU restarted but the robot sits where the record was saved, 2026-11-01); or an
        aligned 'cog' heading that is stale for planning (_heading_cog_stale, 2026-10-14)."""
        i = self.inputs
        if not self._hcal_enabled():
            return False
        if i.heading_source == 'cog':
            return bool(self._heading_cog_stale())
        return i.heading_source == 'file' and i.heading_prior in checks

    def _hcal_blind_ok(self):
        """No usable prior, but a blind drive is allowed: enabled, status fresh, heading not
        aligned (2026-11-01). The disc check (_hcal_disc_clear) decides at the drive."""
        return bool(self.p.heading_calibration_blind) and self._hcal_enabled() \
            and not self.heading_ok()

    def _hcal_prior_text(self):
        i = self.inputs
        if i.heading_source == 'cog':
            return 'cog %s' % (self._heading_cog_stale() or 'fresh')
        return i.heading_prior or 'unchecked'

    def _hcal_reach(self, direction, dist=None):
        """Length of the straight the robot body sweeps: drive + footprint ahead of it."""
        p = self.p
        if dist is None:
            dist = float(p.heading_calibration_distance_m)
        return dist + float(p.robot_front_m if direction > 0 else p.robot_rear_edge_m)

    def _hcal_disc_radius(self):
        """Radius the body can sweep in a blind drive of unknown direction: the drive plus
        the body's front corner radius."""
        p = self.p
        return float(p.heading_calibration_blind_distance_m) + math.hypot(
            float(p.robot_front_m), float(p.robot_width_m) / 2.0)

    def _hcal_disc_clear(self, pose):
        """The disc of _hcal_disc_radius around ``pose`` lies inside one mowing area (outside
        its keepout holes): a blind drive cannot leave it whatever the true heading. Returns
        the area index or None."""
        m = self.mission
        if m is None or not m.areas or pose is None:
            return None
        reach = self._hcal_disc_radius()
        rings = max(1, int(math.ceil(reach / 0.1)))
        pts = [(pose[0], pose[1])]
        for k in range(1, rings + 1):
            r = reach * k / rings
            n = max(8, int(math.ceil(2.0 * math.pi * r / 0.1)))
            pts.extend((pose[0] + r * math.cos(2.0 * math.pi * j / n),
                        pose[1] + r * math.sin(2.0 * math.pi * j / n)) for j in range(n))
        for idx in sorted(m.areas):
            if all(geo.inside_area(pt, m.areas[idx]) for pt in pts):
                return idx
        return None

    def _hcal_clear(self, pose, direction):
        """The straight ``direction`` (+1 ahead, -1 behind) from ``pose`` by the PRIOR
        heading, widened by the robot width and +-heading_calibration_cone_deg, lies inside
        one mowing area (outside its keepout holes). Returns the area index or None."""
        m = self.mission
        if m is None or not m.areas or pose is None:
            return None
        p = self.p
        base = pose[2] + (0.0 if direction > 0 else math.pi)
        reach = self._hcal_reach(direction)
        hw = float(p.robot_width_m) / 2.0
        cone = math.radians(float(p.heading_calibration_cone_deg))
        n = max(2, int(math.ceil(reach / 0.1)))
        pts = []
        for da in (-cone, 0.0, cone):
            h = base + da
            c, s_ = math.cos(h), math.sin(h)
            for off in (-hw, 0.0, hw):
                x0, y0 = pose[0] - s_ * off, pose[1] + c * off
                pts.extend((x0 + c * reach * k / n, y0 + s_ * reach * k / n)
                           for k in range(n + 1))
        for idx in sorted(m.areas):
            if all(geo.inside_area(pt, m.areas[idx]) for pt in pts):
                return idx
        return None

    def _hcal_line(self, pose):
        """Obstacle corridor of the calibration drive: the rest of its straight."""
        h = self._hcal
        base = h['yaw'] + (0.0 if h['dir'] > 0 else math.pi)
        left = max(0.0, self._hcal_reach(h['dir'], h['dist']) - geo.dist(pose, h['start']))
        return [(pose[0], pose[1]),
                (pose[0] + left * math.cos(base), pose[1] + left * math.sin(base))]

    def _hcal_maybe_start(self):
        """RTK fixed, heading not aligned, START came off the dock with a saved heading (or
        none, blind drive): give the continuity check heading_calibration_settle_s, then
        drive."""
        now, p, i = self._now, self.p, self.inputs
        if self._hcal_rtk_since is None:
            self._hcal_rtk_since = now
            self._go('WAITING_FOR_RTK', 'RTK fixed: checking the saved heading')
            return
        if now - self._hcal_rtk_since < float(p.heading_calibration_settle_s):
            return
        pose = i.pose
        v = float(p.heading_calibration_speed_mps)
        if self._hcal_prior_ok():
            for direction in (1, -1):
                area = self._hcal_clear(pose, direction)
                if area is not None:
                    break
            else:
                self._hcal_fail('no clear %.1f m straight ahead or behind inside a mowing area'
                                % float(p.heading_calibration_distance_m), drove=False)
                return
            d = float(p.heading_calibration_distance_m)
            word = 'forward' if direction > 0 else 'backward'
            what = 'saved heading %s' % self._hcal_prior_text()
        elif self._hcal_blind_ok():
            # no usable prior (2026-11-01): the whole disc must be clear, then creep forward
            area = self._hcal_disc_clear(pose)
            if area is None:
                self._hcal_fail('no usable heading prior (%s %s) and no clear %.1f m disc '
                                'around the robot inside a mowing area'
                                % (i.heading_source, self._hcal_prior_text(),
                                   self._hcal_disc_radius()), drove=False)
                return
            direction, d = 1, float(p.heading_calibration_blind_distance_m)
            word = 'forward blind'
            what = 'no usable prior (%s %s), %.1f m disc clear' % (
                i.heading_source, self._hcal_prior_text(), self._hcal_disc_radius())
        else:
            self._hcal_fail('saved heading not usable (%s %s)'
                            % (i.heading_source, self._hcal_prior_text()), drove=False)
            return
        if self.blade_on:
            self._blade_off('heading calibration')
        self._hcal = {'dir': direction, 'dist': d, 'start': (pose[0], pose[1]),
                      'yaw': pose[2], 'done_at': None, 'outcome': None}
        self._log('info', 'heading calibration: %s, driving %.1f m %s at %.2f m/s inside '
                          'area %d (blade off)' % (what, d, word, v, area))
        self._go('WAITING_FOR_RTK', 'calibrating heading: driving %.1f m %s' % (d, word))
        self._start_action(ACT_DRIVE if direction > 0 else ACT_BACKUP,
                           {'distance_m': d, 'speed_mps': v},
                           p.heading_calibration_timeout_s, purpose='heading_cal')

    def _hcal_tick(self):
        h, i = self._hcal, self.inputs
        if self._heading_plan_ok():
            if self._action is not None:
                self._fx.append(CancelActions('heading aligned'))
                self._action = None
            pose = i.pose
            moved = geo.dist(pose, h['start']) if pose is not None else float('nan')
            self._hcal = None
            self._log('info', 'heading calibrated (%s) after %.2f m' % (i.heading_source, moved))
            reason = self._preflight()
            if reason:
                self._hcal_fail('preflight after the drive: %s' % reason)
                return
            if i.fix_type is not None and i.fix_type >= self.p.rtk_fix_type:
                self._log('info', 'RTK fixed, heading aligned (%s): planning' % i.heading_source)
                self._next_area()
            else:
                self._go('WAITING_FOR_RTK', '')
            return
        if h['done_at'] is None:
            kind = self._policy()
            if kind != 'none':
                self._hcal_fail('stopped: %s %s in the way' % (
                    kind, i.obstacle_class or 'obstacle'))
            return
        if self._now - h['done_at'] > float(self.p.heading_calibration_status_wait_s):
            out = h['outcome']
            why = 'no COG alignment after the drive' if out == SUCCEEDED else \
                'drive %s%s' % (out, ' (obstacle / costmap)' if out == ABORTED else '')
            self._hcal_fail(why)

    def _hcal_result(self, outcome):
        h = self._hcal
        if h is None:
            return
        if outcome in (REJECTED, UNAVAILABLE, CANCELED) and not self._heading_plan_ok():
            self._hcal_fail('drive %s' % outcome)
            return
        h['done_at'], h['outcome'] = self._now, outcome
        self._go('WAITING_FOR_RTK', 'calibrating heading: waiting for the COG fix')

    def _hcal_fail(self, why, drove=True):
        """Refused or failed: stop (blade off, zero velocity) and stay idle where it is.
        Never dock / navigate here: Nav2 would steer with the unverified heading."""
        text = '%s (heading calibration: %s)' % (HEADING_NOT_ALIGNED, why)
        self._log('error' if drove else 'warn', 'START: %s' % text)
        self._interrupt_mission(text)
        self._cancel_all(text)
        self._blade_off(text)
        if drove:
            self._fx.append(ZeroBurst())
        self._hcal = None
        self._hcal_wanted = False
        self._go_idle(text)
        self._last_status = None

    def _mission_failed(self, reason):
        self._log('error', 'mowing aborted: %s' % reason)
        self._interrupt_mission(reason)
        self._cancel_all(reason)
        self._blade_off(reason)
        if self.inputs.docked:
            self._go_idle('mowing aborted: %s' % reason)
        else:
            self._dock('COVERAGE_FAILED_DOCKING', 'failed', sub=reason)

    def _interrupt_mission(self, why):
        """Store the precise resume cursor and drop the in-memory session."""
        m = self.mission
        if m is None:
            return
        self._blade_off(why)
        if m.area_idx is not None and m.subpaths and m.sub_i < len(m.subpaths):
            sp = m.subpaths[m.sub_i]
            local = m.start_local
            if m.step == 'follow':
                local = self._track_progress()
            elif m.detour is not None:
                local = m.detour['from']     # the skip is recorded only once around
            offs, _ = subpath_offsets(m.subpaths)
            ac = self.cursor.area(m.area_idx)
            ac.resume_index = offs[m.sub_i] + min(local, len(sp) - 1)
            self.cursor.current_area = m.area_idx
        self.cursor.current_command = CMD_START
        self._persist()
        self._log('info', 'mowing interrupted (%s): resume cursor saved' % why)
        if m.settings is not None:
            self._fx.append(PublishAreaSettings({}))
        self.mission = None
