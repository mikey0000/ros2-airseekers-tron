# SPDX-License-Identifier: GPL-3.0-or-later
"""MissionFSM mixin: Stuck guard (slip_detector /stuck -> escape / STUCK_NEEDS_HELP).

Moved verbatim from mission_fsm.py (2026-10-23 code review stage 1.6)."""

from mower_mission.fsm.common import (CMD_HOME, CMD_MANUAL_MOW, CMD_RESET_EMERGENCY, CMD_START,
    CMD_STOP, CMD_UNDOCK, DOCK_PHASES, DriveCmd, RecordIncident, ResetStuckGuard,
    SRV_CLEAR_COSTMAPS, STUCK_NEEDS_HELP_SUB, STUCK_PHASES, SuppressStuckGuard, ZeroBurst,
    math)  # noqa: F401


class StuckGuardMixin:
    """Stuck guard (slip_detector /stuck -> escape / STUCK_NEEDS_HELP)."""

    # ==================================================================
    # stuck guard (mower_control slip_detector /stuck -> escape / STUCK_NEEDS_HELP)
    # ==================================================================
    def _stuck_steps(self):
        v = abs(float(self.p.stuck_step_speed))
        w = v * math.radians(float(self.p.stuck_arc_deg)) / max(0.05, float(self.p.stuck_step_m))
        return [(-v, 0.0, 'reverse'), (v, w, 'forward arc'), (-v, -w, 'reverse opposite arc')]

    def _stuck_operator(self, cmd):
        """Operator STOP / START / MANUAL / HOME / UNDOCK: clear STUCK_NEEDS_HELP's
        history and the slip_detector latch so the next run starts clean."""
        if self.phase == 'STUCK_NEEDS_HELP' and cmd in (CMD_STOP, CMD_START, CMD_MANUAL_MOW,
                                                         CMD_HOME, CMD_UNDOCK,
                                                         CMD_RESET_EMERGENCY):
            self._log('info', 'stuck: cleared by the operator (command %d)' % cmd)
            self._stuck_hist = []
            self._tilt_hist = []
            self._hb_count = 0
            self._hb_spots = []
            self._stuck_ignore_until = self._now + float(self.p.stuck_ignore_after_s)
            self._fx.append(ResetStuckGuard())
            if cmd != CMD_STOP and cmd != CMD_RESET_EMERGENCY:
                self._go_idle('stuck cleared')

    def _stuck_watch(self):
        """Abort an escape when something else took over (emergency, STOP, phase change)."""
        k = self._stk
        if k is None:
            return
        if self._emergency_cause() or self.phase != k['phase']:
            self._log('warn', 'stuck escape aborted (%s)' % (
                self._emergency_cause() or 'state %s' % self.phase))
            self._stk = None
            self._fx.append(SuppressStuckGuard(False))
            self._fx.append(DriveCmd(0.0, 0.0))

    def _stuck_tick(self):
        """Returns True while the stuck guard owns the tick."""
        if self._stk is not None:
            self._stuck_escape_tick()
            return True
        i = self.inputs
        if i.pose is not None:              # how far the robot got from each escaped spot
            for h in self._stuck_hist:
                h[3] = max(h[3], math.hypot(h[1] - i.pose[0], h[2] - i.pose[1]))
        if not (self.p.stuck_guard and i.stuck and self.phase in STUCK_PHASES) \
                or self._now < self._stuck_ignore_until:
            return False
        if self.phase in ('TRANSIT', 'MOWING') and self.mission is None:
            return False
        if self.phase in DOCK_PHASES and self._at_dock():
            # 2026-10-22 01:13:34: the contacts closed in the same tick as a /stuck latch from
            # the final push against the charger, and the escape drove the robot off the
            # dock. On the contacts, wheels turning without progress is the dock itself.
            self._log('info', 'stuck latch ignored: on the dock contacts (%s)' % self.phase)
            self._stuck_ignore_until = self._now + float(self.p.stuck_ignore_after_s)
            self._fx.append(ResetStuckGuard())
            return False
        pose = i.pose
        if pose is None:
            self._stuck_needs_help('stuck, no pose', (0.0, 0.0))
            return True
        horizon = self._now - float(self.p.stuck_recovery_period_s)
        self._stuck_hist = [h for h in self._stuck_hist if h[0] >= horizon]
        same = [h for h in self._stuck_hist
                if math.hypot(h[1] - pose[0], h[2] - pose[1]) <= self.p.stuck_same_spot_m
                and h[3] < float(self.p.stuck_same_spot_left_m)]
        if same:
            self._stuck_needs_help('stuck again at the same spot', pose)
            return True
        if len(self._stuck_hist) >= int(self.p.stuck_max_recoveries):
            self._stuck_needs_help('stuck %d times within %.0f s' % (
                len(self._stuck_hist) + 1, self.p.stuck_recovery_period_s), pose)
            return True
        self._log('error', 'STUCK at (%.2f, %.2f) in %s: cancel, blade off, escape %d/%d'
                  % (pose[0], pose[1], self.phase, len(self._stuck_hist) + 1,
                     int(self.p.stuck_max_recoveries)))
        m = self.mission
        self._stk = {'phase': self.phase, 'origin': (float(pose[0]), float(pose[1])),
                     'stage': 'pause', 't': self._now, 'step': -1,
                     'dock_purpose': self._dock_purpose}
        self._cancel_all('stuck')
        self._blade_off('stuck')
        self._fx.append(ZeroBurst())
        self._fx.append(SuppressStuckGuard(True))
        if m is not None:
            if m.step == 'follow':
                m.start_local = self._track_progress()
            m.step = None
        self.sub_state = 'stuck: stopping'
        return True

    def _stuck_escape_tick(self):
        k, pose, now = self._stk, self.inputs.pose, self._now
        if k['phase'] in DOCK_PHASES and self._at_dock():
            # 2026-10-22 01:13:36: escape step 1 (reverse) seated the robot on the charger
            # (charging 3.3 A), then step 2 drove it off again.
            self._stk = None
            self._fx.append(DriveCmd(0.0, 0.0))
            self._fx.append(SuppressStuckGuard(False))
            self._fx.append(ResetStuckGuard())
            self._stuck_ignore_until = now + float(self.p.stuck_ignore_after_s)
            self._log('info', 'stuck escape ended: dock contacts reported (%s)' % k['phase'])
            self._dock_purpose = k['dock_purpose']
            self._dock_done()
            return
        if pose is None:
            self._stk = None
            self._fx.append(SuppressStuckGuard(False))
            self._stuck_needs_help('stuck, pose lost during the escape', k['origin'])
            return
        if k['stage'] == 'pause':
            if now - k['t'] < self.p.stuck_pause_s:
                return
            self._stuck_next_step(pose)
            return
        v, w, name = self._stuck_steps()[k['step']]
        d = math.hypot(pose[0] - k['start'][0], pose[1] - k['start'][1])
        if d >= k['best'] + self.p.stuck_step_progress_m:
            k['best'], k['best_t'] = d, now
        done = d >= self.p.stuck_step_m
        stalled = now - k['best_t'] > self.p.stuck_step_progress_s
        if not done and not stalled:
            self._fx.append(DriveCmd(v, w))
            return
        self._fx.append(DriveCmd(0.0, 0.0))
        moved = math.hypot(pose[0] - k['origin'][0], pose[1] - k['origin'][1])
        self._log('info' if done else 'warn', 'stuck escape step %d (%s): %s, %.2f m from the '
                  'stuck spot' % (k['step'] + 1, name, 'done' if done else
                                  'no progress in %.0f s' % self.p.stuck_step_progress_s, moved))
        if done or moved > self.p.stuck_escape_m:
            self._stuck_escaped(moved)
        elif k['step'] + 1 < len(self._stuck_steps()):
            self._stuck_next_step(pose)
        else:
            self._stk = None
            self._fx.append(SuppressStuckGuard(False))
            self._stuck_needs_help('escape failed', k['origin'])

    def _stuck_next_step(self, pose):
        k = self._stk
        k['stage'] = 'drive'
        k['step'] += 1
        k['start'] = (float(pose[0]), float(pose[1]))
        k['best'], k['best_t'] = 0.0, self._now
        v, w, name = self._stuck_steps()[k['step']]
        self.sub_state = 'stuck: escape %d/3 (%s)' % (k['step'] + 1, name)
        self._log('info', 'stuck escape step %d: %s %.2f m at %.2f m/s'
                  % (k['step'] + 1, name, self.p.stuck_step_m, abs(v)))
        self._fx.append(DriveCmd(v, w))

    def _stuck_escaped(self, moved):
        k, self._stk = self._stk, None
        ox, oy = k['origin']
        self._stuck_hist.append([self._now, ox, oy, float(moved)])
        self._stuck_ignore_until = self._now + float(self.p.stuck_ignore_after_s)
        self._fx.append(SuppressStuckGuard(False))
        self._fx.append(RecordIncident('stuck', ox, oy, 'stuck guard escape (%s, %.2f m)'
                                       % (k['phase'], moved)))
        self._call(SRV_CLEAR_COSTMAPS, {'global': True}, track=False)
        self._log('info', 'stuck: escaped (%.2f m): costmaps cleared, terrain incident at '
                  '(%.2f, %.2f), resuming %s' % (moved, ox, oy, k['phase']))
        if k['phase'] in DOCK_PHASES:
            self._dock(k['phase'], k['dock_purpose'], 'resuming after stuck escape')
            return
        m = self.mission
        if m is None or m.area_idx is None or not m.subpaths:
            self._go_idle('stuck escape: nothing to resume')
            return
        m.detour = None
        m.start_local = self._track_progress()
        self._dispatch()

    def _skip_failing_piece(self):
        """resume_skip_failing: before a STUCK interrupt while mowing, record the failing piece
        as skipped and move the cursor past it (START never re-runs the same pivot first)."""
        m = self.mission
        if not self.p.resume_skip_failing or self.phase != 'MOWING' or m.stretch is not None \
                or m.area_idx is None or not m.subpaths or m.sub_i >= len(m.subpaths) or \
                m.step not in ('corner_pivot', 'pivot_backout', 'follow'):
            return
        last = self._seg_last()
        frm = min(self._track_progress() if m.step == 'follow' else m.start_local, last)
        end = self._skip_end(frm, float(self.p.edge_blocked_skip_m))
        if end <= frm:
            return
        self._record_skip(frm, end)
        self._log('warn', 'resume: area %d sub-path %d poses %d -> %d skipped (failing piece, '
                  're-tried at the area end)' % (m.area_idx, m.sub_i, frm, end))
        m.start_local, m.step, m.detour = end, None, None

    def _stuck_needs_help(self, why, pose):
        x, y = float(pose[0]), float(pose[1])
        self._log('error', 'STUCK_NEEDS_HELP: %s at (%.2f, %.2f): motion stopped, blade off '
                  '(STOP / START / MANUAL to clear)' % (why, x, y))
        if self.mission is not None:
            self._skip_failing_piece()
            self._interrupt_mission('stuck')
        self._cancel_all('stuck')
        self._blade_off('stuck')
        self._fx.append(ZeroBurst())
        self._dock_purpose = None
        self._resume_after = None
        self._stk = None
        self._go('STUCK_NEEDS_HELP', STUCK_NEEDS_HELP_SUB % (x, y))
