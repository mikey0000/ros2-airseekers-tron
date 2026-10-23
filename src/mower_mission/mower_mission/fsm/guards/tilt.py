# SPDX-License-Identifier: GPL-3.0-or-later
"""MissionFSM mixin: Tilt guard (tilt_monitor /tilt/status).

Moved verbatim from mission_fsm.py (2026-10-23 code review stage 1.6)."""

from mower_mission.fsm.common import (AREA_SETTINGS_DEFAULTS, DOCK_PHASES, DriveCmd,
    PARAM_NODE_CONTROLLER, RecordIncident, SRV_SET_PARAMS, STUCK_PHASES, SUB_MANUAL_BLADE_OFF,
    SuppressStuckGuard, ZeroBurst, math)  # noqa: F401


class TiltGuardMixin:
    """Tilt guard (tilt_monitor /tilt/status)."""

    # ==================================================================
    # tilt guard (mower_control tilt_monitor /tilt/status), 2026-10-09
    # ==================================================================
    # The monitor owns filtering, debounce and hysteresis; this only acts on its band.
    # critical -> _emergency_cause; caution -> controller speed cap; limit -> back out
    # along the track just driven (pure pursuit on /cmd_vel_emergency, either direction),
    # 'steep' terrain incident, then resume like the stuck escape (mowing: detour past it).
    def _tilt_band(self):
        """The monitor's band, or 'unknown' when the guard is off or the status is stale."""
        i = self.inputs
        if not self.p.tilt_guard or i.tilt_stamp is None \
                or self._now - i.tilt_stamp > float(self.p.tilt_status_timeout_s):
            return 'unknown'
        return i.tilt_band

    def _tilt_text(self):
        i = self.inputs
        r, p = i.tilt_roll_deg, i.tilt_pitch_deg
        if i.tilt_axis == 'pitch' and p is not None:
            return 'pitch %.0f deg' % p
        if r is not None:
            return 'roll %.0f deg' % r
        return 'tilt'

    def _tilt_watch(self):
        """Abort a back-out when something else took over (emergency, STOP, phase change)."""
        k = self._tob
        if k is None:
            return
        if self._emergency_cause() or self.phase != k['phase']:
            self._log('warn', 'steep-slope back-out aborted (%s)' % (
                self._emergency_cause() or 'state %s' % self.phase))
            self._tob = None
            self._fx.append(SuppressStuckGuard(False))
            self._fx.append(DriveCmd(0.0, 0.0))

    def _tilt_trail_add(self):
        pose = self.inputs.pose
        if pose is None or self._tob is not None:
            return
        tr = self._tilt_trail
        if tr and math.hypot(pose[0] - tr[-1][0], pose[1] - tr[-1][1]) \
                < float(self.p.tilt_trail_step_m):
            return
        tr.append((float(pose[0]), float(pose[1])))
        n = int(float(self.p.tilt_trail_m) / max(0.01, float(self.p.tilt_trail_step_m))) + 1
        del tr[:-n]

    def _tilt_speed(self, band):
        """Caution cap on the coverage + transit controllers; restored when back to ok.
        'unknown' keeps the current state (no flapping on a lost status)."""
        if band == 'unknown':
            return
        want = band in ('caution', 'limit', 'critical')
        if want == self._tilt_capped:
            return
        self._tilt_capped = want
        cap = float(self.p.tilt_caution_speed_mps)
        m = self.mission
        params = {}
        if want:
            params = dict(self.coverage_speed_params(cap))
            params[self.p.tilt_transit_speed_param] = cap
            self._log('warn', 'slope caution (%s): controller speed capped at %.2f m/s'
                      % (self._tilt_text(), cap))
        else:
            params = {self.p.tilt_transit_speed_param: float(self.p.tilt_transit_speed_mps)}
            if m is not None and m.settings:
                params.update(self.coverage_speed_params(min(
                    float(m.settings.get('cut_speed_mps', AREA_SETTINGS_DEFAULTS['cut_speed_mps'])),
                    float(self.p.cut_speed_max_mps))))
            self._log('info', 'slope ok: controller speed restored')
        if self.p.set_cut_speed:
            self._call(SRV_SET_PARAMS, {'node': PARAM_NODE_CONTROLLER, 'params': params},
                       track=False)

    def _tilt_tick(self):
        """Returns True while the tilt guard owns the tick."""
        if not self.p.tilt_guard:
            return False
        self._tilt_trail_add()
        band = self._tilt_band()
        self._tilt_speed(band)
        if self._tob is not None:
            self._tilt_backout_tick(band)
            return True
        if band != 'limit':
            return False
        if self.phase == 'MANUAL_MOWING':
            if self.blade_on:               # the operator drives; only the blade is ours
                self._log('warn', 'manual: blade off (steep slope, %s)' % self._tilt_text())
                self._blade_off('manual: steep slope')
                self._go('MANUAL_MOWING', SUB_MANUAL_BLADE_OFF)
            return False
        if self.phase not in STUCK_PHASES:
            return False
        if self.phase in ('TRANSIT', 'MOWING') and self.mission is None:
            return False
        pose = self.inputs.pose
        if pose is None:
            self._tilt_needs_help('steep slope, no pose', (0.0, 0.0))
            return True
        horizon = self._now - float(self.p.tilt_backout_period_s)
        self._tilt_hist = [h for h in self._tilt_hist if h[0] >= horizon]
        if any(math.hypot(h[1] - pose[0], h[2] - pose[1]) <= float(self.p.tilt_same_spot_m)
               for h in self._tilt_hist):
            self._tilt_needs_help('steep again at the same spot', pose)
            return True
        if len(self._tilt_hist) >= int(self.p.tilt_max_backouts):
            self._tilt_needs_help('steep %d times within %.0f s' % (
                len(self._tilt_hist) + 1, self.p.tilt_backout_period_s), pose)
            return True
        ox, oy = float(pose[0]), float(pose[1])
        self._log('error', 'STEEP SLOPE (%s) at (%.2f, %.2f) in %s: cancel, blade off, '
                  'back out' % (self._tilt_text(), ox, oy, self.phase))
        m = self.mission
        follow = m is not None and m.step == 'follow'
        # back-out path: the driven track, newest first, without the spot itself
        path = [q for q in reversed(self._tilt_trail)]
        self._tob = {'phase': self.phase, 'origin': (ox, oy), 'stage': 'pause',
                     't': self._now, 'path': path, 'i': 0, 'follow': follow,
                     'dock_purpose': self._dock_purpose, 'best': 0.0, 'best_t': self._now}
        self._cancel_all('steep slope')
        self._blade_off('steep slope')
        self._fx.append(ZeroBurst())
        self._fx.append(SuppressStuckGuard(True))
        self._fx.append(RecordIncident('steep', ox, oy, '%s (limit) in %s'
                                       % (self._tilt_text(), self.phase)))
        if m is not None:
            if follow:
                m.start_local = self._track_progress()
            m.step = None
        self.sub_state = 'steep slope (%s): stopping' % self._tilt_text()
        return True

    def _tilt_backout_cmd(self, pose):
        """(v, w) toward the next back-out path point: pure pursuit, forward or reverse
        (whichever side the track lies on), straight reverse once the track is used up."""
        k, p = self._tob, self.p
        v0, la = abs(float(p.tilt_backout_speed)), float(p.tilt_backout_lookahead_m)
        path = k['path']
        while k['i'] < len(path) and math.hypot(path[k['i']][0] - pose[0],
                                                path[k['i']][1] - pose[1]) < la:
            k['i'] += 1
        if k['i'] >= len(path):
            return (k.get('v', -v0), 0.0)
        tx, ty = path[k['i']]
        dx, dy = tx - pose[0], ty - pose[1]
        c, s = math.cos(pose[2]), math.sin(pose[2])
        xr, yr = c * dx + s * dy, -s * dx + c * dy
        v = v0 if xr > 0 else -v0
        k['v'] = v
        wmax = float(p.tilt_backout_max_w)
        w = max(-wmax, min(wmax, v * 2.0 * yr / max(1e-6, xr * xr + yr * yr)))
        return v, w

    def _tilt_backout_tick(self, band):
        k, pose, now, p = self._tob, self.inputs.pose, self._now, self.p
        if pose is None:
            self._tob = None
            self._fx.append(SuppressStuckGuard(False))
            self._tilt_needs_help('steep slope, pose lost during the back-out', k['origin'])
            return
        if k['stage'] == 'pause':
            if now - k['t'] < float(p.tilt_backout_pause_s):
                return
            k['stage'], k['best_t'] = 'drive', now
            self.sub_state = 'steep slope (%s): backing out' % self._tilt_text()
            self._log('info', 'steep slope: backing out along %d track points at %.2f m/s'
                      % (len(k['path']), abs(float(p.tilt_backout_speed))))
        d = math.hypot(pose[0] - k['origin'][0], pose[1] - k['origin'][1])
        if d >= k['best'] + float(p.tilt_backout_progress_m):
            k['best'], k['best_t'] = d, now
        if band not in ('limit', 'critical') and d >= float(p.tilt_backout_min_m):
            self._tilt_backed_out(d)
            return
        if d >= float(p.tilt_backout_max_m) or now - k['best_t'] > float(p.tilt_backout_progress_s):
            why = ('still steep after backing out %.2f m' % d if d >= float(p.tilt_backout_max_m)
                   else 'back-out made no progress in %.0f s' % float(p.tilt_backout_progress_s))
            self._fx.append(DriveCmd(0.0, 0.0))
            self._tob = None
            self._fx.append(SuppressStuckGuard(False))
            self._tilt_needs_help(why, k['origin'])
            return
        self._fx.append(DriveCmd(*self._tilt_backout_cmd(pose)))

    def _tilt_backed_out(self, moved):
        k, self._tob = self._tob, None
        ox, oy = k['origin']
        self._tilt_hist.append([self._now, ox, oy])
        self._tilt_trail = []
        self._fx.append(DriveCmd(0.0, 0.0))
        self._fx.append(SuppressStuckGuard(False))
        self._log('info', 'steep slope: backed out %.2f m from (%.2f, %.2f), resuming %s'
                  % (moved, ox, oy, k['phase']))
        if k['phase'] in DOCK_PHASES:
            self._dock(k['phase'], k['dock_purpose'], 'resuming after steep-slope back-out')
            return
        m = self.mission
        if m is None or m.area_idx is None or not m.subpaths:
            self._go_idle('steep-slope back-out: nothing to resume')
            return
        if k['phase'] == 'MOWING' and k['follow']:
            # skip past the spot along the swath (blade-off detour); the 'steep' incident
            # raises the terrain cost so Nav2's detour / transit plans route around it
            if not self._try_detour(m.start_local, 'steep slope'):
                self._subpath_failed('steep slope')
            return
        m.detour = None
        self._dispatch()

    def _tilt_needs_help(self, why, pose):
        """Like the stuck guard's give-up (same latched phase, same operator clear)."""
        x, y = float(pose[0]), float(pose[1])
        self._log('error', 'STUCK_NEEDS_HELP: %s at (%.2f, %.2f): motion stopped, blade off '
                  '(STOP / START / MANUAL to clear)' % (why, x, y))
        if self.mission is not None:
            self._interrupt_mission('steep slope')
        self._cancel_all('steep slope')
        self._blade_off('steep slope')
        self._fx.append(ZeroBurst())
        self._dock_purpose = None
        self._resume_after = None
        self._tob = None
        self._go('STUCK_NEEDS_HELP', 'steep slope: needs help at (%.2f, %.2f)' % (x, y))
