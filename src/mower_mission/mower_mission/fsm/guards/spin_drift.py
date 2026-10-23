# SPDX-License-Identifier: GPL-3.0-or-later
"""MissionFSM mixin: Spin drift and repeated bumper hits.

Moved verbatim from mission_fsm.py (2026-10-23 code review stage 1.6)."""

from mower_mission.fsm.common import (ABORTED, ACT_SPIN, CancelActions, DOCK_PHASES, DriveCmd,
    SuppressStuckGuard, ZeroBurst, geo, math)  # noqa: F401


class SpinDriftGuardMixin:
    """Spin drift and repeated bumper hits."""

    # ==================================================================
    # Spin drift and repeated bumper hits (step 3 round 2, field 2026-10-21 evening)
    # ==================================================================
    def _spin_drift_tick(self):
        """base_link moved more than spin_drift_m during a pivot Spin. Field 2026-10-23
        afternoon: on wet turf 47 % of the pivots walked 0.14 m along the axis well inside the
        area, and stopping each one (transit aborts, skips, NOT-mowed sub-paths) did the damage.
        So a drift only stops the pivot when the body is within spin_drift_body_margin_m of its
        boundary limit (or outside: the boundary guard owns that), or beyond spin_drift_hard_m;
        then the pivot is re-started from where the robot is (no skip, no transit abort).
        True = handled this tick."""
        a, sf, lim = self._action, self._spin_from, float(self.p.spin_drift_m)
        pose = self.inputs.pose
        if lim <= 0.0 or a is None or a.name != ACT_SPIN or sf is None or sf[0] != a.token \
                or pose is None or len(pose) < 3:
            return False
        d = math.hypot(float(pose[0]) - sf[1], float(pose[1]) - sf[2])
        if d <= lim:
            return False
        near = True
        mode = self._hb_body_mode(pose)
        if mode is not None:
            ev = self._hb_body_eval(pose, mode)
            near = ev is None or ev['excess'] > -float(self.p.spin_drift_body_margin_m)
        if not near and d <= float(self.p.spin_drift_hard_m):
            if not sf[3:]:
                self._spin_from = sf + (True,)
                self._log('info', 'pivot drifted %.2f m from (%.2f, %.2f), body well inside: '
                                  'pivot goes on' % (d, sf[1], sf[2]))
            return False
        self._spin_from = None
        self._action = None
        self._fx.append(CancelActions('spin drift'))
        self._fx.append(ZeroBurst())
        why = 'pivot drifted %.2f m' % d
        self._log('warn', 'SPIN DRIFT: base_link moved %.2f m during a %s pivot from (%.2f, '
                          '%.2f) to (%.2f, %.2f) (%s): stopped, pivoting again from here'
                  % (d, a.purpose or 'Spin', sf[1], sf[2], pose[0], pose[1],
                     'body near its limit' if near else '> %.2f m'
                     % float(self.p.spin_drift_hard_m)))
        self._incident('spin_drift', why)
        m = self.mission
        if a.purpose == 'cov_pivot' and m is not None and self.phase == 'MOWING':
            m.pivot_done, m.pivot_target, m.step = None, None, None
            self._follow()                    # the leg-start pivot again, from this pose
            return True
        self._handle_action(a, ABORTED, {})   # the purpose's own re-send / follow logic
        return True

    def _bumper_tick(self):
        """bumper_skip_hits bumper hits within bumper_skip_radius_m: skip that stretch (a
        later hit near a skipped spot skips at once). True = handled this tick."""
        n, p = int(self.inputs.bumper_hits), self.p
        new = n - self._bump_seen
        self._bump_seen = n
        m, pose, need = self.mission, self.inputs.pose, int(p.bumper_skip_hits)
        if new <= 0 or need <= 0 or m is None or pose is None or \
                self.phase not in ('MOWING', 'TRANSIT'):
            return False
        now, rad = self._now, float(p.bumper_skip_radius_m)
        x, y = float(pose[0]), float(pose[1])
        log = [h for h in self._bump_log if now - h[0] <= float(p.bumper_skip_window_s)]
        log += [(now, x, y, False)] * new
        near = [h for h in log if math.hypot(h[1] - x, h[2] - y) <= rad]
        hits = sum(1 for h in near if not h[3])
        spot = any(h[3] for h in near)
        self._bump_log = log
        self._log('warn', 'bumper hit at (%.2f, %.2f): %d within %.1f m%s'
                  % (x, y, hits, rad, ', a skipped spot' if spot else ''))
        if hits < need and not spot:
            return False
        a = self._action
        step = m.step
        if not ((self.phase == 'MOWING' and step in ('follow', 'corner_pivot')) or
                (self.phase == 'TRANSIT' and step == 'transit')):
            return False
        self._bump_log = [h for h in log if h[3]] + [(now, x, y, True)]
        why = ('%d bumper hits within %.1f m' % (hits, rad)) if not spot else \
            'bumper hit near a skipped spot'
        self._log('warn', 'BUMPER SKIP at (%.2f, %.2f): %s' % (x, y, why))
        self._incident('bumper_skip', why)
        if a is not None:
            self._fx.append(CancelActions('bumper skip'))
            self._action = None
        if self.phase == 'TRANSIT':
            self._transit_abort(why)
            return True
        frm = self._track_progress() if step == 'follow' else m.start_local
        frm = min(max(frm, 0), self._seg_last())
        m.pivot_done, m.pivot_target, m.corner_end, m.step = None, None, None, None
        if self._edge_lap(m.sub_i):
            self._edge_skip(frm, why)
        else:
            self._skip_piece(frm, self._skip_end(frm), why)
        return True

    def _hb_cmd(self, pose):
        """(v, w) of the retrace: reverse toward the next track point (pure pursuit on the
        rear), both wheels rolling backwards (inner >= hard_boundary_min_inner_mps); a large
        heading error is turned on the spot first (counter-rotating pivot >= 0.3 rad/s);
        straight reverse over the last lookahead once the track points are used up (the
        caller stops at the track's length: never a blind reverse beyond the driven track)."""
        k, p = self._hb, self.p
        v = -min(abs(float(p.hard_boundary_retrace_speed_mps)), 0.10)   # owner: slow reverse
        path, la = k['path'], float(p.hard_boundary_lookahead_m)
        while k['i'] < len(path) and math.hypot(path[k['i']][0] - pose[0],
                                                path[k['i']][1] - pose[1]) < la:
            k['i'] += 1
        if k['i'] >= len(path):
            return v, 0.0
        tx, ty = path[k['i']]
        err = geo.wrap_angle(math.atan2(ty - pose[1], tx - pose[0]) - (float(pose[2]) + math.pi))
        if abs(err) > float(p.hard_boundary_pivot_err_rad):
            return 0.0, math.copysign(max(0.25, abs(float(p.hard_boundary_pivot_rps))), err)
        half = 0.5 * float(p.hard_boundary_track_width_m)
        wmax = max(0.0, (abs(v) - float(p.hard_boundary_min_inner_mps)) / max(1e-3, half))
        return v, max(-wmax, min(wmax, 1.5 * err))

    def _hb_stop(self):
        self._hb = None
        self._fx.append(DriveCmd(0.0, 0.0))
        self._fx.append(SuppressStuckGuard(False))

    def _hb_retrace_tick(self):
        k, pose, now, p = self._hb, self.inputs.pose, self._now, self.p
        if pose is None:
            self._hb_stop()
            self._hb_needs_help('pose lost during the boundary retrace', k['origin'])
            return
        if k['stage'] == 'pause':
            if now - k['t'] < float(p.hard_boundary_pause_s):
                return
            if k.get('entry'):
                if not self._hb_inward_start(pose):
                    self._hb_stop()
                    self._hb_needs_help('outside at START / resume and no inward direction',
                                        k['origin'])
                return
            if k.get('mode') is not None and not k.get('side'):
                k['stage'], k['t'], k['back_from'] = 'back', now, tuple(pose)
                self._go(self.phase, 'boundary: moving %.2f m %s' % (
                    float(p.hard_boundary_back_m), 'forward' if k['dir'] > 0 else 'back'))
                self._hb_back_tick(pose)
                return
            k['stage'], k['prog_t'], k['last'] = 'drive', now, tuple(pose)
            self._go(self.phase, 'boundary: reversing along the driven track')
            self._log('info', 'boundary: reversing along %d track points at %.2f m/s'
                      % (len(k['path']), min(abs(float(p.hard_boundary_retrace_speed_mps)),
                                             0.10)))
        if k['stage'] == 'back':
            self._hb_back_tick(pose)
            return
        if k['stage'] == 'inward':
            self._hb_inward_tick(pose)
            return
        last = k['last']
        step = math.hypot(pose[0] - last[0], pose[1] - last[1])
        turn = abs(geo.wrap_angle(float(pose[2]) - float(last[2])))
        k['driven'] += step
        k['prog'] += step + 0.5 * float(p.hard_boundary_track_width_m) * turn
        k['last'] = tuple(pose)
        if k['prog'] >= k['prog_best'] + float(p.hard_boundary_progress_m):
            k['prog_best'], k['prog_t'] = k['prog'], now
        # Success = a fresh ~/outside_m reading inside (<= 1 mm) after reversing
        # hard_boundary_retrace_extra_m past the re-entry point, or inside when the recorded
        # track / hard_boundary_retrace_max_m ends first. Never "back inside" on a reading
        # that is still outside (sim 2026-10-15: the track ran out 0.066 m outside and the
        # old <= hard_boundary_m test resumed from there), stale or missing.
        inside, out = self._hb_inside(pose)
        if inside:
            if k['inside_at'] is None:
                k['inside_at'] = k['driven']
            if k['driven'] - k['inside_at'] >= float(p.hard_boundary_retrace_extra_m):
                self._hb_retraced()
                return
        else:
            k['inside_at'] = None             # outside, or no fresh reading: re-entry unproven
        if k['driven'] >= min(float(p.hard_boundary_retrace_max_m), k['track_m'] + 0.02):
            if inside:
                self._hb_retraced()
                return
            if out is not None and self._hb_inward_start(pose):
                self._log('warn', 'boundary: retrace ended %.3f m out after %.2f m: moving '
                                  'inward' % (out, k['driven']))
                return
            self._hb_stop()
            self._hb_needs_help('retrace ended still %s after reversing %.2f m (driven track '
                                '%.2f m, max %.2f m)'
                                % ('outside by an unknown distance (no fresh outside_m)'
                                   if out is None else '%.3f m outside' % out, k['driven'],
                                   k['track_m'], float(p.hard_boundary_retrace_max_m)),
                                k['origin'])
            return
        if now - k['prog_t'] > float(p.hard_boundary_progress_s):
            self._hb_stop()
            self._hb_needs_help('boundary retrace made no progress in %.0f s'
                                % float(p.hard_boundary_progress_s), k['origin'])
            return
        self._fx.append(DriveCmd(*self._hb_cmd(pose)))

    def _hb_retraced(self):
        k = self._hb
        self._hb_stop()
        self._hb_trail = []
        ox, oy = k['origin']
        n, lim = self._hb_count, int(self.p.hard_boundary_max_aborts)
        if not k.get('counted', True):
            n, lim = 0, max(1, lim)
        spot = 0 if not k.get('counted', True) else sum(1 for q in self._hb_spots
                   if math.hypot(q[0] - ox, q[1] - oy) <= float(self.p.hard_boundary_spot_m))
        slim = int(self.p.hard_boundary_spot_max)
        if slim > 0 and spot >= slim:
            self._hb_needs_help('left the boundary %d times within %.1f m'
                                % (spot, float(self.p.hard_boundary_spot_m)), (ox, oy))
            return
        past = k['driven'] - k['inside_at'] if k['inside_at'] is not None else 0.0
        self._log('info', 'boundary: back inside after reversing %.2f m (%.2f m past the '
                          're-entry%s, excursion %d/%d)'
                  % (k['driven'], past, '' if past >= float(self.p.hard_boundary_retrace_extra_m)
                     else ', track / max reached', n, lim))
        if n >= lim:
            self._hb_needs_help('left the boundary %d times in this mission' % n, (ox, oy))
            return
        why = 'left the boundary (%s)' % k.get('what', '%.2f m outside' % k['out0'])
        if k['phase'] in DOCK_PHASES:
            self._dock(k['phase'], k['dock_purpose'], 'resuming after the boundary retrace')
            return
        m = self.mission
        if m is None or m.area_idx is None or not m.subpaths:
            self._go_idle('boundary retrace: nothing to resume')
            return
        if k['phase'] == 'TRANSIT' and k.get('entry'):
            m.detour = None
            self._dispatch(force_transit=True)    # inside again: the transit, not a failure
            return
        if k['phase'] == 'TRANSIT':
            if m.detour is not None:
                self._detour_failed(why)
                return
            m.route_fallback = False          # the retry asks for the drawn route again
            self._transit_abort(why)
            return
        m.detour = None
        pose = self.inputs.pose
        if k['follow'] and pose is not None and \
                m.sub_i < len(m.subpaths):
            # the recovery moved back along the leg: resume from the robot's projection (a
            # leg-start pivot, not a blade-off transit back to where the trip happened)
            sp = m.subpaths[m.sub_i]
            back = sp[:m.start_local + 1]
            if back:
                j = geo.nearest_index(back, pose)
                if geo.dist(back[j], pose) <= float(self.p.transit_gap_m):
                    m.start_local = m.progress_local = j
        self._dispatch()

    def _hb_needs_help(self, why, pose):
        """Like the stuck guard's give-up (same latched phase, same operator clear)."""
        x, y = float(pose[0]), float(pose[1])
        self._log('error', 'STUCK_NEEDS_HELP: %s at (%.2f, %.2f): motion stopped, blade off '
                  '(STOP / START / MANUAL to clear)' % (why, x, y))
        if self.mission is not None:
            self._interrupt_mission('boundary')
        self._cancel_all('boundary')
        self._blade_off('boundary')
        self._fx.append(ZeroBurst())
        self._dock_purpose = None
        self._resume_after = None
        self._hb = None
        self._go('STUCK_NEEDS_HELP', 'boundary: needs help at (%.2f, %.2f)' % (x, y))
