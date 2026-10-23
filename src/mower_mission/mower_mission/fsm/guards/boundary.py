# SPDX-License-Identifier: GPL-3.0-or-later
"""MissionFSM mixin: Hard boundary.

Moved verbatim from mission_fsm.py (2026-10-23 code review stage 1.6)."""

from mower_mission.fsm.common import (DOCK_PHASES, DriveCmd, HARD_BOUNDARY_PHASES,
    RecordIncident, SuppressStuckGuard, ZeroBurst, bbody, geo, math)  # noqa: F401


class BoundaryGuardMixin:
    """Hard boundary."""

    # ==================================================================
    # hard boundary (task 0.4 a, 2026-10-14 run 3)
    # ==================================================================
    # map_server publishes ~/outside_m: the robot centre's distance outside the mowing
    # areas U drawn paths U dock corridor (0 inside; the return corridor does NOT count).
    # Beyond hard_boundary_m in an autonomous driving phase: stop, blade off, reverse along
    # the driven track until back inside, count it, resume or give up (see Params).
    def _outside(self):
        """The fresh ~/outside_m, else None (no check without a recent value)."""
        i = self.inputs
        if i.outside_m is None or i.outside_stamp is None or \
                self._now - i.outside_stamp > float(self.p.hard_boundary_stale_s):
            return None
        return float(i.outside_m)

    def _hard_boundary_watch(self):
        """Abort a retrace when something else took over (emergency, STOP, phase change)."""
        k = self._hb
        if k is None:
            return
        if self._emergency_cause() or self.phase != k['phase']:
            self._log('warn', 'boundary retrace aborted (%s)' % (
                self._emergency_cause() or 'state %s' % self.phase))
            self._hb = None
            self._fx.append(SuppressStuckGuard(False))
            self._fx.append(DriveCmd(0.0, 0.0))

    def _hb_trail_add(self):
        pose = self.inputs.pose
        if pose is None:
            return
        tr, step = self._hb_trail, max(0.01, float(self.p.hard_boundary_trail_step_m))
        if tr and math.hypot(pose[0] - tr[-1][0], pose[1] - tr[-1][1]) < step:
            return
        tr.append((float(pose[0]), float(pose[1])))
        del tr[:-(int(float(self.p.hard_boundary_trail_m) / step) + 1)]

    def _hb_body_mode(self, pose):
        """edge_mode ('soft' | 'hard') of the body check at ``pose``, or None = the base_link
        point check (a dock phase, the dock corridor, a drawn path only, no area snapshot)."""
        if not self.p.hard_boundary_body or pose is None or len(pose) < 3 or \
                self.phase in DOCK_PHASES or not self._region_areas:
            return None
        x, y = float(pose[0]), float(pose[1])
        mowing, nav, near = [], [], None
        for idx, a in self._region_areas.items():
            ring = a.get('outer') or []
            if len(ring) < 3:
                continue
            if bbody._pip(x, y, ring):
                (nav if a.get('is_navigation_area', False) else mowing).append(idx)
            elif not a.get('is_navigation_area', False):
                d = bbody._edge_dist(x, y, ring)
                if near is None or d < near[0]:
                    near = (d, idx)
        if not mowing and not nav and near is not None and near[0] <= 1.0:
            mowing = [near[1]]       # base_link just outside a mowing area (lap 0 drift)
        if not mowing:
            return None
        m = self.mission
        if m is not None and m.area_idx in mowing and m.settings is not None:
            return 'hard' if str(m.settings.get('edge_mode', 'soft')) == 'hard' else 'soft'
        return 'soft'

    def _hb_body_eval(self, pose, mode):
        p = self.p
        lim = bbody.limits_for(mode, float(p.hard_boundary_body_soft_side_m),
                               float(p.hard_boundary_body_soft_nose_m),
                               float(p.hard_boundary_body_soft_tail_m),
                               float(p.hard_boundary_body_hard_m),
                               float(p.hard_boundary_body_soft_front_m))
        # plan-relative (soft): near a corner whose planned swept body is already out by
        # body_m (lap 0 on a tight driven bend, owner option (c)), any point may reach
        # body_m + hard_boundary_plan_margin_m (tracking p99 + margin) before it trips
        m = self.mission
        if mode == 'soft' and m is not None and m.corner_body and pose is not None:
            rad, add = float(p.hard_boundary_plan_radius_m), float(p.hard_boundary_plan_margin_m)
            near = [b for x, y, b in m.corner_body
                    if math.hypot(x - pose[0], y - pose[1]) <= rad]
            if near:
                allow = max(near) + add
                lim = {k: max(v, allow) for k, v in lim.items()}
        return bbody.evaluate(pose, self._region(), lim)

    def _point_out(self, pose):
        """base_link distance outside: from the mission's area snapshot (areas U paths U
        dock corridor) in TRANSIT / MOWING (hard_boundary_snapshot), else ~/outside_m."""
        if self.p.hard_boundary_snapshot and pose is not None and \
                self.phase in ('TRANSIT', 'MOWING') and self._region_areas:
            o = bbody.point_outside(float(pose[0]), float(pose[1]), self._region())
            if math.isfinite(o):
                return o
        return self._outside()

    def _hb_side_path(self, pose):
        """Side-aware recovery path (a SIDE corner beyond its limit while following a
        coverage leg): the planned leg behind the robot, newest first, about 0.8 m - the
        reverse pure pursuit onto the planned line moves base_link (and the sides) back
        toward the interior; a straight reverse would not. None = not applicable."""
        m = self.mission
        if self.phase != 'MOWING' or m is None or m.step != 'follow' or not m.subpaths:
            return None
        sp = m.subpaths[m.sub_i]
        j = self._track_progress()
        path, acc = [], 0.0
        for q in reversed(sp[:j + 1]):
            if path:
                acc += math.hypot(q[0] - path[-1][0], q[1] - path[-1][1])
            path.append((float(q[0]), float(q[1])))
            if acc >= 0.8:
                break
        if acc < 0.3:
            return None
        return path

    def _hb_jumped(self, val):
        """True while a boundary reading that jumped by > hard_boundary_jump_m since the last
        fresh one is held off (at most hard_boundary_jump_hold_s; logged, never counted)."""
        jm, now = float(self.p.hard_boundary_jump_m), self._now
        prev = self._hb_jump
        if jm <= 0.0:
            return False
        if prev is not None and now - prev[1] <= 1.0 and val - prev[0] > jm:
            since = prev[2] if prev[2] is not None else now
            if now - since < float(self.p.hard_boundary_jump_hold_s):
                if prev[2] is None:
                    self._log('warn', 'boundary: reading jumped %.2f -> %.2f m in one tick '
                                      '(map reload / pose jump?): ignored for up to %.1f s, '
                                      'not counted' % (prev[0], val,
                                                       float(self.p.hard_boundary_jump_hold_s)))
                self._hb_jump = (prev[0], now, since)
                return True
        self._hb_jump = (val, now, None)
        return False

    def _hard_boundary_tick(self):
        """Returns True while the hard boundary owns the tick."""
        if not self.p.hard_boundary:
            return False
        if self._hb is not None:
            self._hb_retrace_tick()
            return True
        self._hb_trail_add()
        hcal = self.phase == 'WAITING_FOR_RTK' and self._hcal is not None
        if self.phase not in HARD_BOUNDARY_PHASES and not hcal:
            self._hb_jump = None
            return False
        if self.phase in ('TRANSIT', 'MOWING') and self.mission is None:
            self._hb_jump = None
            return False
        pose = self.inputs.pose
        mode = None if hcal else self._hb_body_mode(pose)
        ev = self._hb_body_eval(pose, mode) if mode is not None else None
        # START / resume while still outside (field 2026-10-22: START 0.07 m out, the resume
        # pivot re-tripped 3 times): recover inward FIRST, no pivot, not counted
        entry = self._hb_entry and not hcal
        if ev is not None:
            out = ev['out']
            if self._hb_jumped(out):
                return False
            if entry:
                self._hb_entry = False
            if entry and ev['excess'] > -float(self.p.hard_boundary_body_clear_m):
                what = 'body %s %.2f m outside at START / resume' % (ev['point'], out)
            elif ev['excess'] <= 0.0:
                return False
            else:
                entry = False
                what = ('body %s %.2f m outside (%s limit %.2f)'
                        % (ev['point'], out, mode, out - ev['excess']))
        else:
            mode = None
            out = self._point_out(pose)
            if out is None:
                return False
            if self._hb_jumped(out):
                return False
            if entry:
                self._hb_entry = False
            if entry and out > float(self.p.hard_boundary_entry_m):
                what = 'base_link %.2f m outside at START / resume' % out
            elif out <= float(self.p.hard_boundary_m):
                return False
            else:
                entry = False
                what = 'base_link %.2f m outside (> %.2f)' % (out, float(self.p.hard_boundary_m))
        where = '(%.2f, %.2f)' % (pose[0], pose[1]) if pose is not None else 'unknown pose'
        if hcal:
            # unverified heading: no retrace on the EKF track, just stop where it is
            self._hcal_fail('left the mowing area by %.2f m at %s' % (out, where))
            return True
        if not entry:
            self._hb_count += 1
            if pose is not None:
                self._hb_spots.append((float(pose[0]), float(pose[1])))
        if pose is None:
            self._log('error', 'BOUNDARY OUT: %s, no pose (%d/%d this mission)'
                      % (what, self._hb_count, int(self.p.hard_boundary_max_aborts)))
            self._hb_needs_help('outside the boundary, no pose', (0.0, 0.0))
            return True
        m = self.mission
        follow = m is not None and m.step == 'follow'
        ox, oy = float(pose[0]), float(pose[1])
        path = [q for q in reversed(self._hb_trail) if math.hypot(q[0] - ox, q[1] - oy) > 0.02]
        side = None
        if ev is not None and ev['side_only']:
            side = self._hb_side_path(pose)
            if side is not None:
                path = side
        if side is not None:
            how = 'reverse onto the planned leg (side-aware), then resume'
        elif mode:
            how = 'move %.2f m %s, then resume' % (float(self.p.hard_boundary_back_m),
                                                   'forward' if ev['rear'] else 'back')
        else:
            how = 'retrace the driven track'
        if entry:
            how = 'move inward first (no pivot), not counted'
        self._log('error', 'BOUNDARY OUT: %s at %s in %s: cancel, zero, blade off, %s '
                           '(%d/%d this mission)'
                  % (what, where, self.phase, how, self._hb_count,
                     int(self.p.hard_boundary_max_aborts)))
        track = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                    for a, b in zip([(ox, oy)] + path, path))
        self._hb = {'track_m': track, 'phase': self.phase, 'origin': (ox, oy), 'stage': 'pause', 't': self._now,
                    'path': path, 'i': 0, 'follow': follow, 'out0': out, 'what': what,
                    'mode': mode, 'dir': 1.0 if ev and ev['rear'] else -1.0,
                    'side': side is not None, 'counted': not entry, 'entry': entry,
                    'tries': 0,
                    'dock_purpose': self._dock_purpose, 'driven': 0.0, 'last': tuple(pose),
                    'prog': 0.0, 'prog_best': 0.0, 'prog_t': self._now, 'inside_at': None}
        self._cancel_all('hard boundary')
        self._blade_off('hard boundary')
        self._fx.append(ZeroBurst())
        self._fx.append(SuppressStuckGuard(True))
        self._fx.append(RecordIncident('boundary', ox, oy, 'boundary out: %s in %s'
                                       % (what, self.phase)))
        self._loc = None
        if m is not None:
            if follow:
                m.start_local = self._track_progress()
            m.step = None
            m.pivot_done = None           # a cut-short leg-start pivot is pivoted again
        self._go(self.phase, 'boundary: %.2f m outside, stopping' % out)
        return True

    def _hb_inward_dir(self, pose):
        """Unit vector from the worst point (body mode) / base_link toward the nearest
        boundary point, i.e. into the allowed region; None when nothing is outside."""
        k = self._hb
        polys = self._region()
        x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
        px, py = x, y
        if k.get('mode') is not None:
            ev = self._hb_body_eval(pose, k['mode'])
            if ev is None or ev['out'] <= 0.0:
                return None
            bx, by, _kind = bbody.body_points()[ev['point']]
            c, sn = math.cos(yaw), math.sin(yaw)
            px, py = x + c * bx - sn * by, y + sn * bx + c * by
        elif bbody.point_outside(x, y, polys) <= 0.0:
            return None
        q = bbody.nearest_edge_point(px, py, polys)
        if q is None or q[2] < 1e-6:
            return None
        return ((q[0] - px) / q[2], (q[1] - py) / q[2])

    def _hb_inward_start(self, pose):
        """A bounded inward move (field 2026-10-22: the 1.0 m retrace cap ended 0.07 m
        out). False = no inward direction or hard_boundary_inward_tries used up."""
        k = self._hb
        if k['tries'] >= int(self.p.hard_boundary_inward_tries):
            return False
        u = self._hb_inward_dir(pose)
        if u is None:
            return False
        k['tries'] += 1
        h = (math.cos(float(pose[2])), math.sin(float(pose[2])))
        # forward (nose turning into the area) unless the inside lies clearly behind
        k['dir'] = 1.0 if h[0] * u[0] + h[1] * u[1] >= -0.3 else -1.0
        k['u'], k['stage'], k['t'], k['in_from'] = u, 'inward', self._now, tuple(pose)
        self._go(self.phase, 'boundary: moving inward (%d/%d)'
                 % (k['tries'], int(self.p.hard_boundary_inward_tries)))
        self._log('info', 'boundary: inward move %d/%d %s toward (%.2f, %.2f)'
                  % (k['tries'], int(self.p.hard_boundary_inward_tries),
                     'forward' if k['dir'] > 0 else 'reversing', u[0], u[1]))
        self._hb_inward_tick(pose)
        return True

    def _hb_inward_tick(self, pose):
        k, p = self._hb, self.p
        inside, out = self._hb_inside(pose)
        o = k['in_from']
        moved = math.hypot(pose[0] - o[0], pose[1] - o[1])
        if inside:
            self._fx.append(DriveCmd(0.0, 0.0))
            k['driven'], k['inside_at'] = moved, 0.0
            self._log('info', 'boundary: inside after an inward move of %.2f m' % moved)
            self._hb_retraced()
            return
        if moved >= float(p.hard_boundary_inward_max_m) or \
                self._now - k['t'] > float(p.hard_boundary_inward_timeout_s):
            self._fx.append(DriveCmd(0.0, 0.0))
            if self._hb_inward_start(pose):
                return
            self._hb_stop()
            self._hb_needs_help('still %s after %d inward moves'
                                % ('outside' if out is None else '%.3f m out' % out,
                                   k['tries']), k['origin'])
            return
        u, s_ = k['u'], k['dir']
        want = math.atan2(u[1] * s_, u[0] * s_)   # travel along u: heading u fwd, -u reverse
        err = geo.wrap_angle(want - float(pose[2]))
        v = min(abs(float(p.hard_boundary_back_speed_mps)), 0.10)
        half = 0.5 * float(p.hard_boundary_track_width_m)
        wmax = max(0.0, (v - float(p.hard_boundary_min_inner_mps)) / max(1e-3, half))
        self._fx.append(DriveCmd(s_ * v, max(-wmax, min(wmax, 1.5 * err))))

    def _hb_inside(self, pose):
        """(inside, out) for the running recovery: base_link mode = a fresh ~/outside_m
        <= 1 mm; body mode = every body point at least hard_boundary_body_clear_m under its
        limit (out = the worst point's outside distance)."""
        k = self._hb
        if k.get('mode') is None:
            out = self._point_out(pose)
            return out is not None and out <= 1e-3, out
        ev = self._hb_body_eval(pose, k['mode'])
        if ev is None:
            return False, None
        return ev['excess'] <= -float(self.p.hard_boundary_body_clear_m), ev['out']

    def _hb_back_tick(self, pose):
        """Stock onBoundaryOut: hard_boundary_back_m straight (reverse, or forward for a rear
        trip) at <= 0.10 m/s, hard_boundary_back_timeout_s. Then inside -> resume; still
        beyond the limit -> the track retrace (fallback)."""
        k, p = self._hb, self.p
        o = k['back_from']
        moved = math.hypot(pose[0] - o[0], pose[1] - o[1])
        if moved < float(p.hard_boundary_back_m) and \
                self._now - k['t'] < float(p.hard_boundary_back_timeout_s):
            v = min(abs(float(p.hard_boundary_back_speed_mps)), 0.10)
            self._fx.append(DriveCmd(k['dir'] * v, 0.0))
            return
        self._fx.append(DriveCmd(0.0, 0.0))
        inside, out = self._hb_inside(pose)
        if inside:
            k['driven'] = moved
            k['inside_at'] = 0.0
            self._log('info', 'boundary: back within the limits after moving %.2f m %s'
                      % (moved, 'forward' if k['dir'] > 0 else 'back'))
            self._hb_retraced()
            return
        self._log('warn', 'boundary: still %s after moving %.2f m straight: retracing the '
                          'driven track' % ('unknown' if out is None else '%.2f m out' % out,
                                            moved))
        k['stage'], k['prog_t'], k['last'] = 'drive', self._now, tuple(pose)
        k['driven'] = 0.0
