# SPDX-License-Identifier: GPL-3.0-or-later
"""MissionFSM mixin: Hard boundary.

Moved verbatim from mission_fsm.py (2026-10-23 code review stage 1.6)."""

from mower_mission.fsm.common import (DOCK_PHASES, DriveCmd, HARD_BOUNDARY_PHASES, bbody, geo,
    math, walls)
from mower_mission.fsm.escape import Escape

# the hard-boundary retrace (starts as 'hard boundary', gives up as 'boundary')
HB_RETRACE = Escape('_hb', 'boundary', 'boundary retrace', 'boundary: needs help at (%.2f, %.2f)',
                    begin_label='hard boundary')


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
        HB_RETRACE.watch(self)

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
        if mode != 'hard' and self._edge_slope():
            lim['front'] = min(lim['front'], lim['side'])   # no convex-bend allowance on a slope
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

    def _edge_slope(self):
        """True while |roll| (fresh /tilt/status) exceeds edge_slope_roll_deg: the soft
        front-corner allowance is withdrawn (logged once per episode). Hysteresis (slope
        sim 2026-11-03: the rule flapped on/off within 2 s at a region's fade edge): once on
        it stays on until |roll| falls below edge_slope_roll_deg - edge_slope_roll_hyst_deg
        (or the status goes stale) AND it has been on edge_slope_hold_s."""
        p, i = self.p, self.inputs
        thr = float(p.edge_slope_roll_deg)
        was = getattr(self, '_edge_slope_on', False)
        fresh = i.tilt_roll_deg is not None and i.tilt_stamp is not None and \
            self._now - i.tilt_stamp <= float(p.tilt_status_timeout_s)
        roll = abs(float(i.tilt_roll_deg)) if fresh else 0.0
        if thr <= 0.0:
            on = False
        elif not was:
            on = fresh and roll > thr
        else:
            below = not fresh or roll < thr - max(0.0, float(p.edge_slope_roll_hyst_deg))
            held = self._now - getattr(self, '_edge_slope_since', self._now) >= \
                float(p.edge_slope_hold_s)
            on = not (below and held)
        if on != was:
            self._edge_slope_on = on
            if on:
                self._edge_slope_since = self._now
                self._log('warn', 'boundary: side slope (roll %.1f deg > %.1f): front corners '
                                  'get the side limit %.2f m (no convex-bend allowance)'
                          % (float(i.tilt_roll_deg), thr,
                             float(self.p.hard_boundary_body_soft_side_m)))
            else:
                self._log('info', 'boundary: off the side slope: front-corner allowance back')
        return on

    def _hb_worst(self, pose):
        """(name, m): the worst-corner outside distance (raw, no limits; keepouts count),
        the one metric every boundary log line and the recovery planner use."""
        return bbody.worst_outside(pose, self._region(), keepouts=self._hb_keepouts())

    def _hb_lines(self):
        """The boundary lines the guard enforces ([] when disabled / none received)."""
        if not self.p.boundary_lines_enabled:
            return []
        return self.inputs.boundary_lines or []

    def _hb_line_hit(self, pose):
        """(depth, line) of the deepest boundary line the footprint (body lines) or the blade
        disc (blade lines) at ``pose`` reaches more than boundary_line_trip_m into, else None."""
        lines = self._hb_lines()
        if not lines or pose is None or len(pose) < 3:
            return None
        h = walls.hits(lines, pose, float(self.p.boundary_line_trip_m),
                       self.inputs.boundary_blade_radius)
        return h[0] if h else None

    def _hb_keepouts(self):
        return [list(o) for a in (self._region_areas or {}).values()
                if not a.get('is_navigation_area', False)
                for o in (a.get('obstacles') or []) if len(o) >= 3]

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
        self._hb_grace_travel()
        hcal = self.phase == 'WAITING_FOR_RTK' and self._hcal is not None
        if self.phase not in HARD_BOUNDARY_PHASES and not hcal:
            self._hb_jump = None
            return False
        if self.phase in ('TRANSIT', 'MOWING') and self.mission is None:
            self._hb_jump = None
            return False
        pose = self.inputs.pose
        # START / resume while still outside (field 2026-10-22: START 0.07 m out, the resume
        # pivot re-tripped 3 times): recover inward FIRST, no pivot, not counted
        entry = self._hb_entry and not hcal
        # boundary lines (virtual walls, 2026-11-02): the footprint touching a line's TRUE
        # geometry is a trip of its own (reason names the line), recovered by the track retrace
        line = self._hb_line_hit(pose)
        mode = None if hcal or line is not None else self._hb_body_mode(pose)
        ev = self._hb_body_eval(pose, mode) if mode is not None else None
        if line is not None:
            out = max(0.0, line[0])
            if entry:
                self._hb_entry = False
            what = walls.describe(line[0], line[1]) + (' at START / resume' if entry else '')
        elif ev is not None:
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
            wn, wo = max(ev['outs'].items(), key=lambda kv: kv[1])
            if wn != ev['point']:
                what += ', worst corner %s %.2f m' % (wn, wo)
            out = wo                      # the worst-corner metric from here on
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
            self._hcal_fail('%s at %s' % (what, where) if line is not None else
                            'left the mowing area by %.2f m at %s' % (out, where))
            return True
        # a re-trip right after a recovery resume (field 2026-11-03): no spot strike yet, one
        # more recovery with twice the margin; counted when that one is back inside
        since = self._hb_grace_since() if not entry and line is None and mode is not None \
            else None
        grace = since is not None
        self._hb_resumed = None
        if not entry and not grace:
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
        margin = max(0.0, float(self.p.boundary_recover_margin_m))
        if grace:
            margin *= 2.0
            how += ' (re-trip %.1f s after a recovery resume: escalated, margin %.2f m, ' \
                   'no spot strike)' % (since, margin)
        self._log('error', 'BOUNDARY OUT: %s at %s in %s: cancel, zero, blade off, %s '
                           '(%d/%d this mission)'
                  % (what, where, self.phase, how, self._hb_count,
                     int(self.p.hard_boundary_max_aborts)))
        track = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                    for a, b in zip([(ox, oy)] + path, path))
        HB_RETRACE.begin(self, {'track_m': track, 'phase': self.phase, 'origin': (ox, oy),
                                'stage': 'pause', 't': self._now,
                                'path': path, 'i': 0, 'follow': follow, 'out0': out, 'what': what,
                                'mode': mode, 'dir': 1.0 if ev and ev['rear'] else -1.0,
                                'side': side is not None, 'counted': not entry, 'entry': entry,
                                'tries': 0, 'moves': 0,
                                'dock_purpose': self._dock_purpose, 'driven': 0.0,
                                'last': tuple(pose),
                                'prog': 0.0, 'prog_best': 0.0, 'prog_t': self._now,
                                'inside_at': None, 'margin': margin, 'grace': bool(grace),
                                'step': m.step if m is not None else None,
                                'nogo_detour': m.nogo_detour if m is not None else None,
                                'line': line[1].id if line is not None else None},
                         incident=('boundary', ox, oy, 'boundary out: %s in %s'
                                   % (what, self.phase)))
        self._loc = None
        Escape.pause_mission(self, follow)
        if m is not None:
            m.nogo_detour = None          # its follow was cancelled: rejoined on resume
            m.pivot_done = None           # a cut-short leg-start pivot is pivoted again
        self._go(self.phase, 'boundary: %.2f m outside, stopping' % out)
        return True

    def _hb_inward_dir(self, pose):
        """Unit vector from the worst point (body mode) / base_link toward the nearest
        boundary point, i.e. into the allowed region; None when nothing is outside."""
        k = self._hb
        if k.get('line') is not None:
            # a boundary line: away from its nearest point, to the side base_link is on
            d = walls.depth_of(self._hb_lines(), k['line'], pose,
                               self.inputs.boundary_blade_radius)
            if d is None or d <= -float(self.p.hard_boundary_body_clear_m):
                return None
            return walls.away_dir(self._hb_lines(), k['line'], pose)
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

    def _hb_inside(self, pose, clear=None):
        """(inside, out) for the running recovery: base_link mode = a fresh ~/outside_m
        <= 1 mm; body mode = every body point at least ``clear`` (default
        hard_boundary_body_clear_m) under its limit (out = the worst point's outside
        distance)."""
        k = self._hb
        if k.get('line') is not None:
            # boundary line trip: clear of the line by hard_boundary_body_clear_m (a line that
            # was deleted meanwhile counts as clear); the area rules resume after it
            d = walls.depth_of(self._hb_lines(), k['line'], pose,
                               self.inputs.boundary_blade_radius)
            if d is None:
                return True, 0.0
            return d <= -float(self.p.hard_boundary_body_clear_m), max(0.0, d)
        if k.get('mode') is None:
            out = self._point_out(pose)
            return out is not None and out <= 1e-3, out
        ev = self._hb_body_eval(pose, k['mode'])
        if ev is None:
            return False, None
        # out = the worst-corner outside distance (2026-11-02: the trip point's distance was
        # logged instead - rear_left 0.523 while front_left was 0.655 out)
        if clear is None:
            clear = float(self.p.hard_boundary_body_clear_m)
        return ev['excess'] <= -clear, max(ev['outs'].values())

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

    # ------------------------------------------------------------------
    # footprint-planned recovery + wheel-slip check (field 2026-11-02 side slope)
    # ------------------------------------------------------------------
    def _hb_plan_on(self):
        k = self._hb
        return bool(self.p.boundary_recover_footprint_plan) and k is not None and \
            k.get('mode') is not None

    def _hb_motions(self):
        p = self.p
        v = min(abs(float(p.hard_boundary_back_speed_mps)), 0.10)
        half = 0.5 * float(p.hard_boundary_track_width_m)
        w_arc = max(0.0, (v - float(p.hard_boundary_min_inner_mps)) / max(1e-3, half))
        return bbody.candidate_motions(v, w_arc, max(0.25, abs(float(p.hard_boundary_pivot_rps))),
                                       float(p.boundary_recover_move_m),
                                       float(p.boundary_recover_pivot_rad))

    def _hb_plan_start(self, pose, exclude=()):
        """Plan and start the next recovery motion from ``pose``. False = the move budget is
        used up or no candidate keeps the worst corner from growing."""
        k, p = self._hb, self.p
        if k['moves'] >= max(1, int(p.boundary_recover_max_moves)):
            return False
        rows = bbody.plan_recovery(pose, self._region(), self._hb_motions(),
                                   float(p.boundary_recover_max_worsen_m),
                                   keepouts=self._hb_keepouts(), exclude=exclude,
                                   reverse_bias=float(p.boundary_recover_reverse_bias_m))
        if not rows:
            return False
        name, v, w, dur, end, peak = rows[0]
        wn, wo = self._hb_worst(pose)
        k['moves'] += 1
        k['stage'], k['t'] = 'plan', self._now
        k['mv'] = {'name': name, 'v': v, 'w': w, 'dur': dur, 'from': tuple(pose),
                   'worst0': wo, 'yaw_acc': 0.0, 'yaw_last': float(pose[2])}
        self._hb_slip_reset(pose)
        self._go(self.phase, 'boundary: recovery move %d (%s)' % (k['moves'], name))
        self._log('info', 'boundary: recovery move %d/%d %s (v %.2f, w %.2f): worst corner %s '
                          '%.2f m -> %.2f m planned (peak %.2f; %d other candidates)'
                  % (k['moves'], int(p.boundary_recover_max_moves), name, v, w, wn, wo, end,
                     peak, len(rows) - 1))
        self._hb_plan_tick(pose)
        return True

    def _hb_plan_tick(self, pose):
        k, p, mv = self._hb, self.p, self._hb['mv']
        # target (field 2026-11-03): boundary_recover_margin_m under the trip limit, not
        # merely hard_boundary_body_clear_m (the resume re-tripped 5 s later)
        clear = max(float(p.hard_boundary_body_clear_m), k.get('margin', 0.0))
        inside, out = self._hb_inside(pose, clear)
        o = mv['from']
        moved = math.hypot(pose[0] - o[0], pose[1] - o[1])     # map / RTK, not wheel odom
        if self._hb_slip_tick(pose, exclude=(mv['name'],)):
            return
        mv['yaw_acc'] += abs(geo.wrap_angle(float(pose[2]) - mv['yaw_last']))
        mv['yaw_last'] = float(pose[2])
        worst = self._hb_worst(pose)[1]
        why = None
        if worst > mv['worst0'] + float(p.boundary_recover_max_worsen_m):
            why = 'worst corner grew %.2f -> %.2f m' % (mv['worst0'], worst)
        elif (mv['v'] != 0.0 and moved >= float(p.boundary_recover_move_m)) or \
                (mv['v'] == 0.0 and mv['yaw_acc'] >= float(p.boundary_recover_pivot_rad)):
            why = 'done'
        elif self._now - k['t'] > 2.0 * mv['dur'] + 2.0:
            why = 'timed out'
        if why is None:
            self._fx.append(DriveCmd(mv['v'], mv['w']))
            return
        self._fx_now(DriveCmd(0.0, 0.0))   # out before the next (slow) plan_recovery
        if inside:                         # the whole planned move, not just the limit
            k['driven'], k['inside_at'] = moved, 0.0
            self._log('info', 'boundary: inside after recovery move %d (%s %s, %.2f m; '
                              '>= %.2f m under the limit)'
                      % (k['moves'], mv['name'], why, moved, clear))
            self._hb_retraced()
            return
        self._log('info' if why == 'done' else 'warn',
                  'boundary: recovery move %d (%s) %s after %.2f m: worst corner %.2f m out%s'
                  % (k['moves'], mv['name'], why, moved, worst,
                     '' if out is None else ', short of the %.2f m margin' % clear))
        if self._hb_plan_start(pose):
            return
        if self._hb_inside(pose)[0]:
            # under the limit but no admissible move (or the cap) for the full margin
            k['driven'], k['inside_at'] = moved, 0.0
            self._log('warn', 'boundary: inside after %d recovery moves but short of the '
                              '%.2f m margin (no admissible move left): resuming'
                      % (k['moves'], clear))
            self._hb_retraced()
            return
        self._hb_stop()
        self._hb_needs_help('still %.2f m out (worst corner) after %d recovery moves'
                            % (worst, k['moves']), pose)

    def _hb_slip_reset(self, pose):
        k = self._hb
        k['slip'] = {'from': tuple(pose[:2]), 'wheel': 0.0, 'last': self.inputs.wheel_xy}

    def _hb_slip_tick(self, pose, exclude=None):
        """Wheel-vs-RTK check of the running recovery motion. True = it tripped and was
        handled (next-best candidate once when ``exclude`` is given, else needs help)."""
        k, p, i = self._hb, self.p, self.inputs
        sl = k.get('slip')
        if not p.boundary_recover_slip_check or sl is None:
            return False
        w = i.wheel_xy
        if w is not None and sl['last'] is not None:
            sl['wheel'] += math.hypot(w[0] - sl['last'][0], w[1] - sl['last'][1])
        sl['last'] = w
        if i.fix_type is None or int(i.fix_type) != int(p.rtk_fix_type):
            sl['from'], sl['wheel'] = tuple(pose[:2]), 0.0     # no RTK: no real reference
            return False
        real = math.hypot(pose[0] - sl['from'][0], pose[1] - sl['from'][1])
        if sl['wheel'] < float(p.boundary_recover_slip_min_m) or \
                sl['wheel'] <= float(p.boundary_recover_slip_ratio) * real:
            return False
        self._fx_now(DriveCmd(0.0, 0.0))   # out before the next-best plan_recovery
        roll = i.tilt_roll_deg
        why = 'wheel slip on slope (roll %s deg): %.2f m wheel odom, %.2f m real' % (
            '?' if roll is None else '%.0f' % abs(float(roll)), sl['wheel'], real)
        self._log('warn', 'boundary: %s, stopping that motion' % why)
        if exclude is not None and not k.get('slip_retried'):
            k['slip_retried'] = True
            if self._hb_plan_start(pose, exclude=exclude):
                return True
        self._hb_stop()
        self._hb_needs_help(why, pose)
        return True

    # ------------------------------------------------------------------
    # resume after a recovery (field 2026-11-03 area 5, side slope roll 8-11 deg: the trip
    # came during an edge-lap detour, so the resume point stayed at the chunk start 2 m
    # back; the transit pivoted at the recovery spot and re-tripped 5 s later, and the
    # spot rule stopped the mission)
    # ------------------------------------------------------------------
    def _hb_grace_travel(self):
        r, pose = self._hb_resumed, self.inputs.pose
        if r is None or pose is None:
            return
        r['dist'] += math.hypot(pose[0] - r['last'][0], pose[1] - r['last'][1])
        r['last'] = (float(pose[0]), float(pose[1]))

    def _hb_grace_since(self):
        """Seconds since the last recovery resume while its grace window is open
        (boundary_recover_grace_s and _grace_m of travel; not after an escalated one), else
        None."""
        r = self._hb_resumed
        if r is None or not r['grace_ok']:
            return None
        dt = self._now - r['t']
        if dt > float(self.p.boundary_recover_grace_s) or \
                r['dist'] > float(self.p.boundary_recover_grace_m):
            return None
        return dt

    def _hb_mark_resumed(self, k):
        pose = self.inputs.pose
        self._hb_resumed = None if pose is None or k.get('mode') is None else {
            't': self._now, 'last': (float(pose[0]), float(pose[1])), 'dist': 0.0,
            'grace_ok': not k.get('grace')}

    def _hb_rejoin_index(self, k, pose):
        """Local index of the interrupted MOWING leg to rejoin from ``pose`` by driving
        forward along it (a follow or an edge-lap detour was running): the nearest leg pose
        around the resume point within transit_gap_m, moved ahead along the leg until it is
        no longer behind the robot. The leg heading there must be within 60 deg of the
        robot's yaw (no pivot), or the pivot onto it must pass the footprint check
        (boundary_recover_margin_m under the limits). None = no rejoin (a transit)."""
        m, p = self.mission, self.p
        if k.get('step') not in ('follow', 'nogo_detour') or pose is None or len(pose) < 3 \
                or m is None or m.sub_i >= len(m.subpaths):
            return None
        sp = m.subpaths[m.sub_i]
        last = self._seg_last()
        frm = m.start_local
        d = k.get('nogo_detour')
        if d is not None:
            frm = min(frm, int(d['from']))
        lo, acc = frm, 0.0
        while lo > 0 and acc < 1.0:
            acc += geo.dist(sp[lo - 1], sp[lo])
            lo -= 1
        j = min(max(geo.nearest_index(sp, pose, lo, 3.0), 0), last)
        if geo.dist(sp[j], pose) > float(p.transit_gap_m):
            return None
        while j < last:
            h = geo.heading_of(sp, j)
            if (sp[j][0] - pose[0]) * math.cos(h) + (sp[j][1] - pose[1]) * math.sin(h) >= 0.05:
                break
            j += 1
        if j >= last or geo.dist(sp[j], pose) > float(p.transit_gap_m):
            return None
        h = geo.heading_of(sp, j)
        if abs(geo.wrap_angle(h - float(pose[2]))) <= math.radians(60.0):
            return j
        aim = (pose[0] + math.cos(h), pose[1] + math.sin(h))
        return j if self._hb_pivot_ok(pose, aim, k.get('mode')) else None

    def _hb_rejoin(self, k):
        """Resume the interrupted leg (preferred: the edge lap it was on) ahead of the robot,
        offset boundary_recover_margin_m inward over edge_join_m, no leg-start pivot; an
        interrupted edge-lap detour is re-planned from there. True = resumed."""
        m, pose = self.mission, self.inputs.pose
        j = self._hb_rejoin_index(k, pose)
        if j is None:
            return False
        sp = m.subpaths[m.sub_i]
        m.start_local = m.progress_local = j
        d = k.get('nogo_detour')
        if d is not None:
            m.nogo_detoured.discard((m.area_idx, m.sub_i, d['end']))
            if self._nogo_edge_detour(j, '%s (re-planned after a boundary recovery)'
                                      % d['why']):
                self._log('info', 'boundary: rejoining the edge-lap detour at pose %d' % j)
                return True
        margin = max(0.0, float(self.p.boundary_recover_margin_m))
        sgn = self._hb_inward_side(sp, j, margin)
        m.join = (m.sub_i, j, sgn, margin) if sgn and margin > 0.0 else None
        turn = abs(geo.wrap_angle(geo.heading_of(sp, j) - float(pose[2])))
        if turn <= math.radians(60.0):
            m.pivot_done = (m.area_idx, m.sub_i, j)    # no pivot at the recovery spot
        self._log('info', 'boundary: rejoining %s %d at pose %d (%.2f m ahead), %.2f m inward '
                          'bias, %s'
                  % ('edge lap' if self._edge_lap(m.sub_i) else 'sub-path', m.sub_i, j,
                     geo.dist(sp[j], pose), margin if m.join else 0.0,
                     'no pivot' if m.pivot_done else
                     'pivot %.0f deg footprint-checked' % math.degrees(turn)))
        self._dispatch()
        return True

    def _hb_inward_side(self, sp, j, off):
        """+1 / -1: the side of leg pose ``j`` (left / right of its heading) that is further
        from the region edge at ``off``; 0 when neither offset point is inside."""
        if off <= 0.0:
            return 0
        polys = self._region()
        h = geo.heading_of(sp, j)
        best = None
        for sgn in (1, -1):
            x = sp[j][0] - math.sin(h) * sgn * off
            y = sp[j][1] + math.cos(h) * sgn * off
            if bbody.point_outside(x, y, polys) > 0.0:
                continue
            e = bbody.nearest_edge_point(x, y, polys)
            c = e[2] if e is not None else 0.0
            if best is None or c > best[0]:
                best = (c, sgn)
        return best[1] if best else 0

    def _hb_resume_in_start(self, k):
        """Before a transit from the recovery spot: a planned straight of
        boundary_recover_resume_in_m (forward or reverse, the one the footprint scoring
        gains most clearance with), then a footprint check of the pivot toward the transit
        target (one more straight when it fails). True = the stage was started."""
        p, pose = self.p, self.inputs.pose
        if k.get('mode') is None or not self._hb_plan_on() or pose is None or \
                k.get('resume_in') is not None:
            return False
        dist = float(p.boundary_recover_resume_in_m)
        if dist <= 0.0:
            return False
        k['resume_in'] = 0
        return self._hb_resume_in_next(pose)

    def _hb_resume_in_target(self):
        m = self.mission
        if m is None:
            return None
        if self._hb['phase'] == 'TRANSIT' and m.transit_target is not None:
            return m.transit_target
        if m.sub_i < len(m.subpaths) and m.start_local < len(m.subpaths[m.sub_i]):
            return m.subpaths[m.sub_i][m.start_local]
        return None

    def _hb_pivot_ok(self, pose, target, mode):
        """The footprint of the pivot toward ``target`` stays boundary_recover_margin_m
        under the ``mode`` trip limits (True also without a target / mode)."""
        if target is None or mode is None:
            return True
        delta = geo.wrap_angle(math.atan2(target[1] - pose[1], target[0] - pose[0]) -
                               float(pose[2]))
        if abs(delta) < 0.05:
            return True
        w = math.copysign(max(0.25, abs(float(self.p.hard_boundary_pivot_rps))), delta)
        margin = max(0.0, float(self.p.boundary_recover_margin_m))
        for q in bbody.simulate(pose, 0.0, w, abs(delta / w), dt=0.2):
            ev = self._hb_body_eval(q, mode)
            if ev is not None and ev['excess'] > -margin:
                return False
        return True

    def _hb_resume_in_next(self, pose):
        k, p = self._hb, self.p
        target = self._hb_resume_in_target()
        if k['resume_in'] >= 1 and self._hb_pivot_ok(pose, target, k.get('mode')):
            return False
        if k['resume_in'] >= 2:
            self._log('warn', 'boundary: the pivot toward the transit still nears the limit '
                              'after %d resume straights: transit anyway' % k['resume_in'])
            return False
        v = min(abs(float(p.hard_boundary_back_speed_mps)), 0.10)
        dist = float(p.boundary_recover_resume_in_m)
        motions = {'forward': (v, 0.0, dist / v), 'reverse': (-v, 0.0, dist / v)}
        rows = bbody.plan_recovery(pose, self._region(), motions,
                                   float(p.boundary_recover_max_worsen_m),
                                   keepouts=self._hb_keepouts(),
                                   reverse_bias=float(p.boundary_recover_reverse_bias_m),
                                   signed=True)
        if not rows:
            if k['resume_in'] == 0:
                self._log('info', 'boundary: no resume straight gains clearance: transit')
            return False
        name, v, w, dur, end, peak = rows[0]
        k['resume_in'] += 1
        k['stage'], k['t'] = 'resume_in', self._now
        k['mv'] = {'name': name, 'v': v, 'dur': dur, 'from': tuple(pose)}
        self._go(self.phase, 'boundary: inward straight before the transit')
        self._log('info', 'boundary: resume straight %d %s %.2f m before the transit (no pivot '
                          'here; worst point ends %.2f m %s the edge)'
                  % (k['resume_in'], name, dist, abs(end), 'inside' if end <= 0.0 else 'past'))
        self._hb_resume_in_tick(pose)
        return True

    def _hb_resume_in_tick(self, pose):
        k, p, mv = self._hb, self.p, self._hb['mv']
        moved = math.hypot(pose[0] - mv['from'][0], pose[1] - mv['from'][1])
        if moved < float(p.boundary_recover_resume_in_m) and \
                self._now - k['t'] <= 2.0 * mv['dur'] + 2.0 and self._hb_inside(pose)[0]:
            self._fx.append(DriveCmd(mv['v'], 0.0))
            return
        self._fx_now(DriveCmd(0.0, 0.0))
        if self._hb_resume_in_next(pose):
            return
        self._hb_retraced()
