# SPDX-License-Identifier: GPL-3.0-or-later
"""MissionFSM mixin: Coverage turn legs, pivots and edge-lap skips.

Moved verbatim from mission_fsm.py (2026-10-23 code review stage 1.6)."""

from mower_mission.fsm.common import (ACT_BACKUP, ACT_FOLLOW, ACT_SPIN, CANCELED,
    CancelActions, DriveCmd, SUCCEEDED, bbody, bisect, geo, math, subpath_offsets)  # noqa: F401
from mower_mission.fsm.phases.transit import TransitMixin


class PivotsMixin:
    """Coverage turn legs, pivots and edge-lap skips."""

    def _cov_turn_leg(self, k):
        """Sub-path ``k`` is a reverse leg (owner turn_type reverse, or a turn chain that
        could not be conditioned): driven as planned, never pivoted onto."""
        return geo.is_reverse_subpath(self.mission.subpaths[k])

    def _cov_kturn_leg(self, k=None):
        """Sub-path ``k`` is the reverse leg of a conditioner K-turn (P1 -> V)."""
        m = self.mission
        c = m.cond.get(m.sub_i if k is None else k)
        return bool(c and c.get('kturn'))

    def _cov_pivots(self, k=None):
        m = self.mission
        k = m.sub_i if k is None else k
        c = m.cond.get(k)
        return list(c['pivots']) if c else []

    def _cov_pivot(self):
        """Before a coverage follow from ``start_local``: the conditioner's pivot rule at the
        leg's start. The start is a vertex of the path actually driven (robot -> leg): a
        conditioner pivot marker, the sub-path start after a transit, or a resume after a
        retry / retrace / hold. The robot is on the line, so no fillet fits there: when the
        join heading is path_conditioning_pivot_min_rad or more off the robot's yaw it turns
        in place (Nav2 Spin, both wheels) first, as at any pivot vertex; a smaller bend is
        left to the tracker. One mechanism for both coverage controllers (it replaces the
        separate leg-start rules path_conditioning_start_pivot_rad and MPPI's
        coverage_leg_pivot_rad). Not on reverse legs, except a K-turn's (edge layout round 2):
        before it the robot turns to face its poses (the about-turn at P1). A leg whose
        conditioned start carries a planned pivot (start_turn: K-turn P1 / V) turns the
        planned way round (the nose through the area), even the long way. True = started."""
        m, p, pose = self.mission, self.p, self.inputs.pose
        key = (m.area_idx, m.sub_i, m.start_local)
        if m.pivot_done == key:               # just pivoted for this leg: follow it
            m.pivot_done = None
            return False
        m.pivot_done = None
        lim = float(p.path_conditioning_pivot_min_rad)
        kturn = self._cov_kturn_leg()
        if not p.path_conditioning or lim <= 0.0 or pose is None or len(pose) < 3 or \
                (self._cov_turn_leg(m.sub_i) and not kturn):
            return False
        sp, pivots = m.subpaths[m.sub_i], self._cov_pivots()
        corner = m.start_local in pivots
        end = min([c for c in pivots if c > m.start_local] + [self._seg_last()])
        if kturn:                             # face the reverse leg's poses
            heading = float(sp[min(m.start_local, len(sp) - 1)][2]) if len(sp[0]) >= 3 else None
        else:
            heading = self._join_heading(
                self._join_leg(list(sp[m.start_local:end + 1]), m.sub_i, m.start_local), pose,
                sharp=lim)
        if heading is None:
            return False
        delta = geo.wrap_angle(heading - float(pose[2]))
        if abs(delta) < lim:
            return False
        planned = (m.cond.get(m.sub_i) or {}).get('start_turn') if m.start_local == 0 else None
        if planned is not None and delta * planned < 0.0 and \
                (abs(delta) > 0.5 * math.pi or abs(planned) > math.pi):
            delta += math.copysign(2.0 * math.pi, planned)    # the planned way round
        elif planned is None:
            delta = self._pivot_way(pose, delta)
        g = self._nogo_pivot_gate('cov', key, delta)     # option C: pre-pivot sweep check
        if g is True:
            return True
        if g is not None:
            delta = g
        q = sp[m.start_local]
        self._log('info', 'area %d sub-path %d: %s at (%.2f, %.2f): pivot %.0f deg in place, '
                          '%s, then follow' % (m.area_idx, m.sub_i,
                                               'K-turn about-turn' if kturn else
                                               'pivot vertex' if corner else 'leg start',
                                               q[0], q[1], math.degrees(delta),
                                               'blade on' if self.blade_on else 'blade off'))
        m.step = 'corner_pivot'
        m.pivot_done = key
        m.pivot_target = (heading, lim)
        self._go('MOWING', 'area %d sub-path %d/%d: pivot %.0f deg' % (
            m.area_idx, m.sub_i + 1, len(m.subpaths), math.degrees(delta)))
        allowance = abs(delta) / max(0.05, float(p.route_cusp_spin_rps)) + 10.0
        self._start_action(ACT_SPIN, {'angle': delta, 'time_allowance_s': allowance},
                           allowance + 5.0, purpose='cov_pivot')
        return True

    def _pivot_way(self, pose, delta):
        """Rotation direction of an in-place pivot by ``delta`` with no planned way round
        (owner 2026-10-21: turn TOWARD the area interior, even the longer way): the way whose
        swept body (body_boundary points, 5 deg steps) reaches least far outside the
        allowed region; the shorter way unless the other is pivot_way_gain_m better."""
        gain = float(self.p.pivot_way_gain_m)
        if gain < 0.0 or not self._region_areas or pose is None:
            return delta
        polys = self._region()
        zero = {'side': 0.0, 'front': 0.0, 'nose': 0.0, 'tail': 0.0}
        x, y, yaw0 = float(pose[0]), float(pose[1]), float(pose[2])

        def worst(d):
            n = max(1, int(abs(d) / math.radians(5.0)))
            w = 0.0
            for k in range(n + 1):
                ev = bbody.evaluate((x, y, yaw0 + d * k / n), polys, zero)
                if ev is None:
                    return 0.0
                w = max(w, ev['out'])
            return w
        other = delta - math.copysign(2.0 * math.pi, delta)
        w0, w1 = worst(delta), worst(other)
        if w1 + gain < w0:
            self._log('info', 'pivot %.0f deg the long way round (%.0f deg): the swept body '
                              'reaches %.2f m out instead of %.2f (turn toward the interior)'
                      % (math.degrees(delta), math.degrees(other), w1, w0))
            return other
        return delta

    @staticmethod
    def _join_heading(leg, pose, look=0.4, sharp=None):
        """Heading from the robot to the point ``look`` m past its projection onto ``leg``
        (where the tracker joins it; on the leg's start: the leg's own heading). ``sharp``:
        a vertex inside the look-ahead where the leg turns by that much or more is not cut:
        the target is that vertex (turn to the first segment, drive it, turn at the vertex;
        2026-10-21 hand-off: one -138 deg pivot aimed past a vertex instead of -21 then
        -112); the robot at it already: the segment after it."""
        if len(leg) < 2:
            return None
        cum = geo.cumulative_lengths(leg)
        s = geo.path_projection(leg, pose)[0]
        # join at <= ~18 deg from up to 0.5 m beside the leg (an edge-lap start approached
        # from inside: a 0.4 m look-ahead aimed the nose 45 deg across the line)
        px, py = geo._point_at(leg, cum, s)
        lat = math.hypot(px - pose[0], py - pose[1])
        s_end = min(cum[-1], s + max(look, min(1.5, 3.0 * lat)))
        if sharp is not None:
            w = 0.06                     # chords either side (resampling zigs do not count)
            for j in range(1, len(leg) - 1):
                if cum[j] <= s + 1e-3 or cum[j] < w or cum[-1] - cum[j] < w:
                    continue
                if cum[j] >= s_end - 1e-9:
                    break
                a = geo._point_at(leg, cum, cum[j] - w)
                b = geo._point_at(leg, cum, cum[j] + w)
                v = leg[j]
                if geo.dist(a, v) < 0.5 * w or geo.dist(v, b) < 0.5 * w:
                    continue
                h0 = math.atan2(v[1] - a[1], v[0] - a[0])
                h1 = math.atan2(b[1] - v[1], b[0] - v[0])
                if abs(geo.wrap_angle(h1 - h0)) >= sharp:
                    if math.hypot(v[0] - pose[0], v[1] - pose[1]) < 0.1:
                        return h1
                    return math.atan2(v[1] - pose[1], v[0] - pose[0])
        tx, ty = geo._point_at(leg, cum, s_end)
        if math.hypot(tx - pose[0], ty - pose[1]) < 0.1:
            return TransitMixin._leg_heading(leg)
        return math.atan2(ty - pose[1], tx - pose[0])

    def _pivot_unfinished(self, outcome, purpose, key, heading, lim, what):
        """A pivot Spin ended without SUCCEEDED. False: the robot is within ``lim`` of
        ``heading`` (the caller follows the leg, as after a successful pivot). True: handled -
        the Spin is re-sent for the remaining angle (pivot_retries per pivot ``key``), or,
        when those are used up, the robot stops (STUCK_NEEDS_HELP). Following a leg from a
        large heading error is what made MPPI leave the area in field run 6."""
        retries = int(self.p.pivot_retries)
        if retries < 0 or heading is None:
            self._log('warn', '%s pivot %s: following the leg anyway' % (what, outcome))
            return False
        pose = self.inputs.pose
        if pose is None or len(pose) < 3:
            self._log('error', '%s pivot %s and no pose: stopping' % (what, outcome))
            self._stuck_needs_help('%s pivot %s (no pose)' % (what, outcome), (0.0, 0.0))
            return True
        delta = geo.wrap_angle(heading - float(pose[2]))
        if abs(delta) < lim:
            self._log('warn', '%s pivot %s %.0f deg off the leg (< %.0f): following it'
                      % (what, outcome, math.degrees(delta), math.degrees(lim)))
            return False
        n = self._pivot_tries.get(key, 0)
        if n >= retries:
            self._stuck_needs_help('%s pivot %s %d time%s, still %.0f deg off the leg (>= %.0f)'
                                   % (what, outcome, n + 1, '' if n == 0 else 's',
                                      math.degrees(delta), math.degrees(lim)), pose)
            return True
        self._pivot_tries[key] = n + 1
        pause = max(0.0, float(self.p.pivot_retry_pause_s))
        self._log('warn', '%s pivot %s with %.0f deg left (>= %.0f): pivot again %d/%d in '
                          '%.1f s' % (what, outcome, math.degrees(delta), math.degrees(lim),
                                      n + 1, retries, pause))
        # a real back-off between attempts (field 2026-10-22: two "Collision Ahead" re-sends
        # in the same second): the Spin is re-sent after pivot_retry_pause_s (the costmap
        # clears / decays), re-aimed from the pose then
        if pause <= 0.0:
            allowance = abs(delta) / max(0.01, float(self.p.route_cusp_spin_rps)) + 10.0
            self._start_action(ACT_SPIN, {'angle': delta, 'time_allowance_s': allowance},
                               allowance + 5.0, purpose=purpose)
            return True
        self._spin_resend = {'due': self._now + pause, 'heading': heading, 'purpose': purpose,
                             'phase': self.phase}
        self._fx.append(DriveCmd(0.0, 0.0))
        return True

    def _spin_resend_tick(self):
        """The delayed pivot re-send of _pivot_unfinished. True while it owns the tick."""
        r = self._spin_resend
        if r is None:
            return False
        if self.phase != r['phase'] or self._action is not None:
            self._spin_resend = None
            return False
        if self._now < r['due']:
            return True
        self._spin_resend = None
        pose = self.inputs.pose
        if pose is None or len(pose) < 3:
            return False
        delta = geo.wrap_angle(r['heading'] - float(pose[2]))
        allowance = abs(delta) / max(0.01, float(self.p.route_cusp_spin_rps)) + 10.0
        self._start_action(ACT_SPIN, {'angle': delta, 'time_allowance_s': allowance},
                           allowance + 5.0, purpose=r['purpose'])
        return True

    def _cov_pivot_done(self, outcome):
        """Coverage pivot ended: follow the leg (an unfinished pivot first goes through
        _pivot_unfinished: pivot again or stop)."""
        m = self.mission
        if self.phase != 'MOWING' or m.step != 'corner_pivot':
            return
        if outcome == SUCCEEDED:
            self._pivot_tries = {}
        else:
            heading, lim = m.pivot_target if m.pivot_target else (None, 0.0)
            if self._cov_pivot_skip(outcome, heading, lim):
                return
            if self._pivot_unfinished(outcome, 'cov_pivot', ('cov',) + tuple(m.pivot_done or ()),
                                      heading, lim, 'coverage'):
                return
        self._go('MOWING', 'area %d sub-path %d/%d' % (m.area_idx, m.sub_i + 1, len(m.subpaths)))
        self._follow()

    def _cov_pivot_skip(self, outcome, heading, lim):
        """blocked_pivot_policy 'skip' (stock firmware): an unfinished coverage pivot is not
        re-sent; the leg it turned onto (start_local -> next pivot vertex / sub-path end) is
        skipped and recorded, the next leg / sub-path is driven. True = handled. False (the
        robot is within ``lim`` of the leg, the per-mission cap is used up, policy 'retry',
        no pose): the caller's _pivot_unfinished path (follow / re-send / STUCK)."""
        m, p, pose = self.mission, self.p, self.inputs.pose
        if str(p.blocked_pivot_policy).strip().lower() != 'skip' or heading is None or \
                pose is None or len(pose) < 3 or outcome == CANCELED:
            return False
        delta = geo.wrap_angle(heading - float(pose[2]))
        if abs(delta) < lim:
            return False
        cap = int(p.blocked_pivot_max_skips)
        if m.pivot_skips >= cap:
            self._log('warn', 'blocked pivot: %d skips this mission (cap %d): pivot again / '
                      'stop' % (m.pivot_skips, cap))
            return False
        key = (m.area_idx, m.sub_i, m.start_local)
        back = self._safe_backoff(float(p.pivot_backout_m))
        if back > 0.0 and m.stretch is None and m.pivot_backout_key != key:
            m.pivot_backout_key = key
            self._log('warn', 'blocked pivot (%s, %.0f deg off) at (%.2f, %.2f): back up %.2f m, '
                      'then pivot once more' % (outcome, math.degrees(delta), pose[0], pose[1],
                                                back))
            m.step = 'pivot_backout'
            if not self._keep_blade_inside('pivot back-up'):
                self._blade_off('pivot back-up')        # blade_keep_on_obstacles false: off
            self._start_action(ACT_BACKUP, {'distance_m': back,
                                            'speed_mps': float(p.pivot_backout_speed_mps)},
                               back / max(0.02, float(p.pivot_backout_speed_mps))
                               + 15.0, purpose='pivot_backout')
            return True
        m.pivot_skips += 1
        self._pivot_tries = {}
        last = self._seg_last()
        frm = min(m.start_local, last)
        edge = self._edge_lap(m.sub_i)
        end = self._skip_end(frm, float(p.edge_blocked_skip_m) if edge else 0.0)
        why = 'coverage pivot %s, %.0f deg off the leg' % (outcome, math.degrees(delta))
        self._log('warn', 'blocked pivot: skip area %d sub-path %d poses %d -> %d at (%.2f, %.2f) '
                          '(%s), skip %d/%d' % (m.area_idx, m.sub_i, frm, end, pose[0], pose[1],
                                                why, m.pivot_skips, cap))
        self._incident('blocked_pivot', why)
        m.pivot_done, m.pivot_target, m.corner_end = None, None, None
        if edge:
            self._edge_skip(frm, why, end=end)
            return True
        self._skip_piece(frm, end, 'blocked pivot (%s)' % why)
        return True

    def _pivot_backout_done(self, outcome):
        """The back-up before the one extra pivot ended (any outcome): pivot again from here
        (a second refusal skips, see _cov_pivot_skip)."""
        m = self.mission
        if self.phase != 'MOWING' or m is None or m.step != 'pivot_backout':
            return
        m.step = None
        m.pivot_done = None
        if outcome != SUCCEEDED:
            self._log('warn', 'blocked pivot back-up %s: pivoting once more from here' % outcome)
        self._begin_blade()                  # blade spin-up, then _follow -> _cov_pivot

    def _skip_end(self, frm, cap_m=0.0):
        """End of the piece skipped from local ``frm``: the next pivot vertex (or the end of
        what is being mowed), at most ``cap_m`` m along the path when ``cap_m`` > 0."""
        m = self.mission
        last = self._seg_last()
        end = min([c for c in self._cov_pivots() if c > frm] + [last])
        if cap_m > 0.0:
            cum = geo.cumulative_lengths(m.subpaths[m.sub_i])
            idx = bisect.bisect_left(cum, cum[frm] + cap_m)
            end = min(end, max(frm + 1, idx))
        return max(frm, min(end, last))

    def _edge_lap(self, k):
        """Sub-path ``k`` is an edge lap: >= 2 m long, median distance to the area outline or
        an obstacle ring <= edge_lap_max_inset_m (lap 0 runs on the line)."""
        m = self.mission
        ck = (m.area_idx, k)
        if ck not in m.edge_laps:
            area = m.areas.get(m.area_idx) if isinstance(m.areas, dict) else None
            rings = [] if not area else [r for r in [area.get('outer')] +
                                         list(area.get('obstacles') or []) if r and len(r) >= 3]
            sp = m.subpaths[k]
            res = False
            if rings and geo.path_length(sp) >= 2.0:
                step = max(1, len(sp) // 40)
                ds = sorted(min(geo.distance_to_ring(q, r) for r in rings) for q in sp[::step])
                res = ds[len(ds) // 2] <= float(self.p.edge_lap_max_inset_m)
            m.edge_laps[ck] = res
        return m.edge_laps[ck]

    def _skip_piece(self, frm, end, why):
        """Record local ``frm`` -> ``end`` of the current sub-path as skipped (re-tried once at
        the area end, else reported blocked / uncut) and go on after it."""
        m, p = self.mission, self.p
        self._blk = None
        self._soft_blade_off('skip')
        if m.stretch is not None:            # the end-of-area re-try of a skipped piece
            self._stretch_blocked(why)
            return
        last = self._seg_last()
        if end > frm:
            offs, _ = subpath_offsets(m.subpaths)
            if p.blocked_pivot_requeue and p.requeue_skipped:
                self._record_skip(frm, end)
            else:
                m.blocked.append((m.area_idx, offs[m.sub_i] + frm, offs[m.sub_i] + end))
        if end >= last:
            m.step = None
            self._segment_done()
            return
        m.start_local = end
        m.transit_fails = m.follow_fails = 0
        m.route_plan_fails = 0
        m.pivot_done, m.corner_end = None, None
        self._dispatch(force_transit=True)

    def _edge_skip(self, frm, why, end=None):
        """An edge-lap segment is blocked by a real obstacle: end it like stock (no in-place
        retry), skip to the next leg / at most edge_blocked_skip_m."""
        m, pose = self.mission, self.inputs.pose
        last = self._seg_last()
        frm = min(max(frm, 0), last)
        if end is None:
            end = self._skip_end(frm, float(self.p.edge_blocked_skip_m))
        cum = geo.cumulative_lengths(m.subpaths[m.sub_i])
        length = cum[end] - cum[frm] if m.stretch is None else 0.0
        m.edge_skipped_m += length
        x, y = (pose[0], pose[1]) if pose is not None else (float('nan'), float('nan'))
        self._log('warn', 'edge blocked: skip %.1f m (area %d sub-path %d poses %d -> %d at '
                          '(%.2f, %.2f): %s), mission total %.1f m'
                  % (length, m.area_idx, m.sub_i, frm, end, x, y, why, m.edge_skipped_m))
        self._incident('edge_blocked', '%s, %.1f m skipped' % (why, length))
        self._skip_piece(frm, end, 'edge blocked (%s)' % why)

    def _edge_block_tick(self):
        """collision_monitor STOP held edge_blocked_hold_s while following an edge lap (not a
        person / pet): cancel the follow and skip the blocked segment. True = handled."""
        m, p, a = self.mission, self.p, self._action
        since = self.inputs.cm_stop_since
        if not p.edge_blocked_skip or m is None or since is None or self.phase != 'MOWING' or \
                m.step != 'follow' or a is None or a.name != ACT_FOLLOW or \
                self._now - since < float(p.edge_blocked_hold_s) or \
                self._policy() == 'dynamic' or not self._edge_lap(m.sub_i):
            return False
        self._fx.append(CancelActions('edge blocked'))
        self._action = None
        self._edge_skip(self._track_progress(), 'collision_monitor stop held %.1f s'
                        % (self._now - since))
        return True
