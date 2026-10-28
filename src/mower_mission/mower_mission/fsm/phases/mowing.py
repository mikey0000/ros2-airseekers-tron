# SPDX-License-Identifier: GPL-3.0-or-later
"""MissionFSM mixin: Sub-path dispatch, follow, blade and path conditioning.

Moved verbatim from mission_fsm.py (2026-10-23 code review stage 1.6)."""

from mower_mission.fsm.common import (ABORTED, ACT_FOLLOW, CancelActions, PublishRoute, bbody,
    condition_plan, geo, math, pcond)  # noqa: F401


class MowingMixin:
    """Sub-path dispatch, follow, blade and path conditioning."""

    def _next_subpath(self):
        m = self.mission
        ac = self.cursor.area(m.area_idx)
        while m.sub_i < len(m.subpaths) and m.sub_i in ac.completed:
            m.sub_i += 1
        if m.sub_i >= len(m.subpaths):
            self._area_done()
            return
        m.start_local = 0
        m.boundary_recoveries = 0
        m.premature = 0
        if m.resume_sub is not None and m.resume_sub[0] == m.sub_i:
            m.start_local = m.resume_sub[1]
            m.resume_sub = None
        m.retry_used = False
        m.transit_fails = m.follow_fails = m.recovery_fails = 0
        m.route_plan_fails = 0
        m.recovering = False
        m.detours, m.detour = 0, None
        m.cusp_turns = {}
        m.leg_watch = None
        m.corner_end, m.pivot_done = None, None
        self._blk = None
        self._dispatch()

    def _dispatch(self, force_transit=False):
        """Send the current sub-path from ``start_local``: blade-off transit
        first when its start is farther than ``transit_gap_m``."""
        m = self.mission
        sp = m.subpaths[m.sub_i]
        if m.sub_i in m.cond_errors:
            self._subpath_failed('path conditioning: %s' % m.cond_errors[m.sub_i])
            return
        if self._seg_last() - m.start_local < 1:
            self._segment_done()
            return
        if self._turn_leg_driven():
            return
        target = sp[m.start_local]
        if force_transit or geo.needs_transit(self.inputs.pose, target, self.p.transit_gap_m):
            # SAFETY (FollowStrip): blade off before any dispatch across the gap, unless
            # the continuous policy keeps it on for a transit inside the area.
            self._transit_blade(target, 'transit', force_off=force_transit)
            m.step = 'transit'
            self._go('TRANSIT', 'area %d sub-path %d/%d' % (
                m.area_idx, m.sub_i + 1, len(m.subpaths)))
            yaw = geo.heading_of(sp, m.start_local)
            m.transit_target = self._transit_approach(target, yaw)
            m.route_fallback, m.route_aborts = False, 0
            self._send_transit()
        else:
            self._begin_blade()

    def _transit_approach(self, target, yaw):
        """Transit goal for a sub-path start: a start on (or within edge_approach_near_m of)
        the area line - an edge-lap start / resume point - is approached at a point
        edge_approach_inset_m inside, beside it (sim 2026-10-22: transits arriving square to
        the line put the nose 0.40 m out). Not where another edge is closer to that point
        than the start's own (a corner: run5 (-1.83, 4.48)). The leg-start pivot and the join then take the
        robot onto the lap from inside. edge_approach_inset_m <= 0: the start itself."""
        x, y = float(target[0]), float(target[1])
        inset = float(self.p.edge_approach_inset_m)
        if self.mission is not None:
            self.mission.offgraph = None
        if inset <= 0.0 or not self._region_areas:
            return (x, y, yaw)
        polys = self._region()
        q = bbody.nearest_edge_point(x, y, polys)
        if q is None or q[2] > float(self.p.edge_approach_near_m):
            return (x, y, yaw)
        best = None
        for sgn in (1.0, -1.0):
            nx, ny = -math.sin(yaw) * sgn, math.cos(yaw) * sgn
            cx, cy = x + inset * nx, y + inset * ny
            if bbody.point_outside(cx, cy, polys) > 0.0:
                continue
            e = bbody.nearest_edge_point(cx, cy, polys)
            c = e[2] if e is not None else 0.0
            if best is None or c > best[0]:
                best = (c, cx, cy, sgn)
        if self.mission is not None:
            self.mission.join = None
        if best is None or best[0] < inset - 0.02:
            return (x, y, yaw)     # another edge closer than the start's own (a corner)
        self._log('info', 'transit to an edge start (%.2f, %.2f): approach point %.2f m inside '
                          'at (%.2f, %.2f)' % (x, y, inset, best[1], best[2]))
        m = self.mission
        if m is not None and float(self.p.edge_join_m) > 0.0:
            # the leg from this start is merged onto tangentially (_join_leg)
            m.join = (m.sub_i, m.start_local, best[3], inset)
        return (best[1], best[2], yaw)

    def _join_leg(self, poses, k, frm):
        """Round 4 tangent join: the leg sub-path ``k`` from local ``frm``, when the transit
        ended at its edge start's approach point (m.join), offset toward the interior by the
        approach inset fading out (cosine) over edge_join_m of arc length - the robot merges
        onto the lap along the curve instead of aiming across it (owner_far's lap start on a
        drawn arc). Otherwise ``poses`` unchanged."""
        m = self.mission
        j = m.join if m is not None else None
        if j is None or j[0] != k or j[1] != frm or len(poses) < 2:
            return poses
        D, sgn, off = float(self.p.edge_join_m), j[2], j[3]
        cum = geo.cumulative_lengths(poses)
        out = []
        for i, q in enumerate(poses):
            if cum[i] >= D:
                out.append(tuple(q))
                continue
            a, b = poses[max(0, i - 1)], poses[min(len(poses) - 1, i + 1)]
            h = math.atan2(b[1] - a[1], b[0] - a[0])
            w = off * 0.5 * (1.0 + math.cos(math.pi * cum[i] / D))
            nx, ny = -math.sin(h) * sgn, math.cos(h) * sgn
            out.append((q[0] + w * nx, q[1] + w * ny) + tuple(q[2:]))
        res = []
        for i, q in enumerate(out):
            if len(q) >= 3:
                a, b = out[max(0, i - 1)], out[min(len(out) - 1, i + 1)]
                q = (q[0], q[1], math.atan2(b[1] - a[1], b[0] - a[0])) + tuple(q[3:])
            res.append(q)
        return res

    def _is_turn_leg(self):
        """The current sub-path is a turn leg of the turn_leg_skip rule: a (short) reverse
        leg of the coverage bridge, or the forward piece that ends at the cusp of one."""
        m = self.mission
        sp = m.subpaths[m.sub_i]
        leg_max = float(self.p.turn_leg_max_m)

        def reverse_leg(poses):     # a (short) reverse turn leg of the coverage bridge
            return geo.is_reverse_subpath(poses) and geo.path_length(poses) <= leg_max

        nxt = m.subpaths[m.sub_i + 1] if m.sub_i + 1 < len(m.subpaths) else None
        cusp_next = nxt is not None and reverse_leg(nxt) and geo.dist(nxt[0], sp[-1]) <= 0.01
        # Task 0.4 (f): this rule stays for turn_leg_skip (distance + yaw, safe on a long
        # swath ending at a cusp: the 2026-10-13 retry case); the _leg_watch cap no longer
        # uses it (a 49 m ring ending at a cusp was capped in run 3).
        return reverse_leg(sp) or cusp_next

    def _turn_leg_driven(self):
        """Skip the rest of a turn leg the robot has already all but driven (see
        ``turn_leg_skip_m``): done -> True (the next sub-path has been dispatched)."""
        m, pose = self.mission, self.inputs.pose
        skip_m = float(self.p.turn_leg_skip_m)
        if skip_m <= 0.0 or pose is None or len(pose) < 3 or m.stretch is not None:
            return False
        sp = m.subpaths[m.sub_i]
        end = sp[-1]
        if len(end) < 3:
            return False
        if not self._is_turn_leg():
            return False
        d = geo.dist(pose, end)
        dyaw = abs(math.atan2(math.sin(pose[2] - end[2]), math.cos(pose[2] - end[2])))
        if d > skip_m or dyaw > float(self.p.turn_leg_skip_yaw):
            return False
        self._log('info', 'area %d sub-path %d: turn leg end %.2f m / %.0f deg away: leg '
                          'done, next leg from here (no re-send, no turn in place)'
                  % (m.area_idx, m.sub_i, d, math.degrees(dyaw)))
        m.step = None
        self._subpath_done()
        return True

    def _leg_watch(self):
        """Fix 10 (2026-10-14 run 2), called on every MOWING tick while a follow_path runs.

        * Turn-leg end cap: once the leg's end is reached in xy (within turn_leg_skip_m of
          the end pose; task 0.4 f: never the tracked pose index, which can jump to the last
          pose early, and only for legs no longer than turn_leg_max_m) the leg is done after
          turn_leg_done_after_s OR turn_leg_done_after_rot_rad of accumulated |yaw change|
          of the map pose, whatever the yaw. A leg whose end yaw is met is left to the
          controller's leg goal checker (and the turn_leg_skip rule on a re-send), as before.
        * Loop guard: a follow_path whose tracked pose index has not advanced for
          leg_loop_abort_s is cancelled and handled as a follow_path abort.

        Returns True when it ended / aborted the follow (the tick is done)."""
        m, a, pose, p = self.mission, self._action, self.inputs.pose, self.p
        if a is None or a.name != ACT_FOLLOW or m.step != 'follow' or not m.subpaths:
            return False
        sp = m.subpaths[m.sub_i]
        key = (m.area_idx, m.sub_i, m.stretch)
        w = m.leg_watch
        by_dist = bool(p.turn_leg_end_by_distance)
        if w is None or w['key'] != key:
            short = geo.path_length(sp) <= float(p.turn_leg_max_m)
            w = m.leg_watch = {
                'key': key, 'reached': None, 'rot': 0.0, 'yaw': None, 'token': None,
                'idx': None, 'since': self._now,
                # task 0.4 (f): capped only by the leg's own length
                'capped': short if by_dist else (self._is_turn_leg() or short)}
        if w['token'] != a.token:            # a new goal (retry / resume): fresh loop timer
            w['token'], w['idx'], w['since'], w['yaw'] = a.token, m.progress_local, self._now, None
        elif m.progress_local != w['idx']:
            w['idx'], w['since'] = m.progress_local, self._now
        # ---- turn-leg end cap ----
        cap_s, cap_rot = float(p.turn_leg_done_after_s), float(p.turn_leg_done_after_rot_rad)
        if w['capped'] and m.stretch is None and pose is not None and len(pose) >= 3:
            end = sp[-1]
            skip_m = float(p.turn_leg_skip_m)
            d = geo.dist(pose, end)
            if w['reached'] is not None and w['yaw'] is not None:
                w['rot'] += abs(geo.wrap_angle(float(pose[2]) - w['yaw']))
            w['yaw'] = float(pose[2])
            # task 0.4 (f): distance only; progress_local can jump to the last pose early
            at_end = (skip_m > 0.0 and d <= skip_m) or \
                (not by_dist and m.progress_local >= len(sp) - 1)
            if w['reached'] is None and at_end:
                w['reached'], w['rot'] = self._now, 0.0
            if w['reached'] is not None:
                dyaw = abs(geo.wrap_angle(float(pose[2]) - float(end[2]))) if len(end) >= 3 \
                    else math.inf
                why = None
                if cap_s > 0.0 and self._now - w['reached'] >= cap_s:
                    why = 'time cap: %.0f s since the end was reached' % (self._now - w['reached'])
                elif cap_rot > 0.0 and w['rot'] >= cap_rot:
                    why = 'rotation cap: %.0f deg turned since the end was reached' \
                        % math.degrees(w['rot'])
                if why is not None:
                    self._log('warn', 'area %d sub-path %d: turn leg end %.2f m / %.0f deg away (%s): '
                              'leg done, next leg from here (no re-send, no turn in place)'
                              % (m.area_idx, m.sub_i, d, math.degrees(min(dyaw, math.pi)), why))
                    self._fx.append(CancelActions('turn leg done'))
                    self._action = None
                    m.step = None
                    m.leg_watch = None
                    self._subpath_done()
                    return True
        # ---- loop guard ----
        loop_s = float(p.leg_loop_abort_s)
        if loop_s > 0.0 and self._now - w['since'] >= loop_s:
            msg = 'pose %d not advanced for %.0f s (leg loop guard)' % (
                m.progress_local, self._now - w['since'])
            self._log('warn', 'area %d sub-path %d: %s: cancelling follow_path, handled as an '
                              'abort' % (m.area_idx, m.sub_i, msg))
            self._fx.append(CancelActions('leg loop guard'))
            self._action = None
            w['token'] = None
            self._handle_action(a, ABORTED, {'message': msg, 'leg_loop': True})
            return True
        return False

    def _begin_blade(self):
        m = self.mission
        if self.inputs.boundary_violation:
            self._boundary_pause()       # never spin the blade up outside the boundary
            return
        self._go('MOWING', 'area %d sub-path %d/%d' % (m.area_idx, m.sub_i + 1, len(m.subpaths)))
        if self._cov_kturn_leg():
            # K-turn reverse leg (and its about-turn at P1): never reverse with the blade on;
            # the forward leg after it spins the blade up again at V
            if self.blade_on:
                self._blade_off('K-turn reverse leg')
            self._follow()
            return
        if self.blade_on and (self.inputs.is_cutting or not self.p.require_blade_confirmation):
            self._follow()
            return
        self._spin_attempt = 1
        self._spin_up()

    def _spin_up(self):
        m = self.mission
        m.step = 'spinup'
        self._blade_on()
        wait = self.p.blade_confirm_timeout_s if self.p.require_blade_confirmation \
            else self.p.blade_spinup_s
        self._spin_deadline = self._now + wait

    def _track_progress(self):
        """Advance ``progress_local`` to the pose nearest the robot, searching
        only ``progress_window_m`` ahead (never backwards, never onto a later lap)."""
        m, pose = self.mission, self.inputs.pose
        if pose is None or not m.subpaths:
            return m.progress_local
        m.progress_local = geo.nearest_index(m.subpaths[m.sub_i], pose, m.progress_local,
                                             self.p.progress_window_m)
        return m.progress_local

    def _tick_blade(self):
        m = self.mission
        if m.step == 'spinup':
            if self.p.require_blade_confirmation:
                if self.inputs.is_cutting:
                    self._follow()
                elif self._now >= self._spin_deadline:
                    self._blade_off('blade start attempt %d not confirmed' % self._spin_attempt)
                    if self._spin_attempt >= self.p.blade_start_attempts:
                        self._mission_failed('blade did not start (is_cutting false after %d '
                                             'attempts)' % self._spin_attempt)
                        return
                    self._log('warn', 'blade not cutting after %.0fs (attempt %d/%d): retrying'
                              % (self.p.blade_confirm_timeout_s, self._spin_attempt,
                                 self.p.blade_start_attempts))
                    m.step = 'spin_pause'
                    self._spin_deadline = self._now + self.p.blade_retry_pause_s
            elif self._now >= self._spin_deadline:
                self._follow()
        elif m.step == 'spin_pause' and self._now >= self._spin_deadline:
            self._spin_attempt += 1
            self._spin_up()

    def _follow(self):
        m = self.mission
        if self._cov_pivot():
            return
        m.step = 'follow'
        m.progress_local = m.start_local
        sp = m.subpaths[m.sub_i]
        m.chunk_end = min(geo.chunk_end(sp, m.start_local, self.p.follow_chunk_m,
                                        self.p.follow_chunk_clearance_m), self._seg_last())
        m.corner_end = None
        for c in self._cov_pivots():
            if m.start_local < c <= m.chunk_end and c < self._seg_last():
                m.chunk_end = m.corner_end = c       # this chunk ends on the pivot vertex
                break
        if m.join is not None and m.join[0] == m.sub_i and m.join[1] == m.start_local:
            # the gate's (h) rebuilds the sent path from this line (keep the format)
            self._log('info', 'edge start join: sub %d local %d side %+d inset %.3f over %.2f m'
                      % (m.sub_i, m.start_local, int(m.join[2]), float(m.join[3]),
                         float(self.p.edge_join_m)))
        poses = self._join_leg(list(sp[m.start_local:m.chunk_end + 1]), m.sub_i, m.start_local)
        m.join = None
        reverse = geo.is_reverse_subpath(sp)
        checker = self._follow_goal_checker(poses, reverse)
        if (m.corner_end is not None or m.sub_i in m.cond) and not reverse and \
                self.p.route_corner_goal_checker_id:
            # a conditioned forward leg (into a pivot vertex, to a chunk end or to the
            # sub-path end) ends when the robot has come up to its end along the path
            # (coverage_corner_goal_checker, along_path), not on a position disc
            checker = self.p.route_corner_goal_checker_id
        self._start_action(ACT_FOLLOW, {
            'poses': poses,
            'controller_id': self.coverage_controller_id(reverse),
            'goal_checker_id': checker,
            'progress_checker_id': self.p.follow_progress_checker_id},
            self.p.follow_timeout_s)

    # ---- path conditioning (design 2026-10-15 steps 2-3) --------------------
    def _cond_params(self):
        p = self.p
        return pcond.Params(
            r_min=float(p.path_conditioning_r_min_m), r_max=float(p.path_conditioning_r_max_m),
            margin_m=float(p.path_conditioning_margin_m),
            half_width_m=float(p.path_conditioning_half_width_m),
            slack_m=float(p.path_conditioning_slack_m),
            pivot_min_rad=float(p.path_conditioning_pivot_min_rad),
            edge_pivot_rad=float(p.path_conditioning_edge_pivot_rad),
            edge_clearance_m=float(p.path_conditioning_edge_clearance_m))

    def _plan_outline(self, outer):
        """An area / path outline as the planner and the conditioner see it: drawing jogs
        dropped and Douglas-Peucker'd in memory (geometry.clean_area_ring; the owner's areas
        file and the mission's area snapshot, which the body boundary check uses, keep the
        recorded outline)."""
        pts = [tuple(q[:2]) for q in (outer or [])]
        tol, jog = float(self.p.area_outline_simplify_m), float(self.p.area_outline_jog_m)
        if len(pts) < 4 or (tol <= 0.0 and jog <= 0.0):
            return list(pts)
        ring = geo.clean_area_ring(pts, max(0.0, tol), max(0.0, jog))
        return ring if len(ring) >= 3 else list(pts)

    def _region(self):
        """Allowed region polygons: mowing areas U drawn path channels U the dock corridor
        (the dock -> approach capsule as a rectangle); outlines as the planner got them."""
        polys = [self._plan_outline(a['outer']) for a in (self._region_areas or {}).values()
                 if len(a.get('outer') or []) >= 3]
        dock = self.inputs.dock_pose
        if dock is not None and len(dock) >= 3:
            x, y, yaw = float(dock[0]), float(dock[1]), float(dock[2])
            L, w = float(self.p.dock_approach_distance_m), 0.35
            c, sn = math.cos(yaw), math.sin(yaw)
            polys.append([(x - w * c + w * sn, y - w * sn - w * c),
                          (x + (L + w) * c + w * sn, y + (L + w) * sn - w * c),
                          (x + (L + w) * c - w * sn, y + (L + w) * sn + w * c),
                          (x - w * c - w * sn, y - w * sn + w * c)])
        return polys

    def _conditioning_job(self, subpaths):
        p, prm = self.p, self._cond_params()
        mode = str((self.mission.settings or {}).get('edge_mode', 'soft')).strip().lower()
        lim = float(p.path_conditioning_body_limit_hard_m if mode == 'hard'
                    else p.path_conditioning_body_limit_soft_m)
        prm.body_limit_m = lim if lim >= 0.0 else None
        cap = float(p.path_conditioning_body_cap_hard_m if mode == 'hard'
                    else p.path_conditioning_body_cap_soft_m)
        prm.body_cap_m = cap if cap >= 0.0 else None
        prm.convex_r_max = float(p.path_conditioning_convex_r_max_m)
        prm.kturn = bool(p.path_conditioning_kturn)
        prm.kturn_min_rad = float(p.path_conditioning_kturn_min_rad)
        prm.kturn_gentle_m = float(p.path_conditioning_kturn_gentle_m)
        prm.fillet_envelope_margin_m = float(p.path_conditioning_fillet_envelope_margin_m)
        prm.kturn_back_max_m = float(p.path_conditioning_kturn_back_max_m)
        return {'subpaths': [list(sp) for sp in subpaths], 'region': self._region(),
                'prm': prm, 'leg_max_m': float(self.p.turn_leg_max_m),
                'area': self.mission.area_idx}

    def _condition_subpaths(self, subpaths, pre=None):
        """Apply a coverage plan's path conditioning: ``pre`` (condition_plan output the
        node computed off the lock for these sub-paths) or computed inline."""
        m = self.mission
        if pre is None or pre.get('n_in') != len(subpaths):
            pre = condition_plan(self._conditioning_job(subpaths))
        m.cond, m.cond_errors = dict(pre['cond']), dict(pre['cond_errors'])
        m.corner_body = list(pre.get('corner_body') or [])
        for level, text in pre['logs']:
            self._log(level, text)
        return [list(sp) for sp in pre['subpaths']]

    def _condition_route(self, poses, what):
        """A transit / dock route -> FollowPath legs split at the conditioner's pivots."""
        prm = self._cond_params()
        # routes are tracked at transit speed by the route controller: a fillet tighter than
        # path_conditioning_route_r_min_m is not drivable there (sim gate 2026-10-16: MPPI ran
        # 0.4 m wide of an r 0.5 m dock-path fillet and left the area) -> pivot at the vertex
        prm.r_min = max(prm.r_min, float(self.p.path_conditioning_route_r_min_m))
        prm.r_max = max(prm.r_max, prm.r_min)
        r = pcond.condition(poses, self._region(), prm, ring=False)
        if not r.valid:
            self._log('error', '%s: path conditioning failed (%s): raw route split as before'
                      % (what, r.errors[0]))
            return None
        legs, s0 = [], 0
        for c in r.pivots:
            if 0 < c < len(r.poses) - 1:
                legs.append(list(r.poses[s0:c + 1]))
                s0 = c
        legs.append(list(r.poses[s0:]))
        legs = [lg for lg in legs if len(lg) >= 2] or [list(r.poses)]
        self._fx.append(PublishRoute(what, list(r.poses), list(r.types)))
        self._log('info', '%s: conditioned: %s' % (what, ', '.join(
            '%s %+.0f deg%s' % (c.choice, math.degrees(c.turn), ' r %.1f' % c.r if c.r else '')
            for c in r.corners) or 'no corners'))
        return legs

    def coverage_controller_id(self, reverse):
        """Nav2 controller id for a coverage sub-path (forward or reverse leg)."""
        return (self.p.mppi_reverse_controller_id if reverse
                else self.p.mppi_follow_controller_id)

    def coverage_speed_params(self, speed):
        """{controller_server param: value} for a coverage cut speed."""
        return {self.p.mppi_controller_speed_param:
                min(float(speed), float(self.p.mppi_speed_max_mps))}

    def _follow_goal_checker(self, poses, reverse):
        """Goal checker for a FollowPath chunk: the leg checker for turn legs."""
        if self.p.turn_leg_goal_checker_id and (
                reverse or geo.path_length(poses) <= self.p.turn_leg_max_m):
            return self.p.turn_leg_goal_checker_id
        return self.p.follow_goal_checker_id
