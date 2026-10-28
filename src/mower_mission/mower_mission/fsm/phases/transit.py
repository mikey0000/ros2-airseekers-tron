# SPDX-License-Identifier: GPL-3.0-or-later
"""MissionFSM mixin: Route-graph transits, transit retries and back-offs.

Moved verbatim from mission_fsm.py (2026-10-23 code review stage 1.6)."""

from mower_mission.fsm.common import (ABORTED, ACT_BACKUP, ACT_DRIVE, ACT_FOLLOW, ACT_NAV,
    ACT_PLAN, ACT_SPIN, ACT_UNDOCK, CANCELED, CancelActions, DOCK_PHASES, NAV2_DOWN,
    NAV2_INACTIVE_KEY, REJECTED, ROUTE_CORNER_TOL_RAD, SRV_PLAN_ROUTE, SUCCEEDED,
    SaveAlternateCounts, TIMEOUT, UNAVAILABLE, bbody, geo, math, nav2_down_why, re)  # noqa: F401
from mower_mission.fsm.budget import Budget


class TransitMixin:
    """Route-graph transits, transit retries and back-offs."""

    # ==================================================================
    # results
    # ==================================================================
    # ---- route-graph transits (2026-10-09, see Params.route_transits) ---------
    TRANSIT_RUN_KEY = '__transit_run__'

    def _transit_run(self):
        """Run index for plan_route's transit variation (0 = base route)."""
        if not self.p.route_variation:
            return 0
        return int(self.alternate_counts.get(self.TRANSIT_RUN_KEY, 0))

    def _bump_transit_run(self):
        if self.p.route_transits and self.p.route_variation:
            self.alternate_counts[self.TRANSIT_RUN_KEY] = self._transit_run() + 1
            self._fx.append(SaveAlternateCounts(dict(self.alternate_counts)))

    def _send_transit(self):
        """(Re-)send the in-flight transit to ``m.transit_target``: the route first, the
        costmap planner (/navigate_to_pose) once this transit fell back."""
        m = self.mission
        m.route_poses = None
        m.route_full, m.route_cusps, m.cusp_resume = None, [], None
        m.final_approach = False
        if self.p.route_transits and not m.route_fallback and m.detour is None \
                and not m.recovering and self.inputs.pose is not None:
            m.fallback_ref = None
            self._call(SRV_PLAN_ROUTE, {'start': tuple(self.inputs.pose),
                                        'goal': tuple(m.transit_target),
                                        'variation': self._transit_run()}, purpose='route')
            return
        self._start_fallback_nav()

    def _start_fallback_nav(self, ref=None):
        """/navigate_to_pose to the transit target. A route fallback gets the corridor bound
        (transit_fallback_max_offpath_m) against ``ref`` (the route it replaces, if any; a
        re-send keeps the previous one) and the mowing areas."""
        m = self.mission
        if ref is not None:
            m.fallback_ref = [tuple(q) for q in ref]
        m.fallback_armed = None
        self._start_action(ACT_NAV, {'pose': m.transit_target}, self.p.transit_timeout_s)

    def _offpath(self, pose):
        """Distance of ``pose`` from the drawn network the fallback must stay on: the route
        it replaces (cross-track), 0 inside a mowing area (outer ring); inf when there is
        nothing to compare with."""
        m = self.mission
        best = math.inf
        if m.fallback_ref:
            best = geo.polyline_distance(pose, m.fallback_ref)
        for area in (m.areas or {}).values():
            outer = list(area.get('outer') or [])
            if len(outer) < 3:
                continue
            if geo.point_in_polygon(pose, outer):
                return 0.0
            best = min(best, geo.distance_to_ring(pose, outer))
        return best

    def _fallback_corridor_tick(self):
        """Fix 11: cancel a costmap-planner route fallback that leaves the drawn network by
        more than transit_fallback_max_offpath_m (live cross-track of the robot's own pose:
        the FSM never sees Nav2's planned path) and count it as a transit abort."""
        m, a, pose = self.mission, self._action, self.inputs.pose
        lim = float(self.p.transit_fallback_max_offpath_m)
        if lim <= 0.0 or a is None or a.name != ACT_NAV or not m.route_fallback or \
                m.detour is not None or m.recovering or pose is None:
            return
        off = self._offpath(pose)
        if math.isinf(off):
            return
        if m.fallback_armed is None:          # first tick of this fallback goal
            m.fallback_armed, m.fallback_off0 = off <= lim, off
            if not m.fallback_armed:
                self._log('info', 'transit fallback starts %.2f m off the drawn network: '
                                  'corridor bound armed once within %.2f m' % (off, lim))
            return
        if not m.fallback_armed:
            if off <= lim:
                m.fallback_armed = True
            elif off <= m.fallback_off0 + lim:
                m.fallback_off0 = min(m.fallback_off0, off)
                return
        if off <= lim:
            return
        self._log('warn', 'transit fallback left the corridor by %.2f m: abort' % off)
        self._fx.append(CancelActions('transit fallback left the corridor'))
        self._action = None
        self._transit_abort('transit fallback left the corridor (%.2f m)' % off)

    def _transit_nav_progress_tick(self):
        """2026-10-15 sim (forced spin): after a route abort the /navigate_to_pose transit
        (costmap planner; its FollowPath keeps the rotation-counting progress_checker) spun in
        place for minutes inside the corridor. The robot's distance to the transit target must
        shrink by route_progress_min_m within route_progress_window_s, else the goal is
        cancelled and counted as a transit abort. True when it fired."""
        win, need = float(self.p.route_progress_window_s), float(self.p.route_progress_min_m)
        m, a, pose = self.mission, self._action, self.inputs.pose
        if win <= 0.0 or a is None or a.name != ACT_NAV or a.purpose is not None or \
                m.detour is not None or pose is None or m.transit_target is None:
            return False
        d = geo.dist(pose, m.transit_target)
        w = self._route_watch
        if w is None or w['token'] != a.token:
            self._route_watch = {'token': a.token, 'd': d, 't': self._now}
            return False
        if d <= w['d'] - need:
            w['d'], w['t'] = d, self._now
            return False
        if self._now - w['t'] < win:
            return False
        self._log('warn', 'transit navigate_to_pose: %.2f m from the target, no %.2f m closer '
                          'for %.0f s (progress watch): abort' % (d, need, self._now - w['t']))
        self._fx.append(CancelActions('transit progress watch'))
        self._action = None
        self._route_watch = None
        self._transit_abort('transit made no progress for %.0f s' % win)
        return True

    def _transit_abort(self, why):
        """A failed mission transit: transit_fails, 'transit_abort' incident, then a retry
        after the back-off or (retries exhausted) the sub-path is dropped."""
        m = self.mission
        m.transit_fails += 1
        self._incident('transit_abort', why)
        if m.transit_fails > self.p.transit_retries:
            self._subpath_failed('%s %d times' % (why, m.transit_fails), transit=True)
            return
        delay = self._retry_delay(m.transit_fails)
        self._schedule_retry('TRANSIT', 'area %d sub-path %d/%d: %s, retry %d/%d in %.0f s'
                             % (m.area_idx, m.sub_i + 1, len(m.subpaths), why,
                                m.transit_fails, self.p.transit_retries, delay), delay=delay)

    def _route_planned(self, ok, resp):
        m = self.mission
        if m is None or self.phase != 'TRANSIT' or m.step != 'transit' or \
                self._action is not None:
            return                       # superseded (stop, guard, wait) while planning
        poses = resp.get('poses') or []
        if not (ok and resp.get('success') and len(poses) >= 2):
            why = resp.get('message') or 'plan_route unavailable'
            if re.search(r'goal .*off the route graph', why) and self._offgraph_goal(why):
                return
            if self._route_replan_far(why):
                return
            m.route_fallback = True
            self._log('info', 'no route (%s): transit via the costmap planner' % why)
            m.fallback_ref = None
            self._start_fallback_nav()
            return
        m.route_plan_fails = 0
        if m.blade_transit and int(resp.get('single_area', -1)) != m.area_idx:
            # the straight segment stays in the area but the route does not (path legs)
            self._blade_off('route leaves area %d' % m.area_idx)
        m.route_poses = self._route_prepare(poses)
        legs = (self._condition_route(m.route_poses, 'route') if self.p.path_conditioning
                else None) or self._split_route(m.route_poses)
        m.route_full = m.route_poses
        m.route_cusps = self._route_cusps(m.route_poses, legs)
        self._log('info', 'route %.1f m (%.1f m on paths, %d leg%s): %s' % (
            float(resp.get('length_m', 0.0)), float(resp.get('path_length_m', 0.0)),
            len(legs), '' if len(legs) == 1 else 's', resp.get('message', '')))
        self._route_legs = {'legs': legs, 'i': 0}
        self._send_route_leg('route')

    def _offgraph_goal(self, why):
        """The transit goal is off the route graph: re-ask the route to the goal moved inside
        (offgraph_goal_insets_m, toward the interior); none left: skip this piece (NOT
        mowed / blocked) instead of a costmap-planner transit to it. True = handled."""
        m = self.mission
        insets = [float(v) for v in (self.p.offgraph_goal_insets_m or [])]
        if not insets or m.transit_target is None:
            return False
        if m.offgraph is None or m.offgraph[0] is None:
            m.offgraph = (tuple(m.transit_target), 0)
        goal, k = m.offgraph
        polys = self._region()
        x, y, yaw = goal[0], goal[1], goal[2] if len(goal) >= 3 else 0.0
        while k < len(insets):
            d = insets[k]
            k += 1
            best = None
            for sgn in (1.0, -1.0):
                cx, cy = x - math.sin(yaw) * sgn * d, y + math.cos(yaw) * sgn * d
                if bbody.point_outside(cx, cy, polys) > 0.0:
                    continue
                e = bbody.nearest_edge_point(cx, cy, polys)
                c = e[2] if e is not None else 0.0
                if best is None or c > best[0]:
                    best = (c, cx, cy)
            if best is None:
                continue
            m.offgraph = (goal, k)
            m.transit_target = (best[1], best[2], yaw)
            self._log('info', 'no route (%s): re-asking with the goal %.1f m inside at '
                              '(%.2f, %.2f)' % (why, d, best[1], best[2]))
            self._call(SRV_PLAN_ROUTE, {'start': tuple(self.inputs.pose),
                                        'goal': tuple(m.transit_target),
                                        'variation': self._transit_run()}, purpose='route')
            return True
        m.offgraph = None
        self._log('warn', 'no route (%s), not even from inside: skipping it (no costmap-planner '
                          'transit to an off-network goal)' % why)
        self._incident('transit_abort', 'goal off the route graph')
        self._subpath_failed('transit goal off the route graph')
        return True

    def _route_replan_far(self, why):
        """plan_route failed for a transit goal beyond transit_fallback_max_goal_m (outside
        the costmap planner's rolling window): re-ask the route after a back-off
        (route_plan_retries), then a transit abort (its own bounded retries). True = handled
        (no costmap-planner fallback)."""
        m, pose, lim = self.mission, self.inputs.pose, float(self.p.transit_fallback_max_goal_m)
        if lim <= 0.0 or pose is None or m.transit_target is None:
            return False
        d = geo.dist(pose, m.transit_target)
        if d <= lim:
            return False
        m.route_plan_fails += 1
        if m.route_plan_fails > int(self.p.route_plan_retries):
            self._transit_abort('no route (%s) to a goal %.0f m away (beyond the costmap '
                                'planner)' % (why, d))
            return True
        delay = Budget.backoff(self.p.route_plan_retry_backoff_s, m.route_plan_fails,
                               self.p.transit_retry_delay_s)
        self._schedule_retry('TRANSIT', 'no route (%s): goal %.1f m away is beyond the costmap '
                                        'planner (%.0f m): re-asking the route %d/%d in %.0f s'
                             % (why, d, lim, m.route_plan_fails, int(self.p.route_plan_retries),
                                delay), delay=delay)
        return True

    def _route_corner_th(self):
        """Corner threshold of the route split and pivots (rad; 0 = off)."""
        th = float(self.p.route_corner_turn_rad)
        if th > 0.0 and float(self.p.route_split_turn_deg) > 0.0:
            th = min(th, math.radians(float(self.p.route_split_turn_deg)))
        return th

    def _route_prepare(self, poses):
        """Planned route -> the poses to drive (2026-10-15 undock incident): stubs shorter
        than route_stub_trim_m before the first / after the last corner are cut off, then the
        route starts where the robot projects onto its first route_start_join_m."""
        poses = [tuple(p) for p in poses]
        th, stub = self._route_corner_th(), float(self.p.route_stub_trim_m)
        # path conditioning: no stub trimming (design 3.3) - a corner near a route end is a
        # pivot marker like any other, and the stub after it a leg that ends along-path on
        # the target (the trim ended transits up to 0.3 m short of the sub-path start)
        if th > 0.0 and stub > 0.0 and not self.p.path_conditioning:
            out = geo.trim_route_stubs(poses, th, stub, tol_rad=ROUTE_CORNER_TOL_RAD)
            if len(out) != len(poses):
                cut0 = poses.index(out[0])
                self._log('info', 'route: %.2f m stub before the corner at (%.2f, %.2f) and '
                                  '%.2f m after (%.2f, %.2f) dropped' % (
                                      geo.path_length(poses[:cut0 + 1]), out[0][0], out[0][1],
                                      geo.path_length(poses[cut0 + len(out) - 1:]),
                                      out[-1][0], out[-1][1]))
                poses = out
        join, pose = float(self.p.route_start_join_m), self.inputs.pose
        if join > 0.0 and pose is not None:
            out = geo.join_route_from(poses, pose, join)
            if out[0] != poses[0]:
                self._log('info', 'route: first leg joins at (%.2f, %.2f), the robot\'s '
                                  'projection, not the start (%.2f, %.2f) behind it'
                          % (out[0][0], out[0][1], poses[0][0], poses[0][1]))
                poses = out
        return poses

    def _split_route(self, poses):
        """Route poses -> FollowPath legs. Task 0.4 (d), route_corner_turn_rad > 0: split at
        every corner of at least that angle (geometry.route_corners: the bend between the
        straight runs either side, robust to pose spacing and drawing wobble; neighbouring
        sharp corners each get their own split), the leg ending exactly on the corner pose
        with its yaw along the run into the corner. 0 = the same split at
        route_split_turn_deg (the 3-pose-chord split it replaced was deleted on 2026-10-23;
        both 0 = one goal)."""
        th = self._route_corner_th()
        if th <= 0.0:
            th = math.radians(max(0.0, float(self.p.route_split_turn_deg)))
        poses = list(poses)
        if th <= 0.0:
            return [poses]
        cuts = [i for i, _t in geo.route_corners(poses, th, tol_rad=ROUTE_CORNER_TOL_RAD)]
        if not cuts:
            return [poses]
        legs, s0 = [], 0
        for c in cuts:
            leg = list(poses[s0:c + 1])
            end = leg[-1]
            if len(end) >= 3:                 # arrive along the run into the corner
                a = poses[c - 1]
                leg[-1] = (end[0], end[1], math.atan2(end[1] - a[1], end[0] - a[0])) + \
                    tuple(end[3:])
            legs.append(leg)
            s0 = c
        legs.append(list(poses[s0:]))
        return [lg for lg in legs if len(lg) >= 2] or [poses]

    def _route_cusps(self, poses, legs):
        """Cusps of a planned route (geometry.route_cusps at route_cusp_angle_rad) plus the
        junctions between its FollowPath legs that turn more than that angle."""
        ang = float(self.p.route_cusp_angle_rad)
        if ang <= 0.0 or int(self.p.route_cusp_retries) <= 0:
            return []
        cusps = list(geo.route_cusps(poses, ang))
        k = 3
        for a, b in zip(legs, legs[1:]):
            if len(a) < 2 or len(b) < 2:
                continue
            p0, j, p2 = a[max(0, len(a) - 1 - k)], a[-1], b[min(len(b) - 1, k)]
            h1 = math.atan2(j[1] - p0[1], j[0] - p0[0])
            h2 = math.atan2(p2[1] - j[1], p2[0] - j[0])
            if abs(geo.wrap_angle(h2 - h1)) < ang or \
                    any(geo.dist(j, (c[1], c[2])) <= 0.3 for c in cusps):
                continue
            i = min(range(len(poses)), key=lambda q: geo.dist(poses[q], j))
            cusps.append((i, float(j[0]), float(j[1]), h2))
        return sorted(cusps)

    def _route_cusp_turn(self, why):
        """Fix 11: a route follow abort near a route cusp. None = no cusp within
        route_cusp_radius_m (the caller goes on as before); True = about-turn started;
        False = this cusp's about-turns are used up (the caller aborts the transit)."""
        m, pose, route = self.mission, self.inputs.pose, self.mission.route_full
        r = float(self.p.route_cusp_radius_m)
        if not route or not m.route_cusps or pose is None or len(pose) < 3 or r <= 0.0:
            return None
        near = [c for c in m.route_cusps if geo.dist(pose, (c[1], c[2])) <= r]
        if not near:
            return None
        i, cx, cy, heading = min(near, key=lambda c: geo.dist(pose, (c[1], c[2])))
        key = (round(cx, 2), round(cy, 2))
        n = m.cusp_turns.get(key, 0)
        limit = int(self.p.route_cusp_retries)
        if n >= limit:
            self._log('warn', 'route cusp at (%.2f, %.2f): %s after %d about-turn%s: transit '
                              'abort (no costmap fallback at a cusp)'
                      % (cx, cy, why, n, '' if n == 1 else 's'))
            return False
        m.cusp_turns[key] = n + 1
        rest = [tuple(q) for q in route[i:]]
        if len(rest) < 2:
            rest = [(cx, cy, heading), tuple(route[-1])]
        m.cusp_resume = rest
        m.route_poses = rest
        m.cusp_heading = heading
        delta = geo.wrap_angle(heading - float(pose[2]))
        self._log('warn', 'route cusp at (%.2f, %.2f): about-turn %d/%d then resume route (%s; '
                          'turn %.0f deg)' % (cx, cy, n + 1, limit, why, math.degrees(delta)))
        self._go(self.phase, 'route cusp: about-turn %d/%d' % (n + 1, limit))
        if abs(delta) < 0.3:                  # already facing the route after the cusp
            self._resume_after_cusp()
            return True
        allowance = abs(delta) / max(0.05, float(self.p.route_cusp_spin_rps)) + 10.0
        self._start_action(ACT_SPIN, {'angle': delta, 'time_allowance_s': allowance},
                           allowance + 5.0, purpose='route_cusp')
        return True

    def _resume_after_cusp(self):
        """Send the route remainder after a cusp about-turn as FollowPath legs."""
        m = self.mission
        rest = m.cusp_resume
        m.cusp_resume = None
        m.route_poses = rest
        # the cusp about-turn was this leg's pivot (task 0.4 d: no second one, even after a
        # failed Spin; the route follow is the judge)
        self._route_legs = {'legs': self._split_route(rest), 'i': 0, 'turned': 0}
        self._send_route_leg('route')

    def _route_cusp_spun(self, outcome):
        """About-turn ended: resume the route (an unfinished about-turn first goes through
        _pivot_unfinished: pivot again or stop)."""
        m = self.mission
        if m is None or self.phase != 'TRANSIT' or m.step != 'transit' or not m.cusp_resume:
            return
        if outcome == SUCCEEDED:
            self._pivot_tries = {}
        else:
            q = m.cusp_resume[0]
            if self._pivot_unfinished(outcome, 'route_cusp', ('cusp', round(q[0], 2),
                                                             round(q[1], 2)),
                                      m.cusp_heading,
                                      self._pivot_follow_rad(), 'route cusp about-turn'):
                return
        self._resume_after_cusp()

    def _pivot_follow_rad(self):
        """Largest heading error a leg is followed from after an unfinished pivot (cusp,
        final approach): the path conditioning pivot threshold, else route_leg_pivot_min_rad."""
        lim = float(self.p.path_conditioning_pivot_min_rad)
        if self.p.path_conditioning and lim > 0.0:
            return lim
        return max(0.3, float(self.p.route_leg_pivot_min_rad))

    @staticmethod
    def _leg_heading(leg, look=0.3):
        """Heading of a route leg's start: chord to the first pose ``look`` m along it."""
        a = leg[0]
        for q in leg[1:]:
            if geo.dist(a, q) >= look:
                return math.atan2(q[1] - a[1], q[0] - a[0])
        b = leg[-1]
        if geo.dist(a, b) < 1e-6:
            return None
        return math.atan2(b[1] - a[1], b[0] - a[0])

    def _route_corner_turn(self, purpose):
        """Task 0.4 (d): before route leg ``i`` (transit or dock route), pivot in place to
        its heading when that is more than route_corner_turn_rad off the robot's yaw and its
        start is within route_cusp_radius_m. True = the about-turn was started."""
        lim, rl, pose = float(self.p.route_corner_turn_rad), self._route_legs, self.inputs.pose
        if lim <= 0.0 or rl is None or pose is None or len(pose) < 3 or \
                rl.get('turned') == rl['i']:
            return False
        leg = rl['legs'][rl['i']]
        if len(leg) < 2 or geo.dist(pose, leg[0]) > float(self.p.route_cusp_radius_m):
            return False
        heading = self._leg_heading(leg)
        if heading is None:
            return False
        delta = geo.wrap_angle(heading - float(pose[2]))
        piv = float(self.p.route_leg_pivot_min_rad)
        if piv > 0.0 and rl['i'] > 0:          # stopped at a split corner
            lim = min(lim, piv)
        if self.p.path_conditioning and float(self.p.path_conditioning_pivot_min_rad) > 0.0:
            # one pivot rule (as _cov_pivot): every leg start, the first one too, is a vertex
            # of the driven path no fillet fits (sim gate 2026-10-16: MPPI turned onto a
            # transit 33 deg off and ran 0.17 m beside it)
            lim = float(self.p.path_conditioning_pivot_min_rad)
        if abs(delta) < lim - ROUTE_CORNER_TOL_RAD:     # inclusive, as the split
            return False
        rl['turned'], rl['purpose'] = rl['i'], purpose
        rl['pivot'] = (heading, lim - ROUTE_CORNER_TOL_RAD)
        if purpose == 'route' and self.mission is not None:
            # option C: pre-pivot sweep check against the no-go zones / static marks
            g = self._nogo_pivot_gate('route', ('corner', round(leg[0][0], 2),
                                                round(leg[0][1], 2)), delta)
            if g is True:
                return True
            if g is not None:
                delta = g
        self._log('info', 'route corner at (%.2f, %.2f): stop and pivot %.0f deg to the next '
                          'leg (%d/%d), then follow it'
                  % (leg[0][0], leg[0][1], math.degrees(delta), rl['i'] + 1, len(rl['legs'])))
        self._go(self.phase, 'route corner: pivot %.0f deg' % math.degrees(delta))
        allowance = abs(delta) / max(0.05, float(self.p.route_cusp_spin_rps)) + 10.0
        self._start_action(ACT_SPIN, {'angle': delta, 'time_allowance_s': allowance},
                           allowance + 5.0, purpose='route_corner')
        return True

    def _route_corner_spun(self, outcome):
        """Corner pivot ended (any outcome: the route follow is the judge): send the leg."""
        rl, m = self._route_legs, self.mission
        if rl is None:
            return
        purpose = rl.get('purpose', 'route')
        if purpose == 'route':
            if m is None or self.phase != 'TRANSIT' or m.step != 'transit':
                return
        elif self.phase not in DOCK_PHASES or self._dock_purpose is None:
            return
        if outcome == SUCCEEDED:
            self._pivot_tries = {}
        else:
            leg = rl['legs'][rl['i']]
            heading, lim = rl.get('pivot') or (None, 0.0)
            if purpose == 'route' and self._route_pivot_backout(
                    outcome, ('corner', round(leg[0][0], 2), round(leg[0][1], 2)), heading, lim):
                return
            if self._pivot_unfinished(outcome, 'route_corner',
                                      ('corner', round(leg[0][0], 2), round(leg[0][1], 2)),
                                      heading, lim, 'route corner'):
                return
        self._send_route_leg(purpose)

    def _safe_backoff(self, dist, pose=None):
        """The longest straight reverse <= ``dist`` (0.05 m steps) whose poses all keep the
        body within its boundary limits with hard_boundary_body_clear_m to spare (field
        2026-10-23: back-offs beside the pole / the line must not trip the boundary guard);
        0 = no room (under 0.10 m). Without a body check (drawn path, no snapshot): ``dist``.
        ``pose``: from there instead of the robot's pose (the pre-pivot check, option C)."""
        pose = self.inputs.pose if pose is None else pose
        if dist <= 0.0 or pose is None or len(pose) < 3:
            return 0.0
        mode = self._hb_body_mode(pose)
        if mode is None:
            return dist
        c, s = math.cos(float(pose[2])), math.sin(float(pose[2]))
        best, d = 0.0, 0.05
        clear = float(self.p.hard_boundary_body_clear_m)
        while d <= dist + 1e-9:
            q = (float(pose[0]) - c * d, float(pose[1]) - s * d, float(pose[2]))
            ev = self._hb_body_eval(q, mode)
            if ev is None or ev['excess'] > -clear:
                break
            best = d
            d += 0.05
        return best if best >= 0.10 - 1e-9 else 0.0

    def _route_pivot_backout(self, outcome, key, heading, lim):
        """A mission-transit route corner pivot refused pivot_retries times (field 2026-10-23
        20:30:52: a Spin refused 0.71 m from a trunk went to STUCK_NEEDS_HELP): back off
        route_pivot_backout_m, then re-plan the transit (a transit abort: re-routed from there;
        transit_retries used up -> the sub-path is skipped). A second refusal at the same
        corner re-plans at once. True = handled."""
        m, p, pose = self.mission, self.p, self.inputs.pose
        dist = float(p.route_pivot_backout_m)
        if dist <= 0.0 or m is None or outcome == CANCELED or heading is None or \
                pose is None or len(pose) < 3:
            return False
        if abs(geo.wrap_angle(heading - float(pose[2]))) < lim:
            return False
        if int(p.pivot_retries) < 0 or self._pivot_tries.get(key, 0) < int(p.pivot_retries):
            return False
        self._pivot_tries.pop(key, None)
        self._route_legs = None
        why = 'route corner pivot refused (%s) at (%.2f, %.2f)' % (outcome, pose[0], pose[1])
        self._incident('blocked_pivot', why)
        if key in m.route_backouts:
            self._log('warn', '%s again: re-planning the transit' % why)
            self._transit_abort(why)
            return True
        m.route_backouts.add(key)
        dist = self._safe_backoff(dist)
        if dist <= 0.0:
            self._log('warn', '%s: no room to back off (body limit behind): re-planning the '
                              'transit' % why)
            self._transit_abort(why)
            return True
        self._log('warn', '%s: back off %.2f m, then re-plan the transit' % (why, dist))
        if not self._keep_blade_inside('route pivot back-off'):
            self._blade_off('route pivot back-off')
        self._start_action(ACT_BACKUP, {'distance_m': dist,
                                        'speed_mps': float(p.pivot_backout_speed_mps)},
                           dist / max(0.02, float(p.pivot_backout_speed_mps)) + 15.0,
                           purpose='route_pivot_backout')
        return True

    def _send_route_leg(self, purpose):
        rl = self._route_legs
        if self._route_corner_turn(purpose):
            return
        checker = self.p.route_goal_checker_id
        if (rl['i'] + 1 < len(rl['legs']) and float(self.p.route_corner_turn_rad) > 0.0 or
                self.p.path_conditioning) and self.p.route_corner_goal_checker_id:
            # task 0.4 (d): end on the corner; path conditioning: every leg (the last one
            # too) ends along-path on its end, not 0.25 m short on the transit disc
            checker = self.p.route_corner_goal_checker_id
        self._start_action(ACT_FOLLOW, {'poses': rl['legs'][rl['i']],
                                        'controller_id': self.p.route_controller_id,
                                        'goal_checker_id': checker,
                                        'progress_checker_id': self.p.route_progress_checker_id},
                           self.p.transit_timeout_s, purpose=purpose)
        self._route_watch = None

    def _route_progress_tick(self):
        """2026-10-15 undock incident: a route leg (transit or dock route) whose robot
        projection has not advanced route_progress_min_m along the leg for
        route_progress_window_s (turning / circling in place) is cancelled and handled as a
        FollowPath abort. True when it fired."""
        win, need = float(self.p.route_progress_window_s), float(self.p.route_progress_min_m)
        a, rl, pose = self._action, self._route_legs, self.inputs.pose
        if a is None or a.name != ACT_FOLLOW or rl is None or pose is None or \
                a.purpose not in ('route', 'dock_route'):
            return False
        leg = rl['legs'][rl['i']]
        s = geo.path_projection(leg, pose)[0]
        w = self._route_watch
        if w is None or w['token'] != a.token:
            w = self._route_watch = {'token': a.token, 's': s, 't': self._now}
        if w['t'] == self._now:
            return False
        if s >= w['s'] + need:
            w['s'], w['t'] = s, self._now
            return False
        if win <= 0.0 or self._now - w['t'] < win:
            return False
        msg = 'no progress along route leg %d/%d for %.0f s (%.2f of %.2f m; route progress ' \
              'watch)' % (rl['i'] + 1, len(rl['legs']), self._now - w['t'], s,
                          geo.path_length(leg))
        self._log('warn', '%s: cancelling follow_path, handled as an abort' % msg)
        self._fx.append(CancelActions('route progress watch'))
        self._action = None
        self._route_watch = None
        self._handle_action(a, ABORTED, {'message': msg})
        return True

    def _route_next_leg(self, outcome, purpose):
        """A route leg succeeded at its end and another follows: send it. True if sent."""
        rl, pose = self._route_legs, self.inputs.pose
        if rl is None or rl['i'] + 1 >= len(rl['legs']) or pose is None:
            return False
        d = geo.dist(pose, rl['legs'][rl['i']][-1])
        if outcome != SUCCEEDED:
            acc = float(self.p.route_leg_end_accept_m)
            if outcome not in (ABORTED, TIMEOUT) or acc <= 0.0 or d > acc or \
                    self._blocked_cause():
                return False
            self._log('info', 'route leg %d/%d %s %.2f m from its corner: counted as reached'
                      % (rl['i'] + 1, len(rl['legs']), outcome, d))
        elif d > self.p.route_end_tolerance_m:
            return False
        rl['i'] += 1
        self._send_route_leg(purpose)
        return True

    def _route_result(self, outcome):
        """Route follow ended. Arrived -> mow. An obstacle abort holds first (blocked wait)
        and re-sends the route up to route_blocked_retries times, then through
        /navigate_to_pose. An abort / timeout at a route cusp turns about and re-sends the
        rest of the route (route_cusp_retries), then counts as a transit abort (fix 11).
        Any other failure re-sends via /navigate_to_pose at once without consuming a
        transit retry (those stay for the costmap planner, bounded to the corridor)."""
        m = self.mission
        if self._route_next_leg(outcome, 'route'):
            return
        self._route_legs = None
        route = m.route_poses
        m.route_poses = None
        pose = self.inputs.pose
        d = geo.dist(pose, m.transit_target) if pose is not None else math.inf
        if (outcome == SUCCEEDED and d <= self.p.route_end_tolerance_m) or \
                (outcome != UNAVAILABLE and self._transit_arrived()):
            if self._route_final_approach(d):
                return
            m.recovering = False
            m.transit_fails = 0
            m.route_plan_fails = 0
            self._blk = None
            self._begin_blade()
            return
        why = ('route follow_path %s' % outcome) if outcome != SUCCEEDED else \
            'route follow_path succeeded %.2f m from the target' % d
        m.route_aborts += 1
        if outcome in (ABORTED, TIMEOUT) and self._blocked_cause():
            # obstacle: wait, then re-send the route (route_blocked_retries), later Nav2
            m.route_fallback = m.route_aborts > int(self.p.route_blocked_retries)
            if m.route_fallback:
                m.fallback_ref = route
            if self._blocked('transit', why):
                return
        if outcome in (ABORTED, TIMEOUT):
            turned = self._route_cusp_turn(why)
            if turned:
                return
            if turned is False:
                self._transit_abort('%s at a route cusp' % why)
                return
        m.route_fallback = True
        self._log('warn', '%s: transit via the costmap planner' % why)
        self._start_fallback_nav(ref=route)

    def _route_final_approach(self, d):
        """Path conditioning: a transit that ended (arrived) more than route_final_approach_m
        from the sub-path start drives onto it first. The junction robot -> start is part of
        the driven path: a straight hop as explicit steps, like a pivot (Nav2 Spin to face
        the start when that is path_conditioning_pivot_min_rad or more off the robot's yaw,
        then Nav2 DriveOnHeading the remaining distance at route_final_approach_speed_mps),
        not a tracker leg (sim gate 2026-10-16: transits ended 0.1-0.23 m beside the start
        and the start pivot / rejoin ran that far off the line; as a 0.1 m FollowPath leg
        MPPI reversed 0.4 m out of the area). Once per transit; the outcome is not judged
        (the transit had arrived). True = started."""
        m, pose, tol = self.mission, self.inputs.pose, float(self.p.route_final_approach_m)
        if not self.p.path_conditioning or tol <= 0.0 or d <= tol or pose is None or \
                len(pose) < 3 or m.transit_target is None or m.final_approach:
            return False
        m.final_approach = True
        b = m.transit_target
        delta = geo.wrap_angle(math.atan2(b[1] - pose[1], b[0] - pose[0]) - float(pose[2]))
        self._log('info', 'transit ended %.2f m from the sub-path start: final approach onto '
                          'it (turn %.0f deg, then straight)' % (d, math.degrees(delta)))
        if abs(delta) >= float(self.p.path_conditioning_pivot_min_rad):
            allowance = abs(delta) / max(0.05, float(self.p.route_cusp_spin_rps)) + 10.0
            self._start_action(ACT_SPIN, {'angle': delta, 'time_allowance_s': allowance},
                               allowance + 5.0, purpose='final_approach_spin')
            return True
        self._final_approach_drive()
        return True

    def _final_approach_drive(self):
        """The straight hop of the final approach: the start's distance along the heading."""
        m, pose = self.mission, self.inputs.pose
        b = m.transit_target
        dist = 0.0
        if pose is not None and len(pose) >= 3:
            dist = (b[0] - pose[0]) * math.cos(pose[2]) + (b[1] - pose[1]) * math.sin(pose[2])
        if dist < 0.02:
            self._final_approach_done(SUCCEEDED)
            return
        speed = float(self.p.route_final_approach_speed_mps)
        self._start_action(ACT_DRIVE, {'distance_m': dist, 'speed_mps': speed,
                                       'wait_for_rtk': False, 'rtk_timeout_s': 0.0},
                           dist / max(0.01, speed) + 15.0, purpose='final_approach')

    def _final_approach_done(self, outcome):
        m = self.mission
        if m is None or self.phase != 'TRANSIT' or m.step != 'transit':
            return
        if outcome != SUCCEEDED:
            self._log('warn', 'final approach %s: mowing from here' % outcome)
        m.recovering = False
        m.transit_fails = 0
        m.route_plan_fails = 0
        self._blk = None
        self._begin_blade()

    def _transit_arrived(self):
        """True when the last transit target is within transit_arrived_radius_m."""
        m = self.mission
        tgt = getattr(m, 'transit_target', None) if m is not None else None
        if tgt is None or self.inputs.pose is None or self.p.transit_arrived_radius_m <= 0:
            return False
        return geo.dist(self.inputs.pose, tgt) <= self.p.transit_arrived_radius_m

    def _handle_action(self, a, outcome, result):
        name = a.name
        if a.purpose == 'preview':
            self._preview_on_plan(outcome, result)
            return
        if outcome != SUCCEEDED:
            self._log('warn', '%s finished: %s %s' % (name, outcome, result.get('message', '')))
        nav2_down = outcome != SUCCEEDED and bool(result.get(NAV2_INACTIVE_KEY))
        if nav2_down:
            self._log('error', '%s %s: %s (%s)' % (name, outcome, NAV2_DOWN,
                                                  result.get(NAV2_INACTIVE_KEY)))
        if a.purpose == 'heading_cal':
            self._hcal_result(outcome)
            return
        if a.purpose in ('nogo_step', 'nogo_detour'):    # option C maneuvers / edge detour
            self._nogo_action_done(a, outcome)
            return
        if a.purpose == 'dock_route':
            self._dock_route_result(outcome, nav2_down)
            return
        if a.purpose == 'route_corner':
            self._route_corner_spun(outcome)
            return
        if a.purpose == 'route_pivot_backout':
            m = self.mission
            if m is not None and self.phase == 'TRANSIT':
                self._transit_abort('route corner pivot refused: backed off, re-routing')
            return
        if a.purpose == 'final_approach_spin':
            m = self.mission
            if m is not None and self.phase == 'TRANSIT' and m.step == 'transit':
                if outcome == SUCCEEDED:
                    self._pivot_tries = {}
                else:
                    b, pose = m.transit_target, self.inputs.pose
                    heading = None if pose is None else \
                        math.atan2(b[1] - pose[1], b[0] - pose[0])
                    if self._pivot_unfinished(outcome, 'final_approach_spin',
                                              ('final', round(b[0], 2), round(b[1], 2)),
                                              heading, self._pivot_follow_rad(),
                                              'final approach'):
                        return
                self._final_approach_drive()
            return
        if a.purpose == 'final_approach':
            self._final_approach_done(outcome)
            return
        if a.purpose == 'dock':
            self._dock_contact_since = None
            if outcome == SUCCEEDED and (result.get('success', True)):
                self._dock_done()
            elif self._at_dock() and outcome != CANCELED:
                self._log('warn', 'docking action ended %s (%s) but the contacts report '
                          'docked: treating as docked' % (outcome, result.get('message', '')))
                self._dock_done()
            elif outcome in (ABORTED, TIMEOUT) and self._blocked_cause() and \
                    self._blocked('dock', 'docking %s' % outcome):
                return
            else:
                why = result.get('message') or outcome
                if outcome == UNAVAILABLE:
                    why = 'docking action server unavailable'
                if nav2_down:
                    # a navigation failure says nothing beyond "Nav2 down"; anything
                    # else the docking server reported is kept as the detail
                    nav = outcome in (REJECTED, UNAVAILABLE) or \
                        str(why).startswith('NAV_TO_DOCK_FAILED')
                    why = NAV2_DOWN if nav else nav2_down_why(why)
                self._dock_failed(why)
            return
        if self._manual_undocking and name in (ACT_UNDOCK, ACT_BACKUP):
            self._manual_undocking = False
            if outcome == SUCCEEDED and result.get('success', True):
                self._log('info', 'UNDOCK done: idle off the dock')
                self._go_idle('undocked')
            else:
                why = result.get('message') or outcome
                if outcome == UNAVAILABLE:
                    why = '/mower_docking/undock action server unavailable' \
                        if name == ACT_UNDOCK else '/backup action server unavailable'
                if nav2_down:
                    why = nav2_down_why(why)
                self._log('error', 'undock failed: %s' % why)
                self._go('UNDOCK_FAILED', why)
            return
        m = self.mission
        if m is None:
            return
        if nav2_down and name in (ACT_FOLLOW, ACT_NAV, ACT_SPIN):
            # Retrying / falling back to the costmap planner cannot help while the stack
            # is down (each attempt waits out nav2_ready_timeout_s again, then every
            # sub-path is skipped as "transit rejected"): abort with the real cause.
            self._mission_failed(nav2_down_why('%s %s' % (name, outcome)))
            return
        if a.purpose == 'route':
            self._route_result(outcome)
            return
        if a.purpose == 'route_cusp':
            self._route_cusp_spun(outcome)
            return
        if a.purpose == 'cov_pivot':
            self._cov_pivot_done(outcome)
            return
        if a.purpose == 'pivot_backout':
            self._pivot_backout_done(outcome)
            return
        if name in (ACT_UNDOCK, ACT_BACKUP):
            if outcome == SUCCEEDED and result.get('success', True):
                m.left_dock = True
                self._go('WAITING_FOR_RTK', '')
            else:
                why = result.get('message') or outcome
                if outcome == UNAVAILABLE:
                    why = '/mower_docking/undock action server unavailable' \
                        if name == ACT_UNDOCK else '/backup action server unavailable'
                if nav2_down:
                    why = nav2_down_why(why)
                self._log('error', 'undock failed: %s' % why)
                self.mission = None
                self._go('UNDOCK_FAILED', why)
        elif name == ACT_PLAN:
            self._on_plan(outcome, result)
        elif name == ACT_NAV:
            if m.detour is not None and outcome != UNAVAILABLE:
                if outcome == SUCCEEDED or self._transit_arrived():
                    self._detour_done()
                else:
                    self._detour_failed(outcome)
                return
            if outcome == SUCCEEDED:
                m.recovering = False
                m.transit_fails = 0
                m.route_plan_fails = 0
                self._blk = None
                self._begin_blade()
            elif outcome == UNAVAILABLE:
                self._mission_failed('/navigate_to_pose action server unavailable')
            elif self._transit_arrived():
                d = geo.dist(self.inputs.pose, m.transit_target)
                self._log('warn', 'transit %s but robot is %.2f m from the target '
                                  '(<= %.2f m): treating as arrived'
                          % (outcome, d, self.p.transit_arrived_radius_m))
                m.recovering = False
                self._blk = None
                self._begin_blade()
            elif m.recovering:
                m.recovery_fails += 1
                why = 'boundary recovery transit %s' % outcome
                if m.recovery_fails > self.p.boundary_recovery_retries:
                    self._boundary_recovery_failed('%s %d times' % (why, m.recovery_fails))
                    return
                self._schedule_retry('TRANSIT', 'area %d sub-path %d/%d: %s, retry %d/%d in %.0f s'
                                     % (m.area_idx, m.sub_i + 1, len(m.subpaths), why,
                                        m.recovery_fails, self.p.boundary_recovery_retries,
                                        self.p.transit_retry_delay_s))
            else:
                if self._blocked_cause() and self._blocked('transit', 'transit %s' % outcome):
                    return
                self._transit_abort('transit %s' % outcome)
        elif name == ACT_FOLLOW:
            if outcome == SUCCEEDED:
                self._blk = None
                cum = m.cum[m.sub_i]
                prog = self._track_progress()
                last = self._seg_last()
                end = min(m.chunk_end, last)
                left = cum[end] - cum[min(prog, end)]
                if left > self.p.follow_end_tolerance_m:
                    # Stock Jazzy goal checkers only look at the final pose; a coverage
                    # path that passes near its own end "succeeds" early.
                    if m.premature >= self.p.follow_premature_retries:
                        self._subpath_failed('follow_path keeps reporting success with '
                                             '%.1f m left' % left)
                        return
                    m.premature += 1
                    m.start_local = prog
                    self._log('warn', 'follow_path succeeded with %.1f m of sub-path %d left '
                                      '(goal checker fired early?): continuing from pose %d '
                                      '(%d/%d)' % (left, m.sub_i, prog, m.premature,
                                                   self.p.follow_premature_retries))
                    self._dispatch()
                    return
                if end < last:
                    m.start_local = end           # next chunk of the same sub-path
                    m.retry_used = False
                    m.follow_fails = 0
                    self._dispatch()
                    return
                if m.stretch is None:
                    self._log('info', 'area %d sub-path %d done (%.1f m)' % (
                        m.area_idx, m.sub_i, cum[-1]))
                self._segment_done()
            elif outcome == UNAVAILABLE:
                self._mission_failed('/follow_path action server unavailable')
            else:
                prog = self._track_progress()
                if self._policy() == 'dynamic':
                    m.start_local = prog
                    m.step = None
                    self._dyn_start('follow')
                    return
                if m.stretch is not None:
                    self._stretch_blocked('follow_path %s at pose %d' % (outcome, prog))
                    return
                pf = int(self.p.detour_unclassified_after)
                static = self._static_recent()
                collided = self._collision_recent()
                if (collided or static or self.inputs.cm_stop_since is not None) and \
                        self._edge_lap(m.sub_i) and self._nogo_edge_detour(
                            prog, 'follow_path %s, obstacle on the lap' % outcome):
                    return     # option C / B6: detour inward around it, rejoin, no stop-wait
                if self.p.edge_blocked_skip and (collided or self.inputs.cm_stop_since
                                                 is not None) and self._edge_lap(m.sub_i):
                    self._edge_skip(prog, 'follow_path %s, collision ahead' % outcome)
                    return
                # aborts inside a blocked wait count toward the unclassified-detour threshold
                nb = self._blk['n'] if self._blk is not None and \
                    self._blk['ctx'] == 'follow' else 0
                if (static or (pf >= 0 and m.follow_fails + nb >= pf and collided)) and \
                        self._try_detour(
                            prog, '%s obstacle' % (self.inputs.obstacle_class or 'static')
                            if static else 'follow_path %s (costmap collision ahead)' % outcome):
                    self._blk = None
                    return
                if collided:
                    m.start_local = prog
                    if self._blocked('follow', 'follow_path %s at pose %d' % (outcome, prog)):
                        return
                m.follow_fails += 1
                m.start_local = prog
                # the leg loop guard's abort is the robot orbiting a bad plan piece, not a
                # traction failure: its own kind, no terrain weight (field 2026-10-28: four
                # loop-guard aborts became follow_abort incidents on plain grass)
                self._incident('leg_loop' if result.get('leg_loop') else 'follow_abort',
                               'follow_path %s at pose %d' % (outcome, prog))
                if m.follow_fails > self.p.follow_retries:
                    self._subpath_failed('follow_path %s %d times (at pose %d)'
                                         % (outcome, m.follow_fails, m.start_local))
                    return
                delay = self._retry_delay(m.follow_fails)
                self._schedule_retry('MOWING', 'area %d sub-path %d/%d: follow_path %s at pose %d, '
                                     'retry %d/%d in %.0f s' % (
                                         m.area_idx, m.sub_i + 1, len(m.subpaths), outcome,
                                         m.start_local, m.follow_fails, self.p.follow_retries,
                                         delay), delay=delay)
