# SPDX-License-Identifier: GPL-3.0-or-later
"""MissionFSM mixin: Docking / undocking.

Moved verbatim from mission_fsm.py (2026-10-23 code review stage 1.6)."""

from mower_mission.fsm.common import (ABORTED, ACT_DOCK, ACT_NAV, CancelActions, DOCK_PHASES,
    SRV_CHARGING, SRV_PLAN_ROUTE, SUCCEEDED, TIMEOUT, bbody, geo, math, re)  # noqa: F401


class DockingMixin:
    """Docking / undocking."""

    # ==================================================================
    # docking
    # ==================================================================
    def _dock(self, name, purpose, sub=''):
        self._dyn = None
        # a localization hold of the interrupted mow / transit ends here: the dock route
        # and docking are exempt from it unless lost > loc_hold_dock_exempt_max_s (holds.py)
        self._loc_drop()
        self._blade_off('dock')
        if self._action is not None:
            self._fx.append(CancelActions('dock'))
            self._action = None
        self._dock_purpose = purpose
        self._go(name, sub)
        if self.inputs.docked:
            self._dock_done()
            return
        if self.p.use_docking_server:
            if self._dock_route_on() and self._dock_routed != (name, purpose):
                self._dock_route_tries = 0
                self._plan_dock_route()
                return
            self._dock_handover()
        elif len(self.p.dock_pose) == 3:
            self._start_action(ACT_NAV, {'pose': tuple(self.p.dock_pose)},
                               self.p.transit_timeout_s, purpose='dock')
        else:
            self._dock_failed('no docking server and no dock_pose configured')

    def _dock_route_on(self):
        return bool(self.p.route_transits and self.p.route_dock and self.inputs.pose is not None)

    def _plan_dock_route(self):
        self._call(SRV_PLAN_ROUTE, {'start': tuple(self.inputs.pose), 'to_dock': True,
                                    'variation': self._transit_run()}, purpose='dock_route')

    def _dock_approach(self):
        """The docking server's approach pose (dock + dock_approach_distance_m along the dock
        yaw, as map_server / mower_docking compute it); else the last dock route's end."""
        d = self.inputs.dock_pose
        if d is not None and len(d) >= 3:
            ad = float(self.p.dock_approach_distance_m)
            return (d[0] + ad * math.cos(d[2]), d[1] + ad * math.sin(d[2]))
        return getattr(self, '_dock_route_end', None)

    def _dock_handover(self):
        """Task 0.4 (e): the docking server (whose own navigate_to_pose drives the approach
        leg across the grass) only within dock_server_nav_max_m of the approach pose. Farther
        away: plan and drive the dock route again (dock_route_retries), then fail in place."""
        lim = float(self.p.dock_server_nav_max_m)
        ap, pose = self._dock_approach(), self.inputs.pose
        if lim > 0.0 and ap is not None and pose is not None and self._dock_route_on():
            d = geo.dist(pose, ap)
            if d > lim:
                tries = getattr(self, '_dock_route_tries', 0)
                if tries >= int(self.p.dock_route_retries):
                    self._dock_failed('dock route did not reach the approach pose (%.2f m away '
                                      'after %d re-sends); not crossing the lawn: the docking '
                                      'server navigates only within %.2f m' % (d, tries, lim))
                    return
                self._dock_route_tries = tries + 1
                self._log('warn', 'dock: %.2f m from the approach pose (> %.2f m): driving the '
                                  'dock route again (%d/%d), not the docking server\'s own '
                                  'navigation' % (d, lim, tries + 1,
                                                  int(self.p.dock_route_retries)))
                self._plan_dock_route()
                return
        self._dock_server()

    def _dock_server(self):
        self._start_action(ACT_DOCK, {'use_vision': self.p.dock_use_vision,
                                      'timeout_s': self.p.dock_timeout_s},
                           self.p.dock_action_timeout_s, purpose='dock')

    def _dock_route_planned(self, ok, resp):
        """plan_route(to_dock) answered: drive the route to the approach pose, or (no route)
        let the docking server navigate there itself."""
        if self.phase not in DOCK_PHASES or self._action is not None or \
                self._dock_purpose is None or self.inputs.docked:
            return
        poses = resp.get('poses') or []
        if not (ok and resp.get('success') and len(poses) >= 2):
            why = resp.get('message') or 'plan_route unavailable'
            if re.search(r'start .*off the route graph', why) and self._dock_join(why):
                return
            self._dock_routed = (self.phase, self._dock_purpose)
            self._log('info', 'dock route: %s: handing over to the docking server'
                      % (resp.get('message') or 'plan_route unavailable'))
            self._dock_handover()
            return
        self._dock_route_end = (float(poses[-1][0]), float(poses[-1][1]))
        prepared = self._route_prepare(poses)
        legs = (self._condition_route(prepared, 'dock route') if self.p.path_conditioning
                else None) or self._split_route(prepared)
        self._log('info', 'dock route %.1f m (%d leg%s): %s' % (
            float(resp.get('length_m', 0.0)), len(legs), '' if len(legs) == 1 else 's',
            resp.get('message', '')))
        self._route_legs = {'legs': legs, 'i': 0}
        self._send_route_leg('dock_route')

    def _dock_route_result(self, outcome, nav2_down=False):
        """The dock route ended (any outcome): the docking server takes over at the approach
        pose (task 0.4 e: only within dock_server_nav_max_m of it; farther the route is driven
        again, see _dock_handover). An obstacle abort waits first (blocked wait, then a fresh
        dock route). Nav2 down: straight to the docking server (it cannot drive the lawn
        without Nav2 either, and its failure names the cause)."""
        if self._route_next_leg(outcome, 'dock_route'):
            return
        self._route_legs = None
        self._dock_routed = (self.phase, self._dock_purpose)
        if self.inputs.docked:
            self._dock_done()
            return
        if nav2_down:
            self._log('warn', 'dock route follow_path %s: Nav2 down, handing over to the '
                              'docking server' % outcome)
            self._dock_server()
            return
        if outcome != SUCCEEDED:
            self._log('warn', 'dock route follow_path %s' % outcome)
            if outcome in (ABORTED, TIMEOUT) and float(self.p.dock_server_nav_max_m) > 0.0 \
                    and self._blocked_cause():
                self._dock_routed = None      # the re-send after the wait plans a fresh route
                if self._blocked('dock', 'dock route %s' % outcome):
                    return
                self._dock_routed = (self.phase, self._dock_purpose)
        self._dock_handover()

    # ---- off the route graph: join it inside the area first (2026-11-02) ------------
    def _dock_join_record(self, poses):
        """A transit route was planned: remember where it entered the area of its goal (the
        first route pose inside that area at least dock_join_inset_m from its edge, i.e. on
        an in-area graph leg) when it started outside that area."""
        if len(poses) < 2 or not self._region_areas:
            return
        a, b = poses[0], poses[-1]
        inset = float(self.p.dock_join_inset_m)
        for idx, area in self._region_areas.items():
            outer = [tuple(q[:2]) for q in (area.get('outer') or [])]
            if len(outer) < 3 or not geo.point_in_polygon(b, outer) or \
                    geo.point_in_polygon(a, outer):
                continue
            for q in poses:
                if geo.point_in_polygon(q, outer) and \
                        geo.distance_to_ring(q, outer) >= inset - 1e-6:
                    entries = getattr(self, '_dock_join_entries', None) or {}
                    entries[idx] = (float(q[0]), float(q[1]))
                    self._dock_join_entries = entries
                    break
            return

    def _dock_join_areas(self, pose):
        """[(idx, name, outer)] of the mowing areas containing ``pose``; none: the area whose
        outline is nearest, if ``pose`` is within dock_join_edge_tol_m outside it (2026-11-02
        incident: 0.013 m outside the area 5 outline)."""
        out, near = [], None
        tol = float(getattr(self.p, 'dock_join_edge_tol_m', 0.0) or 0.0)
        for idx, area in (self._region_areas or {}).items():
            outer = [tuple(q[:2]) for q in (area.get('outer') or [])]
            if len(outer) < 3:
                continue
            cand = (idx, str(area.get('name') or idx), outer)
            if geo.point_in_polygon(pose, outer):
                out.append(cand)
                continue
            d = geo.distance_to_ring(pose, outer)
            if d <= tol and (near is None or d < near[0]):
                near = (d, cand)
        if not out and near is not None:
            out.append(near[1])
        return out

    def _dock_join_reachable(self, pose, q, outer, cap):
        """``q`` inside ``outer``, within ``cap`` and the straight line to it inside too (a
        start just outside the outline may leave it inward: samples outside within
        dock_join_edge_tol_m are allowed until the line has entered)."""
        d = geo.dist(pose, q)
        if d > cap or not geo.point_in_polygon(q, outer):
            return False
        tol = float(getattr(self.p, 'dock_join_edge_tol_m', 0.0) or 0.0)
        n = max(1, int(math.ceil(d / 0.2)))
        entered = False
        for k in range(n + 1):
            s = (pose[0] + (q[0] - pose[0]) * k / n, pose[1] + (q[1] - pose[1]) * k / n)
            if geo.point_in_polygon(s, outer):
                entered = True
            elif entered or geo.distance_to_ring(s, outer) > tol:
                return False
        return True

    def _dock_join_candidates(self, pose, areas, cap):
        """Join points, best first: the last inbound route's entry into each area, then
        points dock_join_insets_m in from the nearest edge (nearest first)."""
        cands = []
        entries = getattr(self, '_dock_join_entries', None) or {}
        for idx, _name, outer in areas:
            e = entries.get(idx)
            if e is not None and geo.dist(pose, e) > 0.3 and \
                    self._dock_join_reachable(pose, e, outer, cap):
                cands.append((e, 'entry of the last inbound transit', outer))
        near = []
        for _idx, _name, outer in areas:
            ne = bbody.nearest_edge_point(pose[0], pose[1], [outer])
            if ne is None or ne[2] <= 1e-6:
                continue
            px, py, d0 = ne
            nx, ny = (pose[0] - px) / d0, (pose[1] - py) / d0
            if not geo.point_in_polygon(pose, outer):   # just outside: inward is to the edge
                nx, ny, d0 = -nx, -ny, -d0
            for inset in self.p.dock_join_insets_m or []:
                step = float(inset) - d0
                if step <= 0.3:
                    continue
                q = (pose[0] + nx * step, pose[1] + ny * step)
                if geo.distance_to_ring(q, outer) >= float(inset) * 0.8 and \
                        self._dock_join_reachable(pose, q, outer, cap):
                    near.append((geo.dist(pose, q), q, outer))
        near.sort(key=lambda t: t[0])
        return cands + [(q, 'nearest', outer) for _d, q, outer in near]

    def _dock_join(self, why, advance=False):
        """plan_route(to_dock) said the start is off the route graph. Inside a mowing area:
        drive to the next join point in it and plan the dock route again from there. True =
        handled (driving, or failed with the reason); False = outside every area (or off):
        the old hand-over. A re-plan after a hold (blocked / dynamic obstacle wait) before
        the join point was reached drives to the same join point again."""
        cap, pose = float(self.p.dock_join_max_m), self.inputs.pose
        if cap <= 0.0 or pose is None:
            return False
        j = getattr(self, '_dock_join_state', None)
        if j is None:
            areas = self._dock_join_areas(pose)
            if not areas:
                return False
            j = {'areas': areas, 'cands': self._dock_join_candidates(pose, areas, cap),
                 'i': -1, 'fails': 0, 'outer': None, 'start': tuple(pose[:2])}
            self._dock_join_state = j
            advance = True
        if advance or j.get('arrived'):
            j['i'] += 1
            j['fails'] = 0
        elif j['i'] < len(j['cands']):
            self._dock_routed = None
            self._dock_join_send()
            return True
        if j['i'] >= len(j['cands']):
            names = ', '.join('%s (%s)' % (n, i) for i, n, _o in j['areas'])
            tried = len(j['cands'])
            st = j['start']
            self._dock_join_state = None
            self._dock_failed('dock route: start (%.2f, %.2f) is off the route graph and no '
                              'join point is reachable inside area %s within %.1f m (%d '
                              'tried); not crossing the lawn: the docking server navigates '
                              'only within %.2f m' % (st[0], st[1], names, cap, tried,
                                                      float(self.p.dock_server_nav_max_m)))
            return True
        q, kind, outer = j['cands'][j['i']]
        j['target'], j['outer'], j['arrived'] = q, outer, False
        self._dock_routed = None
        self._log('info', 'dock: off the route graph, joining it at (%.2f, %.2f) via in-area '
                          'transit (%.1f m, %s; %s)' % (q[0], q[1], geo.dist(pose, q), kind, why))
        self._dock_join_send()
        return True

    def _dock_join_send(self):
        j, pose = self._dock_join_state, self.inputs.pose
        q = j['target']
        yaw = math.atan2(q[1] - pose[1], q[0] - pose[0]) if pose is not None else 0.0
        j['off0'] = None
        self._go(self.phase, 'joining the route graph at (%.2f, %.2f)' % (q[0], q[1]))
        self._start_action(ACT_NAV, {'pose': (q[0], q[1], yaw)}, self.p.transit_timeout_s,
                           purpose='dock_join')

    def _dock_join_tick(self):
        """The join transit stays in its area: cancelled (a failed attempt) once it is more
        than transit_fallback_max_offpath_m outside it (beyond where it started)."""
        j, pose, lim = self._dock_join_state, self.inputs.pose, \
            float(self.p.transit_fallback_max_offpath_m)
        if j is None or pose is None or lim <= 0.0 or not j.get('outer'):
            return
        off = bbody.point_outside(pose[0], pose[1], [j['outer']])
        if j.get('off0') is None:
            j['off0'] = off
            return
        if off <= j['off0'] + lim:
            return
        self._log('warn', 'dock join transit left the area by %.2f m: abort' % off)
        self._fx.append(CancelActions('dock join left the area'))
        self._action = None
        self._dock_join_result(ABORTED)

    def _dock_join_result(self, outcome, nav2_down=False):
        j = self._dock_join_state
        if j is None or self.phase not in DOCK_PHASES or self._dock_purpose is None:
            return
        if self.inputs.docked:
            self._dock_done()
            return
        if outcome == SUCCEEDED:
            self._log('info', 'dock: joined the route graph at (%.2f, %.2f): planning the dock '
                              'route' % tuple(j['target']))
            j['arrived'] = True
            self._dock_route_tries = 0
            self._plan_dock_route()
            return
        if nav2_down:
            self._dock_join_state = None
            self._dock_failed('dock join transit %s: Nav2 down' % outcome)
            return
        if outcome in (ABORTED, TIMEOUT) and self._blocked_cause() and \
                self._blocked('dock', 'dock join transit %s' % outcome):
            return                          # the re-send (_dock) re-plans, re-joins here
        j['fails'] += 1
        if j['fails'] <= int(self.p.transit_retries):
            self._log('warn', 'dock join transit %s: retry %d/%d'
                      % (outcome, j['fails'], int(self.p.transit_retries)))
            self._dock_join_send()
            return
        self._dock_join('join transit %s %d times' % (outcome, j['fails']), advance=True)

    def _dock_done(self):
        self._dock_join_state = None
        self._dock_routed = None
        self._dock_route_end = None
        self._blk = None
        purpose, self._dock_purpose = self._dock_purpose, None
        if not self.p.use_docking_server:
            self._call(SRV_CHARGING, {'enable': True}, track=False)
        if purpose == 'rain':
            self._rain_clear_since = None
            self._resume_after = 'rain'
            self._go('RAIN_WAITING', 'waiting %.0f min after rain' % self.p.rain_delay_minutes)
        elif purpose == 'battery':
            self._go('CHARGING', 'charging to %.0f%%, then resuming' % self._resume_charge_percent())
            self._resume_after = 'charge'
        elif purpose == 'battery_temp':
            self._go_idle('battery temperature high: waiting to cool below %.0f C, then resuming'
                          % self.p.battery_temp_max_start_c)
            self._resume_after = 'battery_temp'
        elif purpose == 'incomplete':
            self._go_idle('docked; %s' % self._incomplete_text or 'mowing incomplete')
        elif purpose == 'failed' and self._failed_text:
            self._go_idle('docked; mowing aborted: %s' % self._failed_text)
            self._failed_text = ''
        else:
            self._go_idle('docked')

    def _dock_failed(self, why):
        self._dock_join_state = None
        self._dock_routed = None
        self._blk = None
        self._dock_purpose = None
        self._resume_after = None
        self._log('error', 'docking failed: %s' % why)
        self._go('NAV_TO_DOCK_FAILED', why)
