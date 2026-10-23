# SPDX-License-Identifier: GPL-3.0-or-later
"""MissionFSM mixin: Docking / undocking.

Moved verbatim from mission_fsm.py (2026-10-23 code review stage 1.6)."""

from mower_mission.fsm.common import (ABORTED, ACT_DOCK, ACT_NAV, CancelActions, DOCK_PHASES,
    SRV_CHARGING, SRV_PLAN_ROUTE, SUCCEEDED, TIMEOUT, geo, math)  # noqa: F401


class DockingMixin:
    """Docking / undocking."""

    # ==================================================================
    # docking
    # ==================================================================
    def _dock(self, name, purpose, sub=''):
        self._dyn = None
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

    def _dock_done(self):
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
        elif purpose == 'incomplete':
            self._go_idle('docked; %s' % self._incomplete_text or 'mowing incomplete')
        else:
            self._go_idle('docked')

    def _dock_failed(self, why):
        self._dock_routed = None
        self._blk = None
        self._dock_purpose = None
        self._resume_after = None
        self._log('error', 'docking failed: %s' % why)
        self._go('NAV_TO_DOCK_FAILED', why)
