# SPDX-License-Identifier: GPL-3.0-or-later
"""MissionFSM mixin: Obstacle avoidance: blocked / dynamic / localization holds, detours, stretches.

Moved verbatim from mission_fsm.py (2026-10-23 code review stage 1.6)."""

from mower_mission.fsm.common import (ACT_NAV, DOCK_PHASES, SRV_GET_AREA,
    SRV_SET_CHANNEL, DriveCmd, accepted_area, SuppressStuckGuard, ZeroBurst, _STILL_V, _STILL_W, _along, _seg_dist, absolute_to_local, geo, json, math,
    subpath_offsets)
from mower_mission.fsm.hold import Hold


class HoldsMixin:
    """Obstacle avoidance: blocked / dynamic / localization holds, detours, stretches."""

    # ==================================================================
    # obstacle avoidance
    # ==================================================================
    def _policy(self):
        """Fresh /obstacle_policy kind: 'none' | 'dynamic' | 'static'."""
        i = self.inputs
        if not self.p.obstacle_avoidance or i.obstacle_stamp is None or \
                self._now - i.obstacle_stamp > self.p.obstacle_policy_timeout_s:
            return 'none'
        kind = i.obstacle_kind if i.obstacle_kind in ('dynamic', 'static') else 'none'
        if i.dock_marker_in_view and self.phase in DOCK_PHASES:
            # Owner: "if the rear camera has the dock marker in vision, all other camera
            # detections should be ignored" (2026-10-07: the owner by the dock cancelled
            # DOCKING twice as 'dynamic obstacle: person'). Bumper/costmap stay active.
            if kind != 'none' and not getattr(self, '_marker_ignore_logged', False):
                self._marker_ignore_logged = True
                self._log('info', 'marker in view: obstacle detections ignored while docking')
            return 'none'
        self._marker_ignore_logged = False
        if self.phase in DOCK_PHASES and self._in_dock_zone():
            if kind != 'none' and not getattr(self, '_zone_ignore_logged', False):
                self._zone_ignore_logged = True
                self._log('info', 'within %.1f m of the dock: obstacle detections ignored '
                          'while docking' % self.p.dock_zone_radius_m)
            return 'none'
        self._zone_ignore_logged = False
        self._path_eval = None
        if kind != 'none':
            on = self._obstacle_on_path()
            self._decide(kind, on)
            if on is False:
                return 'none'
        elif kind == 'none' and self.obstacle_decision.get('kind') != 'none':
            self.obstacle_decision = {'kind': 'none'}
        return kind

    def _in_dock_zone(self):
        """Robot within dock_zone_radius_m of the dock pose (topic, else params.dock_pose)."""
        pose = self.inputs.pose
        dock = self.inputs.dock_pose or (tuple(self.p.dock_pose) if len(self.p.dock_pose) >= 2
                                         else None)
        r = float(self.p.dock_zone_radius_m)
        if pose is None or dock is None or r <= 0.0:
            return False
        return math.hypot(pose[0] - dock[0], pose[1] - dock[1]) <= r

    def _fresh_motion(self):
        """(v, w) of the applied /cmd_vel, else /cmd_vel_nav, younger than
        obstacle_twist_max_age_s; None when neither is fresh."""
        i, age = self.inputs, float(self.p.obstacle_twist_max_age_s)
        for cmd, t in ((i.cmd_vel, i.cmd_vel_stamp), (i.nav_cmd, i.nav_cmd_stamp)):
            if cmd is not None and t is not None and self._now - t <= age:
                return float(cmd[0]), float(cmd[1])
        return None

    def _path_motion(self):
        """Twist the in-path prediction uses: while a dynamic stop holds, the one captured
        when it started (the robot is stopped now; judging the wait against "standing
        still" would call the same person advisory and resume into them), else fresh."""
        if self._dyn is not None:
            return self._dyn.get('motion')
        return self._fresh_motion()

    def _upcoming_path(self):
        """How a detection is judged right now, or None (no path to judge against: it
        counts as before):

        * ``('corridor', pts)``: the path the controller tracks -- the next
          obstacle_path_lookahead_m of the MOWING sub-path or of a route transit. Kept for
          these: it is the true path (a pivot at a swath end follows it too).
        * ``('swept', poses)``: robot poses along the predicted motion (_path_motion) for
          obstacle_sweep_horizon_s -- /navigate_to_pose transits (detours, gaps, retries)
          and dock phases while the robot moves (Nav2 plans its own path; the straight line
          to the target is not where a pivoting robot goes, 2026-10-13).
        * ``('line', pts)``: straight to the transit target, for a TRANSIT standing still
          with no prediction (waits, goal just sent): the intended direction.

        While a dynamic stop holds, the kind chosen when it started is kept."""
        m, pose = self.mission, self.inputs.pose
        if pose is None:
            return None
        frozen = self._dyn.get('path') if self._dyn is not None else None
        motion = self._path_motion()
        moving = motion is not None and (abs(motion[0]) >= _STILL_V or abs(motion[1]) >= _STILL_W)
        if frozen == 'swept' and motion is not None:
            moving = True
        elif frozen in ('line', 'corridor'):
            moving = False

        def swept():
            return 'swept', geo.predict_poses(pose, motion[0], motion[1],
                                              float(self.p.obstacle_sweep_horizon_s))

        if self.phase in DOCK_PHASES:
            return swept() if moving else None
        if self._hcal is not None and self.phase == 'WAITING_FOR_RTK':
            return 'line', self._hcal_line(pose)      # heading calibration drive
        if m is None or self.phase not in ('MOWING', 'TRANSIT'):
            return None
        look = float(self.p.obstacle_path_lookahead_m)
        pts = [(pose[0], pose[1])]
        rp = m.route_poses if (m.step == 'transit' or frozen == 'corridor') else None
        if rp:
            # route transit: the corridor follows the route, not the straight line
            k = min(range(len(rp)), key=lambda j: geo.dist(pose, rp[j]))
            return 'corridor', _along(pts, rp[k:], look)
        if m.step == 'transit' or (self.phase == 'TRANSIT' and m.step in (
                'dyn_wait', 'blocked_wait', 'retry_wait')):
            if moving:
                return swept()
            t = m.transit_target
            if t is None:
                return None
            d = geo.dist(pose, t)
            f = 1.0 if d <= look or d < 1e-6 else look / d
            pts.append((pose[0] + (t[0] - pose[0]) * f, pose[1] + (t[1] - pose[1]) * f))
            return 'line', pts
        if not m.subpaths or m.sub_i >= len(m.subpaths):
            return None
        sp = m.subpaths[m.sub_i]
        i = m.progress_local if m.step == 'follow' else m.start_local
        return 'corridor', _along(pts, sp[max(0, min(i, len(sp) - 1)):], look)

    def _obstacle_on_path(self):
        """True / False: the ranged detection is / is not in the path (see _upcoming_path;
        anything within the circumscribed footprint + margin always is); None: cannot tell
        (unranged, no bearing, no path) -> counts as before. Records how it was judged in
        ``self._path_eval`` (for /mission/obstacle_decision and the dynamic stop)."""
        self._path_eval = None
        pose, o = self.inputs.pose, self._obstacle_xy()
        if o is None:
            return None
        p = self.p
        hw, margin = float(p.robot_width_m) / 2.0, float(p.obstacle_path_margin_m)
        front, rear = float(p.robot_front_m), float(p.robot_rear_edge_m)
        model = self._upcoming_path()
        self._path_eval = {'path': model[0] if model else None,
                           'motion': self._path_motion()}
        if geo.dist(o, pose) <= math.hypot(max(front, rear), hw) + margin:
            self._path_eval['by'] = 'footprint'
            return True
        if model is None:
            return None
        kind, pts = model
        self._path_eval['by'] = kind
        if kind == 'swept':
            return geo.swept_hit(o, pts, front, rear, hw, margin)
        best = float('inf')
        for p0, p1 in zip(pts, pts[1:] if len(pts) > 1 else pts):
            best = min(best, _seg_dist(o, p0, p1))
        return best <= hw + margin

    def _obstacle_xy(self):
        """Map position of the ranged detection (pose + obstacle_distance / bearing), or None
        (unranged, no bearing, no pose)."""
        i = self.inputs
        d, b, pose = i.obstacle_distance, i.obstacle_bearing, i.pose
        if d is None or b is None or pose is None:
            return None
        a = (pose[2] if len(pose) > 2 else 0.0) + math.radians(float(b))
        return pose[0] + d * math.cos(a), pose[1] + d * math.sin(a)

    def _decide(self, kind, on):
        i = self.inputs
        ev = getattr(self, '_path_eval', None) or {}
        mo = ev.get('motion')
        dec = {'kind': kind, 'class': i.obstacle_class, 'distance_m': i.obstacle_distance,
               'bearing_deg': i.obstacle_bearing,
               'on_path': on, 'action': 'advisory' if on is False else 'act',
               'lookahead_m': float(self.p.obstacle_path_lookahead_m),
               'half_width_m': float(self.p.robot_width_m) / 2.0
               + float(self.p.obstacle_path_margin_m),
               'judged_by': ev.get('by', ev.get('path')),
               'motion': None if mo is None else [round(mo[0], 2), round(mo[1], 2)],
               'phase': self.phase}
        if on is False and (self.obstacle_decision.get('action') != 'advisory'
                            or self.obstacle_decision.get('class') != dec['class']):
            how = {'swept': 'predicted motion v %.2f w %.2f' % tuple(mo) if mo else 'motion',
                   'line': 'line to the target'}.get(dec['judged_by'], 'path ahead')
            self._log('info', '%s %s at %.1f m / %.0f deg is off the path corridor (%s): '
                              'advisory only' % (kind, dec['class'] or 'obstacle',
                                                 i.obstacle_distance, i.obstacle_bearing, how))
        self.obstacle_decision = dec

    def _ranged_static(self):
        """A fresh 'static' policy that may explain an abort: it must carry a range
        (obstacle_guard only emits ranged statics; an unranged one is accepted only at the
        'sensitive' level, where the owner can opt in to unranged statics in the guard)."""
        if self._policy() != 'static':
            return False
        return self.inputs.obstacle_distance is not None or self.obstacle_level() == 'sensitive'

    def _static_recent(self):
        return self._ranged_static() or \
            self._now - self._last_static_t <= self.p.obstacle_static_memory_s

    def _collision_recent(self):
        c = self.inputs.collision_stamp
        return (c is not None and self._now - c <= self.p.collision_abort_window_s) or \
            self.inputs.cm_stop_since is not None      # collision_monitor STOP right now

    # ---- path blocked (costmap collision / obstacle policy): wait, no retry ----
    def _blocked_cause(self):
        return self._collision_recent() or self._policy() in ('dynamic', 'static')

    def _blocked_limit(self, b):
        return float(self.p.blocked_wait_home_s if b['phase'] == 'RETURNING_HOME'
                     else self.p.blocked_wait_s)

    def _blocked(self, ctx, why):
        """An abort explained by an obstacle: hold (blade off, no goal) instead of
        consuming a retry.  False once this blocked episode has used up its wait."""
        b, now, m = self._blk, self._now, self.mission
        if b is None or b['ctx'] != ctx:
            b = {'ctx': ctx, 'since': now, 'n': 0, 'phase': self.phase,
                 'purpose': self._dock_purpose, 'sub': self.sub_state}
            if self._blocked_limit(b) <= 0:
                return False
            self._blk = b
            self._log('warn', 'path blocked (%s): waiting up to %.0f s, re-sending every %.0f s '
                              'once no collision is reported (no retry used)'
                      % (why, self._blocked_limit(b), self.p.blocked_resend_s))
        elif now - b['since'] >= self._blocked_limit(b):
            self._blk = None
            self._log('warn', 'path still blocked after %.0f s (%s)'
                      % (self._blocked_limit(b), why))
            return False
        b['n'] += 1
        b['sent_at'] = now
        b['waiting'] = True
        b['why'] = why
        if m is not None and ctx in ('transit', 'follow'):
            m.step = 'blocked_wait'
            self._soft_blade_off('path blocked')
        else:
            self._blade_off('path blocked')
        self._go(self.phase, 'path blocked: waiting (%d s)' % int(now - b['since']))
        return True

    def _blocked_tick(self):
        """Returns True when the blocked wait owns this tick."""
        b, now, m = self._blk, self._now, self.mission
        if b is None or not b.get('waiting'):
            return False
        if b['ctx'] == 'dock':
            valid = self.phase == b['phase'] and self._action is None
        else:
            valid = m is not None and m.step == 'blocked_wait' and \
                self.phase in ('TRANSIT', 'MOWING')
        if not valid:
            self._blk = None
            return False
        waited = now - b['since']
        if waited >= self._blocked_limit(b):
            self._blk = None
            self._blocked_timeout(b)
            return True
        c = self.inputs.collision_stamp
        clear = (c is None or now - c >= float(self.p.blocked_clear_s)) and \
            self._policy() != 'dynamic'
        if clear and now - b['sent_at'] >= float(self.p.blocked_resend_s):
            b['waiting'] = False
            self._log('info', 'path blocked %.0f s: no collision for %.0f s, re-sending'
                      % (waited, float(self.p.blocked_clear_s)))
            Hold.resume(self, b, transit_phase='TRANSIT')
            return True
        Hold.show_wait(self, 'path blocked: waiting (%d s)' % int(waited))
        return True

    def _blocked_timeout(self, b):
        m = self.mission
        why = 'path blocked for %.0f s (%s)' % (self._blocked_limit(b), b.get('why', ''))
        self._log('warn', why)
        if b['ctx'] == 'dock':
            self._dock_failed(why)
        elif b['ctx'] == 'transit':
            m.step = None
            self._subpath_failed(why, transit=True)
        else:
            m.step = None
            if m.stretch is not None:
                self._stretch_blocked(why)
            elif not self._try_detour(m.start_local, why):
                self._no_detour(why)

    def _no_detour(self, why):
        """No detour passes the obstacle on the sub-path being mowed: the sub-path is NOT
        mowed and the mow goes on. On a narrow-strip plan the mower stops there instead
        (STUCK_NEEDS_HELP; owner 2026-11-13: "obstacles will have to stop the mower if it
        cannot go around within the area": no skip of the pass, no re-plan)."""
        pose = self.inputs.pose
        if self.mission.strip and pose is not None:
            self._stuck_needs_help('obstacle in the narrow strip, no detour fits (%s)' % why,
                                   pose)
            return
        self._subpath_failed(why)

    def _pivot_track(self, now):
        """Start time of the current in-place pivot command (fresh /cmd_vel_nav with
        |linear| < 0.01 and |angular| > 0.01), else None."""
        i = self.inputs
        cmd = i.nav_cmd
        fresh = cmd is not None and i.nav_cmd_stamp is not None and now - i.nav_cmd_stamp <= 1.0
        if fresh and abs(cmd[0]) < 0.01 and abs(cmd[1]) > 0.01:
            if getattr(self, '_pivot_since', None) is None:
                self._pivot_since = now
        else:
            self._pivot_since = None

    def pivoting_slowly(self):
        since = getattr(self, '_pivot_since', None)
        return (self.phase in ('TRANSIT', 'MOWING') and since is not None
                and self._now - since > self.p.pivot_sub_after_s)

    def _seg_last(self):
        """Last local index of what is being mowed: the sub-path, or a re-queued stretch."""
        m = self.mission
        last = len(m.subpaths[m.sub_i]) - 1
        return last if m.stretch_end is None else min(last, m.stretch_end)

    def _segment_done(self):
        if self.mission.stretch is not None:
            self._stretch_done()
        else:
            self._subpath_done()

    # ---- dynamic obstacles (person / pet): stop, blade off, wait ----------
    def _dyn_context(self):
        m, a = self.mission, self._action
        if self.phase == 'MOWING' and m is not None and m.step in ('spinup', 'spin_pause',
                                                                    'follow', 'corner_pivot'):
            return 'follow'
        if self.phase == 'TRANSIT' and m is not None and m.step == 'transit' and a is not None:
            return 'transit'
        if self.phase in DOCK_PHASES and a is not None and a.purpose in ('dock', 'dock_route',
                                                                         'dock_join',
                                                                         'route_corner'):
            return 'dock'
        return None

    def _dyn_start(self, ctx):
        m = self.mission
        cls = self.inputs.obstacle_class or 'obstacle'
        if m is not None and ctx == 'follow' and m.step == 'follow':
            m.start_local = self._track_progress()
        self._dyn_ignore = None
        # the in-path judgement that started the stop (same kind, same twist) keeps judging
        # the wait: the clear-hold sees the same evaluation the entry did (2026-10-13)
        ev = getattr(self, '_path_eval', None) or {}
        self._dyn = {'ctx': ctx, 'since': self._now, 'clear_since': None, 'cls': cls,
                     'sub': self.sub_state, 'phase': self.phase, 'purpose': self._dock_purpose,
                     'path': ev.get('path'), 'motion': ev.get('motion')}
        Hold.stop(self, 'dynamic obstacle: %s' % cls)
        if m is not None and ctx in ('follow', 'transit'):
            m.step = 'dyn_wait'
            self._soft_blade_off('dynamic obstacle: %s' % cls)
        else:
            self._blade_off('dynamic obstacle: %s' % cls)
        d = self.inputs.obstacle_distance
        self._log('warn', '%s%s ahead: stopped, blade %s, waiting up to %.0f s' % (
            cls, (' at %.1f m' % d) if d is not None else '',
            'kept on (blade_policy continuous)' if self.blade_on else 'off',
            self._dyn_wait_s(ctx)))
        self._go(self.phase, 'waiting for %s to move (0 s)' % cls)

    def _dynamic_tick(self):
        """Returns True when the dynamic-obstacle logic owns this tick."""
        d, kind = self._dyn, self._policy()
        if d is None:
            if kind != 'dynamic':
                return False
            ctx = self._dyn_context()
            if ctx is None or self._dyn_ignored():
                return False
            self._dyn_start(ctx)
            return True
        if kind == 'dynamic' or (d.get('nogo') and self._nogo_living_blocks(d)):
            d['clear_since'] = None
            d['cls'] = self.inputs.obstacle_class or d['cls']
        elif d['clear_since'] is None:
            d['clear_since'] = self._now
        waited = self._now - d['since']
        if d['clear_since'] is not None and \
                self._now - d['clear_since'] >= self.p.dynamic_clear_hold_s:
            self._dyn = None
            self._log('info', '%s gone for %.1f s: resuming' % (d['cls'],
                                                                 self.p.dynamic_clear_hold_s))
            self._dyn_resume(d)
            return True
        if waited >= self._dyn_wait_s(d['ctx']):
            self._dyn = None
            self._dyn_timeout(d)
            return True
        Hold.show_wait(self, 'waiting for %s to move (%d s)' % (d['cls'], int(waited)))
        return True

    def _dyn_wait_s(self, ctx):
        w = float(self.p.dynamic_wait_s if ctx == 'follow' else self.p.dynamic_wait_transit_s)
        if self.obstacle_level() == 'sensitive':
            w *= max(1.0, float(self.p.dynamic_wait_sensitive_factor))
        return w

    def _dyn_ignored(self):
        """True while a fresh dynamic detection falls in the post-timeout ignore
        window (same class, robot moved <= dynamic_ignore_move_m). Logs once."""
        g = self._dyn_ignore
        if g is None:
            return False
        cls = self.inputs.obstacle_class or 'obstacle'
        pose = self.inputs.pose
        moved = 0.0
        if pose is not None and g['pose'] is not None:
            moved = math.hypot(pose[0] - g['pose'][0], pose[1] - g['pose'][1])
        if self._now >= g['until'] or cls != g['cls'] or \
                moved > float(self.p.dynamic_ignore_move_m):
            self._dyn_ignore = None
            return False
        if not g['logged']:
            g['logged'] = True
            self._log('info', 'ignoring %s detections for %.0f s more (wait already timed out; '
                              'Nav2 / costmap handle it) unless the robot moves > %.1f m'
                      % (cls, g['until'] - self._now, float(self.p.dynamic_ignore_move_m)))
        return True

    def _dyn_resume(self, d):
        Hold.resume(self, d)

    def _dyn_timeout(self, d):
        m = self.mission
        why = '%s did not move within %.0f s' % (d['cls'], self._dyn_wait_s(d['ctx']))
        pose = self.inputs.pose
        self._dyn_ignore = {'until': self._now + float(self.p.dynamic_ignore_after_timeout_s),
                            'cls': d['cls'], 'logged': False,
                            'pose': (pose[0], pose[1]) if pose is not None else None}
        if d.get('nogo') and m is not None and self.phase == 'TRANSIT':
            self._nogo_living_timeout(d, why)
            return
        if d['ctx'] != 'follow' or m is None:
            # transits / docking: Nav2 plans around the marked obstacle
            self._log('warn', '%s: continuing (Nav2 plans around it)' % why)
            self._dyn_resume(d)
            return
        self._log('warn', '%s: treating it as a static obstacle' % why)
        m.step = None
        if m.stretch is not None:
            self._stretch_blocked(why)
        elif not self._try_detour(m.start_local, why):
            self._no_detour(why)

    # ---- localization hold: graded RTK-loss policy (2026-10-09, 2026-11-03) ----
    # docs/localization.md "RTK-loss policy". degraded: speed cap, finish the straight swath
    # when the drift fits the edge clearance, else hold; lost: hold; held loc_lost_retrace_s:
    # retrace the driven track toward the last RTK-fixed pose; held loc_lost_dock_s:
    # MOWING_INCOMPLETE, dock when fixed, else stay + alert. Never charges a retry budget.
    _LOC_BUDGETS = ('transit_fails', 'follow_fails', 'recovery_fails', 'route_plan_fails',
                    'route_aborts', 'detours')

    def _loc_view(self):
        """(state, reason) the hold acts on. Fail closed (review 2026-11-02 H3a): a status
        that was received but is older than loc_status_stale_s reads as 'degraded'
        (monitor crashed / hung); never received ('' state) applies no hold (bench or
        localization_monitor not launched)."""
        i = self.inputs
        if i.loc_stamp is None:
            return '', ''
        age = self._now - i.loc_stamp
        if age > float(self.p.loc_status_stale_s):
            return 'degraded', '/localization/status stale %.0f s' % age
        return i.loc_state, i.loc_reason

    def _loc_fixed(self):
        """A fresh status with the receiver at RTK FIXED."""
        i = self.inputs
        return (i.loc_stamp is not None and i.loc_rtk == 'fixed'
                and self._now - i.loc_stamp <= float(self.p.loc_status_stale_s))

    def _loc_est(self):
        """Fresh est_drift_m, or None (stale / before the first FIXED)."""
        i = self.inputs
        if i.loc_stamp is None or self._now - i.loc_stamp > float(self.p.loc_status_stale_s):
            return None
        return i.loc_est_drift

    def _loc_observe(self, state):
        """Every tick: the 'lost' run timer, the degraded speed cap and the driven track
        (the retrace path, a ring of the last loc_trail_len_m)."""
        if state == 'lost':
            if self._loc_lost_since is None:
                self._loc_lost_since = self._now
        else:
            self._loc_lost_since = None
        want = bool(self.p.loc_hold) and state in ('degraded', 'lost') and \
            self.phase in ('MOWING', 'TRANSIT') + tuple(DOCK_PHASES)
        if want != self._loc_capped:
            self._loc_capped = want
            if want:
                self._log('warn', 'localization %s: controller speed capped at %.2f m/s'
                          % (state, float(self.p.caution_speed_mps)))
            else:
                self._log('info', 'localization speed cap lifted')
            self._push_speed_caps()
        pose, d = self.inputs.pose, self._loc
        if pose is None or (d is not None and d.get('rt') is not None):
            return
        # A continuous ring of the last loc_trail_len_m of the driven track, recorded in
        # every phase whatever the RTK state, so a retrace always has track to follow (it
        # used to restart at each RTK-fixed tick and came out empty when the loss was
        # only noticed after the mower had stopped).
        xy = (float(pose[0]), float(pose[1]))
        tr = self._loc_trail
        if tr and math.hypot(xy[0] - tr[-1][0], xy[1] - tr[-1][1]) \
                < float(self.p.loc_trail_step_m):
            return
        tr.append(xy)
        keep = max(float(self.p.loc_trail_len_m), float(self.p.loc_lost_retrace_max_m) + 1.0)
        acc = 0.0
        for k in range(len(tr) - 1, 0, -1):
            acc += math.hypot(tr[k][0] - tr[k - 1][0], tr[k][1] - tr[k - 1][1])
            if acc > keep:
                del tr[:k - 1]
                break

    def _loc_swath_ahead(self):
        """(end local index, edge clearance m) of the straight swath the follow is on:
        from the tracked progress while the plan heading stays within
        loc_degraded_swath_turn_deg; clearance = the least distance of that stretch to the
        area outline / its obstacle holes. None when the area outline is unknown."""
        m = self.mission
        if m is None or not m.subpaths or m.sub_i >= len(m.subpaths):
            return None
        area = (m.areas or {}).get(m.area_idx) or {}
        rings = [r for r in [area.get('outer') or []] + list(area.get('obstacles') or [])
                 if len(r) >= 3]
        if not rings or len(rings[0]) < 3 or not (area.get('outer')):
            return None
        sp = m.subpaths[m.sub_i]
        i = self._track_progress()
        last = min(self._seg_last(), len(sp) - 1)
        h0, turn = geo.heading_of(sp, i), math.radians(float(self.p.loc_degraded_swath_turn_deg))
        j = i
        while j < last and abs(geo.wrap_angle(geo.heading_of(sp, j + 1) - h0)) < turn:
            j += 1
        pts = [sp[i]]
        for k in range(i, j):
            a, b = sp[k], sp[k + 1]
            n = max(1, int(math.ceil(geo.dist(a, b) / 0.25)))
            pts.extend((a[0] + (b[0] - a[0]) * t / n, a[1] + (b[1] - a[1]) * t / n)
                       for t in range(1, n + 1))
        pose = self.inputs.pose
        if pose is not None:
            pts.append((pose[0], pose[1]))
        clear = min(geo.distance_to_ring(q, r) for q in pts for r in rings)
        if not geo.point_in_polygon(pts[0], rings[0]):
            clear = 0.0
        return j, clear

    def _loc_swath_continue(self, state):
        """Degraded during a mow follow: True while the current straight swath may be
        finished (est_drift + loc_degraded_edge_margin_m <= its edge clearance)."""
        m = self.mission
        if state != 'degraded' or m is None or m.step != 'follow' or self.phase != 'MOWING':
            self._loc_swath = None
            return False
        est, margin = self._loc_est(), float(self.p.loc_degraded_edge_margin_m)
        sw = self._loc_swath
        if sw is None or sw['sub'] != m.sub_i or sw['area'] != m.area_idx:
            r = self._loc_swath_ahead()
            if r is None or est is None or est + margin > r[1] or r[0] <= m.progress_local:
                self._loc_swath = None
                return False
            self._loc_swath = {'sub': m.sub_i, 'area': m.area_idx, 'end': r[0], 'clear': r[1]}
            self._log('warn', 'localization degraded (drift %.2f m): finishing the current '
                      'swath at %.2f m/s (edge clearance %.2f m >= drift + %.2f m)'
                      % (est, float(self.p.caution_speed_mps), r[1], margin))
            return True
        if est is None or est + margin > sw['clear']:
            self._log('warn', 'localization degraded: drift %s + %.2f m > edge clearance '
                      '%.2f m: holding' % ('unknown' if est is None else '%.2f m' % est,
                                           margin, sw['clear']))
            self._loc_swath = None
            return False
        if self._track_progress() >= sw['end']:
            self._log('warn', 'localization degraded: swath end reached, holding')
            self._loc_swath = None
            return False
        return True

    def _loc_drop(self):
        """Forget the hold (another guard / command took over); stops a running retrace."""
        d, self._loc = self._loc, None
        if d is not None and d.get('rt') is not None:
            self._fx.append(DriveCmd(0.0, 0.0))
            self._fx.append(SuppressStuckGuard(False))

    def _loc_hold_valid(self, d):
        m = self.mission
        if self.phase != d['phase']:
            return False
        if d['ctx'] == 'dock':
            return self._action is None
        return m is not None and m.step == 'loc_hold'

    def _loc_hold_tick(self):
        """Returns True while the localization hold owns this tick. Reuses the dynamic-
        obstacle resume path (Hold.resume), so a held follow re-dispatches from its
        progress, a held transit re-sends its goal and a held dock re-docks."""
        m, d = self.mission, self._loc
        state, reason = self._loc_view()
        self._loc_observe(state)
        if d is not None and not self._loc_hold_valid(d):
            self._loc_drop()                 # another guard / command took over
            return False
        if d is None:
            pending = self._loc_fix_pending()
            if not self.p.loc_hold or (state in ('', 'ok') and not pending):
                self._loc_swath = None
                return False
            ctx = self._dyn_context()
            if ctx is None:
                return False
            if state in ('', 'ok') and ctx != 'dock':
                # a fix wait is pending and something else resumed the mission (catch-all:
                # the guarded resume paths call _loc_hold_instead_of_resume first)
                self._loc_hold_begin(ctx, 'rtk %s' % (self.inputs.loc_rtk or 'unknown'),
                                     'resumed before RTK fixed settled')
                return True
            if state in ('', 'ok'):
                return False
            if ctx == 'dock':
                lost = self._loc_lost_since
                if lost is None or self._now - lost <= float(self.p.loc_hold_dock_exempt_max_s):
                    return False             # dock transit / docking exempt (MowgliNext)
            elif self._loc_swath_continue(state):
                return False
            self._loc_hold_begin(ctx, state, reason)
            return True
        held = self._now - d['since']
        fixed = d['ctx'] == 'dock' or not self.p.loc_resume_require_fixed or \
            self._loc_fixed_settled()
        if state == 'ok':
            if d['rt'] is not None:
                self._loc_retrace_stop('localization ok')
            if d['ok_since'] is None:
                d['ok_since'] = self._now
            if not fixed and not d.get('float_logged'):
                d['float_logged'] = True
                self._log('info', 'localization ok on RTK %s: still waiting for RTK fixed '
                          '(held %.1f s) before resuming'
                          % (self.inputs.loc_rtk or 'unknown', float(self.p.loc_fixed_settle_s)))
            if fixed and self._now - d['ok_since'] >= float(self.p.loc_resume_s):
                self._loc = None
                self._loc_fix_wait = None
                self._log('info', 'localization ok again after %.0f s: resuming%s'
                          % (held, '' if d['ctx'] == 'dock' or not self.p.loc_resume_require_fixed
                             else ' (RTK fixed for %.1f s)' % (self._now - self._loc_fixed_since)))
                self._loc_restore_budgets(d)
                Hold.resume(self, d)
                return True
        else:
            d['ok_since'] = None
        if held >= float(self.p.loc_lost_dock_s):
            self._loc_give_up(d, held, state, reason or ('RTK %s, not fixed'
                                                          % (self.inputs.loc_rtk or 'unknown')))
            return True
        if d['rt'] is not None:
            self._loc_retrace_tick(d)
            return True
        if state != 'ok' and not d['retraced'] and held >= float(self.p.loc_lost_retrace_s):
            self._loc_retrace_begin(d)
            return True
        if state == 'ok' and not fixed:
            Hold.show_wait(self, 'localization ok (RTK %s): waiting for RTK fixed (%d s)'
                           % (self.inputs.loc_rtk or 'unknown', int(held)))
        else:
            Hold.show_wait(self, 'localization %s: holding (%d s)'
                           % (state or 'unknown', int(held)))
        return True

    def _loc_hold_begin(self, ctx, state, reason):
        """Start the localization hold for ``ctx`` (follow / transit / dock): stop, blade off,
        keep the goal to resume. A mission hold latches the fix wait (_loc_fix_wait)."""
        m = self.mission
        if ctx == 'follow' and m is not None and m.step == 'follow':
            m.start_local = self._track_progress()
        self._loc = {'ctx': ctx, 'since': self._now, 'ok_since': None, 'sub': self.sub_state,
                     'phase': self.phase, 'purpose': self._dock_purpose, 'rt': None,
                     'retraced': False,
                     'budget': None if m is None else
                     {k: getattr(m, k) for k in self._LOC_BUDGETS}}
        Hold.stop(self, 'localization %s' % state)
        if ctx != 'dock':
            m.step = 'loc_hold'
            if self._loc_fix_wait is None:
                self._loc_fix_wait = self._now
        self._blade_off('localization %s' % state)
        self._log('warn', 'localization %s (%s): stopped, blade off, waiting for RTK fixed'
                  % (state, reason or 'no reason'))
        self._go(self.phase, 'localization %s: holding' % state)

    # ---- fix wait after a dropout (2026-11-13, analysis/2026-11-13_rtk_dropout.md) ----
    # Once a hold has stopped the mission, the mission moves again only on RTK FIXED held
    # loc_fixed_settle_s: float passed the sigma gate 0.3 m off truth, and a boundary
    # recovery resumed on it without going back through the hold.
    def _loc_rtk_class(self):
        """The receiver class of a fresh status, 'stale' for an old one, '' before any."""
        i = self.inputs
        if i.loc_stamp is None:
            return ''
        if self._now - i.loc_stamp > float(self.p.loc_status_stale_s):
            return 'stale'
        return i.loc_rtk or ''

    def _loc_fix_track(self):
        """Every tick: the receiver class change time, the current RTK-fixed run and the fix
        wait latch. A degraded / lost status while an escape manoeuvre owns the tick (the hold
        cannot start then) latches the wait too. The latch clears once fixed has settled or
        no mission is left."""
        rtk, seen = self._loc_rtk_class(), self._loc_rtk_seen
        if seen is None or seen[0] != rtk:
            self._loc_rtk_seen = (rtk, self._now if seen is not None and seen[0] else None)
        if self._loc_fixed():
            if self._loc_fixed_since is None:
                self._loc_fixed_since = self._now
        else:
            self._loc_fixed_since = None
        if self._loc_fix_wait is None and self.p.loc_hold and self.mission is not None and \
                self.phase in ('MOWING', 'TRANSIT') and \
                any(k is not None for k in (self._hb, self._stk, self._tob)) and \
                self._loc_view()[0] in ('degraded', 'lost'):
            self._loc_fix_wait = self._now
            self._log('warn', 'localization %s during a recovery manoeuvre: its resume waits '
                      'for RTK fixed' % self._loc_view()[0])
        if self._loc_fix_wait is not None and (self.mission is None or self._loc_fixed_settled()):
            self._loc_fix_wait = None

    def _loc_rtk_changed_ago(self):
        """Seconds since the receiver class last changed (None: no change seen yet)."""
        seen = self._loc_rtk_seen
        if seen is None or seen[1] is None:
            return None
        return self._now - seen[1]

    def _loc_fixed_settled(self):
        """RTK FIXED (fresh status) continuously for loc_fixed_settle_s."""
        return self._loc_fixed_since is not None and self._loc_fixed() and \
            self._now - self._loc_fixed_since >= float(self.p.loc_fixed_settle_s)

    def _loc_fix_pending(self):
        """A hold stopped the mission and RTK fixed has not settled since."""
        return bool(self.p.loc_resume_require_fixed) and self._loc_fix_wait is not None and \
            not self._loc_fixed_settled()

    def _loc_hold_instead_of_resume(self, who):
        """A guard about to resume the mission (``who``) while the fix wait is pending: start
        the localization hold instead (its resume re-dispatches once RTK fixed settles).
        True = held, the caller must not resume."""
        m = self.mission
        if not self.p.loc_hold or m is None or not self._loc_fix_pending():
            return False
        transit = self.phase == 'TRANSIT' and m.transit_target is not None
        if transit:
            m.step = 'transit'
        self._loc_hold_begin('transit' if transit else 'follow',
                             'rtk %s' % (self.inputs.loc_rtk or 'unknown'),
                             '%s done before RTK fixed settled' % who)
        return True

    def _loc_restore_budgets(self, d):
        """A localization hold is a guard pause (MowgliNext): whatever the stop cost a
        retry / no-progress counter is given back, and the route progress watch restarts."""
        m, b = self.mission, d.get('budget')
        if m is not None and b:
            for k, v in b.items():
                setattr(m, k, v)
        self._route_watch = None

    def _loc_give_up(self, d, held, state, reason):
        """Held loc_lost_dock_s: MOWING_INCOMPLETE (resume cursor kept); RTK fixed -> dock
        (return_home_on_incomplete), else stay stopped (mower_alerts mowIncomplete)."""
        if d['rt'] is not None:
            self._loc_retrace_stop('giving up')
        self._loc = None
        fixed = self._loc_fixed()
        why = 'localization not ok for %.0f s (%s)' % (held, reason or state)
        if d['ctx'] == 'dock':
            if fixed:
                self._log('warn', '%s: RTK fixed, docking again' % why)
                self._dock(d['phase'], d['purpose'], sub=d['sub'])
            else:
                self._blade_off(why)
                self._fx.append(ZeroBurst())
                self._dock_failed('%s, no RTK fixed: stopped' % why)
            return
        m = self.mission
        left = self._not_mowed_text() if m is not None else ''
        self._log('error', '%s: %s' % (why, 'RTK fixed, ending the session' if fixed
                                       else 'no RTK fixed, stopping in place'))
        self._interrupt_mission(why)
        self._cancel_all(why)
        self._blade_off(why)
        self._fx.append(ZeroBurst())
        self._dock_purpose = None
        text = why + ('; ' + left if left else '')
        self._finish_incomplete(text, stay_reason=None if fixed else
                                'no RTK fixed (localization %s)' % (state or 'unknown'))

    # ---- lost: retrace the driven track toward the last RTK-fixed pose ----
    def _loc_retrace_begin(self, d):
        pose = self.inputs.pose
        d['retraced'] = True
        if pose is None:
            return
        max_m = float(self.p.loc_lost_retrace_max_m)
        path, acc, prev = [], 0.0, (float(pose[0]), float(pose[1]))
        for q in reversed(self._loc_trail):
            step = math.hypot(q[0] - prev[0], q[1] - prev[1])
            if step < 0.02:
                continue
            if acc + step > max_m:
                f = (max_m - acc) / step
                path.append((prev[0] + (q[0] - prev[0]) * f, prev[1] + (q[1] - prev[1]) * f))
                break
            acc += step
            path.append(q)
            prev = q
        if not path or max_m <= 0.0:
            self._log('info', 'localization: no driven track to retrace, waiting in place')
            return
        d['rt'] = {'path': path, 'i': 0, 'driven': 0.0, 'last': (float(pose[0]), float(pose[1])),
                   'best_t': self._now, 'len': geo.path_length(
                       [(pose[0], pose[1])] + path)}
        self._fx.append(SuppressStuckGuard(True))
        self._log('warn', 'localization not ok for %.0f s: retracing %.2f m of the driven track '
                  'toward the last RTK-fixed pose at %.2f m/s'
                  % (self._now - d['since'], d['rt']['len'],
                     float(self.p.slow_maneuver_mps)))
        self._go(self.phase, 'localization lost: retracing toward the last fix')

    def _loc_retrace_stop(self, why):
        d = self._loc
        rt = d['rt']
        d['rt'] = None
        self._fx.append(DriveCmd(0.0, 0.0))
        self._fx.append(SuppressStuckGuard(False))
        self._log('info', 'localization retrace ended after %.2f m (%s): waiting'
                  % (rt['driven'], why))

    @staticmethod
    def _track_cmd(pose, target, v0):
        """(v, w) toward ``target`` (x, y) at speed ``v0``: forward when it is ahead of the
        robot, reversing when behind (pure pursuit on the arc through it, |w| <= 0.3)."""
        dx, dy = target[0] - pose[0], target[1] - pose[1]
        c, s = math.cos(pose[2]), math.sin(pose[2])
        xr, yr = c * dx + s * dy, -s * dx + c * dy
        v = v0 if xr > 0 else -v0
        w = max(-0.3, min(0.3, v * 2.0 * yr / max(1e-6, xr * xr + yr * yr)))
        return v, w

    def _loc_retrace_tick(self, d):
        rt, pose = d['rt'], self.inputs.pose
        if pose is None:
            self._loc_retrace_stop('no pose')
            return
        step = math.hypot(pose[0] - rt['last'][0], pose[1] - rt['last'][1])
        if step > 0.005:
            rt['driven'] += step
            rt['best_t'] = self._now
            rt['last'] = (float(pose[0]), float(pose[1]))
        path, la = rt['path'], 0.3
        while rt['i'] < len(path) - 1 and math.hypot(path[rt['i']][0] - pose[0],
                                                    path[rt['i']][1] - pose[1]) < la:
            rt['i'] += 1
        end = path[-1]
        if rt['driven'] >= float(self.p.loc_lost_retrace_max_m) or \
                math.hypot(end[0] - pose[0], end[1] - pose[1]) < 0.1:
            self._loc_retrace_stop('track end' if rt['driven'] < float(
                self.p.loc_lost_retrace_max_m) else 'loc_lost_retrace_max_m')
            return
        if self._now - rt['best_t'] > 10.0:
            self._loc_retrace_stop('no progress for 10 s')
            return
        v, w = self._track_cmd(pose, path[rt['i']], abs(float(self.p.slow_maneuver_mps)))
        self._fx.append(DriveCmd(v, w))
        Hold.show_wait(self, 'localization lost: retracing (%.1f of %.1f m)'
                       % (rt['driven'], rt['len']))

    # ---- static obstacles: blade-off detour to the swath beyond ----------
    def _try_detour(self, cur, why):
        m, p = self.mission, self.p
        if not p.obstacle_avoidance or m.stretch is not None:
            return False
        if m.detours >= p.max_detours_per_subpath:
            self._log('warn', 'area %d sub-path %d: %d detours used (max_detours_per_subpath): '
                              'no detour' % (m.area_idx, m.sub_i, m.detours))
            return False
        return self._detour_send(cur, float(p.detour_skip_m), why)

    def _detour_send(self, cur, skip, why):
        m, p = self.mission, self.p
        sp = m.subpaths[m.sub_i]
        last = self._seg_last()
        idx = geo.advance_index(sp, cur, skip)
        if idx >= last and m.strip:
            # a strip pass is never cut short at an obstacle (_no_detour stops the mower)
            return False
        if idx >= last:
            # obstacle on the sub-path end: skip the rest, re-queued at the area end
            m.detours += 1
            if not self._keep_blade_inside('obstacle at the sub-path end'):
                self._blade_off('obstacle at the sub-path end')
            self._log('warn', 'area %d sub-path %d: %s within %.1f m of the end: skipping '
                              'poses %d -> %d' % (m.area_idx, m.sub_i, why, skip, cur, last))
            self._record_skip(cur, last)
            m.step = None
            self._subpath_done()
            return True
        if not geo.inside_area(sp[idx], m.areas.get(m.area_idx, {}), p.detour_max_leave_m):
            self._log('warn', 'area %d sub-path %d: detour pose %d (%.2f, %.2f) outside the '
                              'area: no detour' % (m.area_idx, m.sub_i, idx, sp[idx][0],
                                                   sp[idx][1]))
            return False
        # force_off: a detour never keeps the blade on by the continuous-area straight-segment
        # rule (review 2026-10-09: the Nav2 detour path, not that segment, passes 0.25-0.6 m
        # from an obstacle YOLO may not have classified). Since 2026-10-23 the owner rule in
        # _keep_blade_inside still keeps it on while the body stays inside the area.
        self._transit_blade(sp[idx], 'detour', force_off=True)
        if not m.detour:
            self._incident('detour', why)
        m.detour = {'from': cur, 'to': idx, 'skip': skip, 'why': why}
        m.start_local = idx
        m.step = 'transit'
        m.transit_target = (sp[idx][0], sp[idx][1], geo.heading_of(sp, idx))
        self._log('warn', 'area %d sub-path %d: %s: blade-%s detour %d/%d to pose %d '
                          '(%.1f m ahead)' % (m.area_idx, m.sub_i, why,
                                              'on' if m.blade_transit else 'off', m.detours + 1,
                                              p.max_detours_per_subpath, idx, skip))
        # costmaps are NOT cleared: the global planner must see the obstacle marks
        self._go('TRANSIT', 'detour around obstacle: area %d sub-path %d/%d' % (
            m.area_idx, m.sub_i + 1, len(m.subpaths)))
        self._start_action(ACT_NAV, {'pose': m.transit_target}, p.transit_timeout_s)
        return True

    def _record_skip(self, lfrom, lto):
        m = self.mission
        offs, _ = subpath_offsets(m.subpaths)
        ac = self.cursor.area(m.area_idx)
        ac.skipped.append((offs[m.sub_i] + int(lfrom), offs[m.sub_i] + int(lto)))
        self._persist()

    def _detour_done(self):
        m = self.mission
        d, m.detour = m.detour, None
        m.detours += 1
        m.transit_fails = m.follow_fails = 0
        m.route_plan_fails = 0
        m.recovering = False
        self._record_skip(d['from'], d['to'])
        self._log('info', 'detour done: sub-path %d poses %d -> %d skipped (re-tried at the '
                          'area end)' % (m.sub_i, d['from'], d['to']))
        self._begin_blade()

    def _detour_failed(self, outcome):
        m, p = self.mission, self.p
        d, m.detour = m.detour, None
        skip = d['skip'] + float(p.detour_skip_step_m)
        if p.detour_skip_step_m > 0 and skip <= float(p.detour_max_skip_m) + 1e-6:
            self._log('warn', 'detour transit %s: extending the skip to %.1f m'
                      % (outcome, skip))
            if self._detour_send(d['from'], skip, d['why']):
                return
        m.start_local = d['from']
        m.follow_fails += 1
        if m.follow_fails > p.follow_retries:
            self._subpath_failed('obstacle could not be bypassed (%s)' % d['why'])
            return
        self._schedule_retry('MOWING', 'area %d sub-path %d/%d: detour failed (%s), retry %d/%d '
                             'in %.0f s' % (m.area_idx, m.sub_i + 1, len(m.subpaths), outcome,
                                            m.follow_fails, p.follow_retries,
                                            p.transit_retry_delay_s))

    # ---- end-of-area pass over the skipped stretches ----------------------
    def _next_stretch(self):
        m = self.mission
        ac = self.cursor.area(m.area_idx)
        pose = self.inputs.pose
        if pose is not None and len(m.requeue) > 1:
            m.requeue.sort(key=lambda s: self._stretch_dist(s, pose))     # nearest first
        fr, to = m.requeue.pop(0)
        a = absolute_to_local(m.subpaths, fr)
        b = absolute_to_local(m.subpaths, to)
        if a is None or b is None or a[0] != b[0] or b[1] <= a[1]:
            if (fr, to) in ac.skipped:
                ac.skipped.remove((fr, to))
                self._persist()
            self._area_done()
            return
        # Field 2026-11-03: a stretch shorter than stretch_min_retry_m is
        # dropped as too short to matter (ignored_short_m), never counted blocked / NOT
        # mowed, so a 99.8 % mow still completes.
        sp = m.subpaths[a[0]]
        cum = geo.cumulative_lengths(sp)
        length = cum[b[1]] - cum[a[1]]
        if length < float(self.p.stretch_min_retry_m):
            if (fr, to) in ac.skipped:
                ac.skipped.remove((fr, to))
                self._persist()
            m.ignored_short_m += length
            self._log('info', 'area %d: stretch sub-path %d poses %d -> %d dropped as too '
                      'short to matter (%.2f m < %.2f m)'
                      % (m.area_idx, a[0], a[1], b[1], length,
                         float(self.p.stretch_min_retry_m)))
            self._area_done()
            return
        m.stretch, m.stretch_end = (fr, to), b[1]
        m.sub_i, m.start_local = a
        m.retry_used, m.premature, m.boundary_recoveries = False, 0, 0
        m.transit_fails = m.follow_fails = m.recovery_fails = 0
        m.route_plan_fails = 0
        m.recovering, m.detour = False, None
        self._log('info', 'area %d: re-trying skipped stretch sub-path %d poses %d -> %d'
                  % (m.area_idx, a[0], a[1], b[1]))
        self._dispatch()

    def _stretch_dist(self, s, pose):
        m = self.mission
        a = absolute_to_local(m.subpaths, s[0])
        if a is None:
            return float('inf')
        q = m.subpaths[a[0]][a[1]]
        return math.hypot(q[0] - pose[0], q[1] - pose[1])

    def _static_block(self):
        """The retry was stopped by a static obstacle on the stretch (not a person or
        animal, which moves): an on-path obstacle decision (act / stop); or a recent ranged
        static on the path (_obstacle_on_path) or within robot_width_m / 2 +
        obstacle_path_margin_m of the stretch; or a recent collision with the robot's front
        that close to the stretch. Review 2026-11-02: a detection anywhere in the frame
        used to exclude the stretch for good (blocked_static, area 'complete')."""
        if self._policy() == 'dynamic':
            return False
        dec = self.obstacle_decision or {}
        if dec.get('kind', 'none') != 'none' and dec.get('on_path') is True and \
                dec.get('action') in ('act', 'stop'):
            return True
        p, pose = self.p, self.inputs.pose
        near = float(p.robot_width_m) / 2.0 + float(p.obstacle_path_margin_m)
        if self._static_recent():
            o = self._obstacle_xy()
            if self._obstacle_on_path() is True or \
                    (o is not None and self._stretch_seg_dist(o) <= near):
                return True
        if self._collision_recent() and pose is not None:
            front = float(p.robot_front_m)
            nose = (pose[0] + front * math.cos(pose[2]), pose[1] + front * math.sin(pose[2]))
            if self._stretch_seg_dist(nose) <= near:
                return True
        return False

    def _stretch_seg_dist(self, pt):
        """Distance from ``pt`` to the polyline of the stretch being re-tried (inf without)."""
        m = self.mission
        if m is None or m.stretch is None:
            return float('inf')
        a = absolute_to_local(m.subpaths, m.stretch[0])
        b = absolute_to_local(m.subpaths, m.stretch[1])
        if a is None or b is None or a[0] != b[0]:
            return float('inf')
        pts = m.subpaths[a[0]][a[1]:b[1] + 1]
        if len(pts) < 2:
            return geo.dist(pt, pts[0]) if pts else float('inf')
        return min(_seg_dist(pt, p0, p1) for p0, p1 in zip(pts, pts[1:]))

    def _next_failed_retry(self):
        """End-of-area pass: re-try one sub-path dropped on transit failure, as a
        stretch from its first missing pose (success marks it completed)."""
        m = self.mission
        e = m.requeue_subs.pop(0)
        if e in m.failed:
            m.failed.remove(e)
        sub = e[1]
        offs, _ = subpath_offsets(m.subpaths)
        last = len(m.subpaths[sub]) - 1
        start = max(0, min(e[3] - offs[sub], last)) if e[3] >= 0 else 0
        m.retry_entry = e
        m.stretch, m.stretch_end = (offs[sub] + start, offs[sub] + last), last
        m.sub_i, m.start_local = sub, start
        m.retry_used, m.premature, m.boundary_recoveries = False, 0, 0
        m.transit_fails = m.follow_fails = m.recovery_fails = 0
        m.route_plan_fails = 0
        m.recovering, m.detour = False, None
        self._blk = None
        self._log('info', 'area %d: end-of-area retry of sub-path %d (%s)'
                  % (m.area_idx, sub, e[2]))
        self._dispatch()

    def _stretch_done(self):
        m = self.mission
        ac = self.cursor.area(m.area_idx)
        if m.retry_entry is not None:
            ac.completed.add(m.retry_entry[1])
            ac.done_fps.add(geo.plan_fingerprint([m.subpaths[m.retry_entry[1]]]))
            m.completed_count += 1
            self._log('info', 'area %d: sub-path %d mowed on the end-of-area retry'
                      % (m.area_idx, m.retry_entry[1]))
            m.retry_entry = None
        elif m.stretch in ac.skipped:
            ac.skipped.remove(m.stretch)
            self._log('info', 'area %d: skipped stretch %s mowed' % (m.area_idx, m.stretch))
        m.stretch, m.stretch_end, m.step = None, None, None
        self._persist()
        self._area_done()

    def _stretch_blocked(self, why):
        m = self.mission
        if m.retry_entry is not None:
            e, m.retry_entry = m.retry_entry, None
            m.failed.append(e)               # NOT mowed after all (original reason kept)
            self._log('warn', 'area %d sub-path %d still NOT mowed on the end-of-area retry '
                      '(%s)' % (m.area_idx, e[1], why))
            m.stretch, m.stretch_end, m.step = None, None, None
            self._blade_off('end-of-area retry failed')
            self._area_done()
            return
        if self._static_block():
            ac = self.cursor.area(m.area_idx)
            st = tuple(m.stretch)
            if st in ac.skipped:
                ac.skipped.remove(st)
            if st not in ac.blocked_static:
                ac.blocked_static.append(st)
            self._persist()
            self._log('warn', 'area %d: stretch %s blocked by a static obstacle on the path '
                      '(%s): excluded (blocked_static)' % (m.area_idx, st, why))
            self._incident('stretch_blocked_static', why)
            m.stretch, m.stretch_end, m.step = None, None, None
            self._blade_off('stretch blocked')
            self._area_done()
            return
        m.blocked.append((m.area_idx,) + tuple(m.stretch))
        self._log('warn', 'area %d: stretch %s still blocked (%s): obstacle cannot be bypassed'
                  % (m.area_idx, m.stretch, why))
        m.stretch, m.stretch_end, m.step = None, None, None
        self._blade_off('stretch blocked')
        self._area_done()

    def _handle_service(self, s, ok, resp):
        if isinstance(s.purpose, str) and s.purpose.startswith('preview_'):
            self._preview_on_service(s.purpose, ok, resp)
            return
        if s.purpose == 'route':
            self._route_planned(ok, resp)
            return
        if s.purpose == 'dock_route':
            self._dock_route_planned(ok, resp)
            return
        if s.purpose == 'enum':
            e = self._enum
            if e is None:
                return
            if ok and resp.get('success'):
                area = accepted_area(resp.get('area', {}))
                if area.get('pending_obstacles') and e['purpose'] == 'start':
                    self._log('info', 'area %d: %d pending (unreviewed) keepout(s) left out of '
                                      'the plan: run time only until accepted'
                              % (e['index'], len(area['pending_obstacles'])))
                e['count'] += 1
                e.setdefault('all', {})[e['index']] = area
                if not area.get('is_navigation_area', False):
                    e['areas'][e['index']] = area
                e['index'] += 1
                if e['index'] < self.p.max_areas:
                    self._call(SRV_GET_AREA, {'index': e['index']}, purpose='enum')
                    return
            self._enum = None
            if e.get('all'):
                self._region_areas = dict(e['all'])     # path conditioning's region
            if e['purpose'] == 'start':
                if self.mission is None:
                    return
                if not ok and e['index'] == 0:
                    self._abort_start('/map_server_node/get_mowing_area unavailable')
                    return
                self._areas_loaded(e['areas'], e['count'])
            else:
                if self._record_poly is not None:
                    self._record_save(len(e['areas']), e['count'], e['areas'])
        elif s.purpose == 'record_obstacle':
            if self._record_poly is not None:
                added = ok and bool(resp.get('success', False))
                if not added:
                    self._log('warn', 'promote_obstacle refused: %s'
                              % (resp.get('message') or 'unavailable'))
                self._record_saved(added, failed_service='promote_obstacle')
        elif s.purpose == 'add_area':
            if self._record_poly is not None:
                added = ok and bool(resp.get('success', False))
                if added and self._record_kind == 'path' and self._record_line:
                    self._call(SRV_SET_CHANNEL, {
                        'index': self._record_index,
                        'settings_json': json.dumps({
                            'points': [[round(x, 3), round(y, 3)] for x, y in self._record_line],
                            'width_m': float(self.p.record_path_width_m)})},
                        purpose='set_channel')
                    return
                self._record_saved(added)
        elif s.purpose == 'set_channel':
            if self._record_poly is not None:
                if not (ok and resp.get('success', False)):
                    # the band polygon is saved; only the centreline metadata is missing
                    self._log('warn', 'set_area_channel failed for "%s": %s' % (
                        self._record_name, resp.get('message', 'unavailable')))
                self._record_saved(True)
        elif s.purpose in ('area_settings', 'coverage_params'):
            m = self.mission
            # coverage parameters also come back mid-mow: a no-go re-plan plans every run
            # of the area's plan (areas._plan_area)
            replanning = (m is not None and s.purpose == 'coverage_params'
                          and m.nogo_replan is not None)
            if m is None or m.area_idx is None or (self.phase != 'PLANNING' and not replanning):
                return
            if s.purpose == 'area_settings':
                self._on_area_settings(ok, resp)
                return
            if not ok:
                self._log('warn', 'area %d: /coverage_server set_parameters failed (%s): planning '
                                  'with its current parameters'
                          % (m.area_idx, resp.get('message') or 'unavailable'))
            self._start_plan()
