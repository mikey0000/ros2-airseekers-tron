# SPDX-License-Identifier: GPL-3.0-or-later
"""MissionFSM mixin: Obstacle avoidance: blocked / dynamic / localization holds, detours, stretches.

Moved verbatim from mission_fsm.py (2026-10-23 code review stage 1.6)."""

from mower_mission.fsm.common import (ACT_NAV, DOCK_PHASES, SRV_GET_AREA, SRV_SET_CHANNEL,
    ZeroBurst, _STILL_V, _STILL_W, _along, _seg_dist, absolute_to_local, geo, json, math,
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
        if kind != 'none' and self.p.obstacle_path_filter:
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
        i = self.inputs
        d, b, pose = i.obstacle_distance, i.obstacle_bearing, i.pose
        if d is None or b is None or pose is None:
            return None
        yaw = pose[2] if len(pose) > 2 else 0.0
        a = yaw + math.radians(float(b))
        o = (pose[0] + d * math.cos(a), pose[1] + d * math.sin(a))
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
        if kind == 'dynamic':
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
            self._subpath_failed(why)

    # ---- localization hold (2026-10-09): drift budget spent -> stop, blade off ----
    def _loc_fresh(self):
        i = self.inputs
        return i.loc_stamp is not None and \
            self._now - i.loc_stamp <= float(self.p.loc_status_timeout_s)

    def _loc_hold_tick(self):
        """Returns True while the localization hold owns this tick. Reuses the dynamic-
        obstacle resume path (_dyn_resume), so a held follow re-dispatches from its
        progress and a held transit re-sends its goal."""
        m, i, d = self.mission, self.inputs, self._loc
        if d is not None and (m is None or self.phase != d['phase'] or m.step != 'loc_hold'):
            self._loc = None                 # another guard / command took over
            return False
        if d is None:
            if not self.p.loc_hold or not self._loc_fresh() or i.loc_state in ('', 'ok'):
                return False
            ctx = self._dyn_context()
            if ctx not in ('follow', 'transit'):
                return False
            if ctx == 'follow' and m.step == 'follow':
                m.start_local = self._track_progress()
            self._loc = {'ctx': ctx, 'since': self._now, 'ok_since': None, 'sub': self.sub_state,
                         'phase': self.phase, 'purpose': self._dock_purpose}
            Hold.stop(self, 'localization %s' % i.loc_state)
            m.step = 'loc_hold'
            self._blade_off('localization %s' % i.loc_state)
            self._log('warn', 'localization %s (%s): stopped, blade off, waiting for RTK fixed'
                      % (i.loc_state, i.loc_reason or 'no reason'))
            self._go(self.phase, 'localization %s: holding' % i.loc_state)
            return True
        if self._loc_fresh() and i.loc_state == 'ok':
            if d['ok_since'] is None:
                d['ok_since'] = self._now
            if self._now - d['ok_since'] >= float(self.p.loc_resume_s):
                self._loc = None
                self._log('info', 'localization ok again after %.0f s: resuming'
                          % (self._now - d['since']))
                self._dyn_resume(d)
                return True
        else:
            d['ok_since'] = None
        held = self._now - d['since']
        if held >= float(self.p.loc_hold_max_s):
            self._loc = None
            why = 'localization not ok for %.0f s (%s)' % (held, i.loc_reason or i.loc_state)
            self._log('error', '%s: stopping in place' % why)
            self._interrupt_mission(why)
            self._cancel_all(why)
            self._blade_off(why)
            self._fx.append(ZeroBurst())
            self._dock_purpose = None
            self._go_idle(why)
            return True
        Hold.show_wait(self, 'localization %s: holding (%d s)'
                       % (i.loc_state or 'unknown', int(held)))
        return True

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
        # Blade always OFF on a detour (review 2026-10-09): with detour_unclassified_after 0 the
        # first "collision ahead" goes straight here, and the obstacle may be a child, pet or
        # hose that YOLO did not classify; the Nav2 path (not the straight segment
        # _transit_blade checks) passes within 0.25-0.6 m of it.
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
        fr, to = m.requeue.pop(0)
        a = absolute_to_local(m.subpaths, fr)
        b = absolute_to_local(m.subpaths, to)
        if a is None or b is None or a[0] != b[0] or b[1] <= a[1]:
            if (fr, to) in ac.skipped:
                ac.skipped.remove((fr, to))
                self._persist()
            self._area_done()
            return
        # field 2026-10-23 afternoon (32 stretch retries, 18 still blocked): a stretch shorter
        # than stretch_min_retry_m, or one near a stretch that could not be bypassed in this
        # mission, is reported blocked without driving to it again
        sp = m.subpaths[a[0]]
        cum = geo.cumulative_lengths(sp)
        length = cum[b[1]] - cum[a[1]]
        p0, p1 = sp[a[1]], sp[b[1]]
        near = [q for q in m.unbypassable
                if min(geo.dist(q, p0), geo.dist(q, p1)) <= float(self.p.stretch_block_radius_m)]
        if length < float(self.p.stretch_min_retry_m) or near:
            m.blocked.append((m.area_idx, fr, to))
            self._log('info', 'area %d: stretch sub-path %d poses %d -> %d not re-tried (%s)'
                      % (m.area_idx, a[0], a[1], b[1],
                         '%.2f m < %.2f m' % (length, float(self.p.stretch_min_retry_m))
                         if not near else 'next to an obstacle that could not be bypassed'))
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
        m.blocked.append((m.area_idx,) + tuple(m.stretch))
        pose = self.inputs.pose
        if pose is not None:
            m.unbypassable.append((float(pose[0]), float(pose[1])))
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
                area = resp.get('area', {})
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
            if m is None or self.phase != 'PLANNING' or m.area_idx is None:
                return
            if s.purpose == 'area_settings':
                self._on_area_settings(ok, resp)
                return
            if not ok:
                self._log('warn', 'area %d: /coverage_server set_parameters failed (%s): planning '
                                  'with its current parameters'
                          % (m.area_idx, resp.get('message') or 'unavailable'))
            self._start_plan()
