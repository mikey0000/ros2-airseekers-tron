# SPDX-License-Identifier: GPL-3.0-or-later
"""MissionFSM mixin: no-go zones in the mission (option C, 2026-10-23,
docs/analysis/2026-10-23_obstacle_nogo_options.md).

* Mid-mow re-plan: a pending / accepted zone of /obstacle_tracker/nogo_zones on the remaining
  path of the current area is added to the area's holes and the remaining coverage is
  re-planned (the planner's hard-obstacle rule: lap 0 bends inward around a zone near the
  line); the new plan is mapped onto the area's mowed raster (progress kept);
  rate-limited; a zone that expires later leaves the plan as it is until the next area.
* Pre-pivot sweep check before every coverage pivot (corners, K-turn about-turns) and transit
  route corner: other direction, back-up, K-turn, pivot moved along the path, then (e) a
  re-plan around the zone or skip the stretch.
* Edge laps (B6, owner): a zone / static mark on the lap ahead -> detour inward around it on
  the lap itself and rejoin, blade on, no stop-wait."""

from mower_mission import nogo
from mower_mission.fsm.common import (ACT_BACKUP, ACT_DRIVE, ACT_FOLLOW, ACT_PLAN, ACT_SPIN,
    AreaCursor, CancelActions, DriveCmd, PublishPlan, SUCCEEDED, CONDITIONED_KEY, geo, math,
    _seg_dist, costmap_lethal_points, subpath_offsets, walls)  # noqa: F401


class NogoMixin:
    """No-go zones: mid-mow re-plan, pre-pivot sweep check, edge-lap detour."""

    # ---- zone input ------------------------------------------------------
    def _nogo_set(self):
        """The latest /obstacle_tracker/nogo_zones (nogo.ZoneSet) if fresh, else None."""
        i, p = self.inputs, self.p
        if not p.nogo_enabled or i.nogo_zones is None or i.nogo_stamp is None or \
                self._now - i.nogo_stamp > float(p.nogo_zones_max_age_s):
            return None
        return i.nogo_zones

    def _nogo_discs(self, marks=True):
        zs = self._nogo_set()
        if zs is None:
            return []
        self._nogo_living_note(zs)
        discs = zs.discs(include_candidates=True, include_marks=marks)
        spots = self.mission.nogo_living_spots if self.mission is not None else []
        if not marks or not spots:
            return discs
        # a static mark where a person / animal was recently is that animal (field
        # 2026-10-28: a 'stone' mark under the dog kept the corner blocked after it left)
        return [z for z in discs if z.state != 'mark' or not any(
            math.hypot(z.x - x, z.y - y) <= r + z.r for (x, y, r, _t) in spots)]

    def _nogo_living_note(self, zs):
        """Remember where living-class zones were in the last nogo_living_memory_s."""
        m = self.mission
        if m is None:
            return
        keep = float(self.p.nogo_living_memory_s)
        spots = [q for q in m.nogo_living_spots if self._now - q[3] <= keep]
        for z in zs.zones:
            if z.label in nogo.LIVING_CLASSES:
                spots = [q for q in spots if math.hypot(q[0] - z.x, q[1] - z.y) > 0.05]
                spots.append((z.x, z.y, z.r, self._now))
        m.nogo_living_spots = spots[-50:]

    def _nogo_area_holes(self, idx):
        m = self.mission
        area = m.areas.get(idx, {}) if isinstance(m.areas, dict) else {}
        return [list(o) for o in area.get('obstacles', [])] + \
            [list(o) for o in m.nogo_extra.get(idx, [])] + \
            self._line_holes(area.get('outer') or [])

    def _line_holes(self, outer):
        """Boundary-line keepouts (walls.area_holes) overlapping the area ``outer``: coverage
        holes, so the planner cuts its swaths at every line (2026-11-02)."""
        p = self.p
        if not p.boundary_lines_enabled:
            return []
        return walls.area_holes(self.inputs.boundary_lines, outer,
                                float(p.boundary_line_keepout_m),
                                walls.blade_keepout(self.inputs.boundary_blade_radius,
                                                    p.boundary_line_blade_keepout_m))

    # ---- tick: re-plan trigger, edge-lap detour ----------------------------
    def _nogo_tick(self):
        """MOWING follow / TRANSIT tick. True = it started something (the tick is done)."""
        m, p = self.mission, self.p
        if not p.nogo_enabled or m is None or m.area_idx is None or not m.subpaths or \
                m.nogo_replan is not None or m.nogo_seq is not None or \
                m.nogo_detour is not None or self._now < m.nogo_check_at:
            return False
        m.nogo_check_at = self._now + float(p.nogo_replan_check_s)
        zs = self._nogo_set()
        if zs is None:
            return False
        self._nogo_living_note(zs)
        zones = self._nogo_replan_zones(zs.zones)
        if zones and self._nogo_replan_budget():
            return self._nogo_replan(zones, 'zone on the remaining path')
        if p.nogo_edge_detour and self.phase == 'MOWING' and m.step == 'follow' and \
                self._action is not None and self._action.name == ACT_FOLLOW and \
                m.stretch is None and self._edge_lap(m.sub_i):
            return self._nogo_edge_detour(self._track_progress(), 'no-go zone / static mark '
                                          'on the edge lap ahead', proactive=True)
        return False

    def _nogo_remaining(self):
        """Remaining coverage of the current area: the current sub-path from the robot on
        and every later sub-path not completed."""
        m = self.mission
        ac = self.cursor.area(m.area_idx)
        out = []
        for k, sp in enumerate(m.subpaths):
            if k < m.sub_i or k in ac.completed:
                continue
            if k == m.sub_i:
                i = m.progress_local if m.step == 'follow' else m.start_local
                sp = sp[max(0, min(i, len(sp) - 1)):]
            out.append(sp)
        return out

    def _nogo_replan_zones(self, zones, require_hit=True):
        """Zones (pending / accepted, per nogo_replan_states) in the current area, not yet a
        hole of its plan, that the body would touch on the remaining path."""
        m, p = self.mission, self.p
        if not p.nogo_replan or m.stretch is not None or m.requeue is not None or \
                m.requeue_subs is not None or m.detour is not None:
            return []
        area = m.areas.get(m.area_idx, {}) if isinstance(m.areas, dict) else {}
        outer = area.get('outer') or []
        holes = self._nogo_area_holes(m.area_idx)
        states = set(p.nogo_replan_states or [])
        cand = [z for z in zones if z.state in states and nogo.zone_in_area(z, outer)
                and not nogo.zone_in_plan(z, holes)]
        if not cand or not require_hit:
            return cand
        rest = self._nogo_remaining()
        pad = float(p.nogo_replan_pad_m)
        return [z for z in cand if nogo.path_hits_zone(rest, z, pad)]

    def _nogo_replan_budget(self):
        m, p = self.mission, self.p
        n = m.nogo_replans.get(m.area_idx, 0)
        if n >= int(p.nogo_replan_max_per_area):
            return False
        return self._now - m.nogo_last_replan >= float(p.nogo_replan_min_interval_s)

    # ---- mid-mow re-plan -------------------------------------------------
    def _nogo_replan(self, zones, why):
        """Stop, add ``zones`` to the area's holes and re-plan the area (see _nogo_on_plan)."""
        m, p = self.mission, self.p
        idx = m.area_idx
        if self._action is not None:
            self._fx.append(CancelActions('no-go re-plan'))
            self._action = None
        self._fx.append(DriveCmd(0.0, 0.0))
        self._spin_resend = None
        # the planner keeps base_link nogo_plan_grow_m + 0.40 from the hole; the nose / the
        # front corners reach 0.60 / 0.645 m ahead (trunk_x1: a lap bending round a 0.40 m
        # keepout drove its nose into the trunk head-on before the arc)
        grow = max(0.0, float(p.nogo_plan_grow_m))
        polys = [nogo.zone_polygon(z.x, z.y, z.r + grow) for z in zones]
        m.nogo_extra.setdefault(idx, []).extend(polys)
        m.nogo_replans[idx] = m.nogo_replans.get(idx, 0) + 1
        m.nogo_last_replan = self._now
        m.nogo_replan = {'sub': m.sub_i, 'why': why,
                         'progress': m.progress_local if m.step == 'follow' else m.start_local,
                         'phase': self.phase}
        m.step = 'nogo_replan'
        m.pivot_done, m.pivot_target, m.corner_end = None, None, None
        self._blk = None
        if self.blade_on and not self._keep_blade_inside('no-go re-plan'):
            self._blade_off('no-go re-plan')
        txt = ', '.join('%s #%d (%.2f, %.2f) r %.2f %s' % (z.label or 'zone', z.id, z.x, z.y,
                                                           z.r, z.state) for z in zones)
        self._log('warn', 'no-go re-plan %d/%d of area %d (%s): %s -> re-planning the area with '
                          'it as a hole (the mowed raster keeps what was cut)'
                  % (m.nogo_replans[idx], int(p.nogo_replan_max_per_area), idx, why, txt))
        self._incident('nogo_replan', txt)
        self._go(self.phase, 'no-go zone: re-planning area %d' % idx)
        self._start_plan(purpose='nogo_replan')
        return True

    def _nogo_on_plan(self, outcome, result):
        """Result of a no-go re-plan: the new sub-paths with the mowed ones marked done (and
        a resume inside the first partly mowed one); a failed plan keeps the old one."""
        m, p = self.mission, self.p
        r, m.nogo_replan = m.nogo_replan, None
        m.step = None
        subpaths = self._plan_subpaths(outcome, result)
        if not subpaths:
            self._log('warn', 'no-go re-plan of area %d failed (%s): keeping the old plan'
                      % (m.area_idx, result.get('message') or outcome))
            m.start_local = min(int(r['progress']), self._seg_last())
            self._dispatch()
            return
        m.cond, m.cond_errors = {}, {}
        if p.path_conditioning:
            subpaths = self._condition_subpaths(subpaths, result.get(CONDITIONED_KEY))
        old_ac = self.cursor.area(m.area_idx)
        done, prefix = self._raster_progress(subpaths, old_ac.raster)
        for k, sp in enumerate(subpaths):
            if geo.plan_fingerprint([sp]) in old_ac.done_fps:
                done.add(k)
        # a new plan (each repeat / cross run, a re-plan) reuses the sub-path indices: no
        # consecutive sweep skips carried over (review 2026-11-02)
        m.sweep_skips = {}
        m.subpaths = subpaths
        m.planned_total += len(subpaths)
        m.cum = [geo.cumulative_lengths(sp) for sp in subpaths]
        m.lengths = [c[-1] for c in m.cum]
        full = result.get('full_path') or [q for sp in subpaths for q in sp]
        self._fx.append(PublishPlan(list(full)))
        self._plan_shown = True
        offs, total = subpath_offsets(subpaths)
        new = AreaCursor(total, geo.plan_fingerprint(subpaths), -1, set(done))
        new.done_fps = {geo.plan_fingerprint([subpaths[k]]) for k in done}
        new.raster = old_ac.raster
        m.resume_sub = None
        first = next((k for k in range(len(subpaths)) if k not in done), None)
        if first is not None and first in prefix:
            m.resume_sub = (first, prefix[first])
            new.resume_index = offs[first] + prefix[first]
        self.cursor.areas[m.area_idx] = new
        # stale indices of the old plan (its skipped / failed pieces are re-covered by the
        # new plan's undone sub-paths)
        m.failed = [f for f in m.failed if f[0] != m.area_idx]
        m.edge_laps = {k: v for k, v in m.edge_laps.items() if k[0] != m.area_idx}
        m.nogo_tries = {}
        self._persist()
        self._log('info', 'no-go re-plan of area %d: %d sub-paths, %.1f m; %d already mowed, '
                          'resume %s' % (m.area_idx, len(subpaths), sum(m.lengths), len(done),
                                         m.resume_sub))
        m.sub_i = 0
        self._next_subpath()

    # ---- pre-pivot sweep check -------------------------------------------
    def _nogo_body(self, poses):
        """Worst body excess (m beyond the edge-mode limit) over ``poses``; None = no check."""
        if not poses:
            return None
        mode = self._hb_body_mode(poses[0])
        if mode is None:
            return None
        worst = None
        for q in poses:
            ev = self._hb_body_eval(q, mode)
            if ev is not None and (worst is None or ev['excess'] > worst):
                worst = ev['excess']
        return worst

    def _nogo_backoff(self, pose, dist):
        return self._safe_backoff(dist, pose=pose)

    def _nogo_pivot_gate(self, site, key, delta):
        """Before a pivot of ``delta`` (site 'cov' or 'route'): None = pivot as planned; a
        number = pivot by that instead (the other way round); True = handled (a maneuver,
        a re-plan, a skip or a re-route was started)."""
        m, p, pose = self.mission, self.p, self.inputs.pose
        if not p.nogo_enabled or not p.nogo_pivot_check or m is None or pose is None or \
                len(pose) < 3:
            return None
        discs = self._nogo_discs()
        if not discs:
            return None
        x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
        plan = nogo.plan_pivot(x, y, yaw, delta, discs, body=self._nogo_body,
                               backoff=self._nogo_backoff, backup_m=float(p.backout_m),
                               shifts=[float(s) for s in (p.nogo_pivot_shifts_m or [])])
        if plan.kind == 'clear':
            return None
        what = 'coverage pivot' if site == 'cov' else 'route corner'
        m.nogo_turns += 1
        if plan.kind == 'other':
            self._log('warn', 'pre-pivot check, %s at (%.2f, %.2f): (a) %s' % (what, x, y, plan.why))
            self._incident('nogo_pivot', 'other direction')
            return plan.steps[0][1]
        n = m.nogo_tries.get(key, 0)
        if plan.kind != 'blocked' and n < int(p.nogo_pivot_max_maneuvers):
            m.nogo_tries[key] = n + 1
            letter = {'backup': '(b)', 'kturn': '(c)', 'shift': '(d)'}[plan.kind]
            self._log('warn', 'pre-pivot check, %s at (%.2f, %.2f): %s %s'
                      % (what, x, y, letter, plan.why))
            self._incident('nogo_pivot', plan.kind)
            m.nogo_seq = {'site': site, 'key': key, 'steps': list(plan.steps[:-1]),
                          'purpose': (self._route_legs or {}).get('purpose', 'route')}
            if self.blade_on and not self._keep_blade_inside('pre-pivot maneuver'):
                self._blade_off('pre-pivot maneuver')
            self._nogo_next_step()
            return True
        why = plan.why if plan.kind == 'blocked' else '%s (maneuvers used up)' % plan.why
        return self._nogo_pivot_blocked(site, key, plan, why, what, (x, y),
                                        pivot=(x, y, yaw, delta))

    def _nogo_pivot_blocked(self, site, key, plan, why, what, xy, pivot=None):
        """(e): coverage - re-plan around a pending / accepted zone in the sweep, else skip
        the leg (blocked_pivot_policy skip); route - a person / animal in the sweep: wait
        for it (the dynamic-obstacle wait), else (and once that wait timed out) re-route
        the transit once per corner. None = pivot as planned (Nav2 judges)."""
        m, p = self.mission, self.p
        if site == 'route' and pivot is not None and self._nogo_living_wait(plan, key, pivot):
            return True
        if site == 'cov':
            zones = self._nogo_replan_zones([h.zone for h in plan.hits if h.zone.state !=
                                             'mark'], require_hit=False)
            if zones and self._nogo_replan_budget():
                self._log('warn', 'pre-pivot check, %s at (%.2f, %.2f): (e) %s: re-plan around '
                                  'it' % (what, xy[0], xy[1], why))
                return self._nogo_replan(zones, 'pivot sweep blocked')
            if str(p.blocked_pivot_policy).strip().lower() == 'skip' and \
                    m.pivot_skips < int(p.blocked_pivot_max_skips):
                self._log('warn', 'pre-pivot check, %s at (%.2f, %.2f): (e) %s: skip the leg'
                          % (what, xy[0], xy[1], why))
                self._nogo_skip_leg('pre-pivot check: ' + why)
                return True
        elif site == 'route' and key not in m.nogo_route_aborts:
            m.nogo_route_aborts.add(key)
            self._log('warn', 'pre-pivot check, %s at (%.2f, %.2f): (e) %s: re-route the '
                              'transit' % (what, xy[0], xy[1], why))
            self._route_legs = None
            self._transit_abort('route corner pivot sweep blocked')
            return True
        self._log('warn', 'pre-pivot check, %s at (%.2f, %.2f): %s: pivot as planned'
                  % (what, xy[0], xy[1], why))
        return None

    def _nogo_living_wait(self, plan, key, pivot):
        """Field 2026-10-28: a dog beside a transit route corner blocked the pivot sweep and
        every retry aborted the transit (47 aborts, sub-paths dropped). A person / animal
        (the dynamic policy, or a living-class zone in the sweep) is waited for instead, as
        the dynamic-obstacle stop does (blade per blade_policy). The wait clears once the
        policy is not dynamic and no living zone is in the sweep; then the transit is
        re-sent. Matching removal: the wait is bounded per corner (dynamic_wait_transit_s,
        summed over re-sends at the same spot); then today's re-route / abort follows.
        True = the wait was started."""
        m = self.mission
        if self.phase != 'TRANSIT' or m is None or m.step != 'transit':
            return False
        x, y, yaw, delta = pivot
        living = nogo.living_sweep_hits(x, y, yaw, delta, plan.hits and
                                        [h.zone for h in plan.hits])
        dyn = self._policy() == 'dynamic'
        if not living and not dyn:
            return False
        w = m.nogo_living_wait
        budget = self._dyn_wait_s('transit')
        if w is None or math.hypot(x - w['xy'][0], y - w['xy'][1]) > 0.5 or \
                self._now - w['since'] > 4.0 * budget:
            w = m.nogo_living_wait = {'xy': (x, y), 'since': self._now}
        if self._now - w['since'] >= budget:
            return False
        cls = (self.inputs.obstacle_class if dyn else None) or (living[0] if living else
                                                                'obstacle')
        self._log('warn', 'pre-pivot check, route corner at (%.2f, %.2f): %s in the sweep: '
                          'waiting for it to move' % (x, y, cls))
        self._incident('nogo_pivot', 'wait for %s' % cls)
        self._dyn_start('transit')
        d = self._dyn
        d['cls'], d['since'] = cls, w['since']
        d['nogo'] = {'pivot': pivot, 'key': key}
        return True

    def _nogo_living_blocks(self, d):
        """Dynamic wait at a route corner: a living-class zone still in the sweep."""
        x, y, yaw, delta = d['nogo']['pivot']
        return bool(nogo.living_sweep_hits(x, y, yaw, delta, self._nogo_discs(marks=False)))

    def _nogo_living_timeout(self, d, why):
        """The corner wait timed out: today's (e) route branch (re-route the transit)."""
        m = self.mission
        m.nogo_route_aborts.add(d['nogo']['key'])
        self._log('warn', '%s at a route corner: re-routing the transit' % why)
        self._route_legs = None
        self._transit_abort('route corner pivot sweep blocked')

    def _nogo_skip_leg(self, why):
        m, p = self.mission, self.p
        m.pivot_skips += 1
        self._pivot_tries = {}
        last = self._seg_last()
        frm = min(m.start_local, last)
        edge = self._edge_lap(m.sub_i)
        end = self._skip_end(frm, float(p.edge_blocked_skip_m) if edge else 0.0)
        self._incident('blocked_pivot', why)
        m.pivot_done, m.pivot_target, m.corner_end = None, None, None
        if edge:
            self._edge_skip(frm, why, end=end)
        else:
            self._skip_piece(frm, end, 'blocked pivot (%s)' % why)

    def _nogo_next_step(self):
        """Run the next pre-pivot maneuver step; none left: back to the pivot site, which
        checks the sweep again from the new pose and pivots."""
        m, p = self.mission, self.p
        s = m.nogo_seq
        if not s['steps']:
            m.nogo_seq = None
            if s['site'] == 'cov':
                m.step, m.pivot_done = None, None
                self._begin_blade()
            else:
                rl = self._route_legs
                if rl is None or self.phase != 'TRANSIT':
                    self._dispatch(force_transit=True)
                    return
                rl['turned'] = None
                self._send_route_leg(s['purpose'])
            return
        kind, val = s['steps'].pop(0)
        speed = max(float(p.slow_maneuver_mps), 0.06)
        m.step = s.get('step', 'nogo_step') if s['site'] == 'cov' else m.step
        if kind == 'spin':
            allowance = abs(val) / max(0.05, float(p.route_cusp_spin_rps)) + 10.0
            self._start_action(ACT_SPIN, {'angle': val, 'time_allowance_s': allowance},
                               allowance + 5.0, purpose='nogo_step')
        elif kind == 'backup':
            self._start_action(ACT_BACKUP, {'distance_m': val, 'speed_mps': speed},
                               val / speed + 15.0, purpose='nogo_step')
        else:
            self._start_action(ACT_DRIVE, {'distance_m': val, 'speed_mps': speed,
                                           'wait_for_rtk': False, 'rtk_timeout_s': 0.0},
                               val / speed + 15.0, purpose='nogo_step')

    def _nogo_action_done(self, a, outcome):
        m = self.mission
        if m is None:
            return
        if a.purpose == 'nogo_detour':
            self._nogo_detour_done(outcome)
            return
        if m.nogo_seq is None:
            return
        if outcome != SUCCEEDED:
            self._log('warn', 'pre-pivot maneuver step %s %s: back to the pivot' % (a.name,
                                                                                   outcome))
            m.nogo_seq['steps'] = []
        self._nogo_next_step()

    # ---- edge-lap detour (B6) ----------------------------------------------
    def _nogo_edge_discs(self):
        """Discs for the edge-lap check: the zones (pending / accepted / candidate) not yet
        a hole of the plan, plus the current on-path static detection (ranged) as a 0.2 m
        disc. Not the 60 s marks: their positions scatter with the range (trunk_x1: a 1.2 m
        detour 2 m from the trunk put the nose out)."""
        m = self.mission
        holes = self._nogo_area_holes(m.area_idx)
        discs = [z for z in self._nogo_discs(marks=False) if not nogo.zone_in_plan(z, holes)]
        i, pose = self.inputs, self.inputs.pose
        if self._policy() == 'static' and i.obstacle_distance is not None and \
                i.obstacle_bearing is not None and pose is not None and len(pose) >= 3:
            a = pose[2] + math.radians(float(i.obstacle_bearing))
            d = float(i.obstacle_distance)
            discs.append(nogo.Zone(pose[0] + d * math.cos(a), pose[1] + d * math.sin(a), 0.2,
                                   label=i.obstacle_class or 'static', state='detection'))
        return discs

    def _nogo_costmap_discs(self, prog, polys):
        """The local costmap's lethal cells (map frame, fresh) inside the allowed region
        (+ 0.05 m), binned to 0.10 m discs: those within nogo_edge_costmap_corridor_m of the
        lap ahead (from local ``prog``, nogo_edge_lookahead_m + 0.5 m) and every cell
        connected to them (the whole obstacle: a branch reaching further in). A costmap-only
        obstacle (stereo depth, no detection class) on an edge lap (field 2026-11-04)."""
        m, p, i = self.mission, self.p, self.inputs
        if not p.nogo_edge_costmap or i.costmap_lethal_stamp is None or \
                self._now - i.costmap_lethal_stamp > float(p.nogo_edge_costmap_max_age_s):
            return []
        pts = i.costmap_lethal if i.costmap_lethal is not None else \
            costmap_lethal_points(i.costmap_grid)
        if not pts:
            return []
        from mower_mission import body_boundary as bb
        sp = m.subpaths[m.sub_i]
        cum = geo.cumulative_lengths(sp)
        k0 = max(0, min(prog, len(sp) - 1))
        reach = float(p.nogo_edge_lookahead_m) + 0.5
        ahead = [tuple(q[:2]) for k, q in enumerate(sp) if k >= k0 and cum[k] - cum[k0] <= reach]
        if len(ahead) < 2:
            return []
        cor = float(p.nogo_edge_costmap_corridor_m)
        cells = {}
        for x, y in pts:
            cells.setdefault((int(math.floor(x / 0.1)), int(math.floor(y / 0.1))), (x, y))
        def xy(c):
            return (c[0] + 0.5) * 0.1, (c[1] + 0.5) * 0.1
        if len(cells) > 2000:            # bound the work (a hedge / fence in view): ahead only
            near = {}
            for c, q in cells.items():
                if min(math.hypot(q[0] - a[0], q[1] - a[1]) for a in ahead[::5] + ahead[-1:]) \
                        <= reach:
                    near[c] = q
            cells = near
        cells = [c for c in cells if bb.point_outside(*xy(c), polys) <= 0.05]
        free = set(cells)
        todo = [c for c in cells if min(_seg_dist(xy(c), a, b) for a, b in
                                        zip(ahead[:-1], ahead[1:])) <= cor]
        free.difference_update(todo)
        keep = []
        while todo:                          # 8-connected cluster of the near cells
            c = todo.pop()
            keep.append(c)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    n = (c[0] + dx, c[1] + dy)
                    if n in free:
                        free.discard(n)
                        todo.append(n)
        if len(keep) > 300:              # merge into 0.2 m discs (edge_detour is O(discs x poses))
            big = {(c[0] // 2, c[1] // 2) for c in keep}
            return [nogo.Zone((b[0] + 0.5) * 0.2, (b[1] + 0.5) * 0.2, 0.142, label='costmap',
                              state='costmap') for b in big]
        return [nogo.Zone(*xy(c), 0.071, label='costmap', state='costmap') for c in keep]

    def _nogo_edge_detour(self, prog, why, proactive=False, costmap=False, backed=False):
        """Bend the edge lap ahead inward around a disc and follow that, blade on, then
        rejoin the lap after it. True = the detour follow was started. ``costmap`` (a
        collision abort): the lethal cells of the local costmap on the lap ahead count as
        discs too (an obstacle no camera classified), with the costmap offset / ramp."""
        m, p = self.mission, self.p
        if not p.nogo_enabled or not p.nogo_edge_detour or m.nogo_detour is not None:
            return False
        sp = m.subpaths[m.sub_i]
        polys = self._region()
        if not polys:
            return False
        import time as _time
        t0 = _time.monotonic()
        discs = self._nogo_edge_discs()
        cm = self._nogo_costmap_discs(prog, polys) if costmap else []
        if not discs and not cm:
            return False
        from mower_mission import body_boundary as bb

        def inside(x, y):
            return bb.point_outside(x, y, polys) <= 0.0
        kw = {}
        if cm:
            kw = {'max_offset_m': max(float(p.nogo_edge_max_offset_m),
                                      float(p.nogo_edge_costmap_max_offset_m)),
                  'ramp_m': float(p.nogo_edge_costmap_ramp_m)}
            why = '%s (%d costmap cells)' % (why, len(cm))
        def bend(extra):
            return nogo.edge_detour(sp, prog, discs + cm, inside,
                                    half_width=float(p.robot_width_m) / 2.0,
                                    clearance=float(p.nogo_edge_clearance_m),
                                    lookahead_m=float(p.nogo_edge_lookahead_m),
                                    **dict({'max_offset_m': float(p.nogo_edge_max_offset_m)},
                                           **dict(kw, **extra)))
        # costmap: the shallowest return that keeps the body inside (a long ramp near a
        # corner can reach out); then the plain ramp. Zones: one try as before.
        per = float(p.nogo_edge_costmap_ramp_per_m) if cm else 0.0
        # costmap: end 0.10 m (boundary_recover_margin_m) inside the lap, merge from there
        end_off = max(0.0, float(p.boundary_recover_margin_m)) if cm else 0.0
        tries = [{'ramp_per_m': r, 'end_offset_m': end_off}
                 for r in (per, per * 2.0 / 3.0) if r > 0.0] + \
            [{'ramp_per_m': 0.0, 'end_offset_m': end_off}]
        res, ex = None, None
        for extra in tries:
            res = bend(extra)
            if res is None:
                break                          # the offset does not fit: no ramp helps
            ex = self._nogo_body(res[0])
            if ex is None or ex <= 0.0:
                break
        dt = _time.monotonic() - t0
        if dt > 0.2:
            self._log('warn', 'edge lap detour geometry took %.2f s (%d discs, %d costmap)'
                      % (dt, len(discs), len(cm)))
        if res is None:
            if cm:
                self._log('warn', 'edge lap: %s: no inward detour fits within %.2f m: no '
                                  'detour' % (why, kw['max_offset_m']))
            return False
        poses, end = res
        if ex is not None and ex > 0.0:
            self._log('warn', 'edge lap: %s: the detour would put the body %.2f m beyond its '
                              'limit: no detour' % (why, ex))
            return False
        key = (m.area_idx, m.sub_i, end)
        if key in m.nogo_detoured:
            return False
        if cm and not backed and self._nogo_detour_backup(prog, cm, why):
            return True
        m.nogo_detoured.add(key)
        if self._action is not None:
            self._fx.append(CancelActions('edge detour'))
            self._action = None
        pose = self.inputs.pose
        m.nogo_detour = {'from': prog, 'end': end, 'why': why,
                         'join': float(p.boundary_recover_margin_m) if cm else 0.0}
        m.nogo_edge_detours += 1
        m.step = 'nogo_detour'
        self._log('warn', 'edge lap: %s: detour inward around it on the lap (poses %d -> %d, '
                          '%.1f m, max offset %.2f m), blade %s, then rejoin'
                  % (why, prog, end, geo.path_length(poses),
                     max(math.hypot(q[0] - sp[min(prog + k, len(sp) - 1)][0],
                                    q[1] - sp[min(prog + k, len(sp) - 1)][1])
                         for k, q in enumerate(poses)),
                     'on' if self.blade_on else 'off'))
        if cm:
            # B6 "tag it on the map": the incident sits on the obstacle (the cells the
            # detour bends round), not on the robot
            used = [z for z in cm if any(math.hypot(z.x - q[0], z.y - q[1]) <= 0.8
                                         for q in poses)] or cm
            ox, oy = (sum(z.x for z in used) / len(used), sum(z.y for z in used) / len(used))
            self._incident('edge_detour', 'costmap obstacle at (%.2f, %.2f): %s'
                           % (ox, oy, why), xy=(ox, oy))
        else:
            self._incident('edge_detour', '%s at (%.2f, %.2f)' % (why, pose[0], pose[1])
                           if pose is not None else why)
        self._start_action(ACT_FOLLOW, {
            'poses': poses, 'controller_id': self.coverage_controller_id(False),
            'goal_checker_id': self.p.follow_goal_checker_id,
            'progress_checker_id': self.p.follow_progress_checker_id},
            self.p.follow_timeout_s, purpose='nogo_detour')
        return True

    def _nogo_detour_backup(self, prog, cm, why):
        """A collision abort stops the robot right at the obstacle: every controller
        trajectory of the detour collides there (sim 2026-11-04: the detour round the branch
        aborted within 1 s, "Optimizer fail to compute path"), and Nav2's BackUp refuses too
        ("Collision Ahead - Exiting DriveOnHeading": its footprint check fails at the start
        pose, whose nose is on the cells). Within nogo_edge_costmap_backup_near_m of the cells
        the mission reverses itself (DriveCmd, the loc retrace's track steering,
        _track_cmd) by up to nogo_edge_costmap_backup_m back along the lap it has just
        driven (clear, and inside: lap 0 follows the line). The detour is then bent from the lap pose
        nearest the robot (_nogo_backup_tick -> _nogo_detour_backup_done). True = started."""
        m, p, pose = self.mission, self.p, self.inputs.pose
        want = float(p.nogo_edge_costmap_backup_m)
        if want <= 0.0 or pose is None or len(pose) < 3:
            return False
        near = min(math.hypot(z.x - pose[0], z.y - pose[1]) for z in cm)
        if near > float(p.nogo_edge_costmap_backup_near_m):
            return False
        # back down the lap just driven (not straight: lap 0 curves with the line, a
        # straight reverse took base_link 0.14 m outside, sim 2026-11-05 branch-scen8)
        sp = m.subpaths[m.sub_i]
        path, run, k = [], 0.0, max(0, min(prog, len(sp) - 1))
        while k > 0 and run < want:
            run += math.hypot(sp[k][0] - sp[k - 1][0], sp[k][1] - sp[k - 1][1])
            k -= 1
            path.append((float(sp[k][0]), float(sp[k][1])))
        back = min(run, want)
        if back < 0.1 or not path:
            return False
        if self._action is not None:
            self._fx.append(CancelActions('edge detour back-up'))
            self._action = None
        speed = max(float(p.slow_maneuver_mps), 0.06)
        m.nogo_detour = {'backup': True, 'from': prog, 'why': why, 'dist': back, 'v': speed,
                         'start': (float(pose[0]), float(pose[1])), 'path': path, 'i': 0,
                         'until': self._now + back / speed + 5.0}
        m.step = 'nogo_detour'
        self._log('warn', 'edge lap: %s: obstacle %.2f m from the robot: back up %.2f m, then '
                          'detour' % (why, near, back))
        if self.blade_on and not self._keep_blade_inside('edge detour back-up'):
            self._blade_off('edge detour back-up')
        self._fx.append(DriveCmd(*self._track_cmd(pose, path[0], speed)))
        return True

    def _nogo_backup_tick(self):
        """The straight reverse of _nogo_detour_backup, one tick. True while it runs."""
        m = self.mission
        d = m.nogo_detour if m is not None else None
        if d is None or not d.get('backup') or m.step != 'nogo_detour':
            return False
        pose = self.inputs.pose
        driven = 0.0 if pose is None else math.hypot(pose[0] - d['start'][0],
                                                    pose[1] - d['start'][1])
        path = d['path']
        if pose is not None:
            while d['i'] < len(path) - 1 and math.hypot(path[d['i']][0] - pose[0],
                                                        path[d['i']][1] - pose[1]) < 0.3:
                d['i'] += 1
        done = pose is not None and math.hypot(path[-1][0] - pose[0],
                                               path[-1][1] - pose[1]) < 0.05
        if driven >= d['dist'] - 0.02 or done or self._now >= d['until'] or pose is None:
            self._fx.append(DriveCmd(0.0, 0.0))
            self._nogo_detour_backup_done(SUCCEEDED if driven >= 0.5 * d['dist'] else
                                          'stopped after %.2f m' % driven)
            return True
        self._fx.append(DriveCmd(*self._track_cmd(pose, path[d['i']], d['v'])))
        return True

    def _nogo_detour_backup_done(self, outcome):
        """The back-up before a costmap detour ended (any outcome): bend the detour from the
        lap pose nearest the robot now (searched back from the abort pose), else skip."""
        m = self.mission
        d = m.nogo_detour if m is not None else None
        if d is None or not d.get('backup') or self.phase not in ('MOWING', 'TRANSIT'):
            return
        m.nogo_detour = None
        m.step = None
        frm, why = int(d['from']), d['why']
        sp, pose = m.subpaths[m.sub_i], self.inputs.pose
        start = frm
        if pose is not None:
            lo = max(0, frm - 200)
            start = min(range(lo, frm + 1), key=lambda k: math.hypot(sp[k][0] - pose[0],
                                                                    sp[k][1] - pose[1]))
        if outcome != SUCCEEDED:
            self._log('warn', 'edge detour back-up %s: detour from here' % outcome)
        m.progress_local = start
        if self._nogo_edge_detour(start, why, costmap=True, backed=True):
            return
        self._edge_skip(frm, why)

    def _nogo_detour_done(self, outcome):
        m = self.mission
        d, m.nogo_detour = m.nogo_detour, None
        if d is None or self.phase not in ('MOWING', 'TRANSIT'):
            return
        if outcome == SUCCEEDED:
            m.start_local = min(d['end'], self._seg_last())
            m.step = None
            off = float(d.get('join') or 0.0)
            sgn = self._hb_inward_side(m.subpaths[m.sub_i], m.start_local, off) if off > 0.0 \
                else 0
            if sgn:
                # the detour ended ``off`` inside the lap: merge tangentially (the boundary
                # rejoin's join), no rejoin pivot (sim 2026-11-05: a rejoin straight onto the
                # line swung the rear 0.35 m out)
                m.join = (m.sub_i, m.start_local, sgn, off)
                m.pivot_done = (m.area_idx, m.sub_i, m.start_local)
            self._log('info', 'edge detour done: rejoining the lap at pose %d%s'
                      % (d['end'], ', merging from %.2f m inside' % off if sgn else ''))
            self._dispatch()
            return
        self._log('warn', 'edge detour %s: skipping the stretch' % outcome)
        m.step = None
        self._edge_skip(d['from'], 'edge detour %s (%s)' % (outcome, d['why']),
                        end=min(d['end'], self._seg_last()))
