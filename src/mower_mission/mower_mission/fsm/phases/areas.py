# SPDX-License-Identifier: GPL-3.0-or-later
"""MissionFSM mixin: Area progression, per-area mowing settings, planning.

Moved verbatim from mission_fsm.py (2026-10-23 code review stage 1.6)."""

from mower_mission.fsm.common import (ACT_PLAN, AREA_SETTINGS_DEFAULTS, AreaCursor,
    BLADE_POLICIES, CONDITIONED_KEY, CUTTER_HEIGHT_MAX_MM, CUTTER_HEIGHT_MIN_MM, EDGE_MODES,
    MOTION_PHASES, OBSTACLE_DETECTION_LEVELS, PARAM_NODE_CONTROLLER, PARAM_NODE_COVERAGE,
    PARAM_NODE_DET_RANGE, PARAM_NODE_OBSTACLE_GUARD, PATH_MODES, PIVOT_SUB,
    PublishAreaSettings, PublishPlan, ROUTE_ORDERS, SRV_CUTTER_HEIGHT, SRV_GET_AREA_SETTINGS,
    SRV_SET_PARAMS, STEREO_STALE_SUB, SUCCEEDED, TURN_TYPES, UNAVAILABLE, absolute_to_local,
    coverage_route_params, geo, json, math, subpath_offsets)  # noqa: F401


class AreasMixin:
    """Area progression, per-area mowing settings, planning."""

    # ==================================================================
    # area / sub-path progression (FollowStrip port)
    # ==================================================================
    def _next_area(self):
        m = self.mission
        if self.blade_on:
            self._blade_off('area finished')
        while m.queue and m.queue[0] in self.cursor.completed_areas:
            m.queue.pop(0)
        if not m.queue:
            if m.edge_skipped_m > 0.0:
                self._log('warn', 'edge blocked: %.1f m of edge laps skipped this mission'
                          % m.edge_skipped_m)
                self._incident('edge_blocked_total', '%.1f m' % m.edge_skipped_m)
            if m.failed or m.blocked:
                self._mission_incomplete()
            else:
                self._mission_complete()     # status still shows the last area's counts
            return
        m.area_idx = None
        m.subpaths, m.lengths, m.cum = [], [], []
        idx = m.queue.pop(0)
        area = m.areas[idx]
        m.area_idx = idx
        m.sub_i, m.start_local, m.skipped, m.step = 0, 0, 0, None
        m.requeue, m.stretch, m.stretch_end, m.detour = None, None, None, None
        m.requeue_subs, m.retry_entry = None, None
        self.cursor.current_area = idx
        m.settings, m.runs = None, []
        m.run_i = int(self.cursor.area_runs.get(idx, 0))
        if self.p.use_area_settings:
            self._go('PLANNING', 'area %d %s: loading settings' % (idx, area.get('name', '')))
            self._call(SRV_GET_AREA_SETTINGS, {'index': idx}, purpose='area_settings')
        else:
            self._on_area_settings(False, {})

    # ------------------------------------------------------------------
    # per-area mowing settings
    # ------------------------------------------------------------------
    @staticmethod
    def height_mm(mm):
        """cutter_height_mm -> MCU height position: absolute deck height in mm,
        clamped to the vendor range 30-90 (mower_bt_nodes CutterControl::onRunning)."""
        return int(round(max(float(CUTTER_HEIGHT_MIN_MM),
                             min(float(CUTTER_HEIGHT_MAX_MM), float(mm)))))

    def _merge_settings(self, ok, resp, area_idx=None):
        st = dict(AREA_SETTINGS_DEFAULTS)
        src = 'built-in defaults'
        got = {}
        if ok and resp.get('success'):
            try:
                got = json.loads(resp.get('settings_json') or '{}')
                if isinstance(got, dict):
                    st.update({k: v for k, v in got.items() if k in st and v is not None})
                    src = 'map_server'
            except ValueError:
                self._log('warn', 'area settings: bad JSON from get_area_settings, using defaults')
        elif self.p.use_area_settings:
            self._log('warn', 'area %d: get_area_settings unavailable: using built-in defaults'
                      % (self.mission.area_idx if area_idx is None else area_idx))
        # Slope-aware angle (terrain memory): the map server derives
        # slope_mow_angle_deg for areas with slope_mode != off; used only while
        # the area's own angle is auto, ahead of the global parameter.
        if float(st['mow_angle_deg']) < 0.0 and st.get('slope_mode', 'off') != 'off':
            sa = got.get('slope_mow_angle_deg') if src == 'map_server' else None
            if isinstance(sa, (int, float)) and not isinstance(sa, bool) and sa >= 0.0:
                st['mow_angle_deg'] = math.fmod(float(sa), 180.0)
                self._log('info', 'slope_mode %s: swath angle %.1f deg (%s)'
                          % (st['slope_mode'], st['mow_angle_deg'],
                             got.get('slope_angle_why', '')))
            else:
                self._log('info', 'slope_mode %s: planner angle kept (%s)'
                          % (st['slope_mode'], got.get('slope_angle_why', 'no slope angle')
                             if src == 'map_server' else 'map_server unavailable'))
        # Global mow_angle_deg parameter: fallback for areas left on auto.
        if float(st['mow_angle_deg']) < 0.0 and self.p.mow_angle_deg >= 0.0:
            st['mow_angle_deg'] = float(self.p.mow_angle_deg)
        if st['route_order'] not in ROUTE_ORDERS:
            self._log('warn', 'unknown route_order %r: boustrophedon' % st['route_order'])
            st['route_order'] = 'boustrophedon'
        if st['turn_type'] not in TURN_TYPES:
            self._log('warn', 'unknown turn_type %r: auto' % st['turn_type'])
            st['turn_type'] = 'auto'
        if st.get('edge_mode') not in EDGE_MODES:
            self._log('warn', 'unknown edge_mode %r: soft' % st.get('edge_mode'))
            st['edge_mode'] = 'soft'
        st['edge_layout'] = st['edge_mode']
        st['route_spiral_size'] = max(2, int(st['route_spiral_size']))
        st['min_turn_radius_m'] = max(0.0, float(st['min_turn_radius_m']))
        if st['path_mode'] not in PATH_MODES:
            self._log('warn', 'unknown path_mode %r: zigzag' % st['path_mode'])
            st['path_mode'] = 'zigzag'
        st['repeat'] = max(1, int(st['repeat']))
        if st['obstacle_detection'] not in OBSTACLE_DETECTION_LEVELS:
            self._log('warn', 'unknown obstacle_detection %r: standard' % st['obstacle_detection'])
            st['obstacle_detection'] = 'standard'
        if st.get('blade_policy') not in BLADE_POLICIES:
            self._log('warn', 'unknown blade_policy %r: continuous' % st.get('blade_policy'))
            st['blade_policy'] = 'continuous'
        st['perimeter_laps'] = max(0, int(st['perimeter_laps']))
        try:
            st['edge_laps'] = max(0, min(10, int(st.get('edge_laps',
                                                         AREA_SETTINGS_DEFAULTS['edge_laps']))))
        except (TypeError, ValueError):
            self._log('warn', 'bad edge_laps %r: %d' % (st.get('edge_laps'),
                                                        AREA_SETTINGS_DEFAULTS['edge_laps']))
            st['edge_laps'] = AREA_SETTINGS_DEFAULTS['edge_laps']
        return st, src

    def _swath_spacing(self, st):
        """Path spacing (m) = the area's swath_width_m, used as-is (cut_width_m plays no part)."""
        try:
            w = float(st.get('swath_width_m') or 0.0)
        except (TypeError, ValueError):
            w = 0.0
        return w if w > 0.0 else AREA_SETTINGS_DEFAULTS['swath_width_m']

    def _area_name(self):
        m = self.mission
        return str(m.areas.get(m.area_idx, {}).get('name', ''))

    def _build_runs(self, st, area_name=None):
        """One entry per PlanCoverage goal of this area, in mowing order.

        cross: two plans per repeat, the second at +90 deg. alternate: the
        angle turns by alternate_angle_offset_deg on every run, continuing
        across sessions (alternate_counts). On an AUTO angle a +90 deg turn is
        the goal's ``perpendicular`` flag (the bridge learns the auto angle);
        any other turn is taken from 0 deg (the map x axis).
        """
        base = float(st['mow_angle_deg'])
        mode = st['path_mode']
        n = int(st['repeat'])

        def run(rot):
            rot = math.fmod(rot, 180.0)
            if base >= 0.0:
                return {'angle': math.fmod(base + rot, 180.0), 'perpendicular': False}
            if abs(rot) < 1e-6:
                return {'angle': -1.0, 'perpendicular': False}
            if abs(rot - 90.0) < 1e-6:
                return {'angle': -1.0, 'perpendicular': True}
            return {'angle': rot, 'perpendicular': False}

        if mode == 'cross':
            return [r for _ in range(n) for r in (run(0.0), run(90.0))]
        if mode == 'alternate':
            off = float(st['alternate_angle_offset_deg'])
            done = int(self.alternate_counts.get(
                self._area_name() if area_name is None else area_name, 0))
            return [run((done + k) * off) for k in range(n)]
        return [run(0.0) for _ in range(n)]

    def display_sub_state(self):
        """sub_state as reported: a fresh stale-stereo costmap warning wins while driving
        (Nav2 aborts every FollowPath then; the generic retry text hides the cause)."""
        i = self.inputs
        if (self.phase in ('TRANSIT', 'MOWING') and i.stereo_stale_stamp is not None
                and self._now - i.stereo_stale_stamp <= self.p.stereo_stale_window_s
                and not self.sub_state.startswith('waiting for ')):
            return STEREO_STALE_SUB % int(round(i.stereo_stale_s or 0.0))
        if (self._tilt_capped and self._tob is None and self._tilt_band() == 'caution'
                and self.phase in MOTION_PHASES):
            return 'slope caution (%s): slowed; %s' % (self._tilt_text(), self.sub_state)
        if self.pivoting_slowly() and not self.sub_state.startswith('waiting for '):
            return PIVOT_SUB + (': ' + self.sub_state if self.sub_state else '')
        return self.sub_state

    def obstacle_level(self):
        m = self.mission
        lvl = (m.settings or {}).get('obstacle_detection') if m is not None else None
        return lvl if lvl in OBSTACLE_DETECTION_LEVELS else 'standard'

    def _push_obstacle_detection(self, level):
        if not self.p.set_obstacle_detection:
            return
        self._call(SRV_SET_PARAMS, {'node': PARAM_NODE_OBSTACLE_GUARD,
                                    'params': {'obstacle_detection': str(level)}}, track=False)
        self._call(SRV_SET_PARAMS, {'node': PARAM_NODE_DET_RANGE,
                                    'params': {'publish_obstacle_points': level != 'none'}},
                   track=False)

    def _on_area_settings(self, ok, resp):
        m = self.mission
        st, src = self._merge_settings(ok, resp)
        if m.height_override_mm is not None:
            # live height change during this mow wins over the saved area height
            st = dict(st, cutter_height_mm=m.height_override_mm)
            src = src + ', live height'
        m.settings = st
        self._push_obstacle_detection(st['obstacle_detection'])
        m.runs = self._build_runs(st)
        if m.run_i >= len(m.runs):
            m.run_i = 0
        hmm = self.height_mm(st['cutter_height_mm'])
        speed = min(float(st['cut_speed_mps']), float(self.p.cut_speed_max_mps))
        if self._tilt_capped:               # area starts on a slope: keep the caution cap
            speed = min(speed, float(self.p.tilt_caution_speed_mps))
        self._log('info', 'area %d %s: settings (%s) %s; blade height %d mm, '
                          'cut speed %.2f m/s, %d run(s)'
                  % (m.area_idx, self._area_name(), src, json.dumps(st, sort_keys=True),
                     hmm, speed, len(m.runs)))
        if self.p.set_cutter_height:
            # Blade is off here (PLANNING); height only, before any mowing.
            self._call(SRV_CUTTER_HEIGHT, {'height_mm': hmm}, track=False)
        if self.p.set_cut_speed:
            self._call(SRV_SET_PARAMS, {'node': PARAM_NODE_CONTROLLER,
                                        'params': self.coverage_speed_params(speed)},
                       track=False)
        self._plan_run()

    def active_settings(self):
        """JSON-able active settings (latched ~/active_area_settings)."""
        m = self.mission
        if m is None or m.settings is None or not m.runs:
            return {}
        run = m.runs[m.run_i]
        out = dict(m.settings)
        out.update({
            'area_index': m.area_idx, 'area_name': self._area_name(),
            'run': m.run_i + 1, 'runs': len(m.runs),
            'run_mow_angle_deg': run['angle'], 'run_perpendicular': run['perpendicular'],
            'operation_width_m': round(self._swath_spacing(m.settings), 4)})
        return out

    def _plan_run(self):
        m = self.mission
        st, run = m.settings, m.runs[m.run_i]
        self._fx.append(PublishAreaSettings(self.active_settings()))
        self._go('PLANNING', 'area %d %s [%s, run %d/%d]' % (
            m.area_idx, self._area_name(), st['path_mode'], m.run_i + 1, len(m.runs)))
        self._call(SRV_SET_PARAMS, {'node': PARAM_NODE_COVERAGE, 'params': {
            'operation_width': self._swath_spacing(st),
            'headland_rings': int(st['perimeter_laps']),
            'path_mode': str(st['path_mode']),
            'mow_angle_deg': float(run['angle']),
            'edge_first': bool(st['edge_first']),
            **coverage_route_params(st)}}, purpose='coverage_params')

    def _start_plan(self):
        m = self.mission
        area, run = m.areas[m.area_idx], m.runs[m.run_i]
        self._start_action(ACT_PLAN, {
            'outer_boundary': self._plan_outline(area.get('outer', [])),
            'obstacles': [list(o) for o in area.get('obstacles', [])],
            'mow_angle_deg': float(run['angle']),
            'perpendicular': bool(run['perpendicular'])}, self.p.plan_timeout_s)

    def _on_plan(self, outcome, result):
        m = self.mission
        if outcome == UNAVAILABLE:
            self._mission_failed('/plan_coverage action server unavailable')
            return
        subpaths = self._plan_subpaths(outcome, result)
        if not subpaths:
            why = result.get('message') or outcome
            self._log('warn', 'area %d: coverage planning failed (%s): skipping it'
                      % (m.area_idx, why))
            m.failed.append((m.area_idx, None, 'area %d planning failed: %s' % (m.area_idx, why),
                             -1))
            self._go('AREA_UNREACHABLE', 'area %d: planning failed: %s' % (m.area_idx, why))
            self._next_area()
            return
        m.cond, m.cond_errors = {}, {}
        if self.p.path_conditioning:
            subpaths = self._condition_subpaths(subpaths, result.get(CONDITIONED_KEY))
        m.subpaths = subpaths
        m.planned_total += len(subpaths)
        m.cum = [geo.cumulative_lengths(sp) for sp in subpaths]
        m.lengths = [c[-1] for c in m.cum]
        full = result.get('full_path') or [p for sp in subpaths for p in sp]
        self._fx.append(PublishPlan(list(full)))
        self._plan_shown = True
        self._preview_shown = False
        fp = geo.plan_fingerprint(subpaths)
        _offs, total = subpath_offsets(subpaths)
        ac = self.cursor.areas.get(m.area_idx)
        m.resume_sub = None
        if ac is not None and ac.fingerprint == fp and ac.pose_count == total:
            if ac.resume_index >= 0:
                m.resume_sub = absolute_to_local(subpaths, ac.resume_index)
            self._log('info', 'area %d: resuming (%d/%d sub-paths done, cursor %s)' % (
                m.area_idx, len(ac.completed), len(subpaths), m.resume_sub))
        else:
            new = AreaCursor(total, fp, -1, set())
            kept = 0
            if ac is not None and ac.done_fps:
                for k, sp in enumerate(subpaths):
                    f = geo.plan_fingerprint([sp])
                    if f in ac.done_fps:
                        new.completed.add(k)
                        new.done_fps.add(f)
                        kept += 1
            if ac is not None and (ac.completed or ac.resume_index >= 0):
                self._log('warn', 'area %d: plan geometry changed, stale resume cursor '
                                  'discarded%s' % (m.area_idx, '' if not kept else
                                                   '; kept %d done sub-path(s) whose geometry '
                                                   'is unchanged' % kept))
            self.cursor.areas[m.area_idx] = new
        self._persist()
        self._log('info', 'area %d: %d drivable sub-paths, %.1f m' % (
            m.area_idx, len(subpaths), sum(m.lengths)))
        m.sub_i = 0
        self._next_subpath()

    @staticmethod
    def _plan_subpaths(outcome, result):
        subpaths = []
        if outcome == SUCCEEDED and result.get('success'):
            subpaths = [list(sp) for sp in result.get('drivable_subpaths', []) if len(sp) >= 2]
            if not subpaths:
                subpaths = [list(sp) for sp in result.get('segments', []) if len(sp) >= 2]
            if not subpaths and len(result.get('full_path', [])) >= 2:
                subpaths = [list(result['full_path'])]
        return subpaths
