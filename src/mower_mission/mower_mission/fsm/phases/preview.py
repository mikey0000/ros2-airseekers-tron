# SPDX-License-Identifier: GPL-3.0-or-later
"""MissionFSM mixin: Plan preview (~/preview_plan): PLANNING without a mission.

Moved verbatim from mission_fsm.py (2026-10-23 code review stage 1.6)."""

from mower_mission.fsm.common import (ACT_PLAN, CancelActions, PARAM_NODE_COVERAGE,
    PREVIEW_ALL, PREVIEW_CLEAR, PREVIEW_PHASES, PublishPlan, PublishPreviewSummary,
    SEGMENT_RING, SEGMENT_SWATH, SRV_GET_AREA, SRV_GET_AREA_SETTINGS, SRV_SET_PARAMS,
    SUCCEEDED, coverage_route_params, geo)  # noqa: F401


class PreviewMixin:
    """Plan preview (~/preview_plan): PLANNING without a mission."""

    # ==================================================================
    # plan preview (~/preview_plan): PLANNING without a mission
    # ==================================================================
    def _preview_refusal(self):
        if self._emergency or self.phase in ('EMERGENCY', 'BOUNDARY_EMERGENCY_STOP'):
            return 'emergency active'
        if self.mission is not None:
            return 'mowing session running'
        if self.phase not in PREVIEW_PHASES:
            return 'not idle (%s)' % self.phase
        if self._enum is not None:
            return 'busy (loading areas)'
        if (self._action is not None and self._action.purpose != 'preview') or \
                (self._service is not None and not str(self._service.purpose).startswith(
                    'preview_')):
            return 'busy'
        return ''

    def _preview_request(self, area):
        why = self._preview_refusal()
        if why:
            return False, 'refused: %s' % why
        if self._preview is not None:
            self._preview_drop('superseded by a new preview')
        if area == PREVIEW_CLEAR:
            self._preview_drop('cleared', clear=True)
            return True, 'preview cleared'
        if area != PREVIEW_ALL and not 0 <= area < int(self.p.max_areas):
            return False, 'refused: bad area index %d' % area
        self._preview_id += 1
        self._preview = {'id': self._preview_id, 'area': -1 if area == PREVIEW_ALL else area,
                         'index': 0, 'areas': {}, 'queue': [], 'cur': None, 'results': []}
        self._fx.append(PublishPreviewSummary({
            'id': self._preview_id, 'status': 'planning', 'area': self._preview['area']}))
        self._call(SRV_GET_AREA, {'index': 0}, purpose='preview_enum')
        return True, 'planning preview of %s' % (
            'all areas' if area == PREVIEW_ALL else 'area %d' % area)

    def _preview_drop(self, why, clear=False):
        """Forget a running preview (its pending calls become stale) and, when
        ``clear`` or a preview is drawn, clear /coverage/full_plan + the summary."""
        pv = self._preview
        if pv is not None:
            self._preview = None
            if self._action is not None and self._action.purpose == 'preview':
                self._action = None
                self._fx.append(CancelActions('preview %s' % why))
            if self._service is not None and str(self._service.purpose).startswith('preview_'):
                self._service = None
        if pv is not None or self._preview_shown or clear:
            if self._preview_shown or clear:
                self._fx.append(PublishPlan([]))
            self._preview_shown = False
            self._fx.append(PublishPreviewSummary({
                'id': self._preview_id, 'status': 'cleared', 'message': why}))

    def _preview_on_service(self, purpose, ok, resp):
        pv = self._preview
        if pv is None:
            return
        if purpose == 'preview_enum':
            if ok and resp.get('success'):
                a = resp.get('area', {})
                if not a.get('is_navigation_area', False):
                    pv['areas'][pv['index']] = a
                pv['index'] += 1
                if pv['index'] < self.p.max_areas:
                    self._call(SRV_GET_AREA, {'index': pv['index']}, purpose='preview_enum')
                    return
            if not ok and pv['index'] == 0:
                self._preview_finish('failed', '/map_server_node/get_mowing_area unavailable')
                return
            if pv['area'] >= 0:
                if pv['area'] not in pv['areas']:
                    self._preview_finish('failed', 'area %d does not exist or is a navigation '
                                                   'area' % pv['area'])
                    return
                pv['queue'] = [pv['area']]
            else:
                pv['queue'] = sorted(pv['areas'])
            if not pv['queue']:
                self._preview_finish('failed', 'no mowing areas defined')
                return
            self._preview_next()
        elif purpose == 'preview_settings':
            cur = pv['cur']
            st, src = self._merge_settings(ok, resp, area_idx=cur['index'])
            cur['settings'], cur['source'] = st, src
            cur['runs'] = self._build_runs(st, area_name=cur['name'])
            cur['preview_runs'] = self._preview_runs(cur['runs'])
            cur['run_k'], cur['acc'] = 0, []
            self._preview_params(cur)
        elif purpose == 'preview_params':
            cur = pv['cur']
            if not ok:
                self._log('warn', 'preview area %d: /coverage_server set_parameters failed (%s): '
                                  'planning with its current parameters'
                          % (cur['index'], resp.get('message') or 'unavailable'))
            area, run = pv['areas'][cur['index']], cur['preview_runs'][cur['run_k']]
            self._start_action(ACT_PLAN, {
                'outer_boundary': self._plan_outline(area.get('outer', [])),
                'obstacles': [list(o) for o in area.get('obstacles', [])],
                'mow_angle_deg': float(run['angle']),
                'perpendicular': bool(run['perpendicular'])}, self.p.plan_timeout_s,
                purpose='preview')

    @staticmethod
    def _preview_runs(runs):
        """The distinct row directions of an area's runs, in mowing order: a preview draws
        every direction one session mows (cross = both row sets, 90 deg apart), not repeats."""
        out = []
        for r in runs:
            if not any(abs(float(o['angle']) - float(r['angle'])) < 1e-6
                       and bool(o['perpendicular']) == bool(r['perpendicular']) for o in out):
                out.append(r)
        return out or list(runs[:1])

    def _preview_params(self, cur):
        st, run = cur['settings'], cur['preview_runs'][cur['run_k']]
        self._call(SRV_SET_PARAMS, {'node': PARAM_NODE_COVERAGE, 'params': {
            'operation_width': self._swath_spacing(st),
            'headland_rings': int(st['perimeter_laps']),
            'path_mode': str(st['path_mode']),
            'mow_angle_deg': float(run['angle']),
            'edge_first': bool(st['edge_first']),
            **coverage_route_params(st)}}, purpose='preview_params')

    def _preview_next(self):
        pv = self._preview
        if not pv['queue']:
            self._preview_finish('ok', '')
            return
        idx = pv['queue'].pop(0)
        pv['cur'] = {'index': idx, 'name': str(pv['areas'][idx].get('name', ''))}
        if self.p.use_area_settings:
            self._call(SRV_GET_AREA_SETTINGS, {'index': idx}, purpose='preview_settings')
        else:
            self._preview_on_service('preview_settings', False, {})

    def _preview_on_plan(self, outcome, result):
        """One run of the current area planned. Every distinct direction (see _preview_runs)
        is planned in turn and the area's results are concatenated."""
        pv = self._preview
        if pv is None:
            return
        cur = pv['cur']
        area = pv['areas'][cur['index']]
        st = cur['settings']
        run0 = cur['preview_runs'][0]
        subpaths = []
        if outcome == SUCCEEDED and result.get('success'):
            subpaths = [list(sp) for sp in result.get('drivable_subpaths', []) if len(sp) >= 2]
            if not subpaths:
                subpaths = [list(sp) for sp in result.get('segments', []) if len(sp) >= 2]
            if not subpaths and len(result.get('full_path', [])) >= 2:
                subpaths = [list(result['full_path'])]
        info = {'index': cur['index'], 'name': cur['name'], 'path_mode': st['path_mode'],
                'headland_rings': int(st['perimeter_laps']), 'edge_first': bool(st['edge_first']),
                'mow_angle_deg': float(run0['angle']),
                'perpendicular': bool(run0['perpendicular']),
                'runs': len(cur['runs']), 'settings_source': cur['source'],
                'preview_runs': len(cur['preview_runs']),
                'run_angles_deg': [float(r['angle']) for r in cur['preview_runs']],
                'run_perpendicular': [bool(r['perpendicular']) for r in cur['preview_runs']],
                'operation_width_m': round(self._swath_spacing(st), 4),
                'swath_width_m': round(self._swath_spacing(st), 4),
                'edge_margin_m': round(max(0.0, float(st['edge_margin_m'])), 4)}
        if not subpaths:
            msg = str(result.get('message') or outcome)
            if len(cur['preview_runs']) > 1:
                msg = 'run %d/%d: %s' % (cur['run_k'] + 1, len(cur['preview_runs']), msg)
            info['error'] = msg
            info.update({'rings': 0, 'swaths': 0, 'length_m': 0.0, 'sub_paths': 0,
                         'inset_m': None})
            pv['results'].append((info, [], [], []))
            self._preview_next()
            return
        segs = [list(sp) for sp in result.get('segments', []) if len(sp) >= 2]
        types = list(result.get('segment_types', []))
        if len(types) != len(segs):
            types = []
        rings = int(result.get('ring_count', types.count(SEGMENT_RING) if types else 0))
        swaths = int(result.get('swath_count', types.count(SEGMENT_SWATH) if types else 0))
        length = float(result.get('total_distance') or 0.0) or \
            sum(geo.path_length(sp) for sp in subpaths)
        full = result.get('full_path') or [p for sp in subpaths for p in sp]
        cur['acc'].append({'rings': rings, 'swaths': swaths, 'length': length,
                           'full': list(full), 'typed': list(zip(types, segs)),
                           'subpaths': subpaths})
        cur['run_k'] += 1
        if cur['run_k'] < len(cur['preview_runs']):
            self._preview_params(cur)          # next direction (cross: the 90 deg row set)
            return
        acc = cur['acc']
        subpaths = [sp for a in acc for sp in a['subpaths']]
        typed = [ts for a in acc for ts in a['typed']]
        outer = list(area.get('outer', []))
        probe = [p for t, s in typed if t == SEGMENT_RING for p in s] or \
            [p for sp in subpaths for p in sp]
        step = max(1, len(probe) // 400)
        inset = min((geo.distance_to_ring(p, outer) for p in probe[::step]), default=None) \
            if len(outer) >= 3 else None
        info.update({'rings': sum(a['rings'] for a in acc),
                     'swaths': sum(a['swaths'] for a in acc),
                     'length_m': round(sum(a['length'] for a in acc), 2),
                     'sub_paths': len(subpaths),
                     'inset_m': None if inset is None else round(inset, 3)})
        full = [p for a in acc for p in a['full']]
        # typed segments of a run with no segment_types are drawn from its sub-paths
        typed = [ts for a in acc for ts in (a['typed'] or
                 [(SEGMENT_SWATH, sp) for sp in a['subpaths']])]
        pv['results'].append((info, full, typed, subpaths))
        self._preview_next()

    def _preview_finish(self, status, message):
        pv, self._preview = self._preview, None
        if self._action is not None and self._action.purpose == 'preview':
            self._action = None
        if self._service is not None and str(self._service.purpose).startswith('preview_'):
            self._service = None
        results = pv['results'] if pv else []
        infos = [r[0] for r in results]
        planned = [r for r in results if r[1]]
        if status == 'ok' and not planned:
            status = 'failed'
            message = '; '.join('area %d: %s' % (i['index'], i.get('error', '?'))
                                for i in infos) or 'nothing planned'

        def pts(seq):
            return [[round(float(p[0]), 2), round(float(p[1]), 2)] for p in seq]

        full, segments, transits, prev_end = [], [], [], None
        for _info, path, typed, subpaths in planned:
            full.extend(path)
            for t, s in typed:
                segments.append({'type': 'ring' if t == SEGMENT_RING else 'swath',
                                 'points': pts(s)})
            for sp in subpaths:
                if prev_end is not None and geo.dist(prev_end, sp[0]) > 1e-3:
                    transits.append(pts([prev_end, sp[0]]))
                prev_end = sp[-1]
            if not typed:
                segments.extend({'type': 'swath', 'points': pts(sp)} for sp in subpaths)
        insets = [i['inset_m'] for i in infos if i.get('inset_m') is not None]
        summary = {
            'id': pv['id'] if pv else self._preview_id, 'status': status,
            'area': pv['area'] if pv else -1, 'message': message,
            'areas': infos,
            'rings': sum(i['rings'] for i in infos), 'swaths': sum(i['swaths'] for i in infos),
            'length_m': round(sum(i['length_m'] for i in infos), 2),
            'sub_paths': sum(i['sub_paths'] for i in infos),
            'inset_m': min(insets) if insets else None,
            'segments': segments, 'transits': transits}
        if planned:
            self._fx.append(PublishPlan(list(full)))
            self._preview_shown = True
            self._plan_shown = False    # the mission's _go_idle must not clear the preview
        self._fx.append(PublishPreviewSummary(summary))
        self._log('info' if status == 'ok' else 'warn',
                  'plan preview %s: %d area(s), %d rings, %d swaths, %.1f m%s' % (
                      status, len(infos), summary['rings'], summary['swaths'],
                      summary['length_m'], (' (%s)' % message) if message else ''))
