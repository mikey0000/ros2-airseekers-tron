# SPDX-License-Identifier: GPL-3.0-or-later
"""MissionFSM mixin: Recording (RecordArea port).

Moved verbatim from mission_fsm.py (2026-10-23 code review stage 1.6)."""

from mower_mission.fsm.common import (PublishTrajectory, SRV_ADD_AREA, SRV_GET_AREA,
    SRV_PROMOTE_OBSTACLE, STATE_AUTONOMOUS, SaveRecordingFallback, geo)  # noqa: F401


class RecordingMixin:
    """Recording (RecordArea port)."""

    # ==================================================================
    # recording (RecordArea port)
    # ==================================================================
    def _record_start(self, kind='area'):
        if self.state == STATE_AUTONOMOUS:
            self._log('warn', 'RECORD_%s refused: autonomous run active (STOP first)'
                      % kind.upper())
            return False
        if self.phase == 'RECORDING':
            return True
        self._leave_current_mode('record %s' % kind)
        self._blade_off('recording')
        self._track = []
        self._record_poly = None
        self._record_line = None
        self._record_kind = kind
        self._last_sample = -1e9
        hint = {'path': 'drive the path with the joystick',
                'obstacle': 'drive around the obstacle with the joystick'}.get(
                    kind, 'drive the boundary with the joystick')
        self._go('RECORDING', hint)
        self._sample(force=True)
        return True

    def _sample(self, force=False):
        pose = self.inputs.pose
        if pose is None or len(self._track) >= self.p.record_max_points:
            return
        pt = (float(pose[0]), float(pose[1]))
        if self._track and geo.dist(self._track[-1], pt) < self.p.record_min_spacing_m:
            return
        self._track.append(pt)
        self._fx.append(PublishTrajectory(list(self._track)))

    def _tick_record(self):
        if self._record_poly is not None:
            return                       # saving
        if self._now - self._last_sample >= 1.0 / max(0.1, self.p.record_rate_hz) - 1e-6:
            self._last_sample = self._now
            self._sample()

    def _record_finish(self):
        if self.phase != 'RECORDING':
            self._log('warn', 'RECORD_FINISH refused: not recording')
            return False
        if self._record_poly is not None:
            return True
        if self._record_kind == 'path':
            return self._record_finish_path()
        n = len(self._track)
        poly = geo.simplify_ring(self._track, self.p.record_simplify_tolerance_m,
                                 close_radius=self.p.record_close_radius_m)
        if len(poly) < 3 and n >= 3:
            poly = list(self._track)     # upstream: keep raw if DP too aggressive
        area = geo.polygon_area(poly)
        # a recorded no-go zone is much smaller than a mowing area: its own minimum
        min_area = (self.p.record_min_obstacle_area_m2 if self._record_kind == 'obstacle'
                    else self.p.record_min_area_m2)
        if len(poly) < 3 or area < min_area:
            why = 'polygon has %d vertices / %.2f m^2 (need 3 and %.1f m^2)' % (
                len(poly), area, min_area)
            self._log('warn', 'recording rejected: %s' % why)
            self._track = []
            self._fx.append(PublishTrajectory([]))
            self._go_idle('recording rejected: %s' % why)
            return False
        self._record_poly = poly
        self._log('info', 'recording finished: %d samples -> %d vertices, %.1f m^2' % (
            n, len(poly), area))
        self._go('RECORDING', 'saving obstacle' if self._record_kind == 'obstacle'
                 else 'saving area')
        self._enum = {'purpose': 'record', 'index': 0, 'areas': {}, 'count': 0}
        self._call(SRV_GET_AREA, {'index': 0}, purpose='enum')
        return True

    def _record_finish_path(self):
        n = len(self._track)
        line = geo.simplify_open_path(self._track, self.p.record_simplify_tolerance_m)
        length = geo.path_length(line) if len(line) >= 2 else 0.0
        if len(line) < 2 or length < self.p.record_path_min_length_m:
            why = 'path is %.2f m long (need %.1f m)' % (length, self.p.record_path_min_length_m)
            self._log('warn', 'recording rejected: %s' % why)
            self._track = []
            self._fx.append(PublishTrajectory([]))
            self._go_idle('recording rejected: %s' % why)
            return False
        self._record_line = line
        self._record_poly = geo.buffer_polyline(line, self.p.record_path_width_m / 2.0)
        self._log('info', 'path recording finished: %d samples -> %d points, %.1f m' % (
            n, len(line), length))
        self._go('RECORDING', 'saving path')
        self._enum = {'purpose': 'record', 'index': 0, 'areas': {}, 'count': 0}
        self._call(SRV_GET_AREA, {'index': 0}, purpose='enum')
        return True

    def _record_save(self, mowing_count, total_count=None, areas=None):
        if self._record_kind == 'obstacle':
            areas = areas or {}
            idx = self._record_obstacle_area(areas)
            if idx is None:
                # no mowing area to store the keepout under (the server rejects
                # obstacles on navigation areas) -> keep the ring like add_area failures
                self._record_name = 'No-go %d' % (mowing_count + 1)
                self._log('warn', 'cannot save no-go zone: no mowing area to attach it to')
                self._record_saved(False, failed_service='promote_obstacle')
                return
            obstacles = sum(len(a.get('obstacles') or []) for a in areas.values())
            self._record_index = idx
            self._record_name = 'No-go %d' % (obstacles + 1)
            self._call(SRV_PROMOTE_OBSTACLE, {'area_index': idx,
                                              'polygon': list(self._record_poly),
                                              'name': self._record_name},
                       purpose='record_obstacle')
            return
        if self._record_kind == 'path':
            total = mowing_count if total_count is None else total_count
            self._record_index = total            # add_area appends at the end
            self._record_name = 'Path %d' % (total - mowing_count + 1)
            self._call(SRV_ADD_AREA, {'name': self._record_name,
                                      'polygon': list(self._record_poly),
                                      'is_navigation_area': True}, purpose='add_area')
            return
        name = 'Area %d' % (mowing_count + 1)
        self._record_name = name
        self._call(SRV_ADD_AREA, {'name': name, 'polygon': list(self._record_poly),
                                  'is_navigation_area': False}, purpose='add_area')

    def _record_obstacle_area(self, areas):
        """Mowing area a recorded no-go zone is stored under: the one containing the
        ring centroid, else the first mowing area. None when there is none. The
        containment is soft on purpose — keepout and nav masks paint obstacle
        polygons by their own geometry, so a zone outside every work area (dock
        corridor, gap between areas) still holds once stored under any mowing area."""
        if not areas:
            return None
        ring = self._record_poly or []
        c = (sum(p[0] for p in ring) / len(ring),
             sum(p[1] for p in ring) / len(ring)) if ring else None
        for i in sorted(areas):
            if c is None or geo.point_in_polygon(c, areas[i].get('outer') or []):
                return i
        return sorted(areas)[0]

    def _record_saved(self, ok, failed_service='add_area'):
        poly, self._record_poly = self._record_poly, None
        self._track = []
        self._fx.append(PublishTrajectory([]))
        if ok:
            self._log('info', '%s "%s" saved (%d vertices)' % (
                self._record_kind, self._record_name, len(poly)))
            self._go('RECORDING_COMPLETE', self._record_name)
        else:
            self._log('error', '%s failed: polygon kept in a fallback file' % failed_service)
            self._fx.append(SaveRecordingFallback(list(poly), self._record_name))
            self._go_idle('%s failed; polygon saved locally' % failed_service)
