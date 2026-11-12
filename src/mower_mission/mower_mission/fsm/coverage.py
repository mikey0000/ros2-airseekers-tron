# SPDX-License-Identifier: GPL-3.0-or-later
"""MissionFSM mixin: the mowed raster of the current area (A3 coverage map, 2026-11-05).

The blade disc (cut_width_m, at the base_link -> blade_link offset from /tf_static) is stamped
into the area's MowedRaster on every tick while the cutter reports cutting and the
localisation is trusted. The raster lives in the area cursor (AreaCursor.raster), so it is
persisted with the resume cursor, starts empty with a new full mow / a new repeat run, and is
cleared when the area's boundary no longer matches it. It drives the honest % mowed, resume
by position (_raster_progress, used by _cursor_remap, the same-plan resume and the no-go
re-plan) and the end-of-area missed-patch report.
"""

from mower_mission import mowed_raster as mr
from mower_mission.fsm.common import geo, subpath_offsets

# while cutting, the cursor (and with it the raster) is saved at least this often, so a crash
# or power loss costs at most this much of the record (the other saves are event driven)
RASTER_SAVE_S = 30.0


class CoverageMixin:
    """Mowed raster: stamping, area %, resume progress, missed patches."""

    def _coverage_tick(self):
        """Stamp the blade disc swept since the last tick into the current area's raster."""
        i = self.inputs
        raster = self._current_raster()
        pt = None
        if raster is not None and i.is_cutting and i.pose is not None and \
                self._loc_trusted():
            if i.blade_offset is None:
                if not self._blade_tf_warned:
                    self._blade_tf_warned = True
                    self._log('warn', 'mowed raster: no base_link -> blade_link transform '
                                      '(/tf_static) yet: the cut area is not recorded')
            else:
                pt = mr.blade_point(i.pose[0], i.pose[1], i.pose[2], i.blade_offset)
        last, self._blade_last = self._blade_last, pt
        if pt is None:
            return
        if last is None or (last[0] - pt[0]) ** 2 + (last[1] - pt[1]) ** 2 > mr.MAX_STEP_M ** 2:
            last = pt
        raster.stamp_segment(last[0], last[1], pt[0], pt[1], 0.5 * float(self.p.cut_width_m))
        if raster.stamps != self._raster_saved[0] and \
                self._now - self._raster_saved[1] >= RASTER_SAVE_S:
            self._persist()

    def _loc_trusted(self):
        """The localisation may place the blade: status ok, or no localization_monitor at all
        (never received, as the hold reads it: bench, a sim without RTK injection). Degraded
        (RTK float, sigma over the limit), lost and a stale status are not trusted."""
        return self._loc_view()[0] in ('ok', '')

    def _current_raster(self):
        m = self.mission
        if m is None or m.area_idx is None:
            return None
        ac = self.cursor.areas.get(m.area_idx)
        return ac.raster if ac is not None else None

    def _area_raster(self, ac):
        """The raster for the current area from its cursor ``ac`` (None = none yet): kept
        while the area's outer boundary is the one it was recorded on, else a new empty one
        (an edited boundary, or an area index that now names another area)."""
        m = self.mission
        outer = m.areas.get(m.area_idx, {}).get('outer') or []
        key = mr.area_key(outer)
        old = ac.raster if ac is not None else None
        if old is not None and old.key == key:
            return old
        if old is not None:
            self._log('warn', 'area %d: boundary changed since its mowed raster was recorded: '
                              'raster cleared' % m.area_idx)
        return mr.MowedRaster.for_area(outer, key) if len(outer) >= 3 else None

    def _raster_progress(self, subpaths, raster):
        """(done, prefix) of ``subpaths`` on ``raster`` (MowedRaster.plan_progress, footprint
        = the area's swath width at the blade point); empty without a raster or before the
        blade offset is known (a base_link reading would put the resume 0.23 m ahead)."""
        off = self.inputs.blade_offset
        if raster is None or off is None or not raster.data.any():
            return set(), {}
        half = 0.5 * self._swath_spacing(self.mission.settings or {})
        return raster.plan_progress(subpaths, half, off)

    def _run_raster_progress(self, subpaths, raster, sub_runs):
        """_raster_progress of the sub-paths of the run in progress only (``sub_runs``: the
        run of each sub-path). The raster records what this run cut (a new one starts with
        each run, _advance_run): a cross plan's later rows cross ground the earlier rows cut,
        and the earlier rows are not judged by a later run's raster."""
        done, prefix = self._raster_progress(subpaths, raster)
        cur = self.mission.run_i
        if len(sub_runs) != len(subpaths) or all(r == cur for r in sub_runs):
            return done, prefix
        return ({k for k in done if sub_runs[k] == cur},
                {k: v for k, v in prefix.items() if sub_runs[k] == cur})

    def _earlier_run_subpaths(self, sub_runs):
        """Indices of the sub-paths of runs before the run in progress (``sub_runs``: the
        run of each sub-path): the part of a cross plan a resume in a later run is past."""
        cur = self.mission.run_i
        return {k for k, r in enumerate(sub_runs) if r < cur}

    def _raster_resume(self, ac, subpaths):
        """Same plan as the cursor: the sub-paths the raster shows mowed are done as well, and
        the resume point moves forward to the end of the mowed lead-in of the first sub-path
        left when that lies beyond the stored one (never backwards: the cursor's progress is
        kept; an uncut gap is the missed-patch report's)."""
        m = self.mission
        done, prefix = self._run_raster_progress(subpaths, ac.raster, m.sub_runs)
        added = done - ac.completed
        for k in added:
            ac.completed.add(k)
            ac.done_fps.add(geo.plan_fingerprint([subpaths[k]]))
        first = next((k for k in range(len(subpaths)) if k not in ac.completed), None)
        if first is None:
            m.resume_sub = None
        elif first in prefix:
            cur = m.resume_sub[1] if m.resume_sub is not None and m.resume_sub[0] == first else 0
            if prefix[first] > cur:
                m.resume_sub = (first, prefix[first])
                offs, _ = subpath_offsets(subpaths)
                ac.resume_index = offs[first] + prefix[first]
        return added

    def mowed_area_percent(self):
        """Cut share (%) of the current area's mowable polygon (outer minus accepted no-go
        zones) from the mowed raster; None without an area / raster."""
        m = self.mission
        raster = self._current_raster()
        if raster is None:
            return None
        outer, holes = self._mowable_polygon(m.area_idx)
        return raster.percent(outer, holes)

    def _mowable_polygon(self, idx):
        """(outer, holes) of area ``idx``: its accepted obstacles (pending ones were left out
        at enumeration) and the zones added by mid-mow re-plans."""
        m = self.mission
        area = m.areas.get(idx, {})
        return (area.get('outer') or [],
                [list(o) for o in area.get('obstacles', [])] +
                [list(o) for o in m.nogo_extra.get(idx, [])])

    def _area_cut_report(self):
        """End of an area run: the cut share of the area, and with missed_patch_min_m2 > 0
        the uncut patches the blade fits into (log + 'missed_patches' incident)."""
        m = self.mission
        raster = self._current_raster()
        if raster is None:
            return
        outer, holes = self._mowable_polygon(m.area_idx)
        cut, total = raster.cut_m2(outer, holes)
        if total <= 0.0:
            return
        pct = 100.0 * cut / total
        # the finish message's cut share; of several runs of one area (cross, repeat: a new
        # raster each) the best one counts
        if cut >= m.cut_m2.get(m.area_idx, (0.0, total))[0]:
            m.cut_m2[m.area_idx] = (cut, total)
        self._log('info', 'area %d: %.1f %% of the mowable area cut (mowed raster)'
                  % (m.area_idx, pct))
        min_m2 = float(self.p.missed_patch_min_m2)
        if min_m2 <= 0.0:
            return
        patches = raster.missed_patches(outer, holes, float(self.p.cut_width_m), min_m2)
        if not patches:
            return
        txt = ', '.join('%.2f m2 at (%.2f, %.2f)' % q for q in patches[:10])
        self._log('warn', 'area %d: %d uncut patch(es) >= %.2f m2 left: %s'
                  % (m.area_idx, len(patches), min_m2, txt))
        self._incident('missed_patches', 'area %d: %s' % (m.area_idx, txt))
