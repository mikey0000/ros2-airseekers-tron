# SPDX-License-Identifier: GPL-3.0-or-later
"""MissionFSM mixin: Sub-path / area / mission outcomes.

Moved verbatim from mission_fsm.py (2026-10-23 code review stage 1.6)."""

from mower_mission.fsm.common import (AreaCursor, CMD_START, DeleteResume, HEADING_NOT_ALIGNED,
    PublishAreaSettings, PublishResumeAvailable, ResumeCursor, SaveAlternateCounts, ZeroBurst,
    blocked_text, geo, subpath_offsets)  # noqa: F401


class OutcomeMixin:
    """Sub-path / area / mission outcomes."""

    def _subpath_done(self):
        m = self.mission
        ac = self.cursor.area(m.area_idx)
        ac.completed.add(m.sub_i)
        ac.done_fps.add(geo.plan_fingerprint([m.subpaths[m.sub_i]]))
        m.completed_count += 1
        offs, _total = subpath_offsets(m.subpaths)
        nxt = m.sub_i + 1
        ac.resume_index = offs[nxt] if nxt < len(offs) else -1
        self._persist()
        m.sub_i = nxt
        self._next_subpath()
        if self.mission is not None and self.phase in ('MOWING', 'TRANSIT'):
            self._go(self.phase)   # refresh completed_swaths / coverage

    def _subpath_failed(self, why, transit=False):
        """Retries exhausted: remember the sub-path as NOT mowed (it stays out of
        ``completed``, so a resume retries it) and go on with the next one.
        ``transit``: it never got to its start; re-tried once in the end-of-area pass."""
        m = self.mission
        self._blk = None
        if m.stretch is not None:
            self._stretch_blocked(why)
            return
        if transit:
            m.transit_failed.append((m.area_idx, m.sub_i))
        m.skipped += 1
        sp = m.subpaths[m.sub_i]
        local = self._track_progress() if m.step == 'follow' else m.start_local
        offs, _ = subpath_offsets(m.subpaths)
        m.failed.append((m.area_idx, m.sub_i, why, offs[m.sub_i] + min(local, len(sp) - 1)))
        self._incident('subpath_failed', why)
        self._log('warn', 'area %d sub-path %d NOT mowed: %s' % (m.area_idx, m.sub_i, why))
        m.step = None
        m.sub_i += 1
        self._next_subpath()

    def _not_mowed_text(self, extra_remaining=0):
        m = self.mission
        n = sum(1 for x in m.failed if x[1] is not None) + extra_remaining
        reason = m.failed[-1][2] if m.failed else ''
        unplanned = sum(1 for x in m.failed if x[1] is None)
        txt = ''
        if n or unplanned or not m.blocked:
            txt = '%d of %d sub-paths not mowed' % (n, m.planned_total)
            if unplanned:
                txt += ', %d area(s) not planned' % unplanned
        if m.blocked:
            b = blocked_text(len(m.blocked))
            txt = '%s, %s' % (txt, b) if txt else b
        return txt + (': %s' % reason if reason else '')

    def _finish_incomplete(self, text):
        """Session over with sub-paths left: latch MOWING_INCOMPLETE (resume kept)."""
        self._cancel_all('mowing incomplete')
        self._blade_off('mowing incomplete')
        self._fx.append(ZeroBurst())
        self.cursor.current_command = CMD_START
        self._persist()
        self._log('warn', 'MOWING_INCOMPLETE: %s (START resumes the missing sub-paths)' % text)
        self._go('MOWING_INCOMPLETE', text)
        if self.p.return_home_on_incomplete:
            self._incomplete_text = text
            self._dock('RETURNING_HOME', 'incomplete', sub=text)

    def _mission_incomplete(self):
        m = self.mission
        text = self._not_mowed_text()
        first = next((x for x in m.failed if x[1] is not None),
                     m.failed[0] if m.failed else m.blocked[0])
        self.cursor.current_area = first[0]
        self.mission = None
        self._fx.append(PublishAreaSettings({}))
        self._finish_incomplete(text)

    def _boundary_recovery_failed(self, why):
        """Recovery transits keep failing: stop in place, never dock from here."""
        m = self.mission
        ac = self.cursor.area(m.area_idx)
        left = sum(1 for i in range(m.sub_i, len(m.subpaths)) if i not in ac.completed)
        m.failed.append((m.area_idx, m.sub_i, why, -1))
        text = self._not_mowed_text(extra_remaining=left - 1)
        unvisited = len([a for a in m.queue if a not in self.cursor.completed_areas])
        if unvisited:
            text += ' (%d more area(s) not started)' % unvisited
        self._interrupt_mission(why)
        self._finish_incomplete(text)

    def _area_done(self):
        m = self.mission
        ac = self.cursor.area(m.area_idx)
        if m.requeue is None and ac.skipped and self.p.requeue_skipped:
            m.requeue = sorted(ac.skipped)
            self._log('info', 'area %d: re-trying %d stretch(es) skipped by obstacle detours'
                      % (m.area_idx, len(m.requeue)))
        if m.requeue:
            self._next_stretch()
            return
        if m.requeue_subs is None:
            m.requeue_subs = [x for x in m.failed if x[0] == m.area_idx and x[1] is not None
                              and (x[0], x[1]) in m.transit_failed] \
                if self.p.requeue_failed_transits else []
            if m.requeue_subs:
                self._log('info', 'area %d: re-trying %d sub-path(s) whose transit failed'
                          % (m.area_idx, len(m.requeue_subs)))
        if m.requeue_subs:
            self._next_failed_retry()
            return
        blocked = [b for b in m.blocked if b[0] == m.area_idx]
        failed = [x for x in m.failed if x[0] == m.area_idx and x[1] is not None]
        self._log('info' if not failed else 'warn',
                  'area %d done: %d/%d sub-paths mowed, %d NOT mowed' % (
                      m.area_idx, len(ac.completed), len(m.subpaths), len(failed))
                  + ('' if not blocked else ', %s' % blocked_text(len(blocked))))
        if failed or blocked:
            # not complete: keep it out of completed_areas, keep its run index, and
            # point the cursor at the first missing pose so START resumes there.
            if self.blade_on:
                self._blade_off('area finished with failures')
            ac.resume_index = min(x[3] for x in failed if x[3] >= 0) \
                if any(x[3] >= 0 for x in failed) else -1
            self._persist()
            self._next_area()
            return
        if m.runs and m.run_i + 1 < len(m.runs):
            # repeat / cross: plan and mow the same area again.
            if self.blade_on:
                self._blade_off('area run finished')
            m.run_i += 1
            self.cursor.area_runs[m.area_idx] = m.run_i
            self.cursor.areas[m.area_idx] = AreaCursor()
            self._persist()
            m.subpaths, m.lengths, m.cum = [], [], []
            m.sub_i, m.start_local, m.skipped, m.step = 0, 0, 0, None
            m.requeue, m.stretch, m.stretch_end = None, None, None
            m.requeue_subs, m.retry_entry = None, None
            self._log('info', 'area %d: run %d/%d (%s)' % (
                m.area_idx, m.run_i + 1, len(m.runs), m.settings['path_mode']))
            self._plan_run()
            return
        if m.settings is not None and m.settings['path_mode'] == 'alternate':
            name = self._area_name()
            self.alternate_counts[name] = int(self.alternate_counts.get(name, 0)) + len(m.runs)
            self._fx.append(SaveAlternateCounts(dict(self.alternate_counts)))
        self.cursor.area_runs.pop(m.area_idx, None)
        self.cursor.completed_areas.add(m.area_idx)
        ac.resume_index = -1
        self._persist()
        self._next_area()

    def _mission_complete(self):
        self._blade_off('mowing complete')
        self._go('MOWING_COMPLETE', '')
        self.mission = None
        self._fx.append(PublishAreaSettings({}))
        self._log('info', 'all areas mowed: returning home')
        self.cursor = ResumeCursor()
        self._fx.append(DeleteResume())
        self._fx.append(PublishResumeAvailable(False))
        self._dock('RETURNING_HOME', 'home')

    def _heading_failed(self):
        """No absolute heading: stop where we are. Never dock/navigate with Nav2 here, it
        would steer with the wrong heading."""
        self._log('error', 'mowing aborted: %s' % HEADING_NOT_ALIGNED)
        self._interrupt_mission(HEADING_NOT_ALIGNED)
        self._cancel_all(HEADING_NOT_ALIGNED)
        self._blade_off(HEADING_NOT_ALIGNED)
        self._go_idle(HEADING_NOT_ALIGNED)
        self._last_status = None
