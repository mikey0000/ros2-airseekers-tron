# Copyright 2026 Airseekers Tron ROS 2 port contributors
# SPDX-License-Identifier: GPL-3.0-or-later
#
# On-disk format ported from MowgliNext mowgli_behavior/src/coverage_persistence.cpp
# (GPL-3.0): header ``mowgli_coverage_resume v2``.
"""Coverage resume cursor and its upstream-compatible text format (no ROS).

File layout (one record per line, unknown tags ignored, exactly as upstream)::

    mowgli_coverage_resume v2
    current_command 1
    single_area_target 2            # only for a start_in_area run
    current_area 0
    completed_areas 1 3
    area <idx> <pose_count> <fingerprint> <resume_pose_index|-1> completed <i> <j> ...

``pose_count`` is the number of poses over all drivable sub-paths of the
area plan, ``resume_pose_index`` an absolute index into their concatenation,
and the ``completed`` indices are drivable sub-path indices (upstream calls
the same units "swaths").  Cross-hatch rows are not written (we do not
alternate the mow angle) and are skipped when read.
"""

HEADER = 'mowgli_coverage_resume v2'


class AreaCursor:
    __slots__ = ('pose_count', 'fingerprint', 'resume_index', 'completed')

    def __init__(self, pose_count=0, fingerprint=0, resume_index=-1, completed=None):
        self.pose_count = int(pose_count)
        self.fingerprint = int(fingerprint)
        self.resume_index = int(resume_index)
        self.completed = set(completed or ())

    def __eq__(self, other):
        return isinstance(other, AreaCursor) and all(
            getattr(self, k) == getattr(other, k) for k in self.__slots__)

    def __repr__(self):
        return 'AreaCursor(%d, %d, %d, %r)' % (
            self.pose_count, self.fingerprint, self.resume_index, sorted(self.completed))


class ResumeCursor:
    """Where an interrupted mowing session continues."""

    def __init__(self):
        self.current_command = 0
        self.single_area_target = None
        self.current_area = -1
        self.completed_areas = set()
        self.areas = {}

    def __eq__(self, other):
        return isinstance(other, ResumeCursor) and self.__dict__ == other.__dict__

    def __repr__(self):
        return 'ResumeCursor(%r)' % self.__dict__

    @property
    def available(self):
        """True when START would resume rather than start fresh: a START
        session that made progress (a finished area, a finished sub-path or
        a stored resume pose)."""
        return self.current_command == 1 and (
            bool(self.completed_areas)
            or any(a.completed or a.resume_index >= 0 for a in self.areas.values()))

    def area(self, idx):
        return self.areas.setdefault(int(idx), AreaCursor())

    # ------------------------------------------------------------------
    def dumps(self):
        out = [HEADER, 'current_command %d' % int(self.current_command)]
        if self.single_area_target is not None:
            out.append('single_area_target %d' % int(self.single_area_target))
        out.append('current_area %d' % int(self.current_area))
        out.append(' '.join(['completed_areas'] + [str(i) for i in sorted(self.completed_areas)]))
        for idx in sorted(set(self.areas) | self.completed_areas):
            a = self.areas.get(idx, AreaCursor())
            out.append(' '.join(
                ['area', str(idx), str(a.pose_count), str(a.fingerprint), str(a.resume_index),
                 'completed'] + [str(s) for s in sorted(a.completed)]))
        return '\n'.join(out) + '\n'

    @classmethod
    def loads(cls, text):
        """Parse; returns None for empty text or an unknown header (start fresh)."""
        lines = (text or '').splitlines()
        if not lines or lines[0].strip() != HEADER:
            return None
        cur = cls()
        for line in lines[1:]:
            tok = line.split()
            if not tok:
                continue
            tag, args = tok[0], tok[1:]
            try:
                if tag == 'current_command' and args:
                    cur.current_command = int(args[0])
                elif tag == 'single_area_target' and args:
                    cur.single_area_target = int(args[0])
                elif tag == 'current_area' and args:
                    cur.current_area = int(args[0])
                elif tag == 'completed_areas':
                    cur.completed_areas = {int(a) for a in args}
                elif tag == 'area' and len(args) >= 5 and args[4] == 'completed':
                    idx = int(args[0])
                    cur.areas[idx] = AreaCursor(int(args[1]), int(args[2]), int(args[3]),
                                                {int(s) for s in args[5:]})
            except ValueError:
                continue  # malformed row: skip it, like upstream
        return cur


def subpath_offsets(subpaths):
    """Absolute index of the first pose of each sub-path in the concatenation."""
    offs, total = [], 0
    for sp in subpaths:
        offs.append(total)
        total += len(sp)
    return offs, total


def absolute_to_local(subpaths, absolute):
    """Absolute pose index -> (sub-path index, local index); None if out of range."""
    offs, total = subpath_offsets(subpaths)
    if absolute < 0 or absolute >= total:
        return None
    for i in range(len(subpaths) - 1, -1, -1):
        if absolute >= offs[i] and len(subpaths[i]) > 0:
            return i, absolute - offs[i]
    return None
