#!/usr/bin/env python3
"""Sim gate resume scenario: turn the cursor a priming START wrote into a mid-path resume.

    resume_cursor.py <coverage_resume.txt> <area> <abs_pose_index>

The priming START planned the area and persisted ``area <idx> <pose_count> <fingerprint>``
(mower_mission/resume.py format). This rewrites the file as an interrupted START session
of that plan that continues at absolute pose ``abs_pose_index`` of the area's drivable
sub-paths (for sub-path 0 the local index: field run 6 resumed at cursor (0, 279), the
pivot vertex (-0.50, 5.32) 0.14 m from the boundary, and at (0, 402)). Exit 1 when the file
has no planned area (the priming START did not get that far).
"""
import os
import sys

HEADER = 'mowgli_coverage_resume v2'


def main(argv):
    path, area, index = argv[1], int(argv[2]), int(argv[3])
    try:
        with open(path, 'r', encoding='utf-8') as fh:
            lines = fh.read().splitlines()
    except OSError:
        return 1
    rows = [ln.split() for ln in lines]
    plan = next((r for r in rows if len(r) >= 5 and r[0] == 'area' and int(r[1]) == area
                 and int(r[2]) > 0), None)
    if not lines or lines[0].strip() != HEADER or plan is None:
        return 1
    count, fingerprint = int(plan[2]), int(plan[3])
    if not 0 <= index < count:
        print('resume_cursor: index %d outside the plan (%d poses)' % (index, count),
              file=sys.stderr)
        return 2
    out = [HEADER, 'current_command 1', 'current_area %d' % area, 'completed_areas',
           'area %d %d %d %d completed' % (area, count, fingerprint, index)]
    tmp = path + '.gate'
    with open(tmp, 'w', encoding='utf-8') as fh:
        fh.write('\n'.join(out) + '\n')
    try:
        os.remove(path)          # written by the (root) mission; the directory is ours
    except OSError:
        pass
    os.replace(tmp, path)
    print('resume_cursor: area %d resumes at pose %d of %d (fingerprint %d)'
          % (area, index, count, fingerprint))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
