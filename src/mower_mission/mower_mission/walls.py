# SPDX-License-Identifier: GPL-3.0-or-later
"""Boundary lines (virtual walls, 2026-11-02) in the mission. Pure Python, no ROS.

The geometry and the wire format live in ``mower_map.boundary_lines`` (map_server stores and
latches the set on /map_server_node/boundary_lines). The mission uses them twice:

* coverage holes: every line whose thin keepout (width ``width_m + boundary_line_keepout_m``)
  overlaps the area is one more hole of the PlanCoverage goal, so swaths are cut at the line
  (``area_holes``);
* the hard boundary guard: the footprint at the current pose touching the TRUE line (plus its
  ``width_m``) by more than ``boundary_line_trip_m`` is a trip naming the line
  (``hits``; fsm/guards/boundary.py), recovered by retracing the driven track.
"""

import math
import os
import sys

try:
    from mower_map import boundary_lines as blines
except ImportError:          # source tree without the mower_map install (unit tests)
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'mower_map'))
    from mower_map import boundary_lines as blines      # noqa: E402

from mower_common import geometry as rgeo

BoundaryLine = blines.BoundaryLine


def parse(text):
    """Lines from the latched JSON, or None when it does not parse (keep the last set)."""
    ok, _msg, lines = blines.parse_json(text)
    return lines if ok else None


def area_holes(lines, outer, keepout_m):
    """Keepout polygons (lists of (x, y)) of the lines that overlap the area ``outer``."""
    if not lines or not outer or len(outer) < 3:
        return []
    return [list(k) for _ln, k in blines.keepouts_for_polygon(lines, outer, keepout_m)]


def hits(lines, pose, min_depth=0.0):
    """[(depth, line)] of the lines the footprint at ``pose`` reaches deeper than
    ``min_depth`` into, deepest first (owner-measured footprint, mower_common.geometry)."""
    return blines.body_hits(lines or [], pose, rgeo.FOOTPRINT_FRONT_M, rgeo.FOOTPRINT_REAR_M,
                            rgeo.FOOTPRINT_HALF_WIDTH_M, min_depth)


def depth_of(lines, line_id, pose):
    """Footprint depth into line ``line_id`` at ``pose`` (<= 0 = clear by that much); None when
    the line is gone."""
    for ln in lines or []:
        if ln.id == line_id and len(ln.points) >= 2 and pose is not None:
            corners = blines.body_corners(float(pose[0]), float(pose[1]), float(pose[2]),
                                          rgeo.FOOTPRINT_FRONT_M, rgeo.FOOTPRINT_REAR_M,
                                          rgeo.FOOTPRINT_HALF_WIDTH_M)
            return blines.body_depth(ln, corners)
    return None


def away_dir(lines, line_id, pose):
    """Unit vector from the nearest point of line ``line_id`` to base_link (the side the robot
    came from), or None."""
    for ln in lines or []:
        if ln.id == line_id and len(ln.points) >= 2 and pose is not None:
            qx, qy, d = blines.nearest_point(ln, float(pose[0]), float(pose[1]))
            if d < 1e-6:
                return None
            return ((float(pose[0]) - qx) / d, (float(pose[1]) - qy) / d)
    return None


def describe(depth, line):
    return 'body crosses boundary line %s by %.2f m' % (line.label(), max(0.0, depth)) \
        if math.isfinite(depth) else 'body crosses boundary line %s' % line.label()
