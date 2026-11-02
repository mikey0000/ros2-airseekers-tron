# SPDX-License-Identifier: GPL-3.0-or-later
"""Boundary lines (virtual walls, 2026-11-02) in the mission. Pure Python, no ROS.

The geometry and the wire format live in ``mower_map.boundary_lines`` (map_server stores and
latches the set on /map_server_node/boundary_lines). The mission uses them twice:

* coverage holes: every line whose thin keepout (width ``width_m + boundary_line_keepout_m``,
  blade lines ``width_m + blade keepout``) overlaps the area is one more hole of the
  PlanCoverage goal, so swaths are cut at the line (``area_holes``);
* the hard boundary guard: at the current pose, the footprint (body lines) or the blade disc
  (blade lines, 2026-11-03) touching the TRUE line (plus its ``width_m``) by more than
  ``boundary_line_trip_m`` is a trip naming the line (``hits``; fsm/guards/boundary.py),
  recovered by retracing the driven track.

The blade disc: radius from map_server's latched ``blade_radius_m`` (its blade_disc_mm; the
220 mm disc's 0.11 before one arrives), centre ``DECK_CENTRE_X_M`` (0.23) ahead of base_link.
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


DEFAULT_BLADE_RADIUS_M = rgeo.BLADE_DISC_M / 2.0


def parse(text):
    """Lines from the latched JSON, or None when it does not parse (keep the last set)."""
    ok, _msg, lines = blines.parse_json(text)
    return lines if ok else None


def blade_radius_of(text):
    """map_server's blade disc radius from the latched JSON, None when it does not carry one."""
    return blines.blade_radius_of(text)


def blade_keepout(blade_radius=None, override_m=-1.0):
    """Planner keepout of a blade line: ``override_m`` when > 0, else 2 x (radius + 0.05)."""
    if override_m is not None and float(override_m) > 0.0:
        return float(override_m)
    return blines.blade_keepout_m(blade_radius or DEFAULT_BLADE_RADIUS_M)


def geometry(blade_radius=None):
    """The owner-measured footprint (mower_common.geometry) and the blade disc."""
    return blines.Geometry(rgeo.FOOTPRINT_FRONT_M, rgeo.FOOTPRINT_REAR_M,
                           rgeo.FOOTPRINT_HALF_WIDTH_M, blade_radius or DEFAULT_BLADE_RADIUS_M,
                           rgeo.DECK_CENTRE_X_M, 0.0)


def area_holes(lines, outer, keepout_m, blade_keepout_m=blines.DEFAULT_BLADE_KEEPOUT_M):
    """Keepout polygons (lists of (x, y)) of the lines that overlap the area ``outer``."""
    if not lines or not outer or len(outer) < 3:
        return []
    found = blines.keepouts_for_polygon(lines, outer, keepout_m, blade_keepout_m)
    return [list(k) for _ln, k in found]


def hits(lines, pose, min_depth=0.0, blade_radius=None):
    """[(depth, line)] of the lines reached deeper than ``min_depth`` at ``pose``, deepest
    first: the footprint for body lines, the blade disc for blade lines."""
    return blines.line_hits(lines or [], pose, geometry(blade_radius), min_depth)


def depth_of(lines, line_id, pose, blade_radius=None):
    """Depth into line ``line_id`` at ``pose`` (footprint or blade disc by its mode; <= 0 =
    clear by that much); None when the line is gone."""
    for ln in lines or []:
        if ln.id == line_id and len(ln.points) >= 2 and pose is not None:
            return blines.line_depth(ln, pose, geometry(blade_radius))
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
    what = 'blade' if line.is_blade else 'body'
    return '%s crosses boundary line %s by %.2f m' % (what, line.label(), max(0.0, depth)) \
        if math.isfinite(depth) else '%s crosses boundary line %s' % (what, line.label())
