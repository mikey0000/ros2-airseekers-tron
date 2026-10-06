#!/usr/bin/env python3
"""Remove the closing-overshoot spike from a recorded area in areas.dat.

A boundary recorded by driving (RECORDING mode) before the
trim_closure_overshoot fix kept the bit of track driven past the start point,
so the stored ring ends with a vertex lying beyond vertex 0 and the ring closes
with a near-180 deg sliver spike. This drops such trailing vertices (turn at
the vertex >= --min-turn degrees, i.e. the ring doubles back on itself).

Dry run by default: prints the before/after polygon. Pass --apply to write
(a timestamped backup is made next to the file first). map_server_node only
reads areas.dat at startup and rewrites it from memory on every map change,
so after --apply restart the stack before touching the map again.

  python3 scripts/fix_area_closure.py /userdata/ros2/maps/areas.dat --area "Area 1"
  python3 scripts/fix_area_closure.py /userdata/ros2/maps/areas.dat --area "Area 1" --apply
"""
import argparse
import math
import os
import shutil
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'mower_map'))
from mower_map import areas as core  # noqa: E402


def turn_deg(a, b, c):
    h1 = math.atan2(b[1] - a[1], b[0] - a[0])
    h2 = math.atan2(c[1] - b[1], c[0] - b[0])
    return abs(math.degrees((h2 - h1 + math.pi) % (2 * math.pi) - math.pi))


def trim_closing_spike(poly, min_turn=150.0):
    """Drop trailing vertices at which the ring doubles back (sliver spike)."""
    poly = list(poly)
    removed = []
    while len(poly) > 3 and turn_deg(poly[-2], poly[-1], poly[0]) >= min_turn:
        removed.append(poly.pop())
    return poly, removed


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('path')
    ap.add_argument('--area', required=True, help='area name, e.g. "Area 1"')
    ap.add_argument('--min-turn', type=float, default=150.0)
    ap.add_argument('--apply', action='store_true')
    a = ap.parse_args()

    with open(a.path, encoding='utf-8') as f:
        text = f.read()
    areas, datum = core.parse_areas_dat(text)
    hits = [x for x in areas if x.name == a.area]
    if len(hits) != 1:
        sys.exit('area %r: %d matches in %s' % (a.area, len(hits), a.path))
    area = hits[0]
    new, removed = trim_closing_spike(area.polygon, a.min_turn)
    print('%s: %d vertices, %.2f m^2' % (a.area, len(area.polygon), core.polygon_area(area.polygon)))
    if not removed:
        print('no closing spike (turn at last vertex %.0f deg); nothing to do'
              % turn_deg(area.polygon[-2], area.polygon[-1], area.polygon[0]))
        return
    print('remove trailing vertices: %s' % ', '.join('(%.3f, %.3f)' % p for p in removed))
    print('after: %d vertices, %.2f m^2' % (len(new), core.polygon_area(new)))
    print('new polygon: %s' % core.polygon_to_string(new))
    if not a.apply:
        print('dry run: pass --apply to write')
        return
    backup = '%s.bak-%s' % (a.path, time.strftime('%Y%m%d-%H%M%S'))
    shutil.copy2(a.path, backup)
    area.polygon = new
    lat, lon = datum if datum else (0.0, 0.0)
    core.save_areas_file(a.path, areas, lat, lon)
    print('written %s (backup %s); restart the stack so map_server_node reloads it' % (a.path, backup))


if __name__ == '__main__':
    main()
