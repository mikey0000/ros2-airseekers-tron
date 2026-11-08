# SPDX-License-Identifier: GPL-3.0-or-later
"""Propose smoothed boundaries for existing mowing areas (never writes the live map).

    ros2 run mower_map smooth_areas /ros2_ws/maps/areas.dat \\
        [--area "Front lawn" ...] [--tolerance 0.08] [--out areas.smoothed.dat]

Reads an areas.dat, smooths the outer boundary of every selected mowing area with the same
inner-hull smoothing as area recording (mower_map.boundary_smoothing: zigs within the
tolerance go, the line only moves inward, corners stay), and writes

- the proposal: a complete areas.dat with only the selected boundaries changed (navigation
  areas, no-go zones, boundary lines and the datum stamp are copied unchanged), default
  ``areas.smoothed.dat`` next to the input;
- a report per area (vertices, max deviation, area change, wiggle and excess turning
  before / after; ``boundary_smoothing.wiggle_m`` / ``excess_turning_deg``) on stdout
  and as JSON next to the proposal.

It refuses to write over its input. To apply the proposal, keep a copy of areas.dat, copy the
proposal over it and call ``/map_server_node/load_areas`` (mower_map README, "Smoothing
existing boundaries").

``--restore-from DIR`` instead puts back the rings as recorded, from the recording quality
files in DIR (mission recording_quality_dir), for areas smoothed when they were recorded.
"""

import argparse
import json
import math
import os
import sys

from mower_map import areas as core
from mower_map import boundary_smoothing as bs


def _percentile(values, q):
    v = sorted(values)
    if not v:
        return 0.0
    k = (len(v) - 1) * q / 100.0
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    return v[lo] + (v[hi] - v[lo]) * (k - lo)


def area_report(name, before, after, res=None):
    """One area's before/after numbers (a dict, also what the JSON report holds)."""
    zb, za = bs.wiggle_m(before), bs.wiggle_m(after)
    ab, aa = core.polygon_area(before), core.polygon_area(after)
    return {
        'name': name, 'vertices_before': len(before), 'vertices_after': len(after),
        'max_deviation_m': round(res.max_deviation_m if res else 0.0, 4),
        'area_before_m2': round(ab, 3), 'area_after_m2': round(aa, 3),
        'area_change_m2': round(aa - ab, 3),
        'area_change_pct': round(100.0 * (aa - ab) / ab, 2) if ab > 0 else 0.0,
        'wiggle_before_m': {'p50': round(_percentile(zb, 50), 3),
                            'p90': round(_percentile(zb, 90), 3), 'max': round(max(zb), 3)},
        'wiggle_after_m': {'p50': round(_percentile(za, 50), 3),
                           'p90': round(_percentile(za, 90), 3), 'max': round(max(za), 3)},
        'excess_turning_before_deg': round(bs.excess_turning_deg(before), 1),
        'excess_turning_after_deg': round(bs.excess_turning_deg(after), 1),
        'skipped': (res.reason if res else ''),
    }


def smooth_areas(areas, names, tolerance):
    """(new area list, reports). Pure: ``areas`` is not changed."""
    out, reports = [], []
    for a in areas:
        b = core.Area(name=a.name, polygon=list(a.polygon), is_navigation=a.is_navigation,
                      obstacles=a.obstacles, channel=a.channel,
                      channel_width_m=a.channel_width_m)
        if not a.is_navigation and (not names or a.name in names):
            res = bs.smooth_ring(a.polygon, tolerance)
            b.polygon = list(res.ring)
            reports.append(area_report(a.name, list(a.polygon), b.polygon, res))
        out.append(b)
    return out, reports


def restore_recorded(areas, names, quality_dir):
    """(new area list, reports): the recorded rings back from the quality sidecars."""
    out, reports = [], []
    for a in areas:
        b = core.Area(name=a.name, polygon=list(a.polygon), is_navigation=a.is_navigation,
                      obstacles=a.obstacles, channel=a.channel,
                      channel_width_m=a.channel_width_m)
        if not a.is_navigation and (not names or a.name in names):
            path = os.path.join(quality_dir, '%s.json' % a.name.replace(' ', '_'))
            ring = None
            try:
                with open(path, encoding='utf-8') as f:
                    ring = ((json.load(f).get('smoothing') or {}).get('recorded_polygon'))
            except (OSError, ValueError):
                pass
            if ring and len(ring) >= 3:
                b.polygon = [(float(p[0]), float(p[1])) for p in ring]
                reports.append(area_report(a.name, list(a.polygon), b.polygon))
            else:
                r = area_report(a.name, list(a.polygon), list(a.polygon))
                r['skipped'] = 'no recorded ring in %s' % path
                reports.append(r)
        out.append(b)
    return out, reports


def format_report(reports):
    lines = ['%-24s %9s %8s %16s %13s %17s %s' % (
        'area', 'vertices', 'max dev', 'area m2', 'change', 'wiggle p50/p90', 'excess turn')]
    for r in reports:
        lines.append('%-24s %4d->%-4d %6.3f m %7.2f->%-7.2f %+6.2f %5.2f%% %.2f/%.2f->%.2f/%.2f '
                     '%4.0f->%-4.0f deg%s' % (
                         r['name'][:24], r['vertices_before'], r['vertices_after'],
                         r['max_deviation_m'], r['area_before_m2'], r['area_after_m2'],
                         r['area_change_m2'], r['area_change_pct'],
                         r['wiggle_before_m']['p50'], r['wiggle_before_m']['p90'],
                         r['wiggle_after_m']['p50'], r['wiggle_after_m']['p90'],
                         r['excess_turning_before_deg'], r['excess_turning_after_deg'],
                         ('  (' + r['skipped'] + ')') if r['skipped'] else ''))
    return '\n'.join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('areas', help='areas.dat to read (not written)')
    ap.add_argument('--area', action='append', default=[],
                    help='mowing area name to smooth (repeat; default every mowing area)')
    ap.add_argument('--tolerance', type=float, default=bs.DEFAULT_TOLERANCE_M,
                    help='largest inward move of the line, m (default %(default)s, the '
                         'mission record_smooth_tolerance_m default)')
    ap.add_argument('--out', default=None,
                    help='proposed areas file (default areas.smoothed.dat next to the input)')
    ap.add_argument('--restore-from', default=None, metavar='DIR',
                    help='instead: put back the recorded rings from the recording quality '
                         'files in DIR (default proposal name areas.restored.dat)')
    args = ap.parse_args(argv)
    if args.tolerance <= 0.0 and not args.restore_from:
        ap.error('--tolerance must be > 0')

    src = os.path.abspath(args.areas)
    out = os.path.abspath(args.out or os.path.join(
        os.path.dirname(src), 'areas.restored.dat' if args.restore_from else
        'areas.smoothed.dat'))
    if out == src:
        print('error: refusing to write over the input; apply the proposal yourself',
              file=sys.stderr)
        return 2
    with open(src, encoding='utf-8') as f:
        text = f.read()
    areas, datum = core.parse_areas_dat(text)
    lines, _ = core.parse_boundary_lines_dat(text)
    known = {a.name for a in areas if not a.is_navigation}
    missing = [n for n in args.area if n not in known]
    if missing:
        print('error: no mowing area named %s (have: %s)' % (
            ', '.join(repr(n) for n in missing), ', '.join(sorted(known))), file=sys.stderr)
        return 2
    names = set(args.area)
    if args.restore_from:
        new, reports = restore_recorded(areas, names, args.restore_from)
    else:
        new, reports = smooth_areas(areas, names, args.tolerance)
    lat, lon = datum if datum else (0.0, 0.0)
    core.save_areas_file(out, new, lat, lon, lines)
    report_path = os.path.splitext(out)[0] + '.json'
    core.write_text_atomic(report_path, json.dumps(
        {'source': src, 'proposal': out,
         'mode': 'restore' if args.restore_from else 'smooth',
         'tolerance_m': None if args.restore_from else args.tolerance,
         'areas': reports}, indent=1) + '\n')
    print(format_report(reports))
    print('\nwrote %s (proposal; %s is unchanged)\nwrote %s (report)' % (out, src, report_path))
    return 0


if __name__ == '__main__':
    sys.exit(main())
