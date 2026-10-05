# SPDX-License-Identifier: GPL-3.0-or-later
"""Convert an Airseekers vendor map.geojson into areas.dat + dock_pose.yaml.

    ros2 run mower_map import_vendor_geojson map.geojson [--out areas.dat]

Then either copy the files into the map server's maps_dir and restart it, or
call /map_server_node/load_areas.
"""

import argparse
import math
import os
import sys

from mower_map import areas as core


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('geojson', help='vendor map.geojson')
    ap.add_argument('--out', default='areas.dat', help='areas.dat to write (default ./areas.dat)')
    ap.add_argument('--dock-out', default=None,
                    help='dock pose file (default dock_pose.yaml next to --out)')
    ap.add_argument('--robot-yaml', default=None,
                    help='also splice dock_pose_x/y/yaw into this mowgli_robot.yaml')
    ap.add_argument('--channel-half-width', type=float, default=0.35,
                    help='half width (m) of the navigation area built from each channel')
    ap.add_argument('--no-channels', action='store_true',
                    help='do not import type-3 channels as navigation areas')
    ap.add_argument('--keep-dock-zone-obstacles', action='store_true',
                    help='keep type-4 obstacles that contain the charge point')
    ap.add_argument('--datum-lat', type=float, default=0.0)
    ap.add_argument('--datum-lon', type=float, default=0.0)
    args = ap.parse_args(argv)

    res = core.import_vendor_geojson_file(
        args.geojson, channel_half_width=args.channel_half_width,
        import_channels=not args.no_channels,
        keep_dock_zone_obstacles=args.keep_dock_zone_obstacles)
    for w in res.warnings:
        print('warning: ' + w, file=sys.stderr)
    if not res.areas:
        print('error: no areas found in %s' % args.geojson, file=sys.stderr)
        return 1

    core.save_areas_file(args.out, res.areas, args.datum_lat, args.datum_lon)
    print('wrote %s:' % args.out)
    for i, a in enumerate(res.areas):
        print('  [%d] %-12s %-10s %3d vertices, %d obstacles, %.1f m2'
              % (i, a.name, 'navigation' if a.is_navigation else 'mowing', len(a.polygon),
                 len(a.obstacles), core.polygon_area(a.polygon)))
    if res.dock is not None:
        dock_out = args.dock_out or os.path.join(os.path.dirname(os.path.abspath(args.out)),
                                                 'dock_pose.yaml')
        core.save_dock_file(dock_out, res.dock)
        print('wrote %s: x=%.3f y=%.3f yaw=%.3f rad (%.1f deg)%s'
              % (dock_out, res.dock.x, res.dock.y, res.dock.yaw, math.degrees(res.dock.yaw),
                 ', with outline' if res.dock.outline else ''))
        if args.robot_yaml:
            ok = core.update_robot_yaml_dock_pose(args.robot_yaml, res.dock.x, res.dock.y,
                                                  res.dock.yaw)
            print(('updated ' if ok else 'could NOT update ') + args.robot_yaml)
    return 0


if __name__ == '__main__':
    sys.exit(main())
