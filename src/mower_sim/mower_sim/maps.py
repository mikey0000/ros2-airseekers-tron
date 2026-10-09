# SPDX-License-Identifier: Apache-2.0
"""Write a world's lawn / dock into a mower_map maps directory.

The sim hands map_server_node a scratch ``maps_dir`` holding ``areas.dat`` (the
lawn as area 0, the world's extra areas after it) and ``dock_pose.yaml``
(measured). Obstacles are NOT written unless marked ``known: true`` in the
world: the robot has to find them with its own sensors.

    ros2 run mower_sim write_maps <world.yaml> <maps_dir>
"""

from __future__ import annotations

import os
import sys

from .world import World, load_world


def write_maps(world: World, maps_dir: str) -> None:
    from mower_map import areas as A
    os.makedirs(maps_dir, exist_ok=True)
    lawn = A.Area(name='lawn', polygon=list(world.lawn))
    for o in world.obstacles:
        if not o.known or o.moving:
            continue
        lawn.obstacles.append(A.Obstacle(polygon=o.polygon(0.0), name=o.name))
    out = [lawn]
    for i, a in enumerate(world.extra_areas):
        poly = [(float(p[0]), float(p[1])) for p in a.get('polygon', [])]
        if len(poly) >= 3:
            out.append(A.Area(name=str(a.get('name', 'area_%d' % (i + 1))), polygon=poly,
                              is_navigation=bool(a.get('is_navigation', False))))
    A.save_areas_file(os.path.join(maps_dir, 'areas.dat'), out,
                      world.datum_lat, world.datum_lon)
    # vendor dock outline 0.4 x 0.55 m, behind the docked robot (dock-local frame)
    outline = [(-0.65, -0.275), (-0.25, -0.275), (-0.25, 0.275), (-0.65, 0.275)]
    dock = A.DockPose(world.dock.x, world.dock.y, world.dock.yaw, outline=outline,
                      measured=True)
    A.save_dock_file(os.path.join(maps_dir, 'dock_pose.yaml'), dock)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 2:
        print('usage: write_maps <world.yaml> <maps_dir>', file=sys.stderr)
        return 2
    world = load_world(argv[0])
    write_maps(world, argv[1])
    print('wrote %s/areas.dat and dock_pose.yaml (%s)' % (argv[1], world.name))
    return 0


if __name__ == '__main__':
    sys.exit(main())
