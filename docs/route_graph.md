# Route-graph transits (2026-10-09)

Owner request: "split navigation up: a specific path for on paths, a different navigation
e.g. follow perimeter / direct to the next path, then path again, to keep the lines within
the areas and paths."

Before this change Nav2 planned every mission transit on the global costmap. That covers the
blade-off move to the next sub-path start, the end-of-area retries and the return home. The
nav mask makes paths cheap (centreline profile 0..`path_edge_cost`) and the lawn dearer
(`area_transit_cost`). NavFn and Smac still cut corners. In the sim the transit left "Path 1"
at x ≈ 6.8 and crossed the lawn. Transits are now planned on a graph built from the map and
driven with FollowPath. Nav2's costmap planner is the fallback.

## Planner: `mower_map/route_graph.py`

It is pure Python with numpy only. It is unit-tested in `test/test_route_graph.py`, on the sim
map, on the owner's field map (`test/data/field_2026-10-08_*`, datum zeroed) and on synthetic
obstacle and concave cases.

- **Path edges.** Each navigation area that has a drawn centreline (`Area.channel`) is
  followed exactly along that polyline, in both directions. The dock -> approach-pose leg is
  a pseudo path called `dock`. The approach pose is `approach_distance` along the dock yaw,
  as in `mower_docking`. When the approach pose lies outside every polygon, the pseudo path
  goes on to the nearest area or path edge, as `areas.dock_corridor` does.
- **Portals: paths are driven END TO END** (owner decision, 2026-10-09). A path is
  entered or left only at:
  - its ends;
  - a junction with an AREA, meaning where its centreline crosses the area boundary;
  - a real crossing with another path, at 25° or more (a shallower crossing counts as
    an overlap).

  These ends are not exits (`_free_end_list`):
  - the dock end of the dock pseudo path;
  - the dock approach end when it lies on a drawn path's band (leaving the dock uses that
    path);
  - a drawn path end that lies inside the dock corridor (the path starts at the dock);
  - a path end inside another path's band away from that path's ends (a spur), or where the
    other path carries on (chained paths).

  Coincident ends of duplicated paths stay exits, as ends of the network.

  A start or goal on a path band is driven along that path: no lawn hop straight off it or
  onto it. There is one exception: the start and goal lie in the same area and the start is
  not at the dock. An in-area mowing transit that happens to begin on a band must not ride the
  path to its far end and back.

  The mid-path portals from earlier versions (`route_portal_step_m`, a portal every 0.25 m
  inside areas) are kept behind the parameter but are off (0) by default.
- **Overlapping paths.** Every `overlap_step_m` (0.5 m) along a path that runs inside another
  path's band, plus at both of its ends, is joined to that path's centreline. Duplicated,
  partly overlapping, end-inside-band and shallow-crossing paths therefore act as one path
  along the overlap: either can be used, and the route can switch from one to the other
  anywhere. Mid-path portals are still placed wherever a centreline lies inside an area with
  the clearance, whether or not another path's band covers it.

  This was added after the owner drew "Dock path" almost on top of Path 1
  (`test/data/sim_garden_paths_v3.dat`). The synthetic cases and a seed-fixed fuzz are in
  `test_route_graph.py`.
- **In-area legs.** These are the "direct to the next path" and "follow perimeter" moves.
  Inside one mowing area, or a navigation polygon without a centreline, two points are joined
  by a straight segment when it keeps `route_clearance_m` (0.35 m: half the robot width 0.29
  plus 0.06) from the area edge and from every obstacle polygon. Otherwise the route bends
  round the inset perimeter.

  The bend points form a visibility graph. They sit `route_corner_margin_m` (0.2 m) beyond
  the clearance away from every reflex area corner and every convex obstacle corner, as arc
  points with 30° spacing. Where that does not fit, they sit at the bare clearance. The
  margin is there because MPPI cuts corners: in the sim it came 0.25 m inside a bend point
  next to a drawn obstacle. This
  gives the shortest path inside the area shrunk by the clearance: it hugs the inset ring at
  concave corners and takes straight shortcuts elsewhere. Overlapping areas connect through
  the corner points they share.
- **Links.** Some points sit closer than the clearance to the edge: a path end drawn onto the
  boundary, or a coverage sub-path start on the outer ring. Such a point is joined to the
  nearest point that has the clearance (at most 1.5 m away) by a short link. Every 5 cm sample
  of the link must lie inside an area or path polygon. A start and goal in the same area also
  get a straight hop if it keeps the smaller of their own edge clearances.
- **Start and goal.** The robot pose snaps onto every path whose band contains it and into
  every area that contains it. A pose outside everything gets no route, and the mission falls
  back to Nav2, whose return corridor exists for that case.
- **Costs.** Among the valid end-to-end options the cheapest wins. Path metres cost 1
  (`path_weight`). In-area metres cost `route_area_weight`, 8 by default. Navigation-polygon metres cost `route_nav_area_weight`, 1 by default.

  The in-area weight was 3 with end-only portals and is now 8. A lawn metre is expensive, so
  the route rides a path to near the perpendicular foot of the hop to the next path:
  - the hop leaves the path about hop length / 8 before the foot;
  - on the original sim map it leaves Path 1 at x ≈ 7.5, the closest point that clears the
    drawn triangle (Path 2 starts at x = 8.59);
  - on the redrawn map it leaves at x ≈ 4.8 (Path 2 starts at x = 5.37).

  With a weight of 1 it leaves Path 1 almost at once
  (`test_sim_area_weight_one_cuts_across_lawn`).
- **Bends (optional, off by default).** A bend next to a path leg can be rounded with an
  arc of `route_fillet_radius_m`. The arc is used only where it stays inside the path polygon,
  or keeps the clearance in an area; otherwise the largest of 0.7, 0.5 or 0.3 times the radius
  that fits is used.

  It is off by default. In the sim, MPPI cut the rounded bends by as much again: on the
  redrawn map, with 1.0 m fillets, the robot centre was outside every polygon for 58 samples,
  by at most 0.27 m. Without fillets it was 14 samples, at most 0.09 m.
- **Output.** Poses every 0.1 m (`route_resample_m`), with the yaw along the route and the
  last yaw equal to the goal yaw. The output also gives a per-leg summary, the length, the
  metres on paths, and `single_area`: the mowing area index when every leg stays inside that
  one area, else -1.

nav2_route exists in the Humble image (`ComputeRoute`), but it was not used, for these reasons:

- It routes on a fixed GeoJSON node/edge graph and knows nothing about area polygons,
  clearance or obstacles. The in-area visibility legs, portals and links would have to be
  generated into that file anyway.
- It adds a lifecycle server and its route tracking (`ComputeAndTrackRoute`).
- It is not verified on the arm64 mower image.

The Python planner needs about 20 ms to build the graph and about 5 ms per query on these
maps, and it is fully unit-testable.

## Transit variation (2026-10-09)

Owner request: "options to randomize paths a little bit on an area, e.g. follow perimeter,
as driving the same line eventually creates tyre tracks".

Each area has a setting `transit_variation` (area settings, GUI area form):

| Value | Effect on each run |
|---|---|
| `none` | Always the base route. |
| `lanes` (default) | Path legs are shifted sideways inside the drawn band, cycling through 0, +a, -a, +a/2 and -a/2, where a = band half width - 0.29 - 0.03. In-area pieces of 2 m or more get a mid waypoint shifted sideways, cycling through 0, + and -, by up to 0.5 m or 15 % of the piece. |
| `perimeter` | In-area legs cycle through the direct leg and the inset-perimeter ring (clearance + 0.05 m inset), clockwise and counter-clockwise. A ring variant is used only if it is no longer than `perimeter_max_factor` (2) times the direct leg. |
| `mixed` | Run k % 3 picks the base route, lanes or perimeter. |

For paths, a = band half width - robot half width 0.29 - margin 0.03:
- A 0.7 m band gives 0.03 m, below the 0.05 m minimum, so it has no lanes.
- A 1.5 m band allows lanes of ±0.43 m.
- The offset tapers to zero over the first and last metre, so a path is still entered and
  left exactly at its ends and junctions.

Every variant keeps the hard rules:
- the robot centre stays in the band or keeps the clearance in the area;
- the route stays end to end;
- obstacles are kept clear;
- only the clearance core of an in-area leg is varied, never its links to a boundary
  crossing or a near-edge start or goal.

The variant is deterministic per run index; it is not re-drawn on a replan. The mission
increments `alternate_counts['__transit_run__']` (the same persisted file as the alternate
mow angles) on every mission start and passes it as `PlanRoute.variation`. Run 0 is the base
route. `route_variation: false` always sends 0. map_server reads each area's setting, logs
the variant used and returns it in `PlanRoute.variant`.

Sim on the v4 map (garden world without the scripted person, domain 96, default `lanes`;
both paths are 0.7 m bands, so they have no lanes):

| Run | Variant | Lawn hop x at y = -2.5 / -3.5 / -4.5 | Outside every polygon |
|---|---|---|---|
| 0 | base | 8.54 / 8.18 / 7.81 | 15 samples, max 0.09 m |
| 1 | lawn and area 1 shift +1 | 8.84 / 8.48 / 7.99 | 14 samples, max 0.10 m |
| 2 | lawn and area 1 shift -1 | 8.16 / 7.84 / 7.61 | 15 samples, max 0.10 m |

All outside samples are at Path 2's sharp bends (MPPI), not on the varied legs. Between runs
1 and 2 the tracks are on average 0.43 m apart on the lawn hop and 0.49 m apart in area 1.

Run 1 docked again in the same container, then ran mission 2. That mission crawled (7.5 m in
440 s) because the host load average was 17 to 30 from other work, so the stereo cloud went
stale. Run 2 was therefore repeated in a fresh container with the run counter file seeded to
1, which makes that start run 2.

## Service: `/map_server_node/plan_route` (`mower_interfaces/srv/PlanRoute`)

The request is `start` and `goal` (Pose2D, map frame), or `to_dock: true`. With `to_dock`,
the goal is the approach pose facing the dock (`route_dock_facing`, as mower_docking's
`approach_facing_dock`). The response is `success`, `message` (the failure reason, or the
leg summary such as `Path 1 8.9 m, lawn 5.7 m, Path 2 17.4 m, area 1 3.3 m`), `path`
(nav_msgs/Path), `single_area`, `length_m` and `path_length_m`.

The graph is rebuilt on the first query after the areas, paths, obstacles, dock or `route_*`
parameters change. The rebuild key is the areas.dat text plus the dock pose.

Each route is published latched on `/map_server_node/route`. It is also published on `/plan`
(`route_plan_topic`; `''` turns this off), which the GUI draws as the transit plan. The GUI
draws `/plan` while Nav2 plans, but a FollowPath goal publishes no `/plan`.
`/map_server_node/route` is in `FOXGLOVE_GUI_TOPICS`.

Parameters (map_server_node): `route_enabled`, `route_clearance_m`, `route_area_weight`,
`route_corner_margin_m`, `route_portal_step_m`, `route_fillet_radius_m`,
`route_nav_area_weight`, `route_resample_m`, `route_dock_facing` and `route_plan_topic`. The
dock leg reuses `approach_distance`, `corridor_half_width_m` and `dock_corridor_max_m`.

## Mission (`mower_mission`, `route_transits: true`)

`_send_transit` is used by the normal sub-path transit (`_dispatch`, which also serves the
end-of-area stretch and failed-sub-path retries), the blocked-wait re-send, the
dynamic-obstacle resume and the localization-hold resume.

1. It calls `plan_route(robot pose -> transit target)`.
2. On success it sends the route to `/follow_path` with `route_controller_id` (FollowPath =
   MPPI) and `route_goal_checker_id` (general_goal_checker). The action purpose is `route`,
   so it never mixes with the coverage follow.
3. On no route (off graph, no connection, service unavailable or timed out) the transit falls
   back to `/navigate_to_pose`, the old behaviour.
4. When the route follow aborts:
   - with a collision ahead (`_blocked_cause`), the existing blocked wait runs. Its re-send
     drives the route again up to `route_blocked_retries` (1) times per transit, and after
     that goes through `/navigate_to_pose`. In the sim the scripted person crossed Path 1;
     sending Nav2 straight away cut across the lawn into a drawn obstacle corner. A static
     unknown obstacle therefore costs one extra wait before Nav2 plans round its costmap
     marks;
   - otherwise it re-sends at once through `/navigate_to_pose` without consuming a transit
     retry. Those retries stay with the costmap planner, under the existing retry and
     back-off semantics.

   A route "success" more than `route_end_tolerance_m` (0.5 m) from the target counts as a
   failure. A route abort within `transit_arrived_radius_m` counts as arrived.
5. A dynamic-obstacle wait does not mark the transit as fallen back: the resume asks for the
   route again. A new dispatch, which includes a retry, tries the route again.

Detours round static obstacles and boundary-recovery transits always use
`/navigate_to_pose`. A detour needs the costmap planner, and a recovering robot is off the
map.

Blade: `_transit_blade` decides as before, from the straight robot-to-target segment inside
the area. When the blade was kept on (continuous policy) but the route's `single_area` is not
the current area, it is turned off before the route starts. The blade therefore runs only on
legs fully inside one mowing area, never on path legs.

The obstacle path filter (`_upcoming_path`) uses the route poses during a route follow
instead of the straight line to the target.

Return to dock (`route_dock: true`, with the docking server): `_dock` first asks
`plan_route(to_dock)` and drives the route to the approach pose (purpose `dock_route`). Then,
whatever the outcome, it starts the docking action. Within `approach_skip_radius_m` (0.6 m)
the docking server skips its own Nav2 leg; after a failed route it navigates there itself.
The route runs once per docking episode (`_dock_routed`), so a re-entry after a dynamic or
blocked wait goes straight to the docking server.

Set `route_transits: false` in `config/mission.yaml` to restore the Nav2-only behaviour.

## Sim verification (2026-10-09, garden world, owner-drawn paths, domain 96)

The test map was `test/data/sim_garden_paths.dat` with `keep_maps:=true`, started with
`start_in_area` area 1. Ground truth came from `/sim/status`.

- Before this change the transit left Path 1 at x ≈ 6.8 and crossed the lawn.
- With the first version (weight 3, portals only at path ends): transit to area 1, route `Path 1 8.9 m, lawn 5.7 m, Path 2 17.4 m, area 1 1.4 m` (33.3 m),
  with no fallback. TRANSIT to MOWING took 142 s.
  - It followed Path 1 to its end: at most 0.23 m off the centreline, 0.16 m from the end
    point.
  - It crossed the lawn round the drawn triangle. The robot centre came within 0.19 m of the
    triangle, because MPPI cuts the bend; the bend points sit at 0.55 m.
  - It reached 0.29 m from Path 2's start and followed Path 2: 0.20 m mean and 0.48 m max off
    the centreline.
  - The centre was outside every polygon for 2.5 s, by at most 0.13 m. That was at Path 2's
    112° bend at (8.11, -13.09): MPPI cuts acute bends, since the robot turns at no more than
    0.3 rad/s.
- An earlier run, before `route_blocked_retries`, hit the scripted person on Path 1. The
  Nav2 fallback then left the robot on the drawn triangle's edge, where every later route
  was "off the route graph". A start pose may now link out of an obstacle polygon.
- HOME from area 1 used the dock route `area 1 7.7 m, Path 2 17.4 m, lawn 5.8 m, Path 1
  8.9 m` (40 m). It was outside every polygon for 0.6 s, at the same Path 2 bend. The docking
  server logged "0.20 m from the approach pose: skipping Nav2", and the robot docked and
  charged.

### Redrawn map (2026-10-09, `test/data/sim_garden_paths_v2.dat`, mid-path portals)

The route was `Path 1 4.0 m, lawn 4.6 m, Path 2 16.2 m, area 1 9.2 m` (33.9 m). Before mid-path
portals it was `lawn 6.6 m, Path 2 16.2 m, area 1 9.2 m`.

Sim ground truth, final settings (no fillets):
- TRANSIT took 135 s.
- The robot was on Path 1's band for 18 s, from the dock approach to (4.58, -0.75), where it
  left Path 1.
- It joined Path 2 at (5.76, -4.57), near Path 2's start at (5.37, -4.85), and stayed on
  Path 2's band for 63 s, into area 1.
- It was outside every polygon for 14 samples, by at most 0.09 m, at Path 2's 96° bend.

Graph build and plan times:

| Map | Build | Plan | Nodes / edges |
|---|---|---|---|
| Original sim map, offline | 25 ms | 5 ms | 65 / 751 |
| Redrawn sim map, offline | 57 ms | 11 ms | 116 / 3661 |
| Redrawn sim map, in the sim | 88 ms | 27 ms | 116 / 3661 |
| Field map, offline | 21 ms | 7 ms | 12 / 17 |

On the original map with mid-path portals and 1.0 m fillets, the robot left Path 1 at
x = 7.58. It was outside every polygon for 36 samples, by at most 0.17 m. Before that change
it was 25 samples, at most 0.13 m.

### "Dock path" on top of Path 1 (v3 map, 2026-10-09)

The route was `Path 1 4.0 m, lawn 4.6 m, Path 2 16.2 m, area 1 9.2 m`. The graph had 128 nodes
and 3712 edges; the node built it in 116 ms and planned in 20 ms.

Sim ground truth:
- TRANSIT took 135 s.
- The robot rode Path 1 (inside Dock path's band too) for 18 s and left it at (4.57, -0.76).
- It joined Path 2 at (5.76, -4.57) and stayed on it for 63 s, into area 1.
- It was outside every polygon for 14 samples, by at most 0.095 m, at Path 2's bend.

A map_server_node that had been started before a rebuild kept planning with the old code: it
logged 26 nodes against 116 offline, and that run ignored the paths. The `route graph:` log
line now prints the area weight, portal step and overlap step, so a stale process is easy to
spot. `test_plan_route_node` checks that the service plans exactly the pure function's route.

### End to end (v4 map, 2026-10-09)

The owner's map has two paths, both named "Path 2"; the first starts at the dock. The route
was `Path 2 8.4 m, lawn 6.1 m, Path 2 14.0 m, area 1 9.2 m`. It rides the first path to its
end (9.18, -0.29), hops the lawn, joins the second path where it crosses the lawn boundary
(a junction with an area), follows it, then enters area 1.

The route is sent as FollowPath legs, split at turns sharper than `route_split_turn_deg`
(100°). Here that is the 128° hairpin at the path end.

Sim on the garden world with the scripted person removed (the person walks x = 9, right
across that path end, and made MPPI fail there twice):
- No fallback; TRANSIT took 161 s.
- It left the first path 0.11 m from its end.
- It joined Path 2 0.43 m from its lawn crossing.
- It was outside every polygon for 15 samples, by at most 0.09 m, at Path 2's bends.

## Limitations

- Unknown obstacles on the route are handled only through the fallback: MPPI aborts, then
  Nav2 plans round them. The graph sees only obstacle polygons stored in areas.dat.
- Two areas that touch but do not overlap, with no path between them, are not connected.
  Their transits fall back to Nav2.
- A narrow neck of an area (narrower than twice the clearance) disconnects the area for the
  graph, and the transit falls back to Nav2.
- MPPI (FollowPath) cuts acute path bends by up to about 0.13 m beyond the band and about
  0.25 m round in-area bend points. Possible fixes are controller tuning (PathFollow /
  PathAlign weights) or an RPP route controller (`route_controller_id`). Neither was compared
  in the sim.
- The route is not chunked like coverage follows (`follow_chunk_m`). A route that passes
  close to its own end could let the stock goal checker fire early. That case is caught as a
  "success far from the target" and re-sent through Nav2.
