# Watching the simulator in the MowgliNext GUI

This guide runs the headless simulator (`mower_sim`, see [simulation.md](simulation.md)) with
the MowgliNext web GUI on an amd64 dev box. You can then watch the coverage plan, the
transit plan, obstacle detours, mow progress and the mission state live at
http://localhost:4006. The helper script is `scripts/sim_gui.sh`.

> **Keep the sim off the robot's DDS graph.** Every ROS process here must run with
> `ROS_LOCALHOST_ONLY=1` and a `ROS_DOMAIN_ID` other than 0. With host networking and
> domain 0, dev-box nodes join the mower's graph over the LAN. On 2026-10-07 a dev-host
> test feeder's `/odometry/filtered_map` latched a lethal boundary stop on the robot.
> `sim_gui.sh` enforces both settings (domain 90 by default; it refuses 0). Do not start
> the sim with a bare `ros2 launch` from a shell that has no domain set. Nothing in this
> guide talks to the mower.

## How it fits together

```
sim container (ROS_DOMAIN_ID=90, ROS_LOCALHOST_ONLY=1, --network host)
  sim.launch.py world:=... foxglove:=true teleop_relay:=true maps_dir:=/tmp/mower_sim_gui.90/maps
    foxglove_bridge :8765 (all topics) ─┐
    cmd_vel_ws_relay :8766 ────────────┤
GUI container (mower-gui:amd64, --network host)
    :4006 ── FOXGLOVE_URL=ws://localhost:8765
    MOWER_YAML_CONFIG_FILE = <maps_dir>/mowgli_robot.yaml   (not config/gui)
```

`config/gui/mowgli_robot.yaml` belongs to the device, and the sim never reads or writes it.
The script copies `config/gui/mowgli_robot.yaml.seed` into the scratch `maps_dir`. On
start, gui_bridge rewrites that copy's `datum_lat`/`datum_lon` to the world's datum
(48.0, 9.0 for every bundled world). The GUI reads the datum from that copy, so the map
lines up. map_server reads `areas.dat`/`dock_pose.yaml` from the same directory.

## Prerequisites (once)

- Docker, and the dev image `mower:humble-dev-amd64`. `dev_build.sh` builds the image
  if it is missing.
- Our GUI built for amd64 from `third_party/mowglinext/gui`: `./scripts/sim_gui.sh build-gui`
  (= `ARCH=amd64 ./gui/build.sh`, tag `mower-gui:amd64`, a few minutes). Rebuild it after any
  GUI change; `up` warns when the source is newer than the image. Do NOT use an old
  `mower-gui:latest` (2026-10-09: a 3-day-old build without the drawn-path tools was picked up).
  `mower-gui:arm64` is the robot image and does not run here.
- Internet access in the browser. The GUI map uses Mapbox tiles; the robot overlays
  still draw without them.

## Build

```bash
cd ros2_stack
# --packages-up-to mower_sim mower_bringup does NOT cover every package sim.launch.py
# includes (Nav2 config, coverage, docking, mission, gui_bridge). Build them all:
DEV_ROS_DOMAIN_ID=90 ./scripts/dev_build.sh build --packages-up-to mower_sim mower_bringup \
  mower_navigation mower_map mower_coverage_bridge mower_docking mower_mission \
  mower_gui_bridge mower_teleop
```

If the launch fails with `No module named ...`, a package was changed but not rebuilt.
Re-run the build above, or a full `./scripts/dev_build.sh build`.

## Start sim + GUI

```bash
./scripts/sim_gui.sh up                 # world garden, full mission, robot docked
./scripts/sim_gui.sh up transit_box mission:=false mission_stub:=true   # extra args go to sim.launch.py
./scripts/sim_gui.sh status
```

Open **http://localhost:4006**. The GUI log should show `foxglove: connected` and
`cmd_vel relay: connected` (`docker logs mower_sim_gui_gui`). Two log warnings are
expected and harmless: `Cannot connect to the Docker daemon` (the socket is not mounted,
so "Restart ROS 2" and container logs are off) and GPS sidecar warnings.

Worlds (`src/mower_sim/worlds/`):

| World | What it shows |
|---|---|
| `garden` | 20 x 12 m lawn. Dock at the origin. An unknown box at (6, 0.3) and a tree at (12, -3). A known flower bed, which is in the map. A scripted person walking x = 9, y ±5 from t = 30 s. |
| `transit_box` | Straight transit past a 0.6 m box at x = 4.8. The obstacle-reroute case. |
| `dock_approach` | Robot 1.2 m out of the dock; for `scenario dock`. |

Script overrides (environment): `SIM_ROS_DOMAIN_ID` (default 90), `SIM_GUI_DIR` (default
`/tmp/mower_sim_gui.<domain>`), `GUI_IMAGE`, `SIM_IMAGE`. The script runs one sim/GUI pair
at a time because ports 4006, 8765 and 8766 are fixed.

## Start a mow

In the GUI, press **Start** on the dashboard or map page. From a shell, the
equivalent is:

```bash
./scripts/sim_gui.sh ros2 service call /behavior_tree_node/high_level_control \
  mowgli_interfaces/srv/HighLevelControl '{command: 1}'      # 1 START, 2 HOME, 8 STOP
# or through the GUI backend:
curl -X POST -H 'Content-Type: application/json' localhost:4006/api/mowglinext/call/high_level_control -d '{"command":1}'
```

The `ros2 service call` can keep blocking after the mission has accepted the command.
Check the GUI and Ctrl-C the call.

The mission runs UNDOCKING, WAITING_FOR_RTK, PLANNING, TRANSIT and then MOWING (blade on).
On the garden world, MOWING starts about 2 minutes after START.

## What you see

- **Coverage plan** (`/coverage/full_plan`, the GUI's `path` layer). The full
  Fields2Cover plan: headland plus 32 swaths on `garden`. It appears once PLANNING is
  done.
- **Transit / Nav2 plan** (`/plan`). Shown only while Nav2 is navigating, because it is
  not latched. The **obstacle detour** shows up here: the unknown box is not in the map,
  so the plan bends round it once the synthetic stereo marks it in the global costmap.
- **Robot pose** (`/gui/pose`), mission state and sub-state (`highLevelStatus`, e.g.
  `MOWING area 0 sub-path 1/32`), battery and charging, and GNSS (always RTK FIXED).
- **Mow progress** (`/map_server_node/mow_progress`). The mowed-area overlay grows behind
  the robot.
- **Lawn and dock** from `areas.dat`/`dock_pose.yaml`, through map_server services.

The GUI does **not** draw the costmaps, the world's obstacles (`/sim/markers`) or the
stereo points. It has no costmap layer, and its obstacle layer reads
`/obstacle_tracker/obstacles`, which the sim does not run. To see them, connect
**Foxglove Studio** to `ws://localhost:8765`. In the sim, foxglove_bridge serves all
topics, not the robot's `FOXGLOVE_GUI_TOPICS` whitelist. Add a 3D panel with:

- `/global_costmap/costmap` and `/local_costmap/costmap`;
- `/sim/markers` (world obstacles, including the moving person);
- `/stereo_depth/points`;
- `/plan`, `/coverage/full_plan` and `/tf`.

Ground truth: `./scripts/sim_gui.sh ros2 topic echo /sim/status --field data`
(clearance, collisions, cutting, docked).

## Obstacles and the scripted person

Obstacles are fixed per run: the sim has no service to add or move them live. Copy a
world, edit it and start from the copy. The repo is mounted at `/work` in the sim
container, so a path inside the repo works:

```bash
cp src/mower_sim/worlds/garden.yaml src/mower_sim/worlds/my_garden.yaml   # or anywhere under ros2_stack/
# edit obstacles: (box | cylinder); known: true puts one into the map (areas.dat),
# otherwise only stereo/bumper find it. Moving obstacle:
#   - {name: person, type: cylinder, radius: 0.25, height: 1.7, speed: 0.6,
#      mode: pingpong, start_delay: 30.0, path: [[9, 5], [9, -5]]}   # mode: loop | once | pingpong
./scripts/sim_gui.sh down
./scripts/sim_gui.sh up /work/src/mower_sim/worlds/my_garden.yaml
```

`sim.launch.py` accepts any world path. The schema is in
[simulation.md, World YAML](simulation.md#world-yaml). Keep the datum at 48.0, 9.0 unless
you have a reason to change it; the GUI follows whatever the world sets.

## Scenario runner

The scenario client runs inside the sim container, on the same domain. Use it with
`mission:=false mission_stub:=true`:

```bash
./scripts/sim_gui.sh up transit_box mission:=false mission_stub:=true
./scripts/sim_gui.sh ros2 run mower_sim scenario navigate --x 8.0 --y 0.0   # JSON line, exit 0 = pass

./scripts/sim_gui.sh down; ./scripts/sim_gui.sh up dock_approach mission:=false mission_stub:=true
./scripts/sim_gui.sh ros2 run mower_sim scenario dock
```

While `navigate` runs, the GUI shows the robot and the `/plan` detour round the box.

## Teardown

```bash
./scripts/sim_gui.sh down       # stops both containers (they are --rm); frees 4006/8765/8766
```

The scratch dir `/tmp/mower_sim_gui.90` holds root-owned files. The next `up` wipes it.
The GUI database in `db/` is throwaway.

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `foxglove_bridge did not come up` | The launch died. Run `docker logs mower_sim_gui_sim` (the container is `--rm`, so run `up` again and read the log quickly), or run the `docker run ... ros2 launch` line from the script in the foreground. The usual cause is a stale build (`No module named ...`): rebuild. |
| `port 4006/8765/8766 already in use` | Another sim, an old `mower_gui` container, or Foxglove on this box. Run `./scripts/sim_gui.sh down` and `docker ps`. |
| GUI loads but shows no pose or state | Check `docker logs mower_sim_gui_gui` for `foxglove: connected`. The GUI and sim must both use host networking. Check that gui_bridge is running: `/gui/pose` comes from gui_bridge, not the sim. |
| `ros2 topic list` from your own shell shows nothing | You are on a different domain, or without `ROS_LOCALHOST_ONLY=1`. Use `./scripts/sim_gui.sh ros2 ...` (runs inside the sim container), or export `ROS_DOMAIN_ID=90 ROS_LOCALHOST_ONLY=1` in a dev-image shell with `--network host`. Never use domain 0. |
| Robot and lawn drawn in the wrong place, or the map is empty | Wrong datum. `curl -s localhost:4006/api/settings/yaml` must show `datum_lat 48`, `datum_lon 9`. The GUI has to read `<SIM_GUI_DIR>/maps/mowgli_robot.yaml`; gui_bridge only rewrites a datum that already has `datum_lat/datum_lon` keys, so the seed copy must exist before launch (the script does this). Never point the GUI at `config/gui/`. |
| No `/plan` in the GUI | Normal outside TRANSIT or navigation; `/plan` is not latched. |
| Mission stuck in WAITING_FOR_RTK or preflight with `localization:=ekf` | heading_aligner starts unaligned in ekf mode (see simulation.md, Limitations). Use the default `localization:=sim`. |
| GUI robot profile shows YardForce500 | The seed yaml has no `mower_model`, so the settings schema default shows in `/api/settings/yaml`. It is cosmetic for the sim. |

## Verified

Verified on 2026-10-09 on the amd64 dev box, with `mower-gui:latest` (amd64) and
domain 90:

- **Build:** the package set above.
- **`sim_gui.sh up garden`:** the GUI connects to foxglove (8765) and the relay (8766).
  `/api/settings/yaml` reports datum 48/9.
- **GUI websocket streams** (`/api/mowglinext/subscribe/<topic>`) delivered `pose`,
  `highLevelStatus`, `status`, `power`, `gps`, `gnssStatus`, `emergency`, `map` (lawn
  from map_server), `path` (the coverage plan), `plan` (during transit) and
  `mowProgress` (non-zero cells while mowing).
- **Mow started through the GUI API** (`/api/mowglinext/call/high_level_control`): the
  mission ran through TRANSIT to MOWING with the blade on and no collisions. The minimum
  clearance was 0.37 m, past the unknown box.
- **`scenario navigate --x 8 --y 0` on `transit_box` through the script:** passed
  (SUCCEEDED in 32 s, |y| up to 0.88 m round the box, clearance 0.19 m).
- **`sim_gui.sh ros2 ...`:** topic echo and service call both work.

Not verified:

- the browser rendering itself (map layers, buttons): only the backend streams were
  checked;
- the Foxglove Studio costmap view;
- `dock_approach` through the script;
- `missionPlan` (`/behavior_tree_node/mow_plan`), which is not published in this stack,
  so the GUI closes that subscription.
