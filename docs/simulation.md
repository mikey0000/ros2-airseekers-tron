# Simulation (mower_sim)

A headless kinematic simulator for testing obstacle rerouting, the docking approach
and mission logic without the mower. One rclpy node, `mower_sim/sim_node`, stands in
for the hardware drivers. Everything above the drivers is the real stack, run
unmodified:

- twist_mux, cmd_vel_slew (including its motion gate and turn shaper), slip_detector
  and bumper_controller;
- Nav2, the mower_map map server, coverage and mower_docking;
- the mission (`/behavior_tree_node`) and gui_bridge.

The sim is fast enough for CI: the transit regression test runs in about 30 s inside
the dev image.

Gazebo was not used. The control and perception problems found so far are about the
MCU drive limits, costmap marking and clearing, and the mission and docking logic, not
about contact dynamics. A 2.5D raycast and a diff-drive model with the real limits
cover these. They need no new image, no GPU and no X.

## Running it

Everything runs in the dev image and needs no extra packages. Give each run its own
DDS domain with `--network host` and localhost-only discovery
(`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`; `ROS_LOCALHOST_ONLY=1` is deprecated on
Jazzy, set both), so two runs, or a run and the robot, never share a graph.

`sim.launch.py`, `sim_node` and `scenario` enforce this (`mower_sim/isolation.py`,
review B6). They refuse to start when `ROS_DOMAIN_ID` is unset or 0, when discovery
is not localhost-only (or `ROS_STATIC_PEERS` is set), or on the mower itself (`/userdata` or `/userdata/ros2` is a mount point, or
`/userdata/ros2_stack` or `/userdata/mower` exists: the robot's data partition and
its container bind mounts, which dev hosts and the dev image do not have). The robot runs
domain 0 with LAN discovery: a bare launch there would add a second twist_mux,
Nav2 and mission, fake odometry, GPS, TF and e-stop, and (with `--stub-state`) a
latched `/motion_enabled=true` to the live robot. Overrides, each with a loud
warning: `MOWER_SIM_ALLOW_LAN=1` (skip the domain/discovery checks for a deliberate
LAN sim on a private domain) and `MOWER_SIM_ALLOW_ON_ROBOT=1`.

```bash
export DEV_ROS_DOMAIN_ID=87
./scripts/dev_build.sh build --packages-up-to mower_sim mower_bringup

# The full stack on the demo garden: robot on the dock, mission idle.
docker run --rm -it --network host -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID=142 \
  -v $PWD:/work -w /work mower:jazzy-dev-amd64 bash -c \
  "source install/setup.bash; ros2 launch mower_bringup sim.launch.py world:=garden"

# Second shell, same ROS_DOMAIN_ID: start a mow (undock -> RTK -> plan -> transit -> mow).
ros2 service call /behavior_tree_node/high_level_control \
  mowgli_interfaces/srv/HighLevelControl '{command: 1}'
ros2 topic echo /sim/status --field data       # ground truth, collisions, dock contact
```

To watch it in the MowgliNext GUI or in Foxglove Studio, add `foxglove:=true`
(port 8765) and `teleop_relay:=true` (joystick, port 8766). The world's obstacles are
published on `/sim/markers` (MarkerArray in `map`).

Step-by-step GUI setup (sim plus GUI container, datum, mission, scenarios):
[simulation_gui.md](simulation_gui.md), via `./scripts/sim_gui.sh up [world]`.

### Scenarios

`ros2 run mower_sim scenario` drives one action against a running sim. It judges the
run from ground truth, prints one JSON line of metrics, and exits 0 when the run
passes.

```bash
# Every shell (sim and scenario) on the same private, localhost-only domain:
export ROS_DOMAIN_ID=142 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_LOCALHOST_ONLY=1

# Obstacle reroute: NavigateToPose past a box that is not in the map.
ros2 launch mower_bringup sim.launch.py world:=transit_box mission:=false mission_stub:=true
ros2 run mower_sim scenario navigate --x 8.0 --y 0.0

# Docking approach: /mower_docking/dock from 1.2 m out (vision on the rendered marker).
ros2 launch mower_bringup sim.launch.py world:=dock_approach mission:=false mission_stub:=true
ros2 run mower_sim scenario dock
```

`navigate` passes when all of the following hold:

- the Nav2 result is SUCCEEDED;
- the robot ends within 0.5 m of the goal;
- there are no collisions;
- the robot never moves less than 5 cm in more than 20 s.

`dock` passes when the action succeeds and the sim reports both dock contact and
charging.

cmd_vel_slew's motion gate only passes wheel commands while the mission is in a motion
phase and keeps re-asserting `/motion_enabled`. A bare Nav2 or docking goal therefore
needs a stand-in for the mission. The scenario client provides one through
`--stub-state` (default TRANSIT or RETURNING_HOME): it publishes
`/behavior_tree_node/high_level_status` and `/motion_enabled`. Launch the sim with
`mission:=false mission_stub:=true` so that gui_bridge does not serve its own stub
high-level state as well. With the real mission running, drive it through its
services instead (`--stub-state ''` disables the stand-in).

### Regression test

`src/mower_sim/test/test_scenario_transit_box.py` is a launch_testing test and part of
`colcon test`. It is the regression test for the 2026-10-09 obstacle-reroute work:
stereo obstacles in the global costmap, and BTs that wait and replan.

```bash
./scripts/dev_build.sh test --packages-select mower_sim     # 75 tests, ~35 s
```

The test asserts all of the following:

- the robot reaches (8, 0) from (1.5, 0);
- it never touches the 0.6 m box at x = 4.8;
- it actually goes round the box (|y| > 0.4 m);
- its clearance stays above 5 cm;
- it never stalls for more than 20 s.

The test picks its own DDS domain (`MOWER_SIM_ROS_DOMAIN_ID`, default
150 + pid % 50). The test discriminates: with `stereo_costmap:=false` (bumper-only
costmaps) the same run drives into the box twice and Nav2 aborts.

Measured 2026-10-09:

- localization:=sim: SUCCEEDED in 24 s, peak lateral deviation 0.97 m, minimum
  clearance 0.31 m, longest stall 1 s.
- localization:=ekf: SUCCEEDED in 23 s, deviation 0.86 m, clearance 0.27 m.

## Launch file: `launch/sim.launch.py`

The launch file is installed by mower_bringup next to `mower.launch.py`.

| Argument | Default | Meaning |
|---|---|---|
| `world` | `garden.yaml` | World YAML: a path, or a name under `share/mower_sim/worlds/` (`garden`, `transit_box`, `dock_approach`). |
| `localization` | `sim` | `sim`: the sim publishes ground-truth `odom -> base_link`, `/odometry/filtered` and an aligned `/heading_aligner/status` (deterministic). `ekf`: the real gps_gate, navsat_transform, heading_aligner and ekf_node (`launch/nav2.launch.py`) fuse the sim's `/odom`, `/imu/data` and `/fix`. |
| `maps_dir` | `""` (fresh temp dir) | Where the world's lawn is written as `areas.dat` (area 0, plus `known: true` obstacles) and the dock as `dock_pose.yaml` (measured). gui_bridge's robot yaml and datum.env go there too; nothing is written into `config/gui`. |
| `navigation`, `map_server`, `coverage`, `docking`, `mission`, `gui_bridge` | `true` | Each switches the real component on or off. |
| `mission_stub` | `false` | A scenario client stands in for the mission. gui_bridge then serves no stub high-level state. |
| `stereo_costmap` | `true` | `false` removes the stereo sources from both costmaps (as in `mower.launch.py`). |
| `nav2_params_file`, `docking_params_file`, `mission_params_file` | package defaults | Parameter files (a full copy with the values changed). |
| `stereo`, `stereo_noise_m` | `true`, `0.0` | Synthetic stereo clouds and their range noise. |
| `min_wheel_speed_mps` | `0.06` | The MCU wheel floor while turning; 0 turns it off. |
| `rear_marker_size` | `0.04` | Printed size of the rendered dock ArUco. It must equal mower_docking's `marker_size`. |
| `foxglove`, `teleop_relay` | `false` | Bridges that bind ports. |

The launch file does not start the drivers, cameras, perception, det_range,
base_keys, lights, supervisor or mow_recorder. It starts robot_state_publisher, the
static `map -> odom` transform, twist_mux (lanes from
`mower_teleop/config/twist_mux.yaml`), cmd_vel_slew (`robot_settings_file:=''`, so
the device-owned GUI settings are not read), slip_detector and bumper_controller,
with the same parameters as bringup.

Because there is no `det_ros` in sim there is no annotated camera image, so the
`obstacle_tracker` produces no per-obstacle snapshot JPEGs (`snapshot_topic` has
no publisher); it still clusters/tracks/promotes from `/stereo_depth/points_map`
as usual. Unrelated to obstacles, the GUI's tracked-obstacle panel then shows the
marker icon instead of a thumbnail.

## World YAML

A world file has this shape:

```yaml
name: garden
datum: {lat: 48.0, lon: 9.0}     # GPS anchor of the map origin (x east, y north)
lawn: [[-1, -6], [19, -6], [19, 6], [-1, 6]]   # mowing area, map metres
areas: [{name: path, polygon: [...], is_navigation: true}]   # optional
dock: {x: 0, y: 0, yaw: 0}       # base_link pose on the charger (robot faces away)
start: {docked: true}            # or {x, y, yaw}
obstacles:
  - {name: box, type: box, x: 6, y: 0.3, yaw: 0.3, size_x: 0.6, size_y: 0.6, height: 0.5}
  - {name: tree, type: cylinder, x: 12, y: -3, radius: 0.3, height: 2.5}
  - {name: bed, type: box, x: 15, y: 3.5, size_x: 2, size_y: 1, height: 0.3, known: true}
  - {name: person, type: cylinder, radius: 0.25, height: 1.7, speed: 0.6,
     mode: pingpong, start_delay: 30, path: [[9, 5], [9, -5]]}   # loop | once too
battery: {voltage: 24.6, percentage: 0.85}
```

A world can instead take its map from a copy of a real `areas.dat`
(`areas_file`, relative to the world YAML; `areas_select: [Area 1, Dock path, ...]`
keeps those areas only; `lawn_area` picks the mowing area that becomes the lawn,
default the first one). The other selected areas follow with their drawn-path
channels; the file's map obstacles stay map-only. The world's `datum` replaces the
file's (the polygons are map metres, so the geometry is unchanged). Pair it with the
real `dock` pose and `start: {docked: true}`.

The map frame is the vendor frame: the origin is at the dock and x points out of it.
Obstacles are 2.5D: a 2D shape extruded up to `height`. Only `known: true` static
obstacles go into areas.dat. All the others have to be found by the stereo or the
bumper. The datum longitude 9.0 lies on a UTM central meridian, so in
`localization:=ekf` navsat_transform's UTM grid is not rotated against the sim's
local tangent plane (no meridian convergence).

## What the sim reproduces

### Drive (`mower_sim/kinematics.py`)

The drive model reproduces these behaviours:

- **Speed clamps:** linear and angular are clamped separately to ±0.3 m/s and
  ±0.3 rad/s, as in mcu_node `_map_command`. The clamp is not curvature-preserving.
- **Wheel floor:** while turning, a wheel asked for less than 0.06 m/s does not move.
  This is the floor described in the nav2_params comments ("wheel 0.04 m/s, under the
  MCU's ~0.06 m/s floor ... never broke free") and the reason for the cmd_vel_slew
  turn shaper's "inner >= 0.06". The floor applies only while turning: the vendor's
  straight 0.05 m/s final dock reverse works on hardware, so straight driving is not
  floored. `DriveParams.floor_straight` floors that too.
- **Command handling:** `cmd_vel_timeout` is 0.5 s. The wheels follow the command
  with a first-order lag (`wheel_tau_s` 0.15). Integration uses real elapsed time, so
  a late timer does not slow the robot down.
- **Collisions:** a step whose footprint would touch an obstacle is not taken; the
  robot stops dead. Each contact is counted once. If an obstacle walks into the robot,
  the robot may only move away from it.
- **Dock:** the dock is a hard stop when reversing past the charge point while
  roughly lined up with the dock (lateral ≤ 0.25 m). Contact (`is_docking_done`)
  needs |x| ≤ 3 cm, |y| ≤ 6 cm and |yaw| ≤ 0.2 rad in the dock frame.

### Driver topics and services (`mower_sim/sim_node.py`)

| Replaces | Interface |
|---|---|
| mcu_node | Subscribes `/cmd_vel` and `/estop_request` (latches; cleared by `/clear_estop`). Publishes `/odom` 50 Hz, `/mower_base/status` 20 Hz, `/mower_sensor_info` and `/estop` 10 Hz, `/battery` 1 Hz, and latched `/rain` and `/cutter/height_mm`. Serves `/cutter_control`, `/charging`, `/clear_estop`, `/cutter_off` and `/cutter/set_height`. `is_charging` = dock contact AND a prior `/charging true`, as on the robot. Bumper flags come from footprint contact ahead of x = 0.15 m, left or right by side, held 0.3 s. bumper_controller turns them into `/bumper_cloud` and `/cmd_vel_bumper` as usual. |
| wit_node | `/imu/data` 50 Hz in `imu_link`, with the real driver's covariances. Yaw is the true heading. |
| um960_node | `/fix` 10 Hz in `gps_link`: status GBAS_FIX (RTK), σ 1.5 cm. `/fix_status`: `quality=RTK_FIXED solution=NARROW_INT`. `/vel`. |
| stereo_depth | Raycasts a 64x32 grid over 80° x 50° from `stereo_camera_optical` (URDF pose). `/stereo_depth/points` carries obstacle hits at 0.2-4 m. `/stereo_depth/clear_points` carries ground hits up to 3 m, one per 10 cm cell. Both are xyz float32 (point_step 12), use sensor QoS, run at 5 Hz, share a stamp, and are published only while subscribed. Frames differ from the robot: the real stereo_depth_node (`fixed_frame_clouds`, default since 2026-10-09) publishes `/points` and `/clear_points` in `odom`, stamped at the odom->base_link sample it used; the sim still publishes them in `stereo_camera_optical` stamped now() (the local costmap transforms them; the sim's TF is synchronous, so the robot's tf2 wait problem does not show here). `/stereo_depth/points_map` + `/clear_points_map` carry the same clouds in `map`, transformed with the ground-truth pose (optimistic under `localization:=ekf`). |
| rear camera | `/rear_camera/image_raw` (mono8, 640x480) and `camera_info` at 10 Hz while subscribed. The intrinsics come from `config/cameras/rear_camera_info_640x480.yaml`, without distortion. The dock's ArUco (DICT_4X4_50, id 0) is rendered with a pinhole model 0.45 m behind the dock pose (docking.yaml `docked_marker_offset`), so mower_docking runs its real detector. |
| localization (sim mode) | TF `odom -> base_link` and `/odometry/filtered` at 50 Hz. `/heading_aligner/status` latched `{aligned: true, source: dock}`. gui_bridge still derives `/odometry/filtered_map`, `/gps/status` and `/hardware_bridge/*` from these. |
| (sim only) | `/sim/ground_truth` (Odometry in `map`), `/sim/status` (JSON 10 Hz: pose, twist, clearance, min_clearance, blocked, collisions, last_contact, bumper, docked, charging, cutting, estop, battery), `/sim/markers`. |

## Findings from the first runs (2026-10-09)

- **Obstacle reroute:** works in both localization modes (numbers above). It is
  covered by the regression test.
- **Full mission** (garden world, HighLevelControl START): UNDOCKING, then
  WAITING_FOR_RTK, then PLANNING, then TRANSIT (around the unknown box), then MOWING
  with the blade on. No collisions.
- **Docking from the transit_box start** (1.5 m out, facing away from the dock) with
  the configured 4 cm marker fails with DOCK_MAXOUT. The vision chain works: approach,
  ALIGNING, SEARCHING and DOCKING are all reached. It fails for two reasons:
  - **Noisy marker yaw:** solvePnP's yaw estimate of a 4 cm marker is noisy, ±25° at
    about 2 m (exact with a large marker; see `test_marker.py`).
  - **Field of view:** the rear camera's 640x480 FOV is about ±25°. The reverse
    controller's desired approach angle reaches 0.5 rad (28.6°) at
    `k_lateral 7 x lateral`. Above about 17° the marker leaves the image at 0.7 m
    ("marker not visible" then RETRY).

  From 1.2 m out (`dock_approach`) the robot docks and charges after one retry. These
  belong to the rear-dock work. The sim can now reproduce them.
- **Marker size:** `config/cameras/rear_camera_info_640x480.yaml` measures the docked
  marker's edges at x 102 to 496 px, about 0.14 m at the 0.25 m camera distance. That
  is much larger than docking.yaml's `marker_size: 0.04`. Worth checking on the robot.
  `rear_marker_size` and `docking_params_file` let both be varied in the sim.
- **Bumper-only costmaps** (`stereo_costmap:=false`): the robot hits the box,
  bumper_controller backs off, it hits again, and Nav2 aborts.
- **Launch-configuration trap (fixed in sim.launch.py):** launch configurations are
  global. `mower_map/map_server.launch.py` declares `base_frame:=base_footprint`, so
  a `nav2.launch.py` included after it gets an EKF publishing `odom ->
  base_footprint`, which splits the TF tree. `mower.launch.py` is safe only because
  it includes localization first.

## Sim gate: `scripts/sim_gate.sh` (every change, before the field)

One command runs a full mow in the sim and judges it. A change that fails the gate does not
go to the field (owner, 2026-10-15; design: `DESIGN_path_conditioning_2026-10.md` section 4).

```bash
./scripts/dev_build.sh build                    # once per checkout (symlink install)
./scripts/sim_gate.sh                           # ideal drive
./scripts/sim_gate.sh -d                        # with the ground disturbance
./scripts/sim_gate.sh -d -s 2 -n mytest -- mission_params_file:=/work/path/mission.yaml
```

What it does:

1. It starts `sim.launch.py` in the dev image (container `sim<domain>-gate-<name>`, DDS domain
   `MOWER_SIM_ROS_DOMAIN_ID`, default 175, never 0, localhost-only; it refuses
   `MOWER_SIM_ALLOW_LAN` / `MOWER_SIM_ALLOW_ON_ROBOT`). The default world is
   `src/mower_sim/worlds/real_run5.yaml`: a copy of the real `areas.dat` as of field run 5
   (Area 1 plus the three drawn paths, the GPS datum dropped) with the dock at run 5's
   docked EKF pose. Every parameter is the committed one.
2. It waits for the mission to be idle on the dock, starts a bag recorder inside the sim
   container (Fast DDS shared memory), then sends START: undock, transit, every sub-path of
   Area 1, return, dock.
3. It stops at IDLE_DOCKED / CHARGING after the mission ran, at STUCK_NEEDS_HELP, EMERGENCY,
   NAV_TO_DOCK_FAILED, UNDOCK_FAILED or MOWING_INCOMPLETE, or after `-t` seconds (default
   2700).
4. It copies the bag, dumps it (`scripts/sim_gate/extract.py`) and judges it
   (`scripts/sim_gate/evaluate.py`), all in containers. Output in `log/sim_gate/<name>/`
   (`SIM_GATE_OUT`): `verdict.txt`, `result.json`, `mission.log`, `launch.log`, the bag.
   Exit code 0 only on PASS.

Criteria, judged from ground truth (`/sim/ground_truth`), not from the stack's opinion:

| | Criterion | PASS |
|---|---|---|
| a | max distance outside (areas U drawn path polygons U dock corridor), recomputed with `mower_map.areas`, per phase | no sample > 0.10 m |
| b | hard-boundary trips (`HARD BOUNDARY` in the mission log) | 0 |
| c | MOWING_COMPLETE, then docked and charging (sim contacts) | all |
| d | STUCK_NEEDS_HELP / sub-paths NOT mowed / costmap-fallback transits (FollowPath / Spin aborts reported) | 0 / 0 / 0 |
| e | max deviation within 0.8 m of every corner >= 79 deg (coverage sub-paths and transit / dock routes) | <= 0.15 m |
| f | one-wheel ticks on `/cmd_vel` (a turn with the inner wheel in (-0.06, 0); inner in [0, 0.06), stalled by the MCU floor, is reported) | 0 |
| g | time pivoting in place vs moving | reported |
| h | MOWING cross-track to the sub-path actually sent (`~/mow_plan`; conditioned when path conditioning is on), median / p90 / p99 / max per segment type: line, arc, corner (raw plans: within 0.3 m of a drawn vertex >= 20 deg), pivoting | p99 <= 0.10 m on line, arc and corner |

It also lists how often each runtime corner rule of the design's section 3.3 fired (route
corner splits and corner-reached rule, stub trim, cusp about-turn, progress watches, the
fix 10 turn-leg cap, the loop guard, costmap fallback, the shaper's widened arcs /
pivot-instead / pivot hold, pivot-assist creep, the stuck guard). Field reference (run 5,
MOWING, raw plan): cross-track median 0.026 / p90 0.074 / p99 0.204 / max 0.262 m.

### Ground disturbance (`-d`)

`mower_sim/kinematics.py` can make the ground disagree with the wheels, so the sim does not
flatter a controller that only works on a perfect drive. Off by default; `-d` sets:

| Launch argument | `-d` value | Effect |
|---|---|---|
| `disturb_wheel_bias` | 0.04 | the right wheel makes 2 % more ground than commanded, the left 2 % less: a slow left drift (cross slope) |
| `disturb_slip_std` | 0.05 | per-wheel multiplicative slip, a first-order random process (correlation `disturb_slip_tau_s` 1 s) |
| `disturb_pivot_eff` | 0.85 | ground yaw rate / commanded while counter-rotating (turf scrub) |
| `disturb_seed` | `-s` (1) | the slip process is seeded: a run is repeatable |

The commanded wheel speeds, the MCU model and what the stack sees are unchanged; only the
ground truth moves differently (with `localization:=sim` the stack sees that truth, as it
sees RTK on the robot).

### What the gate cannot show

Slope (tilt, downhill drift, the field's Spin timeouts on the slope), real turf slip and
grip loss in pivots beyond the disturbance model, wheel-in-a-hole stalls, RTK float /
dropouts and heading errors, stereo false positives on grass. A gate PASS is necessary, not
sufficient: the field run still decides.

### Gate runs

Recorded runs (domain 175, real_run5 world). Base = master 9c2fdd1 (the code deployed for
run 5) with only the sim / gate files added.

| Run | Code | Disturbance | a max out / > 0.10 | b trips | c complete / docked | d stuck / not mowed / fallback | e corners > 0.15 | f one-wheel | g pivot share | h p99 line / arc / corner | Verdict |
|---|---|---|---|---|---|---|---|---|---|---|---|
| base-ideal-1 | 9c2fdd1 | off | 0.107 m / 61 | 2 | no / no (STUCK 92 s after START) | 1 / 0 / 0 | 1 of 2 (0.195) | 10 | 39 % | - / - / 0.245 | FAIL |
| base-dist-1 | 9c2fdd1 | on (seed 1) | 0.114 m / 65 | 2 | no / no (STUCK 94 s after START) | 1 / 0 / 0 | 0 of 2 (0.145) | 8 | 41 % | - / - / 0.241 | FAIL |

The base reproduces run 5's failure mode on the same outer ring: two hard-boundary trips,
the second one after resuming the same manoeuvre, STUCK_NEEDS_HELP ~30 s into mowing. In the
sim the trip comes at the ring's start (-2.0, 1.1), not at the V corner (-4.74, -1.17): the
transit arrives heading ~20 deg, the ring leaves at -136 deg, FollowCoveragePath rotates part
way (to about -90 deg) and then drives an r = 0.44 m arc out of the area. In the field the same
start arc reached 0.062 m outside at (-2.09, 1.10) (`outside_m`, run 5, t = 12.8 s), a near
miss, and the V corner 20 s later tripped. Same cause either way: a sharp heading change in a
coverage path executed as a forward arc.

## Limitations

- **Simplified physics:** there is no slip, no terrain or slope, no tilt, and no
  wheel-in-a-hole stalls. Turf pivot inefficiency (0.02-0.08 rad/s on grass) is not
  modelled beyond the wheel floor. slip_detector and stuck logic therefore never
  trigger.
- **Idealised stereo:** no false positives on grass, no specular noise, and no
  temporal filtering. det_range, YOLO detections and grass segmentation are not
  simulated (no images except the rear marker).
- **Collisions:** a collision is an instant stop with no push. Only the front bumper
  reports a contact; a contact behind the robot just stalls it. The dock body is not a
  collision object, only the hard stop at the contacts.
- **GPS:** RTK FIXED always. Ground-truth pose with σ in the covariance only; there is
  no noise, multipath or fix loss. In `localization:=ekf`, heading_aligner starts
  unaligned, so the mission's preflight needs a COG drive. The scenario stand-in does
  not check alignment.
- **Clock:** wall time only (no `use_sim_time`), real-time only.
- **Untested here:** rain, lift, button and LoRa/NTRIP flows have no inputs yet. The
  e-stop path works through `/estop_request` and `/clear_estop`.
