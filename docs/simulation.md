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
DDS domain with `--network host` and `ROS_LOCALHOST_ONLY=1`, so two runs, or a run
and the robot, never share a graph:

```bash
export DEV_ROS_DOMAIN_ID=87
./scripts/dev_build.sh build --packages-up-to mower_sim mower_bringup

# The full stack on the demo garden: robot on the dock, mission idle.
docker run --rm -it --network host -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID=142 \
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
| `nav2_params_file`, `docking_params_file` | package defaults | Parameter overrides. |
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
| stereo_depth | Raycasts a 64x32 grid over 80° x 50° from `stereo_camera_optical` (URDF pose). `/stereo_depth/points` carries obstacle hits at 0.2-4 m. `/stereo_depth/clear_points` carries ground hits up to 3 m, one per 10 cm cell. Both are xyz float32 (point_step 12), use sensor QoS, run at 5 Hz, share a stamp, and are published only while subscribed. |
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
