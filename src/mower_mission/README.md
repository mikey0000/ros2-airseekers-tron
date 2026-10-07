# mower_mission

The mission layer of the Airseekers Tron stack. It runs as node **`behavior_tree_node`**, so
the stock MowgliNext GUI, scheduler and MQTT bridge drive it without changes. It replaces the
stub mission state machine in `mower_gui_bridge`.

It is a plain Python state machine, not a behaviour tree. The logic is ported from MowgliNext
`mowgli_behavior` (GPL-3.0): `trees/main_tree.xml`, `coverage_nodes.cpp` (FollowStrip),
`coverage_persistence.cpp`, `recording_nodes.cpp`. It also takes the vendor cutter interlock
(`OpenCutterCheck` retries, blade off before every transit and dock). Licence: GPL-3.0-or-later.

| File | Role |
|---|---|
| `mower_mission/mission_fsm.py` | All decisions; no ROS imports. Input snapshot + commands + action/service results in, list of effects out (`BladeOn`, `BladeOff`, `StartAction`, `CancelActions`, `PublishStatus`, ...). |
| `mower_mission/geometry.py` | Douglas-Peucker, ring closing, distances, transit decision, plan fingerprint. |
| `mower_mission/resume.py` | Resume cursor and the `coverage_resume.txt` format. |
| `mower_mission/mission_node.py` | rclpy glue: `MultiThreadedExecutor`, reentrant callback group, action clients with `wait_for_server` timeouts and cancel handling. |
| `launch/mission.launch.py`, `config/mission.yaml` | Launch file and all thresholds. |

## Running

```
ros2 launch mower_mission mission.launch.py           # params: config/mission.yaml
ros2 run mower_gui_bridge gui_bridge --ros-args -p serve_high_level:=false
```

Start `gui_bridge` with `serve_high_level:=false`. Otherwise both nodes serve
`/behavior_tree_node/*`. In that mode the bridge keeps the `/hardware_bridge/*` facade and
the `/hardware_bridge/emergency` topic this node needs. It also follows our
`high_level_status` to gate the GUI's blade button.

## How the GUI drives it

Every GUI button is a `HighLevelControl` call on `/behavior_tree_node/high_level_control`:

* Start: 1
* Home: 2
* Record area: 3, then Finish (5) or Cancel (6)
* Manual mow: 7
* Stop: 8
* Reset emergency: 254

The zone list's "mow this area" button calls `start_in_area`. The Resume vs Start-fresh
choice uses the latched `coverage_resume_available` topic and `clear_coverage_resume`. The
GUI renders `high_level_status`: `state`, `state_name` and `sub_state_name` (we put the
failure reason there), plus area, sub-path, swath counts and `coverage_percent`. It also
renders `recording_trajectory` while recording and `/coverage/full_plan`.

## Commands

| Command | Behaviour |
|---|---|
| 1 `START` | `PREFLIGHT_CHECK`: no emergency, battery known and > `battery_low_percent`, no rain (rain_mode 1), GNSS `fix_type >= 1` when docked. Then the area list is read from `get_mowing_area` (index 0.. until `success=false`, navigation areas skipped). Docked: `UNDOCKING` (`/mower_docking/undock`, 0.8 m, `wait_for_rtk=false`). Then `WAITING_FOR_RTK` (fix_type 3, `rtk_timeout_s`). Then per area: `PLANNING` (`/plan_coverage`, publishes `/coverage/full_plan`), then `MOWING`/`TRANSIT` per `drivable_subpath`. Finally `MOWING_COMPLETE`, then `RETURNING_HOME`, then `IDLE_DOCKED`, or `CHARGING` if it is charging. If a resume cursor exists, START continues from it. A START while a run is active is a no-op that returns `true`. |
| 2 `HOME` | Blade off, cancel goals, `RETURNING_HOME` via `/mower_docking/dock`. The cursor is kept. |
| 3 `RECORD_AREA` | `RECORDING` (state 3, blade off). Samples `/odometry/filtered_map` at 10 Hz and drops points less than 5 cm apart. Publishes `~/recording_trajectory`. |
| 10 `RECORD_PATH` (ours) | Same as RECORD_AREA, but FINISH keeps the drive as an OPEN polyline. Steps: `geometry.simplify_open_path` (Douglas-Peucker, no closure or overshoot trim), rejected below `record_path_min_length_m` (1 m), `add_area` of a navigation area "Path N" whose polygon is the line buffered by `record_path_width_m`/2 (0.7 m band), then `/map_server_node/set_area_channel` with `{points, width_m}`. This is the same storage the GUI "Draw path" tool uses, and the GUI then opens the path panel. |
| 5 `RECORD_FINISH` | Closes the ring and runs Douglas-Peucker at 0.05 m. Rejects the result if it has fewer than 3 vertices or covers less than 1 m². Counts the areas, then calls `add_area` with name `"Area N"` (N = mowing areas + 1, not a navigation area). Then `RECORDING_COMPLETE` for 5 s, then `IDLE`. If `add_area` fails, the polygon is written to `recording_fallback_dir`. |
| 6 `RECORD_CANCEL` | Discard the recording, `IDLE`. |
| 7 `MANUAL_MOW` | Publishes state 4 `MANUAL_MOWING` first, then blade on. With `manual_blade_requires_enable` (Tron default) the blade stays OFF (sub_state `joystick, blade off`) until `~/manual_blade`(true). Any other command or an emergency turns the blade off first. Refused during an autonomous run. |
| 8 `STOP` | Cancel all goals, blade off (`/cutter_off`), 0.5 s zero burst on `/cmd_vel_emergency`, then `IDLE`/`IDLE_DOCKED`. Saves the resume cursor and publishes `coverage_resume_available=true`. Always accepted. During an emergency the state stays `EMERGENCY`. It also clears `BOUNDARY_EMERGENCY_STOP`. |
| 254 `RESET_EMERGENCY` | Calls `/clear_estop`. It also clears `BOUNDARY_EMERGENCY_STOP`. |
| `start_in_area(i)` | Like START but only area `i`. It resumes only when the cursor belongs to the same target. |
| `manual_blade` (SetBool) | MANUAL_MOWING only: true = blade on (refused on emergency / lift / stop button / docked / charging), false = blade off, stays in manual drive. sub_state becomes `joystick, blade on` / `joystick, blade off`. mower_gui_bridge forwards the GUI `mower_control` (mow_enabled) here while MANUAL_MOWING. A docked/charging robot with the blade on turns it off on the next tick. |
| `clear_coverage_resume` | Deletes the cursor, so the next START is fresh. Refused while a run is active. |

Commands other than STOP and RESET return `false` during `EMERGENCY` and
`BOUNDARY_EMERGENCY_STOP`. RECORD_AREA and MANUAL_MOW are refused during an autonomous run.
START is refused while recording.

## States

`state` codes are the same as upstream: 0 emergency, 1 idle, 2 autonomous, 3 recording,
4 manual mowing.

```
                      RECORD_AREA(3)              FINISH(5): add_area ok
   IDLE / IDLE_DOCKED ───────────────► RECORDING ───────────────► RECORDING_COMPLETE ─5s─► IDLE
   CHARGING (1)       ◄─────────────── (3)        CANCEL(6) / STOP / reject
        │  ▲
        │  │ MANUAL_MOW(7) ─► MANUAL_MOWING (4, blade on) ─ any cmd / emergency: blade off ─►
        │  │
        │  └──── STOP(8) from anywhere: cancel, blade off, zero burst, cursor kept
        │ START(1) / start_in_area
        ▼
   PREFLIGHT_CHECK (2) ─fail─► IDLE (sub_state = reason)
        │ areas loaded (get_mowing_area unavailable / none ─► IDLE)
        ├─ docked ─► UNDOCKING (2) ─fail─► UNDOCK_FAILED (1) ─5s─► IDLE_DOCKED
        ▼
   WAITING_FOR_RTK (2) ─timeout─┐
        ▼                       │
   ┌► PLANNING (2) ─plan fails─► AREA_UNREACHABLE (2) ─► next area
   │    │ server absent ────────┤
   │    ▼                       │
   │  per drivable_subpath:     │
   │   gap > 0.6 m: blade OFF ─► TRANSIT (2, /navigate_to_pose) ─fail─► skip sub-path
   │   MOWING (2): blade ON, is_cutting within 5 s (3 tries) ─fail─┤
   │     /follow_path (FollowCoveragePath, coverage_goal_checker)  │
   │       abort: retry once from nearest pose, then skip          │
   └──── next area                                                 ▼
        all done ─► MOWING_COMPLETE (2) ─► RETURNING_HOME   COVERAGE_FAILED_DOCKING (2)
                                               │ /mower_docking/dock      │
                                ok ◄───────────┴──────────────────────────┘
                                │                         fail ─► NAV_TO_DOCK_FAILED (0) ─5s─► IDLE
                                ▼
                        IDLE_DOCKED / CHARGING (1)

Guards, checked every tick in this order:
  emergency (Emergency.active, lift, stop button, /hardware_bridge/emergency silent > 2 s)
      ─► EMERGENCY (0): cancel, blade off, zero burst, cursor kept ─clear─► IDLE (no auto-resume)
  lethal boundary violation while PLANNING/MOWING/TRANSIT
      ─► BOUNDARY_EMERGENCY_STOP (0), latched until STOP / RESET_EMERGENCY
  boundary violation while MOWING ─► BOUNDARY_PAUSED (2): cancel, blade off, zero burst
      ─clear─► same sub-path from the nearest pose
      ─still outside after 3 s─► blade-off recovery TRANSIT to a pose 1 m ahead on the sub-path
        (a soft violation never interrupts a TRANSIT); after 2 recoveries ─► BOUNDARY_EMERGENCY_STOP
  rain (rain_mode 1, 2 s debounce) during a run ─► RAIN_DETECTED_DOCKING ─► RAIN_WAITING (1)
      ─dry for rain_delay_minutes─► START again from the cursor
  battery < battery_low_percent during a run ─► LOW_BATTERY_DOCKING ─► CHARGING (1)
      ─>= battery_full_percent─► START again from the cursor
```

### Blade invariants (tested)

* The blade is commanded on only in `MANUAL_MOWING`, or in `MOWING` right before (and during)
  a FollowPath goal.
* `BladeOff` precedes every transit, dock, undock, stop, emergency, boundary pause, area
  change and shutdown. On SIGINT/SIGTERM the node sends `/cutter_off` and waits up to 1 s
  before it exits.
* Blade off goes to `/cutter_off` (Trigger), falling back to `/cutter_control enable=false`.
  Blade on is `/cutter_control cutter.enable=true, speed=0` (driver default).
* A safety net in the state machine forces the blade off on entering any state other than
  MOWING or MANUAL_MOWING and logs `safety: ...`. The tests fail if that net ever fires.
  The randomised test drives about 9000 steps of random commands, sensor changes and action
  outcomes through the invariants.

## Resume cursor

`coverage_resume_path` (default `~/.ros/mower_mission/coverage_resume.txt`) uses the
**upstream-compatible** format with header `mowgli_coverage_resume v2`. It is written
atomically (tmp + fsync + rename) and contains:

* `current_command 1`
* `single_area_target` (only for a `start_in_area` run)
* `current_area`
* `completed_areas`
* one row per area: `area <idx> <pose_count> <fingerprint> <resume_pose_index> completed <sub-path indices>`

`resume_pose_index` is an absolute index into the concatenated `drivable_subpaths`. On STOP,
emergency, HOME, rain or low battery it is set to the tracked progress on the sub-path being
followed. While following, the node tracks progress as the pose nearest the robot, searching
only `progress_window_m` (3 m) ahead of the last progress, so it never moves backwards and
never jumps onto an adjacent headland lap. The same tracker drives the live
`coverage_percent` (length-weighted) and `current_path_index`, and gives the restart point
after a FollowPath abort or a boundary pause. `fingerprint` is a 64-bit FNV-1a hash of the plan geometry (mm-rounded). If
the re-planned geometry differs, the area's cursor is discarded, as upstream does. Upstream
cross-hatch rows are ignored. The file is loaded at start-up, but mowing **never**
auto-starts at boot (upstream does when `current_command` was START). The cursor is cleared
when every area is done or by `clear_coverage_resume`.

## Interfaces

Served (node `behavior_tree_node`):

* Services `~/high_level_control` (HighLevelControl), `~/start_in_area` (StartInArea),
  `~/clear_coverage_resume` (Trigger), `~/manual_blade` (SetBool).
* Publishes `~/high_level_status` (on change + 1 Hz), `~/coverage_resume_available` (Bool,
  transient local), `~/recording_trajectory` (Path), `/coverage/full_plan` (Path, transient
  local), `/cmd_vel_emergency` (zero Twist burst), `~/active_area_settings` (String, JSON,
  transient local; see below), `~/mow_plan` and `~/mow_progress` (String JSON, transient local,
  live mow progress for the GUI map; see `mower_mission/mow_progress.py`).

Inputs:

* `/hardware_bridge/emergency`
* `/hardware_bridge/status` (rain, charging)
* `/battery` (percent)
* `/mower_base/status` (docked, is_cutting, lift, stop, rain, charging)
* `/gps/status` (fix_type, quality)
* `/odometry/filtered_map`
* `/map_server_node/{boundary_violation,lethal_boundary_violation}`

Clients:

* `/map_server_node/{get_mowing_area,add_area,get_area_settings}`
* `/coverage_server/set_parameters`, `/controller_server/set_parameters` (per-area settings)
* `/plan_coverage`
* `/follow_path`
* `/navigate_to_pose`
* `/mower_docking/{dock,undock}`
* `/backup` + `/charging` (only with `use_docking_server:=false`)
* `/cutter_control`, `/cutter_off`
* `/clear_estop`

Every name is a parameter (`*_topic`, `*_service`, `*_action`).

Missing servers do not hang the node. Each goal or call waits `server_wait_timeout_s` (3 s)
in a worker thread and then reports `unavailable`. The state machine then fails the step
cleanly, and `sub_state_name` says which server was missing. Every action also has a
watchdog timeout (`*_timeout_s`); on expiry the goal is cancelled and treated as an abort.

## Per-area mowing settings

Before PLANNING an area the node reads its effective settings from
`/map_server_node/get_area_settings` (index = the `get_mowing_area` index; see
`mower_map/README.md` for the keys). If that service is unavailable the built-in defaults
are used (logged). An area left on auto angle falls back to the global `mow_angle_deg`
parameter. Then, with the blade off:

1. **Blade height**: `/cutter_control` with `cutter.enable=false`, `height.enable=true`,
   `height.position` = `cutter_height_mm` as an absolute deck height in millimetres,
   clamped to 30-90 like the vendor (verified on the bench 2026-10-07). Every later
   blade-ON command repeats the same `height.position` (enable=true), because what the MCU
   does with `height.enable=false` is unknown too.
2. **Cut speed**: `set_parameters` on `/controller_server`
   `FollowCoveragePath.desired_linear_vel` = `cut_speed_mps`, capped at `cut_speed_max_mps`
   (0.5). Fire and forget: without Nav2 it just logs.
3. **Coverage parameters**: `set_parameters` on `/coverage_server`: `operation_width` =
   `cut_width_m` (0.20) - `swath_overlap_m`, `headland_rings` = `perimeter_laps`,
   `path_mode`, `mow_angle_deg` (of this run), `edge_first`. The plan goal is sent when this
   call returns; if it fails the plan goes out anyway with the bridge's current parameters
   (logged).
4. **PlanCoverage** goal with the run's `mow_angle_deg` / `perpendicular`.

Runs: each area is planned and mowed once per run, in order:

* `zigzag`, `spiral`, `contour_only`: `repeat` runs at the same angle.
* `cross`: two plans per repeat, at the angle and the angle + 90 deg.
* `alternate`: `repeat` runs, the angle turned by `alternate_angle_offset_deg` on every run,
  continuing across sessions: the number of finished alternate runs per area name is kept in
  `alternate_state_path` (JSON).
* On an AUTO angle (-1) a +90 deg turn is sent as the goal's `perpendicular=true` (the bridge
  learns the auto angle and re-plans at +90); any other turn uses the offset from 0 deg (map
  x axis).

Between runs the blade goes off and the area's resume cursor row is reset; the run in
progress is stored as an extra `area_run <idx> <run>` line in the resume file (ignored by
upstream readers), so a STOP / rain / battery resume continues the same run. The PLANNING
sub-state reads `area <i> <name> [<path_mode>, run k/n]`.

`~/active_area_settings` (latched `std_msgs/String`) carries the active settings as JSON:
every settings key plus `area_index`, `area_name`, `run`, `runs`, `run_mow_angle_deg`,
`run_perpendicular`, `operation_width_m`. It is `{}` when no
session is running.

| Parameter | Default |
|---|---|
| `use_area_settings` | true (false: built-in defaults, no `get_area_settings` call) |
| `cut_width_m` | 0.20 |
| `cut_speed_max_mps` | 0.5 |
| `controller_speed_param` | `FollowCoveragePath.desired_linear_vel` |
| `set_cut_speed` / `set_cutter_height` | true / true |
| `alternate_state_path` | `~/.ros/mower_mission/alternate_counts.json` (yaml: `/ros2_ws/maps/mission_alternate_counts.json`) |
| `get_area_settings_service`, `coverage_server_node`, `controller_server_node`, `active_area_settings_topic` | `/map_server_node/get_area_settings`, `/coverage_server`, `/controller_server`, `~/active_area_settings` |

## Parameters

All parameters are in `config/mission.yaml`, with comments; `test_config_yaml_matches_fsm_parameter_types`
keeps it in sync with `mission_fsm.Params`. The main ones:

| Parameter | Default |
|---|---|
| `battery_low_percent` / `battery_full_percent` | 20 / 95 |
| `emergency_timeout_s` | 2.0 |
| `preflight_min_fix_type_docked` / `rtk_fix_type` / `rtk_timeout_s` | 1 / 3 / 120 |
| `rain_mode` / `rain_debounce_s` / `rain_delay_minutes` | 1 / 2 / 30 |
| `use_docking_server`, `undock_distance_m`, `undock_speed_mps`, `dock_use_vision`, `dock_timeout_s` | true, 0.8, 0.15, true, 180 |
| `transit_gap_m` | 0.6 |
| `follow_controller_id` / `follow_goal_checker_id` | FollowCoveragePath / coverage_goal_checker |
| `require_blade_confirmation`, `blade_confirm_timeout_s`, `blade_start_attempts`, `blade_retry_pause_s` | true, 5, 3, 2 |
| `manual_blade_requires_enable` | false (upstream); `config/mission.yaml` sets true: two-step manual mowing, see `manual_blade` |
| `progress_window_m` | 3.0 |
| `follow_end_tolerance_m` / `follow_premature_retries` | 1.0 / 3 |
| `follow_chunk_m` / `follow_chunk_clearance_m` | 25.0 / 0.5 |
| `boundary_recover_after_s` / `boundary_max_recoveries` / `boundary_recovery_advance_m` | 3 / 2 / 1.0 |
| `record_*` | 10 Hz, 5 cm, DP 0.05 m, min 1 m² |
| `coverage_resume_path`, `recording_fallback_dir` | `~/.ros/mower_mission/...` |

## Differences from MowgliNext `behavior_tree_node`

* **No behaviour tree.** Humble ships BehaviorTree.CPP v3 and the upstream tree is v4, so the
  logic is a Python state machine instead. The guard order is the same as upstream's
  `ReactiveSequence`.
* **Not ported:**
  * LiDAR / scan guards, collision monitor, fusion_graph, heading calibration and the escape
    and dig manoeuvres.
  * Cross-hatch mow angles and keepout filter toggling.
  * `CRITICAL_BATTERY_*` states and the manual resume threshold.
  * Auto-start at boot from the resume file.
  * Retiring an area after 5 dispatches with no progress. Here an area pass always ends, and
    skipped sub-paths stay unmowed until the next fresh run.
* **Docking** goes through our `mower_interfaces/{Dock,Undock}` actions (`mower_docking`),
  not opennav `DockRobot` or `BackUp`. Undock is sent with `wait_for_rtk=false`, because RTK
  is awaited here in `WAITING_FOR_RTK`.
* **The area list is read before undocking**, so a missing map server or an empty map
  refuses START without moving the mower.
* **Soft boundary violation**: a soft violation pauses only while mowing (`BOUNDARY_PAUSED`,
  our own name). It never pauses a blade-off transit. If the robot is still outside after 3 s,
  it makes up to 2 blade-off recovery transits to a pose 1 m ahead on its own sub-path, then
  latches `BOUNDARY_EMERGENCY_STOP`. The blade never spins up while the violation is
  flagged. Upstream instead navigates to the map server's `get_recovery_point`, which is not
  part of our contract.
* **FollowPath goals are chunked, and premature success is caught.** The Humble
  `coverage_goal_checker` is a stock `SimpleGoalChecker` with 0.10 m tolerance, and it only
  looks at the final pose. A coverage path passes near its own end (closed headland rings,
  a last swath ending at the ring start), so the goal "succeeds" early. Seen live: a 5.6 s
  "complete" mow with 38 m left. Upstream avoids this with its own `PathProgressGoalChecker`.
  We handle it in two steps:
  1. A sub-path is sent in chunks of at most `follow_chunk_m` (25 m). Each chunk's final
     pose is more than `follow_chunk_clearance_m` (0.5 m) away from every earlier pose of
     that chunk.
  2. A success reported more than `follow_end_tolerance_m` (1 m) before the chunk end
     (judged by tracked progress) continues from the progress pose, up to 3 times, and the
     sub-path is skipped after that.

  A progress-aware goal checker in `mower_navigation` would be the cleaner fix.
* **Blade start is verified**: `is_cutting` must be true within 5 s, with 3 tries, as in the
  vendor's `OpenCutterCheck`. Upstream waits a fixed spin-up delay.
* **The blade is commanded directly** on `/cutter_control` and `/cutter_off`. Upstream goes
  through `/hardware_bridge/mower_control`.
* **New or changed states:**
  * Short failure states (`UNDOCK_FAILED`, `NAV_TO_DOCK_FAILED`, `RECORDING_COMPLETE`) are
    shown for `result_display_s` and then fall back to IDLE. The reason stays in
    `sub_state_name`.
  * Failures before the mower moves (preflight, no areas, no map server) go straight back to
    IDLE, with the reason in `sub_state_name`.
  * A run aborted after undocking (RTK timeout, planner, controller or blade failure)
    docks via `COVERAGE_FAILED_DOCKING`.
* **Recorded areas are named "Area N"**, not `recorded_area_N`. A failed `add_area` keeps
  the polygon in a local file.

## Tests

```
python3 -m pytest src/mower_mission/test            # host, no ROS needed
./scripts/dev_build.sh test --packages-select mower_mission
```
