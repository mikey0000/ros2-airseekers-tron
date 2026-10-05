# mower_gui_bridge

Adapter node (`ros2 run mower_gui_bridge gui_bridge`) that makes the Tron hardware
stack look like a MowgliNext robot to the MowgliNext web GUI (Go backend via
foxglove_bridge). It also contains a **stub** high-level mission state machine,
which is enough for joystick driving and manual mowing with the blade on.

**Deployed configuration:** `launch/mower.launch.py` starts the real mission layer
(`mower_mission`, `mission:=true` by default) and this bridge with
`serve_high_level:=false`, so everything under `/behavior_tree_node/*` (START, HOME,
`start_in_area`, recording, coverage resume) is served by `mower_mission`, not by
the stub. The stub is used only with `mission:=false`.

All decision logic is in `mower_gui_bridge/state_machine.py` and
`mower_gui_bridge/datum.py` (pure Python, no ROS). `gui_bridge_node.py` only wires
them to topics and services. Inputs are taken off the executor by
`sub_pump.py` (see its docstring; a copy of the same file lives in the other Python
node packages).

## Topics published

| Topic | Type | Source / rate |
|---|---|---|
| `/hardware_bridge/status` | `mowgli_interfaces/Status` | 5 Hz from `/mower_base/status`, `/mower_sensor_info`. `mower_status=255` while the MCU is alive (status within `mcu_timeout_s`), else 0. `mow_enabled` = blade requested on by us OR `is_cutting`. `mower_motor_rpm` = `cutter_motor.speed_rpm`. `firmware_version` = `cutter_board_version`. `firmware_compatible=true`. |
| `/hardware_bridge/emergency` | `mowgli_interfaces/Emergency` | 5 Hz always (clamped to at least 2 Hz), plus immediately when `/estop` changes. `active` = `/estop` OR stop button OR lift OR MCU telemetry stale. `latched` = `/estop`. `lift_warning` = lift. |
| `/hardware_bridge/power` | `mowgli_interfaces/Power` | 2 Hz from `/battery` (+ `is_charging`). |
| `/gps/fix` | `sensor_msgs/NavSatFix` | relay of `/fix` |
| `/gps/status` | `mowgli_interfaces/GnssStatus` | per `/fix`. FIXED vs FLOAT comes from the `/fix_status` string (`quality=` or `solution=` token). Correction fields come from its `corr_src=` / `corr=` / `corr_flow=` / `corr_age=` tokens (see below). |
| `/wheel_odom` | `nav_msgs/Odometry` | relay of `/odom` |
| `/odometry/filtered_map` | `nav_msgs/Odometry` | relay of `/odometry/filtered`, with `frame_id=map` and `child_frame_id=base_footprint` (map == odom identity for now) |
| `/behavior_tree_node/high_level_status` | `mowgli_interfaces/HighLevelStatus` | on every change + 1 Hz (only when `serve_high_level`) |
| `/behavior_tree_node/coverage_resume_available` | `std_msgs/Bool` | latched (transient local, depth 1), always `false` (only when `serve_high_level`) |
| `/estop_request` | `std_msgs/Bool` | `true` on a GUI emergency stop |
| `/cmd_vel_emergency` | `geometry_msgs/Twist` | one zero twist on `COMMAND_STOP` |

## Topics subscribed

`/mower_base/status`, `/mower_sensor_info`, `/battery`, `/estop`, `/fix`,
`/fix_status`, `/odom`, `/odometry/filtered`. When `serve_high_level` is false, it
also subscribes to `/behavior_tree_node/high_level_status`, so the blade interlock
follows the real mission node's state.

## Services served

| Service | Type | Behaviour |
|---|---|---|
| `/hardware_bridge/mower_control` | `mowgli_interfaces/MowerControl` | Blade on/off through `/cutter_control` (speed 0 means the driver default). It waits up to `service_timeout_s` and returns the driver's result. Subject to the interlock below. |
| `/hardware_bridge/emergency_stop` | `mowgli_interfaces/EmergencyStop` | `emergency!=0` publishes `/estop_request: true` and calls `/cutter_off`. `emergency==0` calls `/clear_estop`. |
| `/hardware_bridge/reboot_board` | `std_srvs/Trigger` | no-op, `success=true`, "not supported on Tron" |
| `/behavior_tree_node/high_level_control` | `mowgli_interfaces/HighLevelControl` | the stub state machine (below) |
| `/behavior_tree_node/start_in_area` | `mowgli_interfaces/StartInArea` | stub only (`serve_high_level=true`): `success=false`. In the deployed setup `mower_mission` serves it. |
| `/behavior_tree_node/clear_coverage_resume` | `std_srvs/Trigger` | stub only: `success=true` |
| `/navsat_to_absolute_pose/set_datum` | `std_srvs/Trigger` | Sets the map origin to the current GPS position. See [set_datum](#set_datum). |

Clients: `/cutter_control` (`mower_interfaces/CutterControl`), `/cutter_off`
(`std_srvs/Trigger`), `/clear_estop` (`std_srvs/Empty`), and robot_localization's
`/datum` (`robot_localization/SetDatum`, optional). Service servers and clients
use reentrant callback groups on a `MultiThreadedExecutor`. That way a server can wait
for a client response without deadlocking. The state-machine side effects are
fire-and-forget (`call_async` + done callback).

## set_datum

The GUI's "set datum from GPS" (Settings > Positioning, onboarding) calls
`/navsat_to_absolute_pose/set_datum`:

1. It needs a `/fix` that is newer than `datum_fix_max_age_s` (2 s) and not
   `STATUS_NO_FIX`. Otherwise it returns `success=false`, `message="set_datum refused: <reason>"`.
2. It replies `success=true`, `message="<lat>,<lon>"` (9 decimals). This is the
   contract parsed by `web/src/utils/datumGps.ts` (split on `,`, `parseFloat` each
   part). The GUI then puts the values in its settings form.
3. If a datum was already in force (launch args, else `datum_env_path`, else
   `robot_yaml_path`) and the new one is more than 1 m away, a comma-free warning
   follows the longitude: `"<lat>,<lon> WARNING: datum moved 14.2 m from the previous
   <lat>/<lon>; areas and the dock pose stored in map coordinates now sit 14.2 m off -
   re-record them or restore the old datum"`. Areas, obstacles and the dock pose are stored in
   map-frame metres relative to the datum, so they shift with it.
4. It splices `datum_lat` / `datum_lon` into `robot_yaml_path`
   (`config/gui/mowgli_robot.yaml`) in place. Comments and layout are kept, and
   symlinks (colcon `--symlink-install`) are followed. Missing keys are added under
   `ros__parameters`.
5. It writes `datum_env_path` (`/userdata/ros2/datum.env`, `DATUM_LAT=` / `DATUM_LON=`).
   `launch/mower.launch.py` reads it at launch time as the default for
   `datum_lat` / `datum_lon` when those args are not given, so the next stack start
   pins `navsat_transform_node` to the new datum.
6. If `/datum` (navsat_transform_node's `robot_localization/SetDatum`) is ready, it is
   called fire-and-forget with the fix's lat/lon/alt and yaw 0 (ENU east). This
   re-anchors the live transform; the reply does not wait for it. When `/datum` is
   not available the reply ends with `(saved; restart the stack to re-anchor localization)`.
   Write failures are appended as `NOT SAVED: ...` (still `success=true`, because the
   GUI keeps the values in its own settings).

On start, if the stack was launched with a datum and `robot_yaml_path` holds a
different one, the bridge rewrites the yaml to the launch datum
(`sync_robot_yaml_datum`). This repairs a yaml reset by a deploy `rsync --delete`.

## GNSS corrections

`/fix_status` tokens are mapped onto the source-owned correction fields:
`correction_source` (`ntrip`/`lora`/`none`), `correction_transport_status` (NTRIP
client state; unknown for LoRa, whose radio link the host cannot see),
`correction_flow_status`, `corrections_active` and `correction_age_s`. A field
whose value is unknown has its capability bit set but not its value bit.

The stock GUI reads only the legacy `correction_stream_status` /
`CAP_CORRECTION_STREAM`, so `corr_flow` is mirrored into it as well: `idle` to
`IDLE`, `waiting`/`held` to `WAITING`, `active` to `ACTIVE`, `stale` to
`UNAVAILABLE`, `invalid` to `ERROR`. Without a `corr_flow` token it stays `UNKNOWN`.

## Blade interlock

This is the interlock that the Mowgli firmware and BT normally provide:

* **Blade ON** is allowed only in HL state `AUTONOMOUS (2)` or `MANUAL_MOWING (4)`, and
  only when no emergency is active. Otherwise `mower_control` returns `success=false`.
* **Blade OFF** is always allowed. If `/cutter_control` refuses or is unavailable, the
  node falls back to `/cutter_off`.
* Any transition out of `MANUAL_MOWING` turns the cutter off.
* When an emergency becomes active, the cutter is forced off and the state becomes
  `EMERGENCY (0)`. When the emergency clears, the state returns to `IDLE`, never
  straight back into a blade state. The cutter is still forced off on the rising edge
  when `serve_high_level=false`.
* mcu_node keeps its own interlock: it refuses `/cutter_control` while its interlock or
  e-stop latch is active. The bridge is an extra layer and does not replace it.

## Stub state machine (`serve_high_level=true`, i.e. `mission:=false`)

| Command | Result |
|---|---|
| 3 `RECORD_AREA` | `RECORDING` (3): joystick, blade off |
| 5 `RECORD_FINISH`, 6 `RECORD_CANCEL` | `IDLE`. Nothing is recorded; this is only logged. |
| 7 `MANUAL_MOW` | `MANUAL_MOWING` (4). The state is published first, then the blade is enabled via `/cutter_control`. |
| 8 `STOP` | `IDLE`, `/cutter_off`, one zero `Twist` on `/cmd_vel_emergency`. During an emergency the state stays `EMERGENCY`. |
| 254 `RESET_EMERGENCY` | calls `/clear_estop`. The state leaves `EMERGENCY` only once `/estop`, the stop button and lift are all clear. |
| 1 `START`, 2 `HOME` | `success=false`, "no mission layer yet" |
| anything else | `success=false` |

`state_name` is one of `IDLE`, `IDLE_DOCKED` (when `is_docking_done`), `RECORDING`,
`MANUAL_MOWING` or `EMERGENCY`. `MANUAL_MOW` and `RECORD_AREA` are refused while an
emergency is active. Other fields: `battery_percent` (0..100), `gps_quality_percent`
(0..1 fraction, as upstream `behavior_tree_node` publishes it), `is_charging`,
`emergency`, `current_area=-1`, `coverage_percent=0`.

### What the stub does NOT do

* There is no autonomous mowing, docking, undocking or path following. `START`, `HOME`
  and `start_in_area` are refused, and the stub never enters `AUTONOMOUS`.
* It does not record areas: no trajectory, no polygon, nothing saved to the map server.
* It does not drive anything itself. Teleop comes from the GUI relay on `/cmd_vel_teleop`.
* There is no coverage resume, no battery-based return-to-dock and no rain handling
  (rain is only reported in `Status.rain_detected`).
* `lift_duration_sec` is not tracked and is always 0.
* It does not run a firmware version handshake (`firmware_compatible` is always true).

## Replacing it with the real mission node

1. Launch the real `behavior_tree_node` (or our own mission node). It must provide
   `/behavior_tree_node/high_level_control`, `high_level_status`, `start_in_area`,
   `clear_coverage_resume` and `coverage_resume_available`.
2. Start this bridge with `serve_high_level:=false`. It then stops publishing and
   serving everything under `/behavior_tree_node/*`. It subscribes to
   `high_level_status` so `mower_control` still enforces the interlock against the real
   state. It keeps serving the `/hardware_bridge/*` hardware facade, the GNSS and
   odometry relays, and `set_datum`.

### Running with the real mission node (`mower_mission`)

The real mission layer is `src/mower_mission` (`ros2 launch mower_mission
mission.launch.py`). It runs as node `behavior_tree_node` and serves all of
`/behavior_tree_node/*` itself: START/HOME/STOP, recording, manual mowing, the
coverage resume cursor, and the rain, battery, boundary and emergency guards. Start
this bridge next to it with `serve_high_level:=false`, for example `ros2 run
mower_gui_bridge gui_bridge --ros-args -p serve_high_level:=false`. If you leave it
at true, both nodes serve the same services and topics. With `false` the bridge
stops serving that namespace. It still publishes `/hardware_bridge/emergency` and
`/hardware_bridge/status`, which the mission node uses for its emergency and rain
guards. It still gates the GUI's blade button (`/hardware_bridge/mower_control`) on
the mission node's `high_level_status`. It still forces the cutter off on the rising
edge of an emergency. The mission node switches the blade itself through
`/cutter_control` and `/cutter_off`.

## Parameters

Every topic and service name above is a parameter: `*_topic` / `*_service` (see
`PARAM_DEFAULTS` in `gui_bridge_node.py`). The others are:

| Parameter | Default | Meaning |
|---|---|---|
| `map_frame` / `base_frame` | `map` / `base_footprint` | frames forced on `/odometry/filtered_map` |
| `status_rate_hz` | 5.0 | `/hardware_bridge/status` rate |
| `emergency_rate_hz` | 5.0 | clamped to at least 2 Hz |
| `power_rate_hz` | 2.0 | |
| `high_level_rate_hz` | 1.0 | HL status republish rate |
| `mcu_timeout_s` | 1.0 | `/mower_base/status` age beyond which the MCU counts as not alive |
| `emergency_on_mcu_timeout` | true | treat MCU silence as an active emergency |
| `fix_status_timeout_s` | 3.0 | an older `/fix_status` is ignored for FIXED/FLOAT classification |
| `service_timeout_s` | 1.5 | wait for `/cutter_control` / `/clear_estop` in GUI service calls |
| `serve_high_level` | true | serve the stub mission layer (`mower.launch.py` sets it to `false` when `mission:=true`) |
| `rl_set_datum_service` | `/datum` | robot_localization SetDatum called by `set_datum` (`''` = never) |
| `robot_yaml_path` | `''` | GUI settings file updated by `set_datum` (`''` = skip). `mower.launch.py` passes `config/gui/mowgli_robot.yaml`. |
| `datum_env_path` | `/userdata/ros2/datum.env` | launch-defaults file written by `set_datum` (`''` = skip). `mower.launch.py` passes its `datum_env_file` arg. |
| `datum_lat` / `datum_lon` | 0.0 / 0.0 | datum the stack was launched with (0/0 = unset) |
| `datum_fix_max_age_s` | 2.0 | the newest `/fix` must be at most this old |
| `sync_robot_yaml_datum` | true | on start, rewrite the yaml datum to the launch datum |

## Tests

```
python3 -m pytest src/mower_gui_bridge/test     # plain host, no rclpy needed
colcon test --packages-select mower_gui_bridge  # in the Humble container
```
