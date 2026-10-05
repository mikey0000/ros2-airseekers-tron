# mower_gui_bridge

Adapter node (`ros2 run mower_gui_bridge gui_bridge`) that makes the Tron hardware
stack look like a MowgliNext robot to the MowgliNext web GUI (Go backend via
foxglove_bridge). It also contains a **stub** high-level mission state machine,
which is enough for joystick driving and manual mowing with the blade on. It stays
in place until a real behaviour tree / mission node exists.

All decision logic is in `mower_gui_bridge/state_machine.py` (pure Python, no ROS).
`gui_bridge_node.py` only wires it to topics and services.

## Topics published

| Topic | Type | Source / rate |
|---|---|---|
| `/hardware_bridge/status` | `mowgli_interfaces/Status` | 5 Hz from `/mower_base/status`, `/mower_sensor_info`. `mower_status=255` while the MCU is alive (status within `mcu_timeout_s`), else 0. `mow_enabled` = blade requested on by us OR `is_cutting`. `mower_motor_rpm` = `cutter_motor.speed_rpm`. `firmware_version` = `cutter_board_version`. `firmware_compatible=true`. |
| `/hardware_bridge/emergency` | `mowgli_interfaces/Emergency` | 5 Hz always (clamped to at least 2 Hz), plus immediately when `/estop` changes. `active` = `/estop` OR stop button OR lift OR MCU telemetry stale. `latched` = `/estop`. `lift_warning` = lift. |
| `/hardware_bridge/power` | `mowgli_interfaces/Power` | 2 Hz from `/battery` (+ `is_charging`). |
| `/gps/fix` | `sensor_msgs/NavSatFix` | relay of `/fix` |
| `/gps/status` | `mowgli_interfaces/GnssStatus` | per `/fix`. FIXED vs FLOAT comes from the `/fix_status` string (`quality=` or `solution=` token). |
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
| `/behavior_tree_node/start_in_area` | `mowgli_interfaces/StartInArea` | `success=false` (no mission layer) |
| `/behavior_tree_node/clear_coverage_resume` | `std_srvs/Trigger` | `success=true` |
| `/navsat_to_absolute_pose/set_datum` | `std_srvs/Trigger` | `success=false`, "datum is fixed in config/localization/navsat.yaml" |

Clients: `/cutter_control` (`mower_interfaces/CutterControl`), `/cutter_off`
(`std_srvs/Trigger`), `/clear_estop` (`std_srvs/Empty`). Service servers and clients
use reentrant callback groups on a `MultiThreadedExecutor`. That way a server can wait
for a client response without deadlocking. The state-machine side effects are
fire-and-forget (`call_async` + done callback).

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

## Stub state machine

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
| `serve_high_level` | true | serve the stub mission layer |

## Tests

```
python3 -m pytest src/mower_gui_bridge/test     # plain host, no rclpy needed
colcon test --packages-select mower_gui_bridge  # in the Humble container
```
