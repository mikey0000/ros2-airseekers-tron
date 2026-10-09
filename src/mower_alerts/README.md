# mower_alerts

Theft, lift and mission incident alerts for the Airseekers Tron stack. Node **`alert_node`**
watches existing topics, applies debounce, per-incident dedupe and a rate limit, and publishes
each alert once on `/mower_alerts/events`. The MowgliNext GUI backend delivers them over the
operator's notification channel (Telegram, Pushover, ntfy or webhook: Settings ->
Notifications, one switch per kind) and forwards them to its MQTT broker as `<prefix>/alerts`.
Licence: GPL-3.0-or-later.

| File | Role |
|---|---|
| `mower_alerts/rules.py` | All decisions; no ROS imports. `Inputs` snapshot in, `Alert` list out. |
| `mower_alerts/alert_node.py` | rclpy glue: sampled inputs (`sub_pump.py`), 2 Hz tick, JSON out, parked-position file. |
| `config/alerts.yaml`, `launch/alerts.launch.py` | Thresholds; started by `mower.launch.py` (`alerts:=true`). |

## Alerts

| Incident | GUI kind | Message id | Condition (defaults) |
|---|---|---|---|
| `theft_lift` | `theft` | `theftLift` | lifted (`lift_triggered`) while parked, 2 s |
| `theft_geofence` | `theft` | `theftGeofence` | GPS fix more than 10 m from the parked position, 10 s |
| `theft_outside_map` | `theft` | `theftOutsideMap` | map pose more than 3 m outside every mapped area (`/keepout_mask`), 10 s |
| `lift_during_mow` | `lift` | `liftDuringMow` | lifted during a mission or manual driving, 0.5 s |
| `tilt_during_mow` | `lift` | `tiltDuringMow` | IMU tilt above 35 deg during a mission or manual driving, 1 s |
| `emergency_stop` | `emergency` | `emergencyStop` | `HighLevelStatus.emergency` or `BOUNDARY_EMERGENCY_STOP`, 1 s, with its cause |
| `stuck` | `blocked` | `stuck` | `STUCK_NEEDS_HELP` |
| `path_blocked` | `blocked` | `pathBlocked` | sub state `path blocked: waiting` for 45 s (at most once per 10 min) |
| `mow_incomplete` | `blocked` | `mowIncomplete` | `MOWING_INCOMPLETE` or `COVERAGE_FAILED_DOCKING` |
| `dock_failed` | `blocked` | `dockFailed` | `NAV_TO_DOCK_FAILED` |
| `undock_failed` | `blocked` | `undockFailed` | `UNDOCK_FAILED` |
| `battery_critical` | `battery` | `batteryCritical` | battery at or below 10 % and not docked or charging, 30 s |
| `rtk_lost` | `gpsLost` | `rtkLost` / `rtkRecovered` | no RTK fixed for 5 min while mowing, transiting or docking |

"Parked" means `HighLevelStatus.state` IDLE (idle, docked, charging, result states). The theft
rules arm after 60 s parked, so the operator can carry the robot right after stopping it.
NULL-coded states such as `EMERGENCY` keep the previous mode, so a lift that trips the e-stop
while parked is still a theft. The e-stop a lift trips is reported by the lift or theft rule
only, not a second time as an emergency stop.

The parked position is the first fix with accuracy of 5 m or better after arming; a much more
accurate fix nearby replaces it. It is written to `anchor_path`
(`/userdata/ros2/alerts/parked_anchor.json` under `mower.launch.py`), so a robot taken while
powered off raises the geofence alert once it is powered up elsewhere. Starting a mission or
manual driving forgets it.

Mowing finished, mowing started, zones, rain, the recharge pause and the RTK wait at mission
start are not raised here: the GUI's own `NotifyDetector` derives them from
`/behavior_tree_node/high_level_status`. While this node's heartbeat is live, the GUI drops its
own `emergency` and `blocked` pushes, which this node reports with their cause.

## Spam control

* Debounce: a condition must hold for its debounce time before the incident fires.
* Dedupe: an incident fires once. It re-arms only after its condition has been clear for
  `clear_s` (30 s).
* Repeats: theft incidents re-notify with the current position every `theft_repeat_s`
  (5 min), at most `theft_max_repeats` (6) times.
* Groups: the theft incidents form one group, and lift and tilt form another. Only one push
  per group is sent per window (5 min and 60 s).
* Rate limit: at most `rate_limit_count` (6) alerts per `rate_limit_window_s` (10 min).
  Priority-5 alerts (theft, lift, emergency) bypass it. The next alert that goes out carries
  `params.suppressed` with the number dropped.
* The GUI also drops a repeat of the same message id within 60 s.

## Wire format

`/mower_alerts/events`, `std_msgs/String`, reliable, depth 50. `data` is JSON:

```
{"type": "alert", "id": "theft_geofence-1760000000500", "incident": "theft_geofence",
 "kind": "theft", "message": "theftGeofence", "priority": 5,
 "text": "THEFT ALERT: the robot moved 23 m from where it was parked. Last position: https://...",
 "params": {"distance": "23 m", "lat": "48.123456", "lon": "11.123456",
            "map": "https://maps.google.com/?q=48.123456,11.123456", "state": "IDLE_DOCKED",
            "battery": "54 %"},
 "stamp": 1760000000.5}
{"type": "heartbeat", "active": ["theft_geofence"], "stamp": 1760000010.5}
```

The heartbeat goes out every `heartbeat_s` (10 s) and after every alert. The GUI renders `message`
from its catalogue (en, fr) with `params`, and falls back to `text`.

## Inputs

`/behavior_tree_node/high_level_status`, `/mower_base/status` (lift, stop button, dock
contact, charging), `/hardware_bridge/emergency` (reason), `/fix`, `/gps/status` (RTK fix
type), `/odometry/filtered_map`, latched `/keepout_mask`, `/imu/data` (tilt). All are
existing topics. The node changes nothing in the mission, the map server or the drivers.

## Tests

```
python3 -m pytest -q test                 # host, no ROS needed for test_rules.py
./scripts/dev_build.sh test --packages-select mower_alerts
```
