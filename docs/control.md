# mower_control — cmd_vel shaping, slip detection, IMU calibration

`mower_control` (ROS 2 Humble, `ros2_stack/src/mower_control`) holds the three
control-support nodes that sit between high-level planners/teleop and the
hardware drivers. All three are ports of MowgliNext patterns — see
`ros2_stack/docs/mowglinext_baseline.md` section 4.3 — adapted to the
Airseekers Tron topic contract.

| Node | MowgliNext source pattern | Baseline section |
|---|---|---|
| `cmd_vel_slew` | `cmd_vel_slew.hpp` — shape before the wire, immediate exact stop, echo the applied command | 4.3 item 9 |
| `slip_detector` | `dig_detector.hpp` — commanded vs measured twist, latched dig event, GNSS trust gate | 4.3 item 8 |
| `imu_cal` | at-rest bias calibration with a persisted, validated file | 4.3 item 7 |

## Data flow

```
twist_mux / teleop / Nav2
        │ /cmd_vel_raw (TwistStamped)
        ▼
  cmd_vel_slew ──► /cmd_vel (TwistStamped) ──► mower_mcu_driver ──► MCU
        │                                            │
        │                              /wheel_vel (TwistStamped, vendor stack)
        │                              /odom     (Odometry, mower_mcu_driver)
        │                                            │
        └──────────────► slip_detector ◄──────────────┘
                              ▲
                              │ /fix_status (String, um960_gps_driver)
                              │
                         /dig_stall (Bool, latched, transient_local)

wit_imu_driver ──► /imu/data (Imu) ──► imu_cal ──► /bias_status (String)
                                                └─► ~/mower_control/imu_calibration.yaml
```

## cmd_vel_slew

Slew-rate limiter with a command watchdog. Subscribes `/cmd_vel_raw`
(`geometry_msgs/TwistStamped`), publishes `/cmd_vel`
(`geometry_msgs/TwistStamped`) — the topic `mower_mcu_driver` subscribes to.

| Parameter | Default | Description |
|---|---|---|
| `max_linear_accel` | `0.5` | m/s² cap on linear.x acceleration |
| `max_angular_accel` | `1.0` | rad/s² cap on angular.z acceleration |
| `cmd_timeout` | `0.5` | s after the last `/cmd_vel_raw` before the watchdog fires |
| `rate` | `20.0` | Hz output/update rate |
| `frame_id` | `base_link` | frame stamped on the output |

Behaviour:

* Fixed-rate loop; each tick moves applied linear.x / angular.z at most
  `accel * dt` toward the target. All other twist components pass through
  unchanged.
* **Exact echo logging** — every published command is logged at full float
  precision (the MowgliNext `/cmd_vel_applied` idea as a log line). Silence
  with `--ros-args --log-level cmd_vel_slew:=warn` if too chatty.
* **Watchdog** — when no `/cmd_vel_raw` arrives within `cmd_timeout`, the
  applied velocity is set to exactly 0.0 (immediate exact stop, not slewed)
  and zero is published every tick until a fresh command arrives; recovery
  slews up from zero.

## slip_detector

Compares commanded vs measured twist and latches `/dig_stall`
(`std_msgs/Bool`, `QoS(1).transient_local()`) when the mismatch persists.

| Parameter | Default | Description |
|---|---|---|
| `slip_threshold` | `0.3` | m/s (also applied to rad/s) — \|measured − commanded\| above this on linear.x or angular.z is a slip condition |
| `slip_window` | `2.0` | s of continuous slip before the latch is raised |
| `require_rtk_fixed` | `true` | only latch while `/fix_status` reports RTK_FIXED |
| `measured_topic` | `''` | override the measured source; empty = auto |
| `discovery_timeout` | `5.0` | s to wait for a `/wheel_vel` publisher before falling back to `/odom` |
| `publish_rate` | `10.0` | Hz latch-status publish rate |
| `fix_timeout` | `5.0` | s without a `/fix_status` message before warning |

Subscribed topics:

* `/cmd_vel` (`geometry_msgs/TwistStamped`) — output of `cmd_vel_slew`.
* Measured twist, auto-detected at startup: `/wheel_vel`
  (`geometry_msgs/TwistStamped`, published by the vendor `mower_base` stack)
  if it has a publisher, otherwise `/odom` (`nav_msgs/Odometry`, published by
  `mower_mcu_driver`). Override with `measured_topic`.
* `/fix_status` (`std_msgs/String`) — from `um960_gps_driver`; the RTK gate
  matches the substring `RTK_FIXED` (NMEA GGA quality 4 → `quality=RTK_FIXED`,
  Unicore BESTNAV posType 17 → `solution=INS_RTKFIXED`).

Behaviour:

* A slip condition is `|measured − commanded| > slip_threshold` on linear.x or
  angular.z. It must hold **continuously** for `slip_window` seconds before
  `/dig_stall` latches true; any sample below the threshold restarts the
  window. The latch is never cleared by this node (MowgliNext clears on charger
  contact / escape displacement — port that with a dig_escalation node later).
* With `require_rtk_fixed` the window does not advance unless the latest
  `/fix_status` contains `RTK_FIXED`; missing or stale `/fix_status` suppresses
  the latch and logs a warning.

## imu_cal

At-rest IMU bias calibration. Subscribes `/imu/data` (`sensor_msgs/Imu`,
from `wit_imu_driver`), publishes `/bias_status` (`std_msgs/String`,
`QoS(1).transient_local()`), persists biases to YAML.

| Parameter | Default | Description |
|---|---|---|
| `calibration_file` | `~/mower_control/imu_calibration.yaml` | output/input file |
| `gravity` | `9.81` | m/s² expected \|accel\| at rest |
| `accel_tolerance` | `0.5` | m/s² — \|(\|accel\| − g)\| must be below this |
| `gyro_variance_max` | `0.0001` | (rad/s)² — window gyro variance must be below this |
| `gyro_bias_max` | `0.2` | rad/s — reject windows with larger \|gyro bias\| |
| `min_samples` | `100` | samples per evaluation window |
| `window_timeout` | `60.0` | s before giving up with FAILED |
| `status_rate` | `1.0` | Hz `/bias_status` publish rate |

Plausibility rules per window (MowgliNext style):

* `| |accel| − g | < accel_tolerance` — the mower is at rest, not accelerating.
* Every axis has non-zero gyro variance — the IMU is alive, not stuck.
* Max axis gyro variance < `gyro_variance_max` — the mower is not moving.
* `|gyro bias| <= gyro_bias_max` — sanity, mirrors the file-load validation.

An implausible window is discarded with a warning and collection continues.
On a plausible window: gyro bias = mean gyro; accel bias = mean accel minus g
along the measured gravity direction (any IMU orientation). The result is
written atomically (tmp + rename) as flat YAML with a
`# mower_control_imu_calibration_v1` header. An existing file is validated on
startup (all-zero biases, |gyro bias| > `gyro_bias_max`, missing keys or
unreadable YAML are rejected with a warning) and then overwritten once a
plausible window completes.

## Running

```bash
# after: colcon build --packages-select mower_control
ros2 run mower_control cmd_vel_slew
ros2 run mower_control slip_detector
ros2 run mower_control imu_cal
```

Or from a launch file:

```python
Node(package='mower_control', executable='cmd_vel_slew', name='cmd_vel_slew'),
Node(package='mower_control', executable='slip_detector', name='slip_detector'),
Node(package='mower_control', executable='imu_cal', name='imu_cal'),
```

## Dependencies on other packages

| Consumer | Depends on | Topic |
|---|---|---|
| `cmd_vel_slew` | — | `/cmd_vel_raw` in, `/cmd_vel` out |
| `slip_detector` | `mower_control/cmd_vel_slew` | `/cmd_vel` |
| `slip_detector` | `mower_mcu_driver` (or vendor `mower_base`) | `/odom` or `/wheel_vel` |
| `slip_detector` | `um960_gps_driver` | `/fix_status` (RTK gate) |
| `imu_cal` | `wit_imu_driver` | `/imu/data` |

`mower_mcu_driver` subscribes to `/cmd_vel`, so the intended chain is
`twist_mux → cmd_vel_slew → mower_mcu_driver`. Wiring these three nodes into
`ros2_stack/launch/bringup.launch.py` is a follow-up.
