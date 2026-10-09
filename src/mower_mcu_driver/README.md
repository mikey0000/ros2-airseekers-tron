# mower_mcu_driver

ROS 2 Jazzy (ament_python) driver for the **Airseekers Tron chassis/cutter MCU** — the AT32
board(s) behind `/dev/serial_mower` (`ttyS9`, 115200 8N1). It replaces the stripped vendor
`mower_base_node` for the serial part of its job: battery, odometry, MCU sensor state, and
forwarding `/cmd_vel` to the MCU.

Sources (read-only, both in `ros2_port_handoff/01_mcu_protocol/`):

* `mcu_frame_decode.py` — reference framing/checksum/escape implementation (validated on
  172,850 live frames); ported 1:1 into `mower_mcu_driver/mcu_node.py`.
* `protocol_definitions_recovered.h` — struct layouts and enums recovered from DWARF in
  `libmower_sdk.a`; every `*_FMT` in the node comes from there.

```
0xA5 | total_len | { sub_len | type_id | mod_id | payload }+ | checksum | 0x5A
checksum = sum(total_len .. last payload byte) & 0xFF        escaping: A5→F5 01, 5A→F5 02, F5→F5 03
host→MCU type 0, MCU→host type 8, heartbeat type 255         all multi-byte fields little-endian
```

## Interfaces

| Direction | Topic / frame | Type | Notes |
|---|---|---|---|
| pub | `/battery` | `sensor_msgs/BatteryState` | `MODULE_BATTERY` (5), ~1 Hz. Voltage raw is 0.1 V (`battery_voltage_scale`) |
| pub | `/odom` | `nav_msgs/Odometry` | dead reckoned from `SpeedData` + MCU yaw; degrades to the commanded velocity when nothing is measured |
| pub | `/imu` | `sensor_msgs/Imu` | **placeholder, only while the MCU emits `MODULE_IMU` (9)** — otherwise nothing is published (the real IMU is the WIT JY61P on `/dev/serial_imu`, handled by `wit_imu_driver`) |
| pub | `/mower_sensor_info` | `mower_interfaces/MowerSensorInfo` | published **only if** `mower_interfaces` has been created; otherwise the publisher is not created at all |
| sub | `/cmd_vel` | `geometry_msgs/TwistStamped` | mapped to a `SpeedData` (4) command frame |

Parameters are listed in the module docstring of `mower_mcu_driver/mcu_node.py`
(`port`, `baud`, `heartbeat_period`, `cmd_vel_timeout`, `stop_frames`, `stop_frame_spacing_s`,
`speed_stream_enabled` (debug), `speed_cmd_rate` (debug stream rate), `linear_scale`,
`angular_scale`, `linear_max`, `angular_max`, `use_imu_yaw`, `*_frame`, battery scaling …).

## Known gaps (do not treat as safety-complete)

| Gap | Status |
|---|---|
| **SpeedData TX** | Vendor pass-through: one SpeedData per `/cmd_vel` message (scale + clamp ±0.3), no host PID (the vendor `libpid_controller.so` is the bumper back-off position primitive, not a wheel loop). After motion, a zero / 0.5 s silence / interlock sends 3 zeros 100 ms apart, then silence. See `docs/wheel_control_semantics.md` §5. |
| **Heartbeat period** | Unknown on hardware. Defaults to 100 ms (same as the BLE link) as a starting point; the MCU failsafe timeout must be characterised with the wheels off the ground first. Parameter `heartbeat_period`. |
| **E-stop passthrough** | Not wired. The MCU enforces estop/lift/bumper cut-offs itself; no host→MCU estop frame has been identified. Documented `TODO(estop)`. |
| Odometry | Open-loop integration, drifts. `/reset_odom` and `/tf` broadcasting not implemented yet. |
| `/cmd_vel` unstamped | Vendor accepted both `/cmd_vel` (`Twist`) and `/cmd_vel_stamped`; only `TwistStamped` is wired for now. |
| `MowerSensorInfo` | `key_pressed`, `rain_sensor_value`, `bumper_routing_*`, `is_docking_done` have no source on this bus yet. Field assignment is guarded (`_set_if`), so a reduced interface package still works. `is_fill_light_on` mirrors `/fill_light/state` from `fill_light_node` (host PWM, see `fill_light.py`). |

## Build

```bash
cd ros2_stack
./scripts/build.sh --packages-select mower_mcu_driver
```

`mower_interfaces` is declared as a `<depend>` so colcon orders the build correctly as soon
as the package exists; until then the node logs a warning and skips `/mower_sensor_info`.

```bash
ros2 run mower_mcu_driver mcu_node --ros-args -p port:=/dev/serial_mower
```

Stop the vendor driver first (`systemctl stop mower-base.service`), wheels off the ground,
blade off — two writers on one UART garble both.

Serial backend: pyserial (`python3-serial`) when available, otherwise a built-in raw-mode
`termios` fallback, so the node also runs in a bare `ros:jazzy` image.

## Tests

```bash
python3 src/mower_mcu_driver/test/test_protocol.py   # no ROS needed
python3 src/mower_mcu_driver/test/test_node_smoke.py # no ROS needed
python3 src/mower_mcu_driver/test/test_loopback.py   # no ROS needed, needs socat
```

* `test_protocol.py` replays the real 2.8 MB capture in `mcu_traffic_logs/` through both this
  parser and the reference decoder and requires an **identical sub-packet histogram**.
* `test_node_smoke.py` drives `McuNode` against a fake serial port (battery/IMU/odom
  publishing, scale+clamp+timeout on `/cmd_vel`, heartbeat shape, sensor-info skip path).
* `test_loopback.py` opens a real socat pty pair, runs the synthetic MCU on one end and the
  node on the other, and asserts a full host→MCU→host round trip.

They install tiny `rclpy`/message stand-ins via `test/ros_stubs.py` **only when ROS is not
installed**, so the same files run unchanged under `colcon test` in the container.

Interactive loopback (real `ros2 run`, same socat setup):

```bash
src/mower_mcu_driver/scripts/loopback_test.sh          # add --imu to exercise /imu
# second shell:
ros2 topic pub --rate 5 /cmd_vel geometry_msgs/msg/TwistStamped \
    "{twist: {linear: {x: 0.3}, angular: {z: 0.2}}}"
```

`fake_mcu` mimics the firmware: `SpeedData`+`SensorInfo` at 60 Hz (measured speed echoes the
last command), battery+version+bms+motors batched at 1 Hz, optional `ImuData` (`--imu`), and
it prints the received heartbeat/command counters.

## Layout

```
package.xml setup.py setup.cfg resource/   ament_python packaging
mower_mcu_driver/mcu_node.py               protocol codec + the driver node
mower_mcu_driver/fake_mcu.py               synthetic MCU for the loopback test
scripts/loopback_test.sh                   socat pty pair + fake MCU + node
test/                                      protocol / smoke / loopback tests + ROS stubs
```
