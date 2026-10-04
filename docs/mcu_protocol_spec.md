# Airseekers Tron — MCU Serial Protocol Spec (chassis/cutter MCU ↔ RK3588)

Engineering spec for the ROS 2 driver's MCU-facing half. Sources:
`ros2_port_handoff/01_mcu_protocol/` (`protocol_definitions_recovered.h` regenerated from
DWARF in `libmower_sdk.a`, `mcu_frame_decode.py` validated on 172,850 live frames with 0
checksum errors, `mower_base_node_strings.txt`), plus the handoff README (live topic rates,
ROS 1 contract). All multi-byte fields are **little-endian**; all structs are packed.
Items marked `(?)` are unverified — see §11 and §13.

## 1. Physical layer

| Item | Value |
|---|---|
| Port | `/dev/serial_mower` → `ttyS9` (RK3588 UART9 @ `0xfebc0000`) |
| Settings | 115200 8N1, no flow control |
| Topology | `RK3588 ←serial_mower→ cutter_board ←uart→ chassis_board` — the **cutter board is the gateway**; chassis (wheels/bumper/lift/stop) hangs behind it |
| MCUs | Artery AT32 (firmware running: cutter 0.6.36, chassis 0.6.34, BMS 1.18.0) |
| Log capture | `/userdata/log/mcu.dat` (ProtocolManager::SaveLog) |

## 2. Wire framing

```
SOF 0xA5 | total_len | { sub_len | type_id | mod_id | payload[sub_len] }+ | checksum | EOF 0x5A
```

- `total_len` (u8): byte count from the **first `sub_len` through the last payload byte** (excludes SOF, itself, checksum, EOF).
- `sub_len` (u8): payload length; struct sizes match it exactly. Payload ≤ 255 B, frame body ≤ 255 B.
- `checksum` (u8): `(sum of all bytes from total_len through last payload byte) & 0xFF` — covers `total_len` + all sub-packet bytes, excludes the checksum itself.
- **Multi-subpacket frames are normal**: the MCU batches `BATTERY+VERSION+MOTORS` and `BMS_VERSION+mod7` together once per second.
- Parser contract (as in `mcu_frame_decode.py`): scan for SOF → find next EOF → unescape the inner bytes → accept only if `inner[0] == len(inner)-2` and `sum(inner[:-1]) & 0xFF == inner[-1]`; otherwise discard and resync at the next SOF. SDK receive path uses a 4096-byte ring buffer (`Protocol::ReceivePacket`).

## 3. Byte escaping (both directions, everything between SOF and EOF, including the checksum)

| Raw | Escaped |
|---|---|
| `0xA5` | `0xF5 0x01` |
| `0x5A` | `0xF5 0x02` |
| `0xF5` | `0xF5 0x03` |

Unescape: `F5 01/02/03` → `A5/5A/F5`; unknown `F5 xx` passes `xx` through (defensive; not observed).

## 4. TypeID — direction

| Value | Name | Direction / meaning |
|---|---|---|
| `0x00` | `TYPE_ROS_MOWER` | **host → MCU** (commands) |
| `0x08` | `TYPE_MOWER_ROS` | **MCU → host** (telemetry) |
| `0xFF` | `TYPE_HEARTBEAT` | host → MCU keepalive; payload = `Heartbeat` |
| `21`, `22` | `TYPE_APP_RTK` / `TYPE_APP_RTKBASE` | OTA/bootloader path only (`ota.py`), not runtime SDK; those boards are on `/dev/serial_rtk` |

## 5. ModuleID

| ID | Name | Direction | Payload | Rate (on wire) |
|---|---|---|---|---|
| 0 | `MODULE_ALL` | — | broadcast | — |
| 3 | `MODULE_CHARGE` | h→M | `ChargeControl` (1 B) | on demand |
| 4 | `MODULE_SPEED` | **both** | `SpeedData` (8 B) | h→M on `/cmd_vel`; M→h **~60 Hz** (measured) |
| 5 | `MODULE_BATTERY` | M→h | `BatteryInfo` (11 B) | ~1 Hz |
| 9 | `MODULE_IMU` | h→M | `ImuData` (18 B) | `SendImuData` exists (host forwards WIT IMU); required-ness unknown `(?)` |
| 10 | `MODULE_SENSOR` | **both** | M→h `SensorInfo` (10 B) **~60 Hz**; h→M `SensorInfoControl` (8 B) | — |
| 11 | `MODULE_CUTTER` | h→M | `CutterControl` (12 B) | on demand |
| 12 | `MODULE_VERSION` | M→h | `VersionInfo` (9 B) | ~1 Hz |
| 13 | `MODULE_MOTORS` | M→h | `MotorsInfo` (32 B) | ~1–2 Hz (README live: ~1 Hz; header: ~2 Hz) |
| 18 | `MODULE_CALIB` | M→h | `MCCalib` (3 B) | factory test result |
| 19 | `MODULE_LOG` | M→h | ASCII text | event-driven |
| 24 | `MODULE_BMS_VERSION` | M→h | `Version` (3 B) | ~1 Hz |
| 7 | *(not in SDK enum)* | M→h | 4 B LE uint32, +600/s → uptime/tick `(?)` | ~1 Hz |
| 37 / 23 / 16 | OTA ids (ota.py) | — | bootloader only | — |

## 6. Payloads — host → MCU (`type=0`, or `255` for heartbeat)

```c
struct Heartbeat   { u8 year(?) /*year-2000?*/, month, day, hour, minute, second, ms_h, ms_l; }; // 8 B, wall clock
struct SpeedData   { f32 linear;  f32 angular; };   // 8 B: linear m/s, angular rad/s (cmd_vel)
struct ChargeControl { u8 enable; };                // 1 B: 1=start, 0=stop
struct MotorControl  { u8 enable; u8 direction/*0 fwd,1 rev*/; u16 speed/*% 0-100, cutter may be raw: 1000 seen (?)*/; u16 position/*% 0-100 height target*/; }; // 6 B
struct CutterControl { MotorControl cutter_motor; MotorControl height_motor; };                  // 12 B
struct SensorInfoControl { u8 bumper, rain, lift, stop, power_off, battery_gate, cutter_size, press_module; }; // 8 B, likely per-sensor enable mask (e.g. /enable_bumper) (?)
struct ImuData     { i16 pitch, roll, yaw, accx, accy, accz, gyrox, gyroy, gyroz; };          // 18 B, WIT scaling (?) angle*32768/180, acc*32768/16g, gyro*32768/2000dps
```

## 7. Payloads — MCU → host (`type=8`)

```c
struct SensorInfo { u8 bumper/*any*/, rain, lift, stop/*big red button*/, power_off, battery_gate,
                      cutter_size/*0 ?,1 small,2 big*/, press_module(?), bumper_r, bumper_l; };  // 10 B, 0/1 flags
struct BatteryInfo { u16 voltage/*0.1 V: live 248 @ 99%*/; i16 current/*units (?)*/; u8 percentage;
                     i8 dock_ok/*1 on dock (?)*/; i8 temperature/*°C*/; u32 error/*bitfield→battery_error u64*/; }; // 11 B
struct MotorInfo { i16 speed/*units TBD, rpm? (?)*/; i16 current/*mA? live left -90 right -141 at rest (?)*/;
                   i16 voltage/*scaling differs per board: cutter 2478, L/R 16245 (?)*/;
                   i8 temperature/*°C*/; u8 status; };                                            // 8 B
// status: 0 normal, 1 overcurrent, 2 overvoltage, 3 undervoltage, 4 overheat, 5 stalled
struct MotorsInfo { MotorInfo cutter_motor, left_motor, right_motor, height_motor; };            // 32 B; height_motor shows garbage — not populated on this HW
struct Version     { u8 major, minor, patch; };
struct VersionInfo { Version cutter_board, chassis_board, rtk_board; };                          // 9 B; rtk_board = 0.0.0 (RTK is on /dev/serial_rtk)
struct MCCalib     { u8 calib, left_calib_ret, right_calib_ret; };                             // 3 B, factory motor calibration
// MODULE_LOG: raw ASCII string payload
```

## 8. SDK API to mirror (from `nm`/DWARF of `libmower_sdk.a`; ROS-independent, glog+pthread only)

- `SerialDevice(port, baud)`: `Init/Open/Close/ConfigDevice/Read/Write`.
- `Protocol`: `SendPacket(TypeID, ModuleID, payload)`; `SendSpeedData/SendCutterControl/SendChargeControl/SendHeartbeat/SendImuData/SendSensorInfoControl`; `ReceivePacket(handler(TypeID, ModuleID, payload), scratch)`; static `CalculateChecksum/EscapeData/UnescapeData/BytesToHexString`.
- `ProtocolManager(serial, log_path)`: `Start/Stop/ReceiveLoop/WriteLoop/SaveLog/HandlePacket`; callbacks `SetSensorInfoCallback / SetSpeedDataCallback / SetBatteryInfoCallback / SetMotorsInfoCallback / SetVersionInfoCallback / SetBMSVersionCallback / SetMotorsCalibrationCallback / SetLogCallback`.
- The vendor node adds `dev_health::DevHealthHandler::{updateHeartbeat, checkHeartbeat}`, `mower_base::{Motors, PowerManager, DevStatus}`, `factory::FactoryTest`, `sensor/odom.cpp`, `sensor/fill_light.cpp`, and links `libpid_controller.so`, `libbumper_controller.so`, `libbase_imu.so`.
- Option: link `libmower_sdk.a` directly from the ROS 2 node, or reimplement from the header.

## 9. `/odom` + `/cmd_vel` + PID contract

- **Command path:** `/cmd_vel` (and `/cmd_vel_stamped`) → `PowerManager::wheelVelCB` → `SpeedData{linear m/s, angular rad/s}` → `TYPE_ROS_MOWER / MODULE_SPEED`. **The MCU takes linear/angular only** — there is no per-wheel command on the wire.
- **Feedback path:** MCU publishes measured `SpeedData` at ~60 Hz; the host runs a **wheel-level PID** (`libpid_controller.so`, tuned by the `linear/angular PID` params in `config/base.yaml`) against wheel-velocity feedback and adjusts what it sends to the MCU.
- **Odometry is computed on the host** (`sensor/odom.cpp`) from `SpeedData` + **IMU yaw** (WIT JY61P on `/dev/serial_imu`, separate UART, 100 Hz / 98 Hz BW via `wit_imu_config.py`), published as `/odom` (`nav_msgs/Odometry`) and `/wheel_vel` (`TwistStamped`) at **100 Hz**. Handles time jumps (`"odom time jump detected %f secs"`) and `/reset_odom` (`"Reset odom to (0, 0)"`).
- IMU is also published as `/imu` + `/imu/temperature` at 100 Hz (via `libbase_imu.so`).

## 10. Topic / service inventory and rates (ROS 1 contract to reproduce)

Measured rates (from `mower_base_node` live): `/odom`, `/imu`, `/wheel_vel` **100 Hz**; `/mower_sensor_info` **33 Hz** (wire SENSOR is 60 Hz — decimate); `/mower_base/motor_info` **3.3 Hz**; `/battery` **1.7 Hz**; `/mower_base/status` **1 Hz**; `/fix` 10 Hz (GPS, not this bus).

| Publishes | Type | MCU source |
|---|---|---|
| `/odom` | nav_msgs/Odometry | SPEED + IMU yaw (host-computed) |
| `/imu`, `/imu/temperature` | Imu / Temperature | JY61P, not MCU bus |
| `/wheel_vel` | TwistStamped | SPEED |
| `/mower_sensor_info` | MowerSensorInfo | SENSOR (incl. `battery_error` bitfield, `cutter_size`, `rain_sensor_value`) |
| `/battery` | BatteryState | BATTERY (voltage, current, SOC, temp, dock_ok) |
| `/mower_base/motor_info` | MotorsInfo | MOTORS |
| `/mower_base/status`, `/mower_base/dev_base_info` | — | VERSION + BMS_VERSION |
| `/bumper_cloud` | PointCloud2 | SENSOR bumpers |
| `/tf`, `/mower_base/{button_info,battery_health,net_status}`, `/notice_code`, `/notice_info`, `/calibration_result` (← MCCalib), `/factory_test_result` | — | mixed |

| Subscribes | Services |
|---|---|
| `/cmd_vel`, `/cmd_vel_stamped` | `/cutter_control` (→ CutterControl) |
| `/clear_notice_code`, `/gps_dev_status`, `/mower_gps_node/info`, `/mower_localization_info`, `/rear_camera/camera_status` | `/charging` (→ ChargeControl), `/fake_charging`, `/enable_bumper` (→ SensorInfoControl), `/clear_estop`, `/reset_odom`, `/poweroff`, `/fill_light_control` (PWM `/sys/class/pwm/pwmchip1/pwm0/duty_cycle`), `/factory_test_req` (→ MCCalib), `/base_driver_node/test_bumper_service` |

Auxiliary (same node, non-MCU): rain ADC `/dev/rain` (thresholds `rain_min`/`rain_max` = 2000/4000), GPIO buttons `/dev/keyboard` (`input/event5`: pause/dock/power).

## 11. Heartbeat & failsafe — known vs unknown

**Known:** `TYPE_HEARTBEAT` (255) carries an 8-byte wall-clock timestamp; the host side has `DevHealthHandler::{updateHeartbeat, checkHeartbeat}`; the MCU enforces estop / lift / bumper cut-offs itself.
**Unknown (resolve on hardware before enabling motion):**
- Heartbeat **period** and **timeout** (timeout params exist in `config/base.yaml` but values were not recovered).
- MCU **failsafe behaviour** on missed heartbeats — "most likely stops motors", unverified. Characterise the timeout first (handoff caution).

## 12. ROS 2 driver node — implementation checklist

- [ ] Serial: open `/dev/serial_mower` 115200 8N1, no flow control, exclusive; params for port/baud; single writer (stop vendor `mower-base.service` first).
- [ ] Frame codec: build/parse per §2, escape/unescape per §3, checksum verify, resync on garbage, 4096 B ring buffer; log checksum-error rate.
- [ ] Tx: heartbeat timer (period TBD, §11), `SpeedData` from `/cmd_vel`, `CutterControl`, `ChargeControl`, `SensorInfoControl`, optional `ImuData` forward (decide after §11/§13 testing).
- [ ] Rx: dispatch `(type, mod)` → sensor/speed/battery/motors/version/bms/calib/log callbacks; handle multi-subpacket frames; decode mod 7 (uptime?) opportunistically.
- [ ] `/odom` + `/wheel_vel` @100 Hz from SPEED + IMU yaw; time-jump guard; `/reset_odom`.
- [ ] Sensor flags → `/mower_sensor_info` @33 Hz + `/bumper_cloud`; host-side estop/lift/stop/bumper cut-off in addition to MCU's.
- [ ] Battery → `/battery` BatteryState (scale voltage ×0.1 V; current units TBD); sanity checks (`"charging but percentage decrease"`).
- [ ] Motors → `/mower_base/motor_info` (ignore `height_motor` garbage); surface `status` enum + temperature.
- [ ] Version/BMS → `/mower_base/status`, `dev_base_info` @1 Hz (cutter/chassis versions; rtk is 0.0.0 here).
- [ ] Factory test: `/factory_test_req` → MCCalib → `/calibration_result`.
- [ ] All §10 services, incl. `/enable_bumper` → `SendSensorInfoControl`, `/poweroff`, `/fill_light_control` (PWM sysfs).
- [ ] Params (`config/base.yaml`): ports, baud, rain thresholds (2000/4000), linear/angular PID gains, timeouts (incl. heartbeat).
- [ ] Auxiliary devices: IMU via `/dev/serial_imu` (WIT 0x55 frames, 100 Hz), rain ADC, keyboard.
- [ ] Logging: forward MCU `LOG` (ASCII) to ROS logger; optional `mcu.dat` capture.
- [ ] Safety bring-up: wheels off ground, blade off, characterise heartbeat timeout before any motion test; validate against `mcu_frame_decode.py` (baseline: 172,850 frames, 0 checksum errors).

## 13. Open questions (marked `?` above)

Heartbeat period/timeout & failsafe behaviour; `BatteryInfo.current` units; `MotorInfo.speed/current/voltage` units & per-board scaling; module id 7 semantics; `SensorInfoControl` exact semantics (enable mask?); whether `SendImuData` is required by the MCU; `MotorControl.speed` scale for the cutter (1000 seen); `Heartbeat.year` epoch offset (2000?); `press_module` meaning.
