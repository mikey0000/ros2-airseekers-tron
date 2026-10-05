# base_ble

Bridge between the Airseekers Tron BLE module (phone app) and ROS 2.

The BLE module is a transparent UART passthrough at `/dev/ttyS3` (`/dev/serial_ble`,
115200). Linux has no Bluetooth stack, so GATT UUIDs are only visible from the phone; this
node just reads/writes the framed serial protocol and dispatches commands to ROS.

## What was recovered

- **Framing**: faithful port of the surviving sources
  `src/mower_drivers/base_ble/src/protocol_{read,write}.cc` → [`ble_protocol.py`](base_ble/ble_protocol.py).
  Verified against the vendor worked examples in `mower_docs/reference/proto/ble.md`
  (see [`test/test_ble_protocol.py`](test/test_ble_protocol.py), 14 cases).
- **Dispatch table**: recovered from `BleNode::BleNode` in the stripped binary
  (`native_decompile/dec/base_ble_node/base_ble_node.c`), with the class/method for each
  command id (e.g. `0x84`→`Twist::set_linear`, `0x96`→`BleNode::twistMode`,
  `0x9a`→`BleNode::getRobotPose`).
- **Protobuf payloads**: regenerated as the `mower_proto` package from
  `mower_docs/reference/proto/*.proto`.
- **Safety behaviour** (recovered from `ble_twist.cpp` / `twist_mode.cpp`):
  - velocity = single **signed byte / 100** (m/s linear, rad/s angular);
  - `/cmd_vel` published only while in **teleop mode** (`0x96` START);
  - hard clamps **0.8 m/s** / **1.0 rad/s** (plus a deadband below 1 mm/s / 1 mrad/s);
  - 1 Hz watchdog stops the blade after **8 s with no drive command** while cutting.

## Assumptions / to confirm on hardware

- **Velocity units**: the binary does `byte / 100.0`, i.e. the byte is in cm/s (the BLE
  doc's `mm/s` note appears stale). Angular is the same `/100` (rad/s). Confirm turn rate
  on-device.
- **`0x9a` vs `0x99`**: the doc lists "localization status" as `0x99`, but the binary
  registers `getRobotPose` on **`0x9a`**. This firmware uses `0x9a`.
- **Mapping command strings** for EXTEND/EXPLORE use guesses (`extend`, `explore`,
  `explore_sure`, `explore_extend`) — the `MappingControl.srv` comment only documents
  start/save/cancel.
- **Cutter speed/position** values for START/STOP/height are placeholders
  (speed=100, position=height); confirm the real MotorControl mapping.
- **Keycode-free**: no evdev here, nothing to map.

## Not yet wired (need downstream nodes / types)

- Wi-Fi join (`nmcli`/NetworkManager) — ssid/pwd stored, join TODO.
- `/api/device/iot-cert` HTTP POST (cert activation `0x88`).
- `/localization/GetMapInfo` (`0x98`) — needs `geographic_msgs/GeoPose`; left as a decode.
- Version source for `0x94` (chassis/cutter/mower_package/rtk board) and localization
  state for `0x9a` — currently empty placeholders.
- `/mower_localization_info`, `/mower_gps_node/info`, `/notice_code` subscriptions
  (driver only publishes `/cmd_vel` today).

## Run

```bash
ros2 run base_ble base_ble_node --ros-args -p port:=/dev/serial_ble
```

Loopback test (no phone): `socat` a PTY pair and echo a frame, e.g.
`echo -n 'a5 89 00 00 89 5a' | xxd -r -p > /dev/serial_ble_client`.
