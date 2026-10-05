# Status LEDs (`mower_lights`)

This package ports the vendor status-LED control to ROS 2. It contains the node
`mower_lights/light_controller` and the interface package `mower_lights_interfaces`.

Before this port, nothing in our stack drove the LEDs. A WS2812 strip keeps the last
frame it received, so the LEDs stayed at whatever the vendor node sent last. On the
device, that was `WarnSensorTrigged` (19), which is solid red. The vendor log shows the
sequence:

- `logic.log` 2026-10-05 18:05:32: `lightControl: [-1--->19] ... WarnSensorTrigged`
- Then the vendor stack was killed (`Light control thread exit` at 18:06:47).

## Hardware: a WS2812 chain on SPI 3

The evidence for this comes from three places.

**The vendor node.** The vendor node is
`devel/mower_light_sound/lib/mower_light_sound/mower_light_sound_node`, built from
`src/mower_task/mower_light_sound/src/light_control.cpp`, class `LightManager`. It is
stripped, but its C++ symbols survived. It links `libws_2812.so`
(`src/mower_task/mower_light_sound/lib/`), which exports `ws2812_init`, `ws2812_send`
and `ws2812_exit`.

**The Ghidra decompile of `libws_2812.so`.**

| Item | Value |
|---|---|
| Device | `open("/dev/spidev3.0", O_RDWR)` |
| ioctls | `SPI_IOC_WR_MODE32` = 0, `BITS_PER_WORD` = 8, `MAX_SPEED_HZ` = `0x7a1200` (8 MHz) |
| Frame size | Always 60 LEDs × 24 bit |
| Bit encoding | Each colour bit becomes one SPI byte, MSB first: `0xFC` for a 1 (750 ns high), `0xC0` for a 0 (250 ns high) |
| Padding | 100 zero bytes before and after the data, so a frame is `0x668` = 1640 bytes |
| Transfer | One `SPI_IOC_MESSAGE(1)`, with `usleep(5000)` before and `usleep(15000)` after |

**The device itself.**

- `/sys/bus/spi/devices/spi3.0` is bound to the `spidev` driver (`dh2228fv`), and
  `/dev/spidev3.0` exists.
- `/sys/class/leds` holds only the board heartbeat LED (`led-2`) and `mmc0`. The status
  LEDs are not driven through sysfs, PWM or I2C.
- The fill light (`/fill_light_control`, `pwmchip1`) is a different device. This package
  does not touch it.

The frame buffer has 3 bytes per LED, in wire order **(G, R, B)**. The
`GeneralErrorRed` mode (26, commented "通用错误红灯", "generic error red light") writes
byte 1, which confirms that byte 1 is red.

### LED layout

Two values set the layout:

- F is the number of front LEDs per strip. `readSn("/meta/sn")` sets it: F = 7 when
  `SN[7:10]` is `B02` or `203`, otherwise 14. This device's SN is
  `1001027304000822`, so F = 14. The vendor logs `front led num 14`.
- T = 27 is the number of top LEDs.

| Area | LEDs (F = 14) | Used by |
|---|---|---|
| 0 | 0–13 (strip A) | – |
| 1 | 14–27 (strip B) | – |
| 2, "tail" | 0–27 (both strips) | `tailLightThread` |
| 3, "top" | 28–54 | `topLightThread` |
| 4 | 0–54 | – |

LEDs 55–59 are always clocked out dark. "Top" and "tail" are the vendor's own names.
I could not confirm which physical strip is which.

### Threads (vendor and port)

The port uses the same three threads as the vendor:

- **Sender**, 25 Hz: sends the whole buffer.
- **Top lane**, 20 Hz.
- **Tail lane**, 20 Hz.

Each lane runs the pattern for the current mode. Waits can be interrupted, as with the
vendor `WakeAbleTimer`. A pattern marked "steady" runs only when the mode changes.
Animations re-run in a loop.

`/light_control` arbitration follows the vendor `lightModeService`. Each mode has a
priority (lower is stronger), taken from the map in the vendor `LightManager`
constructor:

| Priority | Modes |
|---|---|
| 0 | 6, 7, 22, 23 |
| 1 | 2, 8, 9, 10, 11, 17, 25 |
| 2 | 19, 20 |
| 3 | 0, 3, 4, 5, 12–16, 18, 21, 26, 27 |
| 4 | 1, and every unmapped mode |

The newest request always becomes the mode. A running animation is cut short only when
the new mode is different and its priority is ≤ the current lane's priority.

The cutter modes 22 and 23 are one-shots that go only to the tail lane. `CloseCutter`
runs only if `/mower_base/status.is_cutting` is set.

## Mode table

The vendor code calls the colours by number. Brightness 120 is "full" and 60 is "dim".
Every value is scaled by `brightness`/100 (vendor config `SetLightBrightness`,
default 100).

| Mode | Top (27) | Tail (2×14) |
|---|---|---|
| 0 Mapping, 16 TaskIng | White, steady | White, steady |
| 1 Idle | White breathing: table values 1..119, 30 ms/step, 97 steps (about 2.9 s) | Same |
| 2 TaskStart | White blinks twice (500 ms on/off), then white steady | Same |
| 3, 5, 11, 12, 18 | No change | No change |
| 4 TaskPause | Yellow (R+G), steady | Same |
| 6 PowerOn | Off, 0.5 s, white fill from both ends to the middle (30 ms/step), then switches to **Idle** | White sweep top-to-bottom (150 ms/step), then off |
| 7 PowerOff | White, 0.5 s, then wipes off from the middle out (100 ms/step) | Wipes off bottom-to-top (150 ms/step) |
| 8 OTAIng | Loop: off, 0.5 s, blue(60) middle-to-ends at 100 ms/step | Loop: off, 0.2 s, blue(60) bottom-to-top at 150 ms/step, off |
| 9 OTAFinished | White, steady | White, steady |
| 10 PairIng | Blue blinks 500/500 ms | Same |
| 13 Docking | Green, steady | Same |
| 14 Charging | Loop: green(60) middle-to-ends at 100 ms/step, then off | Loop: off, 0.2 s, green(60) bottom-to-top at 150 ms/step, off |
| 15 ChargeFinished | Green, steady | Same |
| 17 LowBattery | Red breathing | Same |
| 19 WarnSensorTrigged | **Red, steady** | Same |
| 20 RobotUnusual | Red blinks 500/500 ms | Same |
| 21 SignalError | Red, steady | Same |
| 22 OpenCutter | – | One-shot: off, 0.2 s, red sweep top-to-bottom at 200 ms/step for 4 s |
| 23 CloseCutter | – | One-shot: the same, bottom-to-top |
| 24 RemoteControl | **No change.** The vendor has no pattern for this mode, so the LEDs freeze. | Same |
| 25 PairFailed | Red, two blinks, repeating | Same |
| 26 GeneralErrorRed | Red, steady | Same |
| 27 GeneralNormalWhite | White, steady | Same |
| −1 Unknown | No change | No change |

The breathing table is `RESPIRATION_LAMP_TABLE`, 69 bytes at `.data` 0x210010. It is
copied verbatim into `lights_logic.py`.

## Automatic mapping from stack state (`auto_mode: true`)

The node subscribes to:

- `/behavior_tree_node/high_level_status`
- `/hardware_bridge/emergency` (`active_emergency`, `latched_emergency`, `lift_warning`)
- `/mower_base/status` (`stop_triggered`, `lift_triggered`, `bumper_triggered`,
  `is_charging`, `is_cutting`)
- `/battery`, used only as a fallback for the battery percentage

The mapping is edge-triggered: the node sends a mode only when the mapped mode changes.
A manual `/light_control` call therefore stays in effect until the stack state changes.
Rules are checked in this order:

1. Node shutting down: **PowerOff**.
2. Emergency, lift, stop or bumper is active, the state is `EMERGENCY` or
   `BOUNDARY_EMERGENCY_STOP`, or `HighLevelStatus.emergency` is set:
   **WarnSensorTrigged**. The mode is held for `warn_hold_s` after the cause clears.
3. No status received, or the last one is older than `status_timeout_s`: **Idle**, or
   Charging if the base reports charging.
4. `NAV_TO_DOCK_FAILED` or `UNDOCK_FAILED`: **RobotUnusual**.
5. `MANUAL_MOWING` or `RECORDING` (state 3 or 4): **RemoteControl**. As with the vendor,
   the LEDs do not change.
6. `LOW_BATTERY_DOCKING`: **LowBattery**.
7. `RETURNING_HOME`, `RAIN_DETECTED_DOCKING` or `COVERAGE_FAILED_DOCKING`: **Docking**.
8. `PREFLIGHT_CHECK`, `UNDOCKING`, `WAITING_FOR_RTK`, `PLANNING`, `MOWING`, `TRANSIT`,
   `AREA_UNREACHABLE` or `MOWING_COMPLETE`: **TaskStart** (blinks twice, then white).
9. Charging (`is_charging` or `CHARGING`): **Charging**, or **ChargeFinished** once
   battery ≥ `charge_full_percent`.
10. `BOUNDARY_PAUSED` or `RAIN_WAITING`: **TaskPause**.
11. `IDLE`, `IDLE_DOCKED` or `RECORDING_COMPLETE`: **Idle**, or **LowBattery** if
    0 < battery < `low_battery_percent`.

At startup the node runs **PowerOn**, which hands over to Idle. Auto mapping starts after
that.

## Interfaces

**`/light_control`** (`mower_lights_interfaces/srv/LightControl`): request
`LightMode mode`, empty response. This is the vendor shape. `LightMode.msg` keeps the
vendor values. The names are UPPER_SNAKE (`WARN_SENSOR_TRIGGED = 19`) because ROS 2
rejects the vendor's CamelCase constant names. The vendor name is in a comment on each
line.

```
ros2 service call /light_control mower_lights_interfaces/srv/LightControl "{mode: {light_mode: 19}}"
```

**`/light_info`** (`std_msgs/Int32`): the current mode, as in the vendor.

**`/light_controller/state`** (`std_msgs/String`, JSON, 2 Hz): readback of the frame
buffer and the hardware. It contains:

- the mode
- a per-area colour summary
- the hex buffer
- `frames_sent`
- `last_frame_matches_buffer` (the last SPI frame decoded back and compared with the
  buffer)
- the kernel SPI counters from `/sys/class/spi_master/spi3/statistics`

## Parameters

Defaults are in `src/mower_lights/config/lights.yaml`.

| Parameter | Default | |
|---|---|---|
| `auto_mode` | true | Map the stack state to modes automatically |
| `dry_run` | false | Encode frames but do not write SPI (launch arg `lights_dry_run`) |
| `spi_device` | `/dev/spidev3.0` | |
| `spi_speed_hz` / `spi_mode` / `spi_bits_per_word` | 8000000 / 0 / 8 | |
| `spi_stats_dir` | `/sys/class/spi_master/spi3/statistics` | Source of the readback counters |
| `sn_path` | `/meta/sn` | Not mounted in the container, so F falls back to 14 |
| `front_led_num` | 0 | 0 means derive F from the SN |
| `top_led_num` / `buffer_leds` | 27 / 60 | |
| `brightness` | 100 | Percent |
| `frame_rate_hz` / `lane_rate_hz` | 25 / 20 | |
| `startup_mode` / `shutdown_mode` / `shutdown_anim_s` | 6 / 7 / 2.0 | |
| `low_battery_percent` / `charge_full_percent` | 20 / 100 | |
| `warn_hold_s` / `status_timeout_s` | 2.0 / 5.0 | |
| Topic names | See `DEFAULTS` in `light_controller.py` | |

The container is privileged and bind-mounts `/dev`, so it needs no compose change.

**Launch:** `bringup.launch.py` starts the node with `lights:=true` (the default) and
respawns it if it exits. Use `lights_dry_run:=true` to run without writing SPI.

## Deliberate deviations from the vendor

- **PowerOn hand-over.** After its animation, the vendor PowerOn sets the mode to Idle
  even if another mode arrived during the animation. The port switches to Idle only if
  the mode is still PowerOn.
- **Interrupted patterns stop at once.** The vendor still runs the tail of a pattern
  after an interrupt, for example the final steady white of TaskStart. This lasts at
  most one lane cycle.
- **Not ported.** The vendor `SoundLightParams` dark-mode, volume and brightness config
  (`/app_config`, `/vio/light_status`) and the audio (`/mower_sound/play`).
  `brightness` is a static parameter.

## Tests

`src/mower_lights/test/test_lights_logic.py` (32 tests) covers:

- the enum and priority tables
- GRB colour bytes
- the SPI encoding, checked against `libws_2812`
- each pattern family
- arbitration
- the state-to-mode mapping
- an end-to-end engine run on the fake backend
