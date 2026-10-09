# Top-panel buttons (`base_keys`)

The physical buttons are handled by one node, `src/base_keys` (`base_keys_node`,
node name `base_keys`). It replaces both halves of the vendor's key handling:

- the ROS 1 `mower_base_node` `mower_base::Buttons` class (`button.cpp`), which reads evdev and publishes `/mower_base/button_info`;
- the key branch of the vendor `mower_logic` `TaskManager::sensorStateCallback` (`task_manager.cpp`), which turns keys into actions.

The pure logic (event decoding, device discovery, the press/long-press state machine and the key-to-action table) is in `base_keys/keys_logic.py`. It needs no ROS or evdev and is tested in `src/base_keys/test/test_keys_logic.py`.

## Input device

| | |
|---|---|
| Name (`/proc/bus/input/devices`) | `key input` (virtual input device, `Bus=0000 Vendor=0000 Product=0000`) |
| Node | `/dev/input/event5` |
| Vendor path | `/dev/keyboard`, a udev symlink to `input/event5` (from `Buttons::Buttons`: `"/dev/keyboard"`) |
| `B: KEY=` | `4404082080000`, i.e. key codes 19, 25, 31, 38, 46 and 50 |

The other input devices are not panel keys: `rk805 pwrkey` (event0), `headset-keys` (event1), es8388 headset (event2), `bt-powerkey` (event3) and `adc-keys` (event4).

The node picks the device in this order:
1. the `device` parameter, if set;
2. the device whose name matches `device_name` (default `key input`);
3. `fallback_device` (default `/dev/keyboard`);
4. the first device that exposes the `key_power`, `key_work_pause` and `key_go_docking` codes.

The node reads raw `struct input_event` (24 bytes) without `python-evdev`. If the device returns an error, EOF or POLLHUP, the node closes it, publishes key 0, forgets held keys, and retries discovery every `reopen_period_s`. It logs the failure reason once rather than on every retry. The device does not need to be grabbed, because the vendor never grabbed it either.

## Kernel module `init_test.ko` (where the timing really happens)

The "key input" device is created by the vendor kernel module
`/lib/modules/5.10.209/kernel/drivers/init_test.ko` (dmesg `get key 2` / `get key 3-` /
`input: key input` / `get key 6-` at ~16 s after boot). Disassembled 2026-10-09
(`input_thread`, capstone): a kthread reads three GPIOs (`gpiod0` = power key, `gpiod1`/`gpiod2`
= the other keys, decoded as a 2-bit combination) and sleeps `usleep_range(40000, 40000)`, so the
poll period is **40 ms**. For the power key it counts consecutive pressed ticks in
`power_key_counter`:

| hold (ticks x 40 ms) | while held | on release |
|---|---|---|
| 1 tick (< 80 ms) | nothing | nothing (bounce filter) |
| 2 to 19 ticks (80 to 760 ms) | nothing | `short press detect`: KEY_P press + release |
| 20 to 74 ticks (0.8 to 3.0 s) | nothing | `ignore short press detect`: nothing |
| 75 ticks (**3.0 s**) | `long press detect`: KEY_L press + release, **immediately** | |
| after a long press | | `long press release`: KEY_R (19) press + release, then `start shutdown process`, `call_usermodehelper("/usr/bin/sys_close", UMH_WAIT_EXEC)` as root on the host, `finish shutdown process` |
| > 750 ticks (30 s) | counter pinned at 1000, `stuck detect` | nothing (no KEY_R, no sys_close) |

So the 3 s threshold, the debounce and the "stuck button" filter are all in the kernel, KEY_P
is never emitted for a long hold, and KEY_P press/release always arrive together. On the mower
`/usr/bin/sys_close` is a 12-byte empty bash script (`#!/bin/bash`): the vendor hook does
nothing. `/etc/input-event-daemon.conf` maps `P = sys_close` too, but `event_handle.service`
(input-event-daemon) fails at boot because event5 does not exist yet, so it is inert.
The other keys emit KEY_C (46), KEY_S (31) and KEY_M (50) as press+release on a GPIO change.

## Vendor key table

Source: `Buttons::handleKey` (`native_decompile/dec/mower_base_node/mower_base_node.c` around line 240378). The key-name strings were resolved from the binary at `DAT_00480520..60`.

| evdev code | key | bit (`MowerBaseButtonInfo` / `MowerSensorInfo.key_pressed`) | vendor log name |
|---|---|---|---|
| 0x19 = 25 | KEY_P | 4 `KEY_POWER_SHORT` | 电源键短按 (power key short press) |
| 0x1f = 31 | KEY_S | 2 `KEY_GO_DOCKING` | 回充键 (return-to-dock key) |
| 0x26 = 38 | KEY_L | 8 `KEY_POWER_LONG` | 电源键长按 (power key long press) |
| 0x2e = 46 | KEY_C | 1 `KEY_WORKING_OR_PAUSE` | 暂停键 (pause key) |
| 0x32 = 50 | KEY_M | 16 `KEY_DOCK_AND_PAUSE` | 回充暂停组合键 (dock + pause combination) |
| 19 | KEY_R | not handled: release of a >= 3 s power hold (see above). The vendor logs `MowerBase: Unknown key code: 19` | |

Vendor behaviour in `mower_base_node`:
- Only `type == EV_KEY` is handled.
- `value == 0` (release) publishes a `MowerBaseButtonInfo` and pushes `0` to the key queue.
- Any other value looks up the code, logs `"%s触发"` (e.g. `handleKey: 电源键短按触发`), publishes `/mower_base/button_info`, and pushes the bit into a 10-deep spsc queue.
- `MowerBase::speedDataCallbackSDK` pops one entry every third speed frame into `MowerSensorInfo.key_pressed`. The key is therefore a one-shot event, not a level.
- There is **no long-press timer and no debounce** in software. `BaseKeys::readLoop` just does `poll(1000 ms)` followed by `read(24)`. The "key input" device reports short and long power presses as *different codes*: KEY_P and KEY_L. A user test on 2026-10-06 measured a short power press as KEY_P value 1, then value 0 in the same tick.
- `mower_base_node` takes no action on keys itself.

Vendor actions in `mower_logic` (`libmower_logic_task.so`, `TaskManager::sensorStateCallback`, disassembled around 0x3a05a0):
- **go dock (2)**: on the rising edge, logs `"go dock triggerd, start go dock"` and sets the pending command to dock.
- **start/pause (1)**: on the rising edge, logs `"start or pause task key triggerd"`. The task loop then does `"current task is running, pause task"`, `"current task is pause, resume task"` or `"current no task, start task"`.
- **power short (4)**: logs `"short power key triggerd!!"`, then calls `RobotDataHandler::clearEstopAlarm()` (this calls `/clear_estop`) and `TaskManager::clearAllFaultCode()`. This matches the device log `handleKey: 电源键短按触发` followed by `clearEStopROS: 解除急停命令`.
- **power long (8)**: logs `"long power key triggerd!!"`, then:
  1. `stopTask(false)`;
  2. `sendNotice(900005)`;
  3. sleeps 3000 ms;
  4. `RobotDataHandler::poweroff()` (the `/poweroff` service on mower_base, which sends MCU module 10 with byte 1);
  5. `system("shutdown -h now")`.
- **dock+pause (16)**: no handler was found in `sensorStateCallback`.

## Our mapping

Bits are published on the **press** edge, as the vendor does, on both:
- `/mower_base/button_info` (`mower_interfaces/MowerBaseButtonInfo`, `key_value` = bit);
- `/mower_base/key_pressed` (`std_msgs/UInt8`, `data` = bit).

When all keys are released, both topics get a `0`. The exception is the power key when a software long press is enabled: see `long_press_ms` under Parameters. `/mower_base/key_pressed` is meant for the coordinator to merge into `MowerSensorInfo.key_pressed`. `mcu_node` currently leaves `key_pressed` at 0.

| key | action (default) | parameter |
|---|---|---|
| power short (4) | call `/clear_estop` (`std_srvs/Empty`: clears the host latch and sends the vendor module-10 frame), then `HighLevelControl` `RESET_EMERGENCY` (254). The second call is our equivalent of `clearAllFaultCode`. | `on_power_short`, `power_short_reset_emergency` |
| start/pause (1) | if `high_level_status.state` is AUTONOMOUS (2) or MANUAL_MOWING (4), send `STOP` (8). If IDLE (1), send `START` (1). In RECORDING or NULL, or before any status has arrived, it only logs. | `on_work_pause` |
| go dock (2) | `HighLevelControl` `HOME` (2) | `on_go_docking` |
| dock+pause (16) | log only. `on_dock_and_pause:=true` treats it as go-dock. | `on_dock_and_pause` |
| power long (8) | `power_long_action` (see below) | `power_long_action` |

`power_long_action` takes one of these values:
- `log_only`: only logs.
- `poweroff_service`: sends `STOP`, then calls `/poweroff` (`std_srvs/Empty`) if the service exists, and otherwise logs a warning. Our `mcu_node` does **not** serve `/poweroff` yet. The vendor implementation sends MCU module 10 with `[0,0,0,0,1,0,0,0]`, which cuts power.
- `sequence` (default, see "Power off" below): the clean power-off sequence.
- `shutdown`: sends `STOP`, then writes a timestamp to `shutdown_hook_file`. A clean OS shutdown cannot be done from inside the container, and it is **not implemented** here. A host-side unit, e.g. a systemd `.path` unit watching that file on a bind-mounted `/userdata` path that runs `systemctl poweroff`, must be added separately. If `shutdown_hook_file` is empty, the node only logs a warning.

Services are called asynchronously and only if the service is available. Otherwise the node logs `service X not available, skipped`. `actions_enabled:=false` turns off every action and leaves only the publishers.

## Parameters

| name | default | meaning |
|---|---|---|
| `device` | `''` | explicit `/dev/input/eventN` (skips discovery) |
| `device_name` | `key input` | name in `/proc/bus/input/devices` |
| `fallback_device` | `/dev/keyboard` | vendor path, tried if the name is not found |
| `reopen_period_s` | 2.0 | retry period when the device is missing or lost |
| `key_power` / `key_power_long` / `key_work_pause` / `key_go_docking` / `key_dock_and_pause` | 25 / 38 / 46 / 31 / 50 | evdev codes (vendor values) |
| `long_press_ms` | 3000 | software long press on `key_power`. With a value > 0, power short is emitted on *release* if held for less than the threshold, and `KEY_POWER_LONG` is emitted once the hold reaches it. `0` gives pure vendor behaviour: power short on press, and long press only from the device's KEY_L. The vendor has no software threshold. 3000 ms is our choice, and it only matters if the device ever holds KEY_P down. |
| `debounce_ms` | 50 | ignores a press of the same code, and its release, within this window of the last accepted press. Autorepeat (value 2) is always ignored. |
| `actions_enabled` | true | master switch for actions |
| `on_power_short` | true | `/clear_estop` on power short |
| `power_short_reset_emergency` | true | also `HighLevelControl` 254 |
| `on_work_pause` | true | START/STOP toggle |
| `on_go_docking` | true | HOME |
| `on_dock_and_pause` | false | treat the combo key as go-dock |
| `power_long_action` | `sequence` | `sequence`, `log_only`, `poweroff_service` or `shutdown` |
| `power_request_dir` | `/userdata/ros2/power` | `sequence`: where the shutdown request goes (bind-mounted host dir) |
| `power_settle_s` | 3.0 | `sequence`: delay between the safety stop and the host request (vendor: 3000 ms) |
| `power_min_uptime_s` | 30.0 | `sequence`: long presses earlier than this after host boot are ignored |
| `cutter_off_service` / `light_control_service` | `/cutter_off` / `/light_control` | |
| `shutdown_hook_file` | `''` | file written for `power_long_action=shutdown` |
| `clear_estop_service` | `/clear_estop` | |
| `poweroff_service` | `/poweroff` | |
| `high_level_control_service` | `/behavior_tree_node/high_level_control` | served by `mower_mission`, or by the gui-bridge stub |
| `high_level_status_topic` | `/behavior_tree_node/high_level_status` | |

## Launch

`launch/bringup.launch.py` starts `base_keys` with `respawn=True` and `respawn_delay=2.0`, gated by `keys:=true` (the default). Turn it off with `ros2 launch mower_bringup mower.launch.py keys:=false`, or with `bringup.launch.py keys:=false`.

## Power off (hold the power key for 3 s)

Vendor reference: `mower_logic` on KEY_POWER_LONG did `stopTask(false)`, `sendNotice(900005)`
(PowerOff voice prompt), sleep 3000 ms, `/poweroff` (MCU module 10 byte 4 = 1, cuts power) and
`system("shutdown -h now")`, i.e. it cut MCU power while the OS was still shutting down, with no
charging/dock gate. The kernel module additionally runs `/usr/bin/sys_close` on release (empty).

Our sequence (container side `base_keys`, `power_long_action:=sequence`; host side
`mower-poweroff.path` / `mower-poweroff.service` / `mower-poweroff-host`):

1. KEY_L at 3.0 s (still held). Ignored if host uptime < `power_min_uptime_s` or a sequence is already running.
2. Immediately: `/estop_request true` (mcu_node latches zero motion and sends cutter off), `HighLevelControl STOP`, `/cutter_off`, `/light_control PowerOff`.
3. After `power_settle_s` (3 s): PowerOff light again (the e-stop switched the auto light to WarnSensorTrigged), then `/userdata/ros2/power/shutdown_request` is written atomically (JSON: source, boot_id, uptime_s, wall_time, hl_state).
4. On release the kernel runs `/usr/bin/sys_close`, which we replace with a forwarder that writes a `source=kernel` request. This path works even when the ROS stack is down. The host handler waits `KERNEL_GRACE_S` (6 s) for the stack's own request before acting on a kernel-only one.
5. Host: `mower-poweroff.path` (`PathExists`) starts `mower-poweroff.service` -> `mower-poweroff-host handle`. It validates the request (same `boot_id`, so a stale file from an earlier boot never powers off a fresh boot; age < `MAX_AGE_S`; made after `MIN_UPTIME_S`), deletes it, archives it to `/userdata/ros2/power/last_poweroff.json`, arms `/run/mower-poweroff/mcu_cut` if `MCU_POWER_CUT=1`, runs `sync` and then `systemctl poweroff --no-block`. With `DRY_RUN=1` (the installed default) it only logs (`journalctl -t mower-poweroff`).
6. systemd stops docker (ordered before the `/userdata` unmount). `mower_humble` gets SIGINT with a 30 s grace (`stop_signal`/`stop_grace_period` in `docker/docker-compose.yml`). ros2 launch SIGINTs every node: bag_recorder SIGINTs rosbag2 (metadata written), mow_recorder and mcu_node close (mcu_node sends cutter-off on shutdown), and light_controller plays PowerOff on exit.
7. After every filesystem is unmounted or read-only, `/usr/lib/systemd/system-shutdown/mower-mcu-poweroff` sends the vendor `/poweroff` frame `A5 0B 08 00 0A 00 00 00 00 01 00 00 00 1E 5A` three times on `/dev/ttyS9`, but only for `poweroff` and only when the marker is armed. The MCU then cuts the battery. A plain `sudo poweroff` over ssh does not cut MCU power.

Docked or charging: no gate. The vendor had none, and holding the key for 3 s is an explicit
request. Whether the charger re-powers the mower after the MCU cut is untested.

Why the host and not the container: the container is not in the host PID namespace, so it cannot
run `systemctl poweroff` itself, and the vendor order (MCU cut first, `shutdown -h now` second)
risks cutting power while `/userdata` is still mounted read-write.

Install on the host (root, after copying the files listed in the script):
`sudo bash /userdata/ros2_stack/scripts/install_power_button.sh`. It installs with `DRY_RUN=1`
and `MCU_POWER_CUT=0`; edit `/etc/default/mower-poweroff` to enable each step.
`--uninstall` restores the vendor `sys_close`.

Unknowns, to settle by live test: whether the MCU cuts power immediately on the frame or after a
delay, whether it needs anything else (heartbeat), and what happens if the frame is never sent.
In that last case the SoC halts and the MCUs stay powered; the kernel key module is gone, so only
the MCU/BMS can turn the mower off.

## Open points

- KEY_R (19) marks the release of a long power press. The node logs it.
- The dock+pause combo (KEY_M) does nothing in the vendor `mower_logic`, and we only log it by default.
- Resolved 2026-10-09 from `init_test.ko`: KEY_L fires at 3.0 s while held, and no KEY_P is emitted for a long hold (see the kernel module section).
