# base_keys

Reads the mower's top-panel buttons from an evdev `/dev/input` node and publishes
`mower_interfaces/MowerBaseButtonInfo` on `/mower_base/button_info`.

## Run

```bash
ros2 run base_keys base_keys_node \
  --ros-args -p device:=/dev/input/event0 -p long_press_ms:=3000
```

## Keycode mapping — must be confirmed on device

The physical keycode → semantic mapping was not recoverable from the stripped binary.
Defaults are:

| Parameter | Default | Key |
|---|---|---|
| `key_work_pause` | `119` | `KEY_PAUSE` |
| `key_go_docking` | `102` | `KEY_HOME` |
| `key_power` | `116` | `KEY_POWER` |
| `long_press_ms` | `3000` | short/long power threshold |

Confirm with `evtest` on the mower (press each button and read the `EV_KEY` code) and
override via parameters.
