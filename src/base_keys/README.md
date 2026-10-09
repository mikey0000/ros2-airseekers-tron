# base_keys

Top-panel buttons for the Airseekers Tron. The node reads the `key input` evdev device (`/dev/input/event5`, vendor path `/dev/keyboard`) and publishes on:
- `/mower_base/button_info` (`MowerBaseButtonInfo`);
- `/mower_base/key_pressed` (`std_msgs/UInt8`).

It also maps keys to actions:
- power short calls `/clear_estop`;
- start/pause sends `HighLevelControl` START or STOP;
- dock sends HOME;
- power long (3 s hold, timed by the vendor kernel module) runs the clean power-off sequence: e-stop, mission STOP, cutter off, PowerOff light, then a request to the host helper (`scripts/install_power_button.sh`), which powers the host off and has the MCU cut the battery. See "Power off" in `docs/buttons.md`.

The full key table, vendor behaviour and parameters are in [`docs/buttons.md`](../../docs/buttons.md).

```bash
ros2 run base_keys base_keys_node                         # auto-detects the device
ros2 run base_keys base_keys_node --ros-args -p actions_enabled:=false   # publish only
python3 -m pytest -q src/base_keys/test                   # no ROS / evdev needed
```
