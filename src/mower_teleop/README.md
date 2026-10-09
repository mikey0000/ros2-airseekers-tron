# mower_teleop

Browser teleop path for the mower. The relay is adapted from MowgliNext
(`cmd_vel_ws_relay.py`, GPL-3.0); this package is GPL-3.0.

## Data flow

```
GUI joystick -> ws:8766 JSON -> cmd_vel_ws_relay -> /cmd_vel_teleop (Twist)
  -> twist_mux -> /cmd_vel_raw -> cmd_vel_slew -> /cmd_vel -> MCU
```

Frame format (header ignored): `{"twist":{"linear":{"x":0.2},"angular":{"z":0.5}}}`.
Missing fields are 0; non-numeric values and bad JSON are rejected (logged, not published).

## Relay behaviour

- Clamps: `max_linear` 0.5 m/s, `max_angular` 1.0 rad/s (parameters).
- Replays the last command every `replay_interval` (0.05 s) while it is younger
  than `command_lease` (0.25 s); then publishes one explicit zero. Also publishes
  zero on disconnect. (MCU driver timeout and the teleop lane timeout are 0.5 s.)
- `bind` defaults to `0.0.0.0` because the GUI may run in another container
  (network_mode host). There is no authentication; firewall it or use `bind:=127.0.0.1`.

## twist_mux lanes (Jazzy, twist_mux 4.5.0, geometry_msgs/Twist)

| lane       | topic                | timeout (s) | priority |
|------------|----------------------|-------------|----------|
| navigation | /cmd_vel_nav         | 0.6         | 10       |
| docking    | /cmd_vel_docking     | 0.5         | 15       |
| teleop     | /cmd_vel_teleop      | 0.5         | 20       |
| bumper     | /cmd_vel_bumper      | 0.5         | 50       |
| emergency  | /cmd_vel_emergency   | 0.2         | 100      |

Lock `estop`: `/estop` (std_msgs/Bool, published by mower_mcu_driver), priority
255. While true, twist_mux outputs nothing from any lane.

## Launch

```
ros2 launch mower_teleop teleop.launch.py [port:=8766 bind:=0.0.0.0 max_linear:=0.5 ...]
```

## Known limitation

The GUI's foxglove-publish fallback sends TwistStamped, which does NOT work with
this stack (type mismatch with the Twist lane). The WebSocket relay is the
supported path.
