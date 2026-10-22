# mower_common

Shared Python library (no nodes) for the Airseekers Tron packages. Created 2026-10-23
(code review stage 0.3).

| Module | Contents |
|---|---|
| `sub_pump` | `SubscriptionPump`, `PeriodicRunner`, CDR fast parsers / serializers (was 11 identical copies) |
| `clock` | `NodeClock`, `node_clock`, `node_now`, `WALL_CLOCK`, `wait_first_clock` (monotonic, or ROS time under `use_sim_time`) |
| `angles` | `wrap_angle` (fmod form), `wrap_angle_trig` (atan2 form), `yaw_from_quaternion` |
| `phases` | `DOCK_PHASES`, `MOTION_PHASES`, `DOCKED_MOTION_PHASES`: the one source for the mission FSM and the cmd_vel_slew motion gate |
| `geometry` | track split 0.40 / physical 0.39, footprint 0.60 / 0.10 / +-0.235, blade disc 0.22, deck centre 0.23, marker 0.04 |

The YAML files keep literal values; `test/test_yaml_constants.py` asserts they equal `geometry`.
Every package that imports it lists `<exec_depend>mower_common</exec_depend>`.
