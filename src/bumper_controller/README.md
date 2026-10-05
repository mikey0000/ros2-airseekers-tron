# bumper_controller

Bumper obstacle-avoidance safety controller (back-up + rotate-clear state machine).
Ported from the recovered `libbumper_controller.so` reimplementation
(`ros2_port_handoff/14_proprietary_drivers/bumper_controller/`).

When the bumper strip triggers, the instant safety cutoff is enforced by the MCU /
`mower_base`; this node runs the **higher-level obstacle routing** (`IDLE` → `BACKING_UP`):
back the mower up a fixed distance, then rotate clear (heading PID), then return to idle.

## Topics

| Name | Type | Direction |
|---|---|---|
| `/mower_base/status` | `mower_interfaces/msg/MowerBaseDevStatus` | subscribe (trigger source) |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | publish (reverse + rotate) |
| `/bumper_cloud` | `sensor_msgs/msg/PointCloud2` | publish (on trigger) |
| `/mower_base/bumper_routing_status` | `std_msgs/msg/UInt8` | publish (0=IDLE, 1=BACKING_UP) |
| `/odom` | `nav_msgs/msg/Odometry` | subscribe (heading for rotate PID) |
| `/test_bumper_service` | `std_srvs/srv/Empty` | service (inject synthetic bumper hit) |

## Prerequisite

`/mower_base/status` must be published (with `bumper_triggered`,
`left/right_bumper_triggered`, and `bumper_routing_enabled` set). In the stack today this
comes from the not-yet-ported `mower_base` node; until it lands the controller starts but
sits dormant (a topic subscription — it simply never sees a trigger).

## Design note (option b)

The rotate-clear heading PID was **inlined** into `bumper_controller.cpp` (a small
positional PID + `/odom` yaw tracking) rather than depending on `pid_controller`, which is
deliberately not ported (the MCU owns the fast loop; `opennav_docking` supersedes the
standalone rotate primitive). Tunable via node parameters:

- `pid_kp` (0.8), `pid_ki` (0.0), `pid_kd` (0.05)
- `pid_max_angular` (0.5 rad/s), `pid_tolerance` (0.03 rad)

## Run

```bash
ros2 run bumper_controller bumper_controller_node
```
