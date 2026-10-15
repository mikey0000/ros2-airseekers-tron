# mowgli_nav2_plugins (Jazzy port)

The MowgliNext `FTCController` (follow-the-carrot coverage controller) and
`PathProgressGoalChecker`, ported from `third_party/mowglinext/mowgli_nav2_plugins`
(Kilted) to Jazzy. The control law is unchanged from upstream. The port only
changes API, headers and exception types, plus the items below.

## Port-only changes

* `desired_linear_vel` is an alias for the GUI mowing speed:
  `speed_fast = v` and `speed_slow = clamp(0.8 v, min_speed_mps, v)`.
* Blade telemetry is not wired, so the blade-load slowdown fails open.
* `scale_linear_on_angular_saturation` (default false): when the FOLLOWING
  angular command saturates at `max_cmd_vel_ang`, scale `linear.x` down so the
  commanded curvature is kept.
* The plan is published on `~/<plugin>/global_plan`, which is
  `/controller_server/<plugin>/global_plan`, the `PathProgressGoalChecker`
  `plan_topic`. Before this fix the plan went to `/<plugin>/global_plan` and the
  checker fell back to SimpleGoalChecker semantics after 5 s.

## 2026-10-06 first swath: why FTC left the ring

On the robot, the controller signs, frames and yaw were all correct. Each logged
`ang` and `lat` was negative, and `/cmd_vel` was negative too, which is the
correct CW correction. The chassis could not turn fast enough:

* `mower_mcu_driver` clamps `angular.z` to `angular_max` = 0.3 rad/s. FTC
  commanded -0.8 rad/s (`max_cmd_vel_ang` 0.8) for more than 3 s. The
  slip_detector logged `DIG STALL |d_angular|=0.616`, so only about 0.2 rad/s
  was delivered in turf with the blade on.
* The FTC carrot advances open-loop. At a corner it rotates at `speed_angular`,
  which upstream sets to 45 deg/s (0.79 rad/s). That is about 4x what the Tron
  can turn, so the carrot ran away around the corner and the robot was carried
  0.54 m outside the ring.

`test/test_ftc_tracking_sim.cpp` runs the real controller in closed loop
against a unicycle that has the Tron yaw limit. It reproduces the failure: the
live parameters give 0.49 m on a ring (and abort) and 0.69 m on swath ends.
The fix is `speed_angular` 15 deg/s together with `max_cmd_vel_ang` 0.3 (see
`config/ftc_tron.yaml`). With these the cross-track error stays under 7 cm on
straight swaths, rings and 0.18 m swath-end turns, at delivered yaw rates down
to 0.15 rad/s. The `RingAblation` test shows that `speed_angular` is the
deciding parameter.

## Tron tuning

Use `config/ftc_tron.yaml` and merge it into `mower_navigation/config/nav2_params.yaml`.
The main points:

| param | live 10-06 | Tron | why |
|---|---|---|---|
| speed_angular | 45 | 15 | carrot rotation must stay below the achievable yaw rate (0.2-0.3 rad/s) |
| max_cmd_vel_ang | 0.8 | 0.3 | MCU clamp; anything above it is cut silently |
| scale_linear_on_angular_saturation | - | true | keeps the curvature when saturated |
| kp_ang / kp_ang_following | 1.5 / 1.0 | 1.0 / 0.8 | 0.3 rad/s is already reached at 17 deg error |
| kp_lat / kd_lat | 0.8 / 0.5 | 1.0 / 0.3 | |
| max_goal_angle_error | 30 | 15 | PRE_ROTATE exited at -16 deg |

Corners are slower with this tuning: in the sim, a 10 m ring takes 66 s
instead of about 41 s.

## MPPI critic: WheelFloorCritic (2026-10-15)

`mppi::critics::WheelFloorCritic` (library `mowgli_mppi_critics`, `mppi_critics_plugin.xml`,
exported to `nav2_mppi_controller`). Jazzy loads critics as `"mppi::critics::" + <name>`, so the
class lives in that namespace. It adds `cost_weight * mean_t (violation / floor)^cost_power` per
sampled trajectory, where `violation` is the distance of (vx, wz) from the drivable set of the
Tron's drive (`wheel_floor.hpp`: inner wheel `|v| - |w| b/2` >= +floor for an arc or <= -floor
for a counter-rotating pivot; straight and stopped commands are free). It mirrors
`cmd_vel_slew.both_wheels_legal`, so a command with no violation passes the turn shaper
unchanged. Parameters (`<controller>.WheelFloorCritic.*`): `cost_weight` (10), `cost_power` (1),
`track_width` (0.48), `min_wheel_speed` (0.06), `straight_wz` (0), `stop_speed` (0.02),
`allow_pivots` (true). Revert: drop it from the controller's `critics` list (or `enabled: false`).
Why and measurements: `docs/analysis/2026-10-15_mppi_study.md`. Test: `test_wheel_floor_critic`.

## Tests

`./scripts/dev_build.sh test --packages-select mowgli_nav2_plugins`. The sim
test runs with `ROS_LOCALHOST_ONLY=1` and `ROS_DOMAIN_ID=97`, so it stays
invisible to the mower even though the dev container uses host networking.
