# mowgli_nav2_plugins (Jazzy)

Nav2 plugins for the Airseekers Tron:

* `mowgli_nav2_plugins::LegGoalChecker` (`goal_checker_plugin.xml`): the goal checker for
  short coverage turn legs (`coverage_leg_goal_checker`) and for legs that end on a pivot
  vertex of a conditioned path (`coverage_corner_goal_checker`, `along_path: true`).
  Test: `test_leg_goal_checker`.
* `mppi::critics::WheelFloorCritic` (below).

The package started as the Jazzy port of MowgliNext's `FTCController` and
`PathProgressGoalChecker` (`third_party/mowglinext/mowgli_nav2_plugins`). Both, with the
obstacle deviation and oscillation detector they used, were deleted in the 2026-10-23
cleanup (`docs/analysis/2026-10-23_code_review.md`, stage 0.1): MPPI is the only tracker.
The 2026-10-06 FTC ring-drift analysis is in git history (README before that commit).

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

`./scripts/dev_build.sh test --packages-select mowgli_nav2_plugins` (pure gtests, no ROS graph).
