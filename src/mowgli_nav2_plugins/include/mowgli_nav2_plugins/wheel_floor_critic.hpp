// MPPI critic: penalise sampled commands the Tron cannot drive (inner wheel under the
// 0.06 m/s floor while turning). See wheel_floor.hpp and
// docs/analysis/2026-10-15_mppi_study.md.
//
// Why: Nav2 MPPI (DiffDrive) assumes every (vx, wz) in its box is drivable. Field bags
// (2026-10-15 undock donut, run 4, run 5) show 24-84 % of its transit output ticks in the
// band the robot cannot drive; cmd_vel_slew's turn shaper rewrites each one, so MPPI's
// rollout no longer predicts what the robot does next (dither, slow pivots, circling).
// This critic makes those samples expensive, so the optimal (weighted-average) control
// stays in the drivable set and the shaper has nothing to rewrite.
//
// Jazzy (1.3.x) loads critics as "mppi::critics::" + <name in the critics list>, so the
// class lives in that namespace; its parameters are <controller>.WheelFloorCritic.*:
//   cost_weight (10.0), cost_power (1), track_width (0.40), min_wheel_speed (0.06),
//   straight_wz (0.0), stop_speed (0.02), allow_pivots (true).
// Revert: drop "WheelFloorCritic" from the critics list (or enabled: false).
#ifndef MOWGLI_NAV2_PLUGINS__WHEEL_FLOOR_CRITIC_HPP_
#define MOWGLI_NAV2_PLUGINS__WHEEL_FLOOR_CRITIC_HPP_

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
#pragma GCC diagnostic ignored "-Wstringop-overflow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <xtensor/xtensor.hpp>
#include "nav2_mppi_controller/critic_function.hpp"
#pragma GCC diagnostic pop

#include "mowgli_nav2_plugins/wheel_floor.hpp"

namespace mowgli_nav2_plugins::wheel_floor
{

// Adds weight * mean_t (violation / floor)^power of every sample (row) of (cvx, cwz) to
// costs. Returns the number of infeasible (sample, step) cells (for tests / diagnostics).
inline size_t scoreBatch(
  const xt::xtensor<float, 2> & cvx, const xt::xtensor<float, 2> & cwz,
  xt::xtensor<float, 1> & costs, const Params & p, double weight, unsigned int power)
{
  const size_t batch = cvx.shape(0);
  const size_t steps = cvx.shape(1);
  if (steps == 0 || p.floor <= 0.0) {
    return 0;
  }
  size_t bad = 0;
  for (size_t i = 0; i < batch; ++i) {
    double sum = 0.0;
    for (size_t j = 0; j < steps; ++j) {
      const double viol = violation(cvx(i, j), cwz(i, j), p);
      if (viol > 0.0) {
        ++bad;
        const double x = viol / p.floor;
        sum += power <= 1 ? x : std::pow(x, static_cast<double>(power));
      }
    }
    costs(i) += static_cast<float>(weight * sum / static_cast<double>(steps));
  }
  return bad;
}

}  // namespace mowgli_nav2_plugins::wheel_floor

namespace mppi::critics
{

class WheelFloorCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(CriticData & data) override;

protected:
  mowgli_nav2_plugins::wheel_floor::Params params_;
  unsigned int power_{1};
  float weight_{10.0f};
};

}  // namespace mppi::critics

#endif  // MOWGLI_NAV2_PLUGINS__WHEEL_FLOOR_CRITIC_HPP_
