// MPPI critic: hold the forward speed low at a row entry until the heading is on the row,
// then ramp up (2026-11-12 row straightness review). Logic and rationale: row_entry.hpp.
//
// Jazzy (1.3.x) loads critics as "mppi::critics::" + <name in the critics list>; parameters
// are <controller>.RowEntryCritic.*: cost_weight (30.0), cost_power (1), entry_speed (0.10
// m/s), heading_tol_deg (2.6), ramp_accel (0.10 m/s^2), release_speed (0.30 m/s),
// max_entry_m (3.0), row_min_m (1.5), row_straight_tol_deg (6.0), new_goal_m (0.05).
// Revert: drop "RowEntryCritic" from the critics list (or enabled: false).
#ifndef MOWGLI_NAV2_PLUGINS__ROW_ENTRY_CRITIC_HPP_
#define MOWGLI_NAV2_PLUGINS__ROW_ENTRY_CRITIC_HPP_

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
#pragma GCC diagnostic ignored "-Wstringop-overflow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <xtensor/xtensor.hpp>
#include "nav2_mppi_controller/critic_function.hpp"
#pragma GCC diagnostic pop

#include "mowgli_nav2_plugins/row_entry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mppi::critics
{

class RowEntryCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(CriticData & data) override;

protected:
  mowgli_nav2_plugins::row_entry::Params params_;
  mowgli_nav2_plugins::row_entry::State state_;
  unsigned int power_{1};
  float weight_{30.0f};
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace mppi::critics

#endif  // MOWGLI_NAV2_PLUGINS__ROW_ENTRY_CRITIC_HPP_
