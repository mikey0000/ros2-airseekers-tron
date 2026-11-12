#include "mowgli_nav2_plugins/row_entry_critic.hpp"

#include <cmath>

#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.hpp"

namespace mppi::critics
{

void RowEntryCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 30.0f);
  getParam(params_.entry_speed, "entry_speed", 0.10f);
  float tol_deg = 2.6f, straight_deg = 6.0f;
  getParam(tol_deg, "heading_tol_deg", 2.6f);
  getParam(straight_deg, "row_straight_tol_deg", 6.0f);
  params_.heading_tol = tol_deg * static_cast<float>(M_PI / 180.0);
  params_.row_straight_tol = straight_deg * static_cast<float>(M_PI / 180.0);
  getParam(params_.ramp_accel, "ramp_accel", 0.10f);
  getParam(params_.release_speed, "release_speed", 0.30f);
  getParam(params_.max_entry_m, "max_entry_m", 3.0f);
  getParam(params_.row_min_m, "row_min_m", 1.0f);
  getParam(params_.new_goal_m, "new_goal_m", 0.25f);
  clock_ = parent_.lock()->get_clock();
  RCLCPP_INFO(
    logger_, "RowEntryCritic: weight %.1f power %u, entry %.2f m/s until within %.1f deg of a "
    ">= %.1f m row, then +%.2f m/s^2 to %.2f m/s; at most %.1f m", weight_, power_,
    params_.entry_speed, tol_deg, params_.row_min_m, params_.ramp_accel, params_.release_speed,
    params_.max_entry_m);
}

void RowEntryCritic::score(CriticData & data)
{
  if (!enabled_) {
    return;
  }
  namespace re = mowgli_nav2_plugins::row_entry;
  const auto & pose = data.state.pose.pose;
  float heading = 0.0f;
  const bool row = re::rowHeading(data.path.x, data.path.y, data.path.x.shape(0), params_,
      heading);
  const re::Phase before = state_.phase;
  const float cap = re::update(
    state_, params_, clock_->now().seconds(), static_cast<float>(pose.position.x),
    static_cast<float>(pose.position.y), static_cast<float>(tf2::getYaw(pose.orientation)),
    static_cast<float>(data.goal.position.x), static_cast<float>(data.goal.position.y), row,
    heading);
  if (state_.phase != before) {
    RCLCPP_INFO(
      logger_, "RowEntryCritic: %s at (%.2f, %.2f), heading %.1f deg off the row",
      state_.phase == re::Phase::kHold ? "hold" :
      state_.phase == re::Phase::kRamp ? "aligned, ramp" : "released",
      pose.position.x, pose.position.y,
      row ? re::wrap(static_cast<float>(tf2::getYaw(pose.orientation)) - heading) * 180.0 / M_PI :
      0.0);
  }
  re::scoreBatch(
    data.state.cvx, data.costs, cap,
    state_.phase == re::Phase::kRamp ? params_.ramp_accel : 0.0f, data.model_dt, weight_,
    power_);
}

}  // namespace mppi::critics

PLUGINLIB_EXPORT_CLASS(mppi::critics::RowEntryCritic, mppi::critics::CriticFunction)
