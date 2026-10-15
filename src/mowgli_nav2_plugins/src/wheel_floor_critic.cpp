#include "mowgli_nav2_plugins/wheel_floor_critic.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace mppi::critics
{

void WheelFloorCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 10.0f);
  double track = 0.48;
  getParam(track, "track_width", 0.48);
  params_.half_track = 0.5 * track;
  getParam(params_.floor, "min_wheel_speed", 0.06);
  getParam(params_.straight_wz, "straight_wz", 0.0);
  getParam(params_.stop_speed, "stop_speed", 0.02);
  getParam(params_.allow_pivots, "allow_pivots", true);
  RCLCPP_INFO(
    logger_, "WheelFloorCritic: weight %.1f power %u, track %.2f m, floor %.3f m/s, "
    "straight |wz| <= %.3f, stop < %.3f m/s, pivots %s", weight_, power_, track,
    params_.floor, params_.straight_wz, params_.stop_speed,
    params_.allow_pivots ? "allowed" : "penalised");
}

void WheelFloorCritic::score(CriticData & data)
{
  if (!enabled_) {
    return;
  }
  mowgli_nav2_plugins::wheel_floor::scoreBatch(
    data.state.cvx, data.state.cwz, data.costs, params_, weight_, power_);
}

}  // namespace mppi::critics

PLUGINLIB_EXPORT_CLASS(mppi::critics::WheelFloorCritic, mppi::critics::CriticFunction)
