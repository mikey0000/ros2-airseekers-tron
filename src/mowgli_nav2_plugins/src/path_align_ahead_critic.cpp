#include "mowgli_nav2_plugins/path_align_ahead_critic.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace mppi::critics
{

void PathAlignAheadCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 10.0f);
  getParam(max_path_occupancy_ratio_, "max_path_occupancy_ratio", 0.07f);
  getParam(offset_from_furthest_, "offset_from_furthest", 20);
  getParam(params_.trajectory_point_step, "trajectory_point_step", 4);
  getParam(threshold_to_consider_, "threshold_to_consider", 0.5f);
  getParam(params_.use_path_orientations, "use_path_orientations", false);
  getParam(params_.control_point_x, "control_point_x", 0.0f);
  RCLCPP_INFO(
    logger_, "PathAlignAheadCritic: weight %.1f power %u, control point %.3f m ahead, "
    "step %d, offset_from_furthest %zu", weight_, power_, params_.control_point_x,
    params_.trajectory_point_step, offset_from_furthest_);
}

void PathAlignAheadCritic::score(CriticData & data)
{
  // Gating as PathAlignCritic (Jazzy 1.3.13): off near the goal, before the path bearing,
  // and when dynamic obstacles block a significant part of the local path.
  if (!enabled_ || utils::withinPositionGoalTolerance(
      threshold_to_consider_, data.state.pose.pose, data.goal))
  {
    return;
  }
  utils::setPathFurthestPointIfNotSet(data);
  const size_t furthest = *data.furthest_reached_path_point;
  if (furthest < offset_from_furthest_) {
    return;
  }
  utils::setPathCostsIfNotSet(data, costmap_ros_);
  const std::vector<bool> & valid = *data.path_pts_valid;
  const float furthest_flt = static_cast<float>(furthest);
  float invalid_ctr = 0.0f;
  for (size_t i = 0; i < furthest && i < valid.size(); i++) {
    if (!valid[i]) {invalid_ctr += 1.0f;}
    if (invalid_ctr / furthest_flt > max_path_occupancy_ratio_ && invalid_ctr > 2.0f) {
      return;
    }
  }
  const size_t n_path = mowgli_nav2_plugins::path_align_ahead::pathPointsUsed(
    data.path.x, data.path.y, furthest, valid.size(), params_.control_point_x);
  mowgli_nav2_plugins::path_align_ahead::scoreBatch(
    data.trajectories.x, data.trajectories.y, data.trajectories.yaws, data.path.x,
    data.path.y, data.path.yaws, valid, n_path, params_, cost_);
  const size_t batch = std::min(cost_.size(), static_cast<size_t>(data.costs.shape(0)));
  for (size_t t = 0; t < batch; ++t) {
    const float c = cost_[t] * weight_;
    data.costs(t) += power_ > 1u ? std::pow(c, static_cast<float>(power_)) : c;
  }
}

}  // namespace mppi::critics

PLUGINLIB_EXPORT_CLASS(mppi::critics::PathAlignAheadCritic, mppi::critics::CriticFunction)
