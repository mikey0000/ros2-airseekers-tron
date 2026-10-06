// LegGoalChecker implementation. See the header for the why.

#include "mowgli_nav2_plugins/leg_goal_checker.hpp"

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mowgli_nav2_plugins
{

void LegGoalChecker::initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
                                const std::string& plugin_name,
                                const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /*costmap*/)
{
  auto node = parent.lock();
  if (!node)
  {
    throw std::runtime_error("LegGoalChecker: failed to lock parent LifecycleNode");
  }
  logger_ = node->get_logger();
  auto declare = [&](const std::string& key, double def)
  {
    const std::string full = plugin_name + "." + key;
    if (!node->has_parameter(full))
    {
      node->declare_parameter(full, def);
    }
    return node->get_parameter(full).as_double();
  };
  std::lock_guard<std::mutex> lk(mutex_);
  state_.progress_fraction = declare("progress_fraction", 0.7);
  state_.xy_tol = declare("xy_goal_tolerance", 0.08);
  state_.yaw_tol = declare("yaw_goal_tolerance", 0.35);
  state_.overshoot_xy_tol = declare("overshoot_xy_tolerance", 0.20);
  state_.overshoot_hysteresis = declare("overshoot_hysteresis", 0.03);
  RCLCPP_INFO(logger_, "LegGoalChecker %s: progress %.2f, xy %.3f m, yaw %.2f rad, overshoot %.2f m",
              plugin_name.c_str(), state_.progress_fraction, state_.xy_tol, state_.yaw_tol,
              state_.overshoot_xy_tol);
}

void LegGoalChecker::reset()
{
  std::lock_guard<std::mutex> lk(mutex_);
  state_.reset();
}

bool LegGoalChecker::isGoalReached(const geometry_msgs::msg::Pose& query_pose,
                                   const geometry_msgs::msg::Pose& goal_pose,
                                   const geometry_msgs::msg::Twist& /*velocity*/)
{
  std::lock_guard<std::mutex> lk(mutex_);
  return state_.update(query_pose.position.x, query_pose.position.y,
                       tf2::getYaw(query_pose.orientation), goal_pose.position.x,
                       goal_pose.position.y, tf2::getYaw(goal_pose.orientation));
}

bool LegGoalChecker::getTolerances(geometry_msgs::msg::Pose& pose_tolerance,
                                   geometry_msgs::msg::Twist& vel_tolerance)
{
  const double invalid = std::numeric_limits<double>::lowest();
  std::lock_guard<std::mutex> lk(mutex_);
  pose_tolerance.position.x = state_.xy_tol;
  pose_tolerance.position.y = state_.xy_tol;
  pose_tolerance.position.z = invalid;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, state_.yaw_tol);
  pose_tolerance.orientation = tf2::toMsg(q);
  vel_tolerance.linear.x = invalid;
  vel_tolerance.linear.y = invalid;
  vel_tolerance.linear.z = invalid;
  vel_tolerance.angular.x = invalid;
  vel_tolerance.angular.y = invalid;
  vel_tolerance.angular.z = invalid;
  return true;
}

}  // namespace mowgli_nav2_plugins

PLUGINLIB_EXPORT_CLASS(mowgli_nav2_plugins::LegGoalChecker, nav2_core::GoalChecker)
