// LegGoalChecker: goal checker for the short turn legs of a coverage plan
// (Dubins / Reeds-Shepp pieces between swaths, 0.3-0.9 m long, forward or
// REVERSE). nav2_controller::SimpleGoalChecker with xy 0.25 m declares a
// 0.3 m leg reached on the first control cycle because the robot already
// stands within tolerance at the leg's START, so the leg is never driven
// (2026-10-07: "sub-path 2 done (0.3 m)" 70 ms after the goal was sent).
//
// Reached when BOTH
//   * the robot has travelled >= progress_fraction (0.7) of the straight-line
//     start->goal distance measured at the first check after a new goal, and
//   * it is within xy_goal_tolerance (0.08 m) and yaw_goal_tolerance (0.35 rad)
//     of the goal pose,
// OR (overshoot guard, so a forward-only controller does not dither around a
// 0.08 m target) the travel gate is met, the robot came within
// overshoot_xy_tolerance (0.20 m) and is now moving away again (distance grew
// by > overshoot_hysteresis 0.03 m above its minimum).
#ifndef MOWGLI_NAV2_PLUGINS__LEG_GOAL_CHECKER_HPP_
#define MOWGLI_NAV2_PLUGINS__LEG_GOAL_CHECKER_HPP_

#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_core/goal_checker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace mowgli_nav2_plugins
{

// ROS-free state machine (unit tested in test/test_leg_goal_checker.cpp).
struct LegProgress
{
  double progress_fraction{0.7};
  double xy_tol{0.08};
  double yaw_tol{0.35};
  double overshoot_xy_tol{0.20};
  double overshoot_hysteresis{0.03};
  double max_step{0.5};  // ignore pose jumps larger than this (relocalisation)

  bool started{false};
  double goal_x{0.0}, goal_y{0.0};
  double last_x{0.0}, last_y{0.0};
  double leg_len{0.0};
  double travelled{0.0};
  double min_dist{std::numeric_limits<double>::infinity()};

  void reset()
  {
    started = false;
    travelled = 0.0;
    leg_len = 0.0;
    min_dist = std::numeric_limits<double>::infinity();
  }

  static double angleDiff(double a, double b)
  {
    return std::fabs(std::atan2(std::sin(a - b), std::cos(a - b)));
  }

  // Returns true when the leg is complete.
  bool update(double x, double y, double yaw, double gx, double gy, double gyaw)
  {
    // A different goal pose without reset(): treat as a new leg.
    if (started && std::hypot(gx - goal_x, gy - goal_y) > 1e-3)
    {
      reset();
    }
    if (!started)
    {
      started = true;
      goal_x = gx;
      goal_y = gy;
      last_x = x;
      last_y = y;
      leg_len = std::hypot(gx - x, gy - y);
    }
    else
    {
      const double step = std::hypot(x - last_x, y - last_y);
      if (step <= max_step)
      {
        travelled += step;
      }
      last_x = x;
      last_y = y;
    }
    const double d = std::hypot(gx - x, gy - y);
    const bool travel_ok = travelled >= progress_fraction * leg_len;
    const double prev_min = min_dist;
    if (d < min_dist)
    {
      min_dist = d;
    }
    if (!travel_ok)
    {
      return false;
    }
    if (d <= xy_tol && angleDiff(yaw, gyaw) <= yaw_tol)
    {
      return true;
    }
    return prev_min <= overshoot_xy_tol && d > prev_min + overshoot_hysteresis;
  }
};

class LegGoalChecker : public nav2_core::GoalChecker
{
public:
  LegGoalChecker() = default;
  ~LegGoalChecker() override = default;

  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
                  const std::string& plugin_name,
                  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void reset() override;

  bool isGoalReached(const geometry_msgs::msg::Pose& query_pose,
                     const geometry_msgs::msg::Pose& goal_pose,
                     const geometry_msgs::msg::Twist& velocity) override;

  bool getTolerances(geometry_msgs::msg::Pose& pose_tolerance,
                     geometry_msgs::msg::Twist& vel_tolerance) override;

private:
  rclcpp::Logger logger_{rclcpp::get_logger("leg_goal_checker")};
  std::mutex mutex_;
  LegProgress state_;
};

}  // namespace mowgli_nav2_plugins

#endif  // MOWGLI_NAV2_PLUGINS__LEG_GOAL_CHECKER_HPP_
