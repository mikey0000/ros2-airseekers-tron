// FTCRowController (2026-11-05, row weave study, docs/analysis/2026-11-05_row_weave_ftc.md):
// a Nav2 controller that drives straight coverage rows with the FTC ("follow the carrot")
// law of OpenMower's ftc_local_planner (ftc_row_law.hpp, GPLv3, ported from
// github.com/ClemensElflein/open_mower_ros by Clemens Elflein) and everything else with an
// inner MPPIController.
//
// Why: the field row weave (p90 ~2 cm) is the coverage MPPI's lightly damped lateral mode,
// not the heading estimate (analysis note section 1). MPPI's effective row gains came out at
// w ~ -3.5..4.7 lat - 0.85 yaw however its critics were tuned; a carrot PD sets the heading
// damping directly.
//
// How:
//   * The inner MPPI is configured under the SAME name, so it reads the existing
//     <name>.* MPPI parameters, and the mission's live <name>.vx_max reaches it unchanged
//     (Nav2's RotationShimController wraps its primary controller the same way).
//   * MPPI runs every cycle (warm start, CostCritic, its collision exceptions propagate, so
//     a blocked row aborts exactly as before). Its command is used unless the row law drives.
//   * The row law drives when the robot is on a straight that runs at least
//     handoff_m + 2 x lead past it, within half of max_lateral_m / max_angle_rad of the line,
//     and the footprint is clear (cost < inscribed) along the row for that distance. It hands
//     back to MPPI when a bend or the path end comes within handoff_m of the carrot, the robot
//     leaves max_lateral_m / max_angle_rad, or the row ahead is blocked. Arcs, fillets, swath
//     turns, path ends and detours are therefore always MPPI; pivots and reverse legs are not
//     this controller (mission Spin, FollowCoveragePathReverseMPPI).
//   * Target speed: <name>.vx_max (the cut speed the mission sets) under the speed limit.
//     Angular limit: <name>.wz_max.
//
// Parameters <name>.FTC.*: kp_lon (1.2), kp_lat (2.5), ki_lat (0.0), ki_lat_max (0.1),
// kd_lat (0.0), kp_ang (1.2), kd_ang (0.0), acceleration (0.3 m/s^2), straight_tolerance
// (0.01 m), handoff_distance (0.4 m), max_lateral_error (0.15 m), max_angle_error (0.35 rad).
// A/B: the controller's `plugin:` line in nav2_params.yaml
// ("mowgli_nav2_plugins::FTCRowController" vs "nav2_mppi_controller::MPPIController").
#ifndef MOWGLI_NAV2_PLUGINS__FTC_ROW_CONTROLLER_HPP_
#define MOWGLI_NAV2_PLUGINS__FTC_ROW_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mowgli_nav2_plugins/ftc_row_law.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace mowgli_nav2_plugins
{

class FTCRowController : public nav2_core::Controller
{
public:
  FTCRowController();
  ~FTCRowController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  bool cancel() override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
  void reset() override;

private:
  enum class Mode { MPPI, ROW };

  bool localPath(const std::string & frame);
  bool rowClear(double s0, double length);
  double param(const std::string & key, double fallback) const;
  void endEpisode(const char * why);
  void logSummary();

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("FTCRowController")};
  rclcpp::Clock::SharedPtr clock_;

  pluginlib::ClassLoader<nav2_core::Controller> loader_;
  nav2_core::Controller::Ptr inner_;

  ftc_row::Gains gains_;
  ftc_row::Gate gate_;
  double search_dist_{2.0};

  nav_msgs::msg::Path plan_;
  ftc_row::Path path_;
  size_t seg_{0};
  bool searched_{false};
  Mode mode_{Mode::MPPI};
  ftc_row::Law law_;
  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
  double speed_limit_{0.0};
  bool speed_limit_pct_{false};

  // per-plan bookkeeping for the summary log
  size_t cycles_{0};
  size_t row_cycles_{0};
  size_t entries_{0};
  double episode_start_s_{0.0};
};

}  // namespace mowgli_nav2_plugins

#endif  // MOWGLI_NAV2_PLUGINS__FTC_ROW_CONTROLLER_HPP_
