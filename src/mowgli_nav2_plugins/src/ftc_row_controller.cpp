// FTCRowController: FTC row law on straight coverage rows, inner MPPI elsewhere.
// See include/mowgli_nav2_plugins/ftc_row_controller.hpp. The row law is ported from
// OpenMower's ftc_local_planner (Clemens Elflein, GPLv3); this file is GPLv3 as well.
#include "mowgli_nav2_plugins/ftc_row_controller.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>

#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mowgli_nav2_plugins
{

namespace
{
constexpr char kInnerPlugin[] = "nav2_mppi_controller::MPPIController";
constexpr double kRowWindowM = 1.0;    // projection search ahead of the last foot point
constexpr double kCheckStepM = 0.1;    // footprint check spacing along the row
constexpr double kNominalDt = 0.05;
}  // namespace

FTCRowController::FTCRowController()
: loader_("nav2_core", "nav2_core::Controller")
{
}

void FTCRowController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  auto node = parent.lock();
  if (!node) {
    throw nav2_core::ControllerException("FTCRowController: unable to lock the node");
  }
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  auto declare = [&](const std::string & key, double def) {
      const std::string full = name_ + ".FTC." + key;
      nav2_util::declare_parameter_if_not_declared(node, full, rclcpp::ParameterValue(def));
      return node->get_parameter(full).as_double();
    };
  gains_.kp_lon = declare("kp_lon", gains_.kp_lon);
  gains_.kp_lat = declare("kp_lat", gains_.kp_lat);
  gains_.ki_lat = declare("ki_lat", gains_.ki_lat);
  gains_.ki_lat_max = declare("ki_lat_max", gains_.ki_lat_max);
  gains_.kd_lat = declare("kd_lat", gains_.kd_lat);
  gains_.kp_ang = declare("kp_ang", gains_.kp_ang);
  gains_.kd_ang = declare("kd_ang", gains_.kd_ang);
  gains_.acceleration = declare("acceleration", gains_.acceleration);
  gate_.straight_tol_m = declare("straight_tolerance", gate_.straight_tol_m);
  gate_.handoff_m = declare("handoff_distance", gate_.handoff_m);
  gate_.max_lateral_m = declare("max_lateral_error", gate_.max_lateral_m);
  gate_.max_angle_rad = declare("max_angle_error", gate_.max_angle_rad);

  inner_ = loader_.createUniqueInstance(kInnerPlugin);
  inner_->configure(parent, name_, tf_, costmap_ros_);
  // the inner MPPI declared these; read once here, live values in computeVelocityCommands
  search_dist_ = param("max_robot_pose_search_dist", search_dist_);
  RCLCPP_INFO(
    logger_, "FTCRowController %s: rows by FTC (kp_lat %.2f kp_ang %.2f kp_lon %.2f ki_lat %.2f "
    "handoff %.2f m), the rest by %s", name_.c_str(), gains_.kp_lat, gains_.kp_ang,
    gains_.kp_lon, gains_.ki_lat, gate_.handoff_m, kInnerPlugin);
}

void FTCRowController::cleanup()
{
  if (inner_) {
    inner_->cleanup();
  }
  inner_.reset();
}

void FTCRowController::activate()
{
  inner_->activate();
}

void FTCRowController::deactivate()
{
  inner_->deactivate();
}

double FTCRowController::param(const std::string & key, double fallback) const
{
  auto node = node_.lock();
  rclcpp::Parameter p;
  if (node && node->get_parameter(name_ + "." + key, p) &&
    p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
  {
    return p.as_double();
  }
  return fallback;
}

void FTCRowController::logSummary()
{
  if (cycles_ > 0 && entries_ > 0) {
    RCLCPP_INFO(
      logger_, "FTCRowController %s: plan done: rows %.0f %% of %zu cycles, %zu row entries",
      name_.c_str(), 100.0 * static_cast<double>(row_cycles_) / static_cast<double>(cycles_),
      cycles_, entries_);
  }
  cycles_ = row_cycles_ = entries_ = 0;
}

void FTCRowController::endEpisode(const char * why)
{
  if (mode_ == Mode::ROW) {
    RCLCPP_INFO(
      logger_, "FTCRowController %s: row -> MPPI after %.2f m (%s)", name_.c_str(),
      std::max(0.0, law_.carrot() - episode_start_s_), why);
  }
  mode_ = Mode::MPPI;
}

void FTCRowController::setPlan(const nav_msgs::msg::Path & path)
{
  inner_->setPlan(path);
  plan_ = path;
  seg_ = 0;
  searched_ = false;
  mode_ = Mode::MPPI;
}

void FTCRowController::reset()
{
  inner_->reset();
  logSummary();
  seg_ = 0;
  searched_ = false;
  mode_ = Mode::MPPI;
}

bool FTCRowController::cancel()
{
  mode_ = Mode::MPPI;
  return inner_->cancel();
}

void FTCRowController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  inner_->setSpeedLimit(speed_limit, percentage);
  speed_limit_ = speed_limit;
  speed_limit_pct_ = percentage;
}

bool FTCRowController::localPath(const std::string & frame)
{
  path_.clear();
  if (plan_.poses.size() < 2) {
    return false;
  }
  geometry_msgs::msg::TransformStamped t;
  try {
    t = tf_->lookupTransform(frame, plan_.header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    return false;
  }
  tf2::Transform tr;
  tf2::fromMsg(t.transform, tr);
  for (const auto & ps : plan_.poses) {
    const tf2::Vector3 q = tr * tf2::Vector3(ps.pose.position.x, ps.pose.position.y, 0.0);
    path_.push(q.x(), q.y());
  }
  return path_.size() >= 2;
}

bool FTCRowController::rowClear(double s0, double length)
{
  auto * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(costmap);
  const auto footprint = costmap_ros_->getRobotFootprint();
  for (double s = s0; s <= s0 + length + 1e-9; s += kCheckStepM) {
    const ftc_row::PathPoint p = ftc_row::pointAt(path_, s);
    if (checker.footprintCostAtPose(p.x, p.y, p.yaw, footprint) >=
      nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    {
      return false;
    }
  }
  return true;
}

geometry_msgs::msg::TwistStamped FTCRowController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  // MPPI every cycle: warm start, and its collision / no-control exceptions propagate.
  geometry_msgs::msg::TwistStamped cmd = inner_->computeVelocityCommands(pose, velocity,
      goal_checker);
  ++cycles_;

  const rclcpp::Time now = clock_->now();
  double dt = kNominalDt;
  if (last_time_.nanoseconds() > 0 && now.get_clock_type() == last_time_.get_clock_type()) {
    const double d = (now - last_time_).seconds();
    if (d > 0.0 && d < 0.5) {
      dt = d;
    }
  }
  last_time_ = now;

  if (!localPath(pose.header.frame_id)) {
    endEpisode("no path in the pose frame");
    return cmd;
  }
  const double x = pose.pose.position.x, y = pose.pose.position.y;
  const double yaw = tf2::getYaw(pose.pose.orientation);
  const ftc_row::Projection pr = ftc_row::project(
    path_, x, y, yaw, seg_, searched_ ? kRowWindowM : search_dist_);
  if (!pr.ok) {
    endEpisode("no foot point");
    return cmd;
  }
  seg_ = pr.seg;
  searched_ = true;

  double v_target = param("vx_max", 0.3);
  if (speed_limit_ > 0.0) {
    v_target = speed_limit_pct_ ? v_target * speed_limit_ / 100.0 :
      std::min(v_target, speed_limit_);
  }
  const double w_max = param("wz_max", 0.3);
  const double lead_max = gains_.kp_lon > 0.0 ? 1.5 * v_target / gains_.kp_lon : 0.0;

  if (mode_ == Mode::ROW) {
    if (!ftc_row::mayStay(path_, pr, law_.carrot(), gate_)) {
      endEpisode("bend, path end or off the line");
    } else if (!rowClear(pr.s, std::max(0.0, law_.carrot() - pr.s) + gate_.handoff_m)) {
      endEpisode("row ahead blocked");
    }
  } else if (v_target > 0.0 && ftc_row::mayEnter(path_, pr, lead_max, gate_) &&
    rowClear(pr.s, gate_.handoff_m + 2.0 * lead_max))
  {
    law_.start(pr.s, velocity.linear.x, gains_);
    episode_start_s_ = law_.carrot();
    mode_ = Mode::ROW;
    ++entries_;
  }
  if (mode_ != Mode::ROW) {
    return cmd;
  }

  ++row_cycles_;
  law_.advance(pr.s, v_target, lead_max, dt, gains_);
  const ftc_row::Command c = law_.step(path_, x, y, yaw, dt, v_target, w_max, gains_);
  cmd.twist = geometry_msgs::msg::Twist();
  cmd.twist.linear.x = c.v;
  cmd.twist.angular.z = c.w;
  return cmd;
}

}  // namespace mowgli_nav2_plugins

PLUGINLIB_EXPORT_CLASS(mowgli_nav2_plugins::FTCRowController, nav2_core::Controller)
