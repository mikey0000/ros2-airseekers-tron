#include "mowgli_nav2_plugins/path_align_ahead_critic.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.hpp"

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
  getParam(flat_.control_point_x, "control_point_x", 0.0f);
  getParam(flat_.latency_s, "latency_s", 0.0f);
  getParam(trim_.enabled, "trim_enabled", false);
  getParam(trim_.tau_s, "trim_tau_s", 5.0f);
  flat_.trim_tau_s = trim_.tau_s;
  float slope_tilt_deg = 0.0f;
  getParam(slope_tilt_deg, "slope_tilt_deg", 0.0f);
  slope_.tilt_rad = slope_tilt_deg * static_cast<float>(M_PI / 180.0);
  getParam(slope_.control_point_x, "slope_control_point_x", flat_.control_point_x);
  getParam(slope_.trim_tau_s, "slope_trim_tau_s", trim_.tau_s);
  getParam(trim_.max_m, "trim_max_m", 0.10f);
  getParam(trim_.max_err_m, "trim_max_err_m", 0.30f);
  getParam(trim_.min_speed, "trim_min_speed", 0.05f);
  getParam(trim_.leak_tau_s, "trim_leak_tau_s", 60.0f);
  getParam(trim_.k_roll, "k_roll", 0.0f);
  getParam(roll_timeout_s_, "roll_timeout_s", 1.0f);
  std::string roll_topic;
  getParam(roll_topic, "roll_topic", std::string("/imu/data"));
  auto node = parent_.lock();
  clock_ = node->get_clock();
  const bool want_roll = (trim_.enabled && trim_.k_roll != 0.0f) || slope_.tilt_rad > 0.0f;
  if (want_roll && !roll_topic.empty()) {
    imu_sub_ = node->create_subscription<sensor_msgs::msg::Imu>(
      roll_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        const auto & q = msg->orientation;
        const double r = std::atan2(2.0 * (q.w * q.x + q.y * q.z),
        1.0 - 2.0 * (q.x * q.x + q.y * q.y));
        roll_.store(static_cast<float>(r));
        tilt_.store(mowgli_nav2_plugins::path_align_ahead::tiltAngle(q.x, q.y));
        roll_t_.store(clock_->now().seconds());
      });
  }
  RCLCPP_INFO(
    logger_, "PathAlignAheadCritic: trim %s (tau %.1f s, max %.2f m, k_roll %.2f, roll %s); "
    "slope at tilt >= %.1f deg: control point %.3f m, no latency projection, trim tau "
    "%.1f s", trim_.enabled ? "on" : "off", trim_.tau_s, trim_.max_m, trim_.k_roll,
    imu_sub_ ? roll_topic.c_str() : "unused", slope_tilt_deg, slope_.control_point_x,
    slope_.trim_tau_s);
  RCLCPP_INFO(
    logger_, "PathAlignAheadCritic: weight %.1f power %u, control point %.3f m ahead, "
    "step %d, offset_from_furthest %zu, latency compensation %.2f s", weight_, power_,
    flat_.control_point_x, params_.trajectory_point_step, offset_from_furthest_, flat_.latency_s);
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
  namespace paa = mowgli_nav2_plugins::path_align_ahead;
  const rclcpp::Time now = clock_->now();
  const bool roll_fresh = imu_sub_ && now.seconds() - roll_t_.load() <= roll_timeout_s_;
  const float roll = roll_.load();
  const paa::Tuning tune = paa::tuningAt(flat_, slope_, tilt_.load(), roll_fresh);
  paa::Params params = params_;
  params.control_point_x = tune.control_point_x;
  const auto & pose0 = data.state.pose.pose;
  const paa::Projection proj = paa::projection(
    static_cast<float>(pose0.position.x), static_cast<float>(pose0.position.y),
    static_cast<float>(tf2::getYaw(pose0.orientation)),
    static_cast<float>(data.state.speed.linear.x), static_cast<float>(data.state.speed.angular.z),
    tune.latency_s);
  const size_t n_path = paa::pathPointsUsed(
    data.path.x, data.path.y, furthest, valid.size(), params.control_point_x + proj.arc);
  const xt::xtensor<float, 1> * px = &data.path.x;
  const xt::xtensor<float, 1> * py = &data.path.y;
  if (trim_.enabled) {
    namespace pt = mowgli_nav2_plugins::path_trim;
    float dt = have_last_ ? static_cast<float>((now - last_t_).seconds()) : 0.0f;
    if (dt > 1.0f || dt < 0.0f) {dt = 0.0f;}    // a pause / new leg: no wind-up over the gap
    dt = std::min(dt, 0.2f);
    last_t_ = now;
    have_last_ = true;
    const auto & pose = data.state.pose.pose;
    const float x = static_cast<float>(pose.position.x);
    const float y = static_cast<float>(pose.position.y);
    const float yaw = static_cast<float>(tf2::getYaw(pose.orientation));
    const float v = static_cast<float>(data.state.speed.linear.x);
    const size_t n_ct = std::min(data.path.x.shape(0), furthest + 10);
    const pt::CrossTrack ct = pt::crossTrack(data.path.x, data.path.y, n_ct, x, y);
    pt::update(trim_state_, ct, v, dt, trim_, tune.trim_tau_s);
    float fx = 0.0f, fy = 0.0f;
    if (roll_fresh) {
      pt::feedForward(roll, yaw, v, trim_, fx, fy);
    }
    const float sx = trim_state_.ox + fx, sy = trim_state_.oy + fy;
    if (sx != 0.0f || sy != 0.0f) {
      px_ = data.path.x;
      py_ = data.path.y;
      pt::shiftPath(px_, py_, px_.shape(0), sx, sy);
      px = &px_;
      py = &py_;
    }
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 5000, "PathAlignAheadCritic trim: e %+.3f m, integral (%+.3f, %+.3f), "
      "ff (%+.3f, %+.3f), tau %.0f s, control point %.2f m", ct.signed_m, trim_state_.ox,
      trim_state_.oy, fx, fy, tune.trim_tau_s, tune.control_point_x);
  }
  paa::scoreBatch(
    data.trajectories.x, data.trajectories.y, data.trajectories.yaws, *px, *py, valid, n_path,
    params, cost_, proj);
  const size_t batch = std::min(cost_.size(), static_cast<size_t>(data.costs.shape(0)));
  for (size_t t = 0; t < batch; ++t) {
    const float c = cost_[t] * weight_;
    data.costs(t) += power_ > 1u ? std::pow(c, static_cast<float>(power_)) : c;
  }
}

}  // namespace mppi::critics

PLUGINLIB_EXPORT_CLASS(mppi::critics::PathAlignAheadCritic, mppi::critics::CriticFunction)
