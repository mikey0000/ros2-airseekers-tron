// SPDX-License-Identifier: GPL-3.0-or-later
//
// Closed-loop kinematic simulation of the REAL FTCController (not a copy of
// its control law): the plugin is configured on a lifecycle node, fed a
// map->base_link TF from a unicycle model, and stepped at 20 Hz on an
// overridden ROS clock. The unicycle mirrors the Tron drivetrain:
//
//   * mower_mcu_driver clamps /cmd_vel to |v| <= 0.3 m/s, |w| <= 0.3 rad/s
//     (angular_max, vendor PidControllerROS clamp) — commands beyond that are
//     silently cut, which is what the 2026-10-06 first FTC swath ran into;
//   * a first-order wheel response (tau 0.15 s).
//
// Scenarios: a straight swath from a 16 deg heading error (the logged
// PRE_ROTATE exit), and a closed headland ring with sharp polygon corners.

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <rcl/time.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/buffer.h>

#include "mowgli_nav2_plugins/ftc_controller.hpp"

namespace
{

using Params = std::map<std::string, rclcpp::ParameterValue>;

struct Pt
{
  double x, y;
};

// Parameters live on the robot on 2026-10-06 (FollowCoveragePath = FTC,
// desired_linear_vel 0.3 from the GUI).
Params LiveParams()
{
  return {
      {"desired_linear_vel", rclcpp::ParameterValue(0.3)},
      {"speed_fast", rclcpp::ParameterValue(0.20)},
      {"speed_fast_threshold", rclcpp::ParameterValue(0.5)},
      {"speed_fast_threshold_angle", rclcpp::ParameterValue(10.0)},
      {"speed_slow", rclcpp::ParameterValue(0.16)},
      {"speed_angular", rclcpp::ParameterValue(45.0)},
      {"acceleration", rclcpp::ParameterValue(0.2)},
      {"min_speed_mps", rclcpp::ParameterValue(0.10)},
      {"kp_lon", rclcpp::ParameterValue(1.0)},
      {"kp_lat", rclcpp::ParameterValue(0.8)},
      {"kd_lat", rclcpp::ParameterValue(0.5)},
      {"kp_ang", rclcpp::ParameterValue(1.5)},
      {"kp_ang_following", rclcpp::ParameterValue(1.0)},
      {"derivative_filter_tau", rclcpp::ParameterValue(0.2)},
      {"max_cmd_vel_speed", rclcpp::ParameterValue(0.30)},
      {"max_cmd_vel_ang", rclcpp::ParameterValue(0.8)},
      {"max_goal_distance_error", rclcpp::ParameterValue(0.5)},
      {"max_goal_angle_error", rclcpp::ParameterValue(30.0)},
      {"goal_timeout", rclcpp::ParameterValue(10.0)},
      {"max_follow_distance", rclcpp::ParameterValue(2.0)},
      {"forward_only", rclcpp::ParameterValue(true)},
  };
}

// Proposed Tron parameters (config/ftc_tron.yaml): commands kept inside the
// MCU envelope, carrot rotation no faster than the chassis can turn, and the
// curvature-preserving linear scale-down when the angular command saturates.
Params TronParams()
{
  Params p = LiveParams();
  p["speed_angular"] = rclcpp::ParameterValue(15.0);
  p["max_cmd_vel_ang"] = rclcpp::ParameterValue(0.3);
  p["kp_ang"] = rclcpp::ParameterValue(1.0);
  p["kp_ang_following"] = rclcpp::ParameterValue(0.8);
  p["kp_lat"] = rclcpp::ParameterValue(1.0);
  p["kd_lat"] = rclcpp::ParameterValue(0.3);
  p["speed_fast_threshold_angle"] = rclcpp::ParameterValue(10.0);
  p["scale_linear_on_angular_saturation"] = rclcpp::ParameterValue(true);
  p["max_goal_angle_error"] = rclcpp::ParameterValue(15.0);
  p["min_speed_mps"] = rclcpp::ParameterValue(0.10);
  return p;
}

std::vector<Pt> Straight(double len, double step)
{
  std::vector<Pt> v;
  for (double s = 0.0; s <= len + 1e-9; s += step)
    v.push_back({s, 0.0});
  return v;
}

// Closed rectangular ring driven clockwise (right-hand corners), start == end,
// sampled every `step` like a polygon headland from the coverage server.
std::vector<Pt> RectRing(double w, double h, double step)
{
  const std::vector<Pt> c = {{0, 0}, {w, 0}, {w, -h}, {0, -h}, {0, 0}};
  std::vector<Pt> v;
  for (size_t k = 0; k + 1 < c.size(); ++k)
  {
    const double L = std::hypot(c[k + 1].x - c[k].x, c[k + 1].y - c[k].y);
    const int n = static_cast<int>(std::round(L / step));
    for (int i = 0; i < n; ++i)
    {
      const double t = static_cast<double>(i) / n;
      v.push_back({c[k].x + t * (c[k + 1].x - c[k].x), c[k].y + t * (c[k + 1].y - c[k].y)});
    }
  }
  v.push_back(c.back());
  return v;
}

// Boustrophedon: `n` swaths of length `len`, `spacing` apart (0.18 m on the
// Tron), joined by sharp 90-90 deg swath-end turns.
std::vector<Pt> Boustrophedon(int n, double len, double spacing, double step)
{
  std::vector<Pt> c;
  for (int i = 0; i < n; ++i)
  {
    const double y = -spacing * i;
    if (i % 2 == 0)
    {
      c.push_back({0.0, y});
      c.push_back({len, y});
    }
    else
    {
      c.push_back({len, y});
      c.push_back({0.0, y});
    }
  }
  std::vector<Pt> v;
  for (size_t k = 0; k + 1 < c.size(); ++k)
  {
    const double L = std::hypot(c[k + 1].x - c[k].x, c[k + 1].y - c[k].y);
    const int m = std::max(1, static_cast<int>(std::round(L / step)));
    for (int i = 0; i < m; ++i)
    {
      const double t = static_cast<double>(i) / m;
      v.push_back({c[k].x + t * (c[k + 1].x - c[k].x), c[k].y + t * (c[k + 1].y - c[k].y)});
    }
  }
  v.push_back(c.back());
  return v;
}

// 2026-10-06 headland ring as FTC latched it on /FollowCoveragePath/global_plan
// (135 poses + FTC tail duplicate dropped), map frame.
const std::vector<Pt> kRing20261006 = {
    {-1.799, -1.291}, {-1.875, -1.342}, {-1.951, -1.393}, {-2.027, -1.445}, {-2.104, -1.496}, {-2.180, -1.548},
    {-2.256, -1.599}, {-2.333, -1.650}, {-2.409, -1.702}, {-2.485, -1.753}, {-2.558, -1.729}, {-2.630, -1.705},
    {-2.703, -1.682}, {-2.726, -1.595}, {-2.749, -1.509}, {-2.771, -1.423}, {-2.794, -1.336}, {-2.817, -1.250},
    {-2.840, -1.163}, {-2.863, -1.077}, {-2.885, -0.991}, {-2.908, -0.904}, {-2.921, -0.810}, {-2.934, -0.717},
    {-2.947, -0.623}, {-2.959, -0.529}, {-2.972, -0.435}, {-2.985, -0.341}, {-2.998, -0.247}, {-3.010, -0.154},
    {-3.023, -0.060}, {-3.036, 0.034}, {-3.049, 0.128}, {-3.062, 0.222}, {-3.074, 0.315}, {-3.057, 0.401},
    {-3.040, 0.486}, {-3.023, 0.572}, {-3.006, 0.657}, {-2.961, 0.729}, {-2.917, 0.801}, {-2.872, 0.872},
    {-2.827, 0.944}, {-2.783, 1.016}, {-2.706, 1.078}, {-2.630, 1.140}, {-2.553, 1.203}, {-2.477, 1.265},
    {-2.400, 1.327}, {-2.324, 1.390}, {-2.247, 1.452}, {-2.171, 1.514}, {-2.095, 1.576}, {-2.018, 1.639},
    {-1.942, 1.701}, {-1.865, 1.763}, {-1.789, 1.826}, {-1.712, 1.888}, {-1.636, 1.950}, {-1.559, 2.013},
    {-1.473, 2.046}, {-1.387, 2.080}, {-1.301, 2.113}, {-1.214, 2.147}, {-1.128, 2.181}, {-1.042, 2.214},
    {-0.956, 2.248}, {-0.870, 2.281}, {-0.778, 2.295}, {-0.686, 2.309}, {-0.594, 2.323}, {-0.502, 2.337},
    {-0.410, 2.351}, {-0.318, 2.365}, {-0.226, 2.379}, {-0.135, 2.393}, {-0.043, 2.407}, {0.049, 2.421},
    {0.141, 2.435}, {0.204, 2.370}, {0.267, 2.305}, {0.330, 2.240}, {0.342, 2.145}, {0.353, 2.051},
    {0.365, 1.956}, {0.377, 1.861}, {0.389, 1.767}, {0.430, 1.845}, {0.470, 1.924}, {0.510, 2.002},
    {0.550, 2.081}, {0.591, 2.159}, {0.631, 2.238}, {0.671, 2.316}, {0.634, 2.225}, {0.596, 2.133},
    {0.559, 2.042}, {0.521, 1.950}, {0.484, 1.859}, {0.446, 1.767}, {0.409, 1.676}, {0.371, 1.584},
    {0.334, 1.493}, {0.296, 1.401}, {0.259, 1.310}, {0.221, 1.218}, {0.184, 1.126}, {0.146, 1.035},
    {0.079, 0.970}, {0.012, 0.905}, {-0.055, 0.839}, {-0.122, 0.774}, {-0.189, 0.709}, {-0.256, 0.644},
    {-0.323, 0.578}, {-0.390, 0.513}, {-0.457, 0.448}, {-0.524, 0.383}, {-0.591, 0.317}, {-0.658, 0.252},
    {-0.697, 0.163}, {-0.736, 0.074}, {-0.775, -0.016}, {-0.814, -0.105}, {-0.853, -0.194}, {-0.892, -0.284},
    {-0.931, -0.373}, {-0.970, -0.462}, {-1.027, -0.539}, {-1.084, -0.617}, {-1.142, -0.694}, {-1.199, -0.771},
    {-1.256, -0.848}, {-1.313, -0.925}, {-1.371, -1.003},
};

double PolylineDistance(const std::vector<Pt>& p, double x, double y)
{
  double best = 1e9;
  for (size_t i = 0; i + 1 < p.size(); ++i)
  {
    const double dx = p[i + 1].x - p[i].x, dy = p[i + 1].y - p[i].y;
    const double L2 = dx * dx + dy * dy;
    double t = L2 > 0 ? ((x - p[i].x) * dx + (y - p[i].y) * dy) / L2 : 0.0;
    t = std::clamp(t, 0.0, 1.0);
    best = std::min(best, std::hypot(x - (p[i].x + t * dx), y - (p[i].y + t * dy)));
  }
  return best;
}

struct SimResult
{
  bool finished{false};
  bool threw{false};
  double max_xtrack{0.0};
  double final_xtrack{0.0};
  double final_heading_err_deg{0.0};
  double t_end{0.0};
};

class FtcSim : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  SimResult Run(const Params& params,
                const std::vector<Pt>& path,
                double x0,
                double y0,
                double yaw0,
                double mcu_w_max,
                double t_max)
  {
    const std::string plugin = "FTC";
    rclcpp::NodeOptions opts;
    std::vector<rclcpp::Parameter> ov;
    ov.emplace_back(plugin + ".check_obstacles", false);
    ov.emplace_back(plugin + ".enable_obstacle_deviation", false);
    for (const auto& kv : params)
      ov.emplace_back(plugin + "." + kv.first, kv.second);
    opts.parameter_overrides(ov);
    static int run_id = 0;
    const std::string ns = "/ftc_sim_" + std::to_string(run_id++);
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server", ns, opts);

    auto clock = node->get_clock();
    rcl_clock_t* h = clock->get_clock_handle();
    EXPECT_EQ(rcl_enable_ros_time_override(h), RCL_RET_OK);
    rcl_time_point_value_t now_ns = 1000LL * 1000000000LL;
    (void)rcl_set_ros_time_override(h, now_ns);

    auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("local_costmap", ns, "local_costmap");
    costmap->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{}));
    costmap->configure();

    auto tf = std::make_shared<tf2_ros::Buffer>(clock);
    tf->setUsingDedicatedThread(true);  // data is set synchronously before each lookup
    double x = x0, y = y0, th = yaw0, v = 0.0, w = 0.0;
    auto publish_tf = [&]()
    {
      geometry_msgs::msg::TransformStamped t;
      t.header.frame_id = "map";
      t.header.stamp = rclcpp::Time(now_ns, RCL_ROS_TIME);
      t.child_frame_id = "base_link";
      t.transform.translation.x = x;
      t.transform.translation.y = y;
      t.transform.rotation.z = std::sin(th / 2.0);
      t.transform.rotation.w = std::cos(th / 2.0);
      tf->setTransform(t, "sim", false);
    };
    publish_tf();

    mowgli_nav2_plugins::FTCController ftc;
    ftc.configure(node, plugin, tf, costmap);
    ftc.activate();

    nav_msgs::msg::Path msg;
    msg.header.frame_id = "map";
    for (size_t i = 0; i < path.size(); ++i)
    {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = "map";
      ps.pose.position.x = path[i].x;
      ps.pose.position.y = path[i].y;
      const size_t j = (i + 1 < path.size()) ? i + 1 : i;
      const size_t k = (i + 1 < path.size()) ? i : i - 1;
      const double yaw = std::atan2(path[j].y - path[k].y, path[j].x - path[k].x);
      ps.pose.orientation.z = std::sin(yaw / 2.0);
      ps.pose.orientation.w = std::cos(yaw / 2.0);
      msg.poses.push_back(ps);
    }
    ftc.setPlan(msg);

    SimResult r;
    const double dt = 0.05, tau = 0.15;
    int zero_ticks = 0;
    for (double t = 0.0; t < t_max; t += dt)
    {
      now_ns += static_cast<rcl_time_point_value_t>(dt * 1e9);
      (void)rcl_set_ros_time_override(h, now_ns);
      publish_tf();
      geometry_msgs::msg::PoseStamped pose;
      geometry_msgs::msg::Twist vel;
      vel.linear.x = v;
      vel.angular.z = w;
      geometry_msgs::msg::TwistStamped cmd;
      try
      {
        cmd = ftc.computeVelocityCommands(pose, vel, nullptr);
      }
      catch (const std::exception& e)
      {
        r.threw = true;
        r.t_end = t;
        break;
      }
      const double vc = std::clamp(cmd.twist.linear.x, -0.3, 0.3);
      const double wc = std::clamp(cmd.twist.angular.z, -mcu_w_max, mcu_w_max);
      v += (vc - v) * dt / tau;
      w += (wc - w) * dt / tau;
      th += w * dt;
      x += v * std::cos(th) * dt;
      y += v * std::sin(th) * dt;
      const double xt = PolylineDistance(path, x, y);
      if (t > 1.0)  // ignore the initial-offset transient of the first second
        r.max_xtrack = std::max(r.max_xtrack, xt);
      r.final_xtrack = xt;
      r.t_end = t;
      // FINISHED -> zero command for a while.
      if (cmd.twist.linear.x == 0.0 && cmd.twist.angular.z == 0.0 && t > 2.0)
      {
        if (++zero_ticks > 20)
        {
          r.finished = true;
          break;
        }
      }
      else
      {
        zero_ticks = 0;
      }
    }
    const size_t n = path.size();
    const double path_yaw = std::atan2(path[n - 1].y - path[n - 2].y, path[n - 1].x - path[n - 2].x);
    r.final_heading_err_deg = std::atan2(std::sin(path_yaw - th), std::cos(path_yaw - th)) * 180.0 / M_PI;
    ftc.deactivate();
    ftc.cleanup();
    return r;
  }
};

constexpr double kDeg = M_PI / 180.0;
// Effective yaw rate the Tron achieved in turf with the blade on during the
// 2026-10-06 run: /cmd_vel -0.8 rad/s, slip_detector |d_angular| 0.616 rad/s
// against wheel odom -> ~0.18-0.2 rad/s delivered (driver clamp is 0.3).
constexpr double kTurfYawRate = 0.2;

TEST_F(FtcSim, StraightSwathFrom16DegConvergesLiveParamsNoMcuClamp)
{
  const auto path = Straight(6.0, 0.1);
  const auto r = Run(LiveParams(), path, 0.0, 0.0, 16.0 * kDeg, 10.0, 60.0);
  std::printf("live/noclamp straight: max_xt=%.3f final_xt=%.3f head=%.1f fin=%d threw=%d t=%.1f\n",
              r.max_xtrack, r.final_xtrack, r.final_heading_err_deg, r.finished, r.threw, r.t_end);
  EXPECT_FALSE(r.threw);
  EXPECT_LT(r.max_xtrack, 0.10);
  EXPECT_LT(r.final_xtrack, 0.03);
  EXPECT_LT(std::abs(r.final_heading_err_deg), 5.0);
}

TEST_F(FtcSim, StraightSwathFrom16DegConvergesWithMcuClamp)
{
  const auto path = Straight(6.0, 0.1);
  for (const auto& params : {LiveParams(), TronParams()})
  {
    const auto r = Run(params, path, 0.0, 0.0, 16.0 * kDeg, 0.3, 60.0);
    std::printf("clamp0.3 straight: max_xt=%.3f final_xt=%.3f head=%.1f fin=%d threw=%d t=%.1f\n",
                r.max_xtrack, r.final_xtrack, r.final_heading_err_deg, r.finished, r.threw, r.t_end);
    EXPECT_FALSE(r.threw);
    EXPECT_LT(r.max_xtrack, 0.12);
    EXPECT_LT(r.final_xtrack, 0.03);
    EXPECT_LT(std::abs(r.final_heading_err_deg), 5.0);
  }
}

// Reproduces the 2026-10-06 failure: on a ring the live parameters command
// up to 0.8 rad/s and rotate the carrot at 45 deg/s, the MCU delivers at most
// 0.3 rad/s, and the robot is carried off the ring at the first corner.
TEST_F(FtcSim, RingWithLiveParamsLeavesTheRingUnderMcuClamp)
{
  const auto path = RectRing(3.0, 2.0, 0.1);
  const auto r = Run(LiveParams(), path, 0.0, 0.0, 16.0 * kDeg, kTurfYawRate, 120.0);
  std::printf("live/clamp ring: max_xt=%.3f final_xt=%.3f fin=%d threw=%d t=%.1f\n",
              r.max_xtrack, r.final_xtrack, r.finished, r.threw, r.t_end);
  EXPECT_GT(r.max_xtrack, 0.25) << "expected the live tuning to reproduce the excursion";
}

TEST_F(FtcSim, SwathEndTurnsWithTronParamsStayOnPath)
{
  const auto path = Boustrophedon(4, 3.0, 0.18, 0.1);
  const auto r = Run(TronParams(), path, 0.0, 0.0, 16.0 * kDeg, kTurfYawRate, 200.0);
  std::printf("tron/clamp swaths: max_xt=%.3f final_xt=%.3f fin=%d threw=%d t=%.1f\n",
              r.max_xtrack, r.final_xtrack, r.finished, r.threw, r.t_end);
  EXPECT_FALSE(r.threw);
  EXPECT_TRUE(r.finished);
  EXPECT_LT(r.max_xtrack, 0.15);
}

TEST_F(FtcSim, RingWithTronParamsStaysOnTheRingUnderMcuClamp)
{
  const auto path = RectRing(3.0, 2.0, 0.1);
  const auto r = Run(TronParams(), path, 0.0, 0.0, 16.0 * kDeg, kTurfYawRate, 120.0);
  std::printf("tron/clamp ring: max_xt=%.3f final_xt=%.3f fin=%d threw=%d t=%.1f\n",
              r.max_xtrack, r.final_xtrack, r.finished, r.threw, r.t_end);
  EXPECT_FALSE(r.threw);
  EXPECT_LT(r.max_xtrack, 0.15);
}

}  // namespace

namespace
{
// Which ingredient matters: each change applied alone to the live set.
TEST_F(FtcSim, RingAblation)
{
  const auto path = RectRing(3.0, 2.0, 0.1);
  struct Variant
  {
    const char* name;
    Params p;
  };
  std::vector<Variant> vs;
  vs.push_back({"live", LiveParams()});
  {
    auto p = LiveParams();
    p["scale_linear_on_angular_saturation"] = rclcpp::ParameterValue(true);
    p["max_cmd_vel_ang"] = rclcpp::ParameterValue(0.3);
    vs.push_back({"live+scale+maxang0.3", p});
  }
  {
    auto p = LiveParams();
    p["speed_angular"] = rclcpp::ParameterValue(15.0);
    vs.push_back({"live+speed_angular15", p});
  }
  {
    auto p = LiveParams();
    p["max_cmd_vel_ang"] = rclcpp::ParameterValue(0.3);
    vs.push_back({"live+maxang0.3", p});
  }
  {
    auto p = TronParams();
    p["scale_linear_on_angular_saturation"] = rclcpp::ParameterValue(false);
    vs.push_back({"tron-noscale", p});
  }
  vs.push_back({"tron", TronParams()});
  for (double sa : {10.0, 20.0, 25.0})
  {
    auto p = TronParams();
    p["speed_angular"] = rclcpp::ParameterValue(sa);
    static std::vector<std::string> names;
    names.push_back("tron speed_angular " + std::to_string(static_cast<int>(sa)));
    vs.push_back({names.back().c_str(), p});
  }
  for (const auto& v : vs)
  {
    for (double wmax : {0.3, kTurfYawRate, 0.15})
    {
      const auto r = Run(v.p, path, 0.0, 0.0, 16.0 * kDeg, wmax, 120.0);
      std::printf("ablation %-24s yaw<=%.2f: ring max_xt=%.3f fin=%d threw=%d t=%.1f",
                  v.name, wmax, r.max_xtrack, r.finished, r.threw, r.t_end);
      const auto b = Run(v.p, Boustrophedon(4, 3.0, 0.18, 0.1), 0.0, 0.0, 16.0 * kDeg, wmax, 200.0);
      std::printf(" | swaths max_xt=%.3f fin=%d threw=%d t=%.1f\n",
                  b.max_xtrack, b.finished, b.threw, b.t_end);
    }
  }
}
}  // namespace

namespace
{
// Replay of the real 2026-10-06 ring. Start pose from the first FTC log line:
// carrot (0.19, 0.02) ahead in base_link at angle_err -16.1 deg, i.e. robot
// yaw = -146.1 + 16.1 deg. The ring turns ~110 deg right within 0.25 m at
// idx 9-12, which is where the robot left it.
TEST_F(FtcSim, Replay20261006RingLiveParamsLeavesTheRing)
{
  const double yaw = (-146.1 + 16.1) * kDeg;
  const double x0 = -1.80 - 0.19 * std::cos(yaw), y0 = -1.29 - 0.19 * std::sin(yaw);
  const auto r = Run(LiveParams(), kRing20261006, x0, y0, yaw, kTurfYawRate, 30.0);
  std::printf("replay live: max_xt=%.3f fin=%d threw=%d t=%.1f\n", r.max_xtrack, r.finished, r.threw,
              r.t_end);
  EXPECT_GT(r.max_xtrack, 0.3);
}

TEST_F(FtcSim, Replay20261006RingTronParamsStaysOnTheRing)
{
  const double yaw = (-146.1 + 16.1) * kDeg;
  const double x0 = -1.80 - 0.19 * std::cos(yaw), y0 = -1.29 - 0.19 * std::sin(yaw);
  for (double wmax : {0.3, kTurfYawRate, 0.15})
  {
    const auto r = Run(TronParams(), kRing20261006, x0, y0, yaw, wmax, 200.0);
    std::printf("replay tron yaw<=%.2f: max_xt=%.3f fin=%d threw=%d t=%.1f\n", wmax, r.max_xtrack,
                r.finished, r.threw, r.t_end);
    EXPECT_FALSE(r.threw);
    EXPECT_LT(r.max_xtrack, 0.15);
  }
}
}  // namespace
