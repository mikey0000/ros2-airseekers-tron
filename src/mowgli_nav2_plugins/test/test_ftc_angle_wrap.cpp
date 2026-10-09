// SPDX-License-Identifier: GPL-3.0-or-later
//
// PRE_ROTATE -> FOLLOWING hand-off must not carry a 2*pi offset in the
// heading-error unwrap accumulator.
//
// update_control_point() keeps angle_error_ as an UNWRAPPED accumulator
// (issue #200). PRE_ROTATE and POST_ROTATE gate and steer on the wrapped
// angle, but FOLLOWING feeds angle_error_ itself into the P term. When the
// path start lies behind the robot (heading error near +-180 deg, the usual
// case when a ring starts after a transit/undock) one heading sample on the
// other side of +-pi (pose noise, an IMU/GPS heading correction) makes the
// accumulator and the wrapped angle differ by 2*pi. PRE_ROTATE then pivots the
// short way and exits on the WRAPPED angle (< max_goal_angle_error) while the
// accumulator reads ~2*pi, and FOLLOWING commands kp_ang_following * 2*pi =
// saturated angular velocity: the robot drives a full circle off the path.
//
// Closed-loop unicycle sim of the real plugin, same harness style as
// test_ftc_tracking_sim.cpp (TF map->base_link from the model, ROS clock
// override, 20 Hz).

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

constexpr double kDeg = M_PI / 180.0;

struct Result
{
  bool threw{false};
  double max_abs_y_following{0.0};  // cross-track from the y = 0 path
  double max_abs_w_following{0.0};  // largest |cmd angular| during FOLLOWING
  double yaw_turned_following{0.0}; // integral of |yaw rate| during FOLLOWING (rad)
  bool reached_following{false};
};

class FtcAngleWrap : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  // Straight path along +x from the origin. The robot starts on the path
  // facing (almost) backwards. yaw_first is the heading of the very first
  // sample only; afterwards the model integrates from yaw_after.
  Result Run(double yaw_first, double yaw_after)
  {
    const std::string plugin = "FTC";
    rclcpp::NodeOptions opts;
    std::vector<rclcpp::Parameter> ov;
    ov.emplace_back(plugin + ".check_obstacles", false);
    ov.emplace_back(plugin + ".enable_obstacle_deviation", false);
    // Tron tuning (FollowCoveragePathFTC in nav2_params.yaml).
    ov.emplace_back(plugin + ".desired_linear_vel", 0.3);
    ov.emplace_back(plugin + ".speed_angular", 15.0);
    ov.emplace_back(plugin + ".acceleration", 0.2);
    ov.emplace_back(plugin + ".min_speed_mps", 0.1);
    ov.emplace_back(plugin + ".kp_lon", 1.0);
    ov.emplace_back(plugin + ".kp_lat", 1.0);
    ov.emplace_back(plugin + ".kd_lat", 0.3);
    ov.emplace_back(plugin + ".kp_ang", 1.0);
    ov.emplace_back(plugin + ".kp_ang_following", 0.8);
    ov.emplace_back(plugin + ".derivative_filter_tau", 0.2);
    ov.emplace_back(plugin + ".max_cmd_vel_speed", 0.3);
    ov.emplace_back(plugin + ".max_cmd_vel_ang", 0.3);
    ov.emplace_back(plugin + ".max_goal_angle_error", 15.0);
    // A 180 deg pivot at 0.3 rad/s needs > 10 s; this test is about the
    // hand-off, not the PRE_ROTATE timeout.
    ov.emplace_back(plugin + ".goal_timeout", 30.0);
    ov.emplace_back(plugin + ".max_follow_distance", 2.0);
    ov.emplace_back(plugin + ".forward_only", true);
    ov.emplace_back(plugin + ".scale_linear_on_angular_saturation", true);
    opts.parameter_overrides(ov);
    static int run_id = 0;
    const std::string ns = "/ftc_wrap_" + std::to_string(run_id++);
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server", ns, opts);

    auto clock = node->get_clock();
    rcl_clock_t* h = clock->get_clock_handle();
    EXPECT_EQ(rcl_enable_ros_time_override(h), RCL_RET_OK);
    rcl_time_point_value_t now_ns = 1000LL * 1000000000LL;
    (void)rcl_set_ros_time_override(h, now_ns);

    auto costmap =
        std::make_shared<nav2_costmap_2d::Costmap2DROS>("local_costmap", ns, "local_costmap");
    costmap->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{}));
    costmap->configure();

    auto tf = std::make_shared<tf2_ros::Buffer>(clock);
    tf->setUsingDedicatedThread(true);
    double x = 0.0, y = 0.0, th = yaw_first, v = 0.0, w = 0.0;
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
    for (int i = 0; i <= 60; ++i)
    {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = "map";
      ps.pose.position.x = 0.1 * i;
      ps.pose.orientation.w = 1.0;
      msg.poses.push_back(ps);
    }
    ftc.setPlan(msg);

    Result r;
    const double dt = 0.05, tau = 0.15, w_max = 0.3;
    bool first = true;
    bool moved_forward = false;
    for (double t = 0.0; t < 60.0; t += dt)
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
      catch (const std::exception&)
      {
        r.threw = true;
        break;
      }
      if (first)
      {
        // One heading sample on the other side of +-180 deg, then the model
        // continues from yaw_after (a few degrees of pose noise).
        th = yaw_after;
        first = false;
      }
      if (cmd.twist.linear.x > 0.0)
      {
        moved_forward = true;  // PRE_ROTATE commands linear 0; FOLLOWING > 0
      }
      if (moved_forward)
      {
        r.reached_following = true;
        r.max_abs_w_following = std::max(r.max_abs_w_following, std::abs(cmd.twist.angular.z));
      }
      const double vc = std::clamp(cmd.twist.linear.x, -0.3, 0.3);
      const double wc = std::clamp(cmd.twist.angular.z, -w_max, w_max);
      v += (vc - v) * dt / tau;
      w += (wc - w) * dt / tau;
      th += w * dt;
      x += v * std::cos(th) * dt;
      y += v * std::sin(th) * dt;
      if (moved_forward)
      {
        r.max_abs_y_following = std::max(r.max_abs_y_following, std::abs(y));
        r.yaw_turned_following += std::abs(w) * dt;
      }
      if (x > 5.0)
      {
        break;  // well along the path
      }
    }
    ftc.deactivate();
    ftc.cleanup();
    return r;
  }
};

// Control: no sample across +-180 deg. PRE_ROTATE pivots and FOLLOWING tracks.
TEST_F(FtcAngleWrap, PathBehindRobotNoWrapTracksPath)
{
  const auto r = Run(178.0 * kDeg, 178.0 * kDeg);
  std::printf("no wrap: following=%d max_y=%.3f max_w=%.3f turned=%.0f deg threw=%d\n",
              r.reached_following, r.max_abs_y_following, r.max_abs_w_following,
              r.yaw_turned_following / kDeg, r.threw);
  EXPECT_FALSE(r.threw);
  EXPECT_TRUE(r.reached_following);
  EXPECT_LT(r.max_abs_y_following, 0.15);
  EXPECT_LT(r.yaw_turned_following, 60.0 * kDeg);
}

// One heading sample at -178 deg, then +178 deg (4 deg of noise across the
// +-180 seam). Before the fix the accumulator left PRE_ROTATE at ~+345 deg
// and FOLLOWING saturated the angular command, looping the robot off the path.
TEST_F(FtcAngleWrap, PathBehindRobotWrapAcrossPiDoesNotLoopOffPath)
{
  const auto r = Run(-178.0 * kDeg, 178.0 * kDeg);
  std::printf("wrap: following=%d max_y=%.3f max_w=%.3f turned=%.0f deg threw=%d\n",
              r.reached_following, r.max_abs_y_following, r.max_abs_w_following,
              r.yaw_turned_following / kDeg, r.threw);
  EXPECT_FALSE(r.threw);
  EXPECT_TRUE(r.reached_following);
  EXPECT_LT(r.max_abs_y_following, 0.15);
  EXPECT_LT(r.yaw_turned_following, 60.0 * kDeg);
}

}  // namespace
