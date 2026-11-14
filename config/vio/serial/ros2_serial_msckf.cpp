// SPDX-License-Identifier: GPL-3.0-or-later
// ros2_serial_msckf: run OpenVINS ov_msckf over a recorded ROS 2 bag, serially (2026-11-14).
//
// Upstream OpenVINS has a serial bag runner only for ROS 1 (ros1_serial_msckf). This is the
// ROS 2 counterpart for the offline VIO evaluation (docs/vio.md, scripts/vio/eval_vio_bag.sh):
// messages are read with rosbag2_cpp in bag (receive) order and handed straight to
// ROS2Visualizer's callbacks, with use_multi_threading_subs/pubs off, so every camera update
// runs to completion before the next message. No DDS, no wall clock, no dropped frames:
// the same bag gives the same output on every run, and it runs as fast as the CPU allows.
//
// Copied into ov_msckf/src by scripts/vio/stage_openvins.sh; built only with
// -DOV_ROS2_SERIAL=ON (config/vio/patches/0003-*.patch; the mower build leaves it off).
//
//   ros2 run ov_msckf ros2_serial_msckf --ros-args -p config_path:=.../estimator_config.yaml
//       -p bag:=BAG_DIR -p out_dir:=OUT -p verbosity:=WARNING
//
// Output (CSV, seconds in the bag's ROS time):
//   OUT/odomimu.csv  one row per IMU message once /ov_msckf/odomimu would publish (1 s after
//                    init): t, q_GtoI xyzw (JPL, as published), p_IinG, v (IMU frame, the
//                    published twist.linear), w (gyro), var of v (twist covariance diagonal).
//                    Computed like ROS2Visualizer::visualize_odometry, but after the camera
//                    update this IMU message released (live publishes just before it).
//   OUT/updates.csv  one row per stereo pair OpenVINS processed (also before init): t (image
//                    stamp), wall_ms of the IMU callback that ran it, initialized, MSCKF
//                    features used, SLAM features in the state, camera-IMU time offset.
// The summary line counts the pairs ROS2Visualizer dropped itself (closer than
// 1 / track_frequency to the previous one).

#include <chrono>
#include <cstdio>
#include <fstream>
#include <algorithm>
#include <map>
#include <vector>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "ros/ROS2Visualizer.h"
#include "state/Propagator.h"
#include "state/State.h"
#include "types/Vec.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"

using namespace ov_msckf;

namespace {

// ROS2Visualizer with read access to its camera queue: how many pairs the last callback
// queued or processed.
class SerialVisualizer : public ROS2Visualizer {
public:
  using ROS2Visualizer::ROS2Visualizer;
  size_t queued() {
    std::lock_guard<std::mutex> lck(camera_queue_mtx);
    return camera_queue.size();
  }
  std::vector<double> queued_stamps() {
    std::lock_guard<std::mutex> lck(camera_queue_mtx);
    std::vector<double> out;
    for (const auto &c : camera_queue)
      out.push_back(c.timestamp);
    return out;
  }
};

double stamp_of(const builtin_interfaces::msg::Time &t) { return t.sec + t.nanosec * 1e-9; }

int64_t stamp_ns(const builtin_interfaces::msg::Time &t) { return int64_t(t.sec) * 1000000000LL + t.nanosec; }

// One odomimu row, the same quantities ROS2Visualizer::visualize_odometry publishes.
bool write_odom_row(std::ofstream &out, const std::shared_ptr<VioManager> &sys, double t, const sensor_msgs::msg::Imu &imu) {
  if (!sys->initialized() || (t - sys->initialized_time()) < 1)
    return false;
  Eigen::Matrix<double, 13, 1> sp = Eigen::Matrix<double, 13, 1>::Zero();
  Eigen::Matrix<double, 12, 12> cov = Eigen::Matrix<double, 12, 12>::Zero();
  if (!sys->get_propagator()->fast_state_propagate(sys->get_state(), t, sp, cov))
    return false;
  char buf[512];
  std::snprintf(buf, sizeof(buf), "%.9f,%.9g,%.9g,%.9g,%.9g,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6g,%.6g,%.6g\n", t, sp(0), sp(1),
                sp(2), sp(3), sp(4), sp(5), sp(6), sp(7), sp(8), sp(9), imu.angular_velocity.x, imu.angular_velocity.y,
                imu.angular_velocity.z, cov(6, 6), cov(7, 7), cov(8, 8));
  out << buf;
  return true;
}

} // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("ros2_serial_msckf", options);
  std::string config_path, bag_path, out_dir = ".";
  node->get_parameter("config_path", config_path);
  node->get_parameter("bag", bag_path);
  node->get_parameter("out_dir", out_dir);
  if (config_path.empty() || bag_path.empty()) {
    std::fprintf(stderr, "usage: ros2 run ov_msckf ros2_serial_msckf --ros-args -p config_path:=YAML -p bag:=BAG [-p out_dir:=DIR]\n");
    return 2;
  }

  auto parser = std::make_shared<ov_core::YamlParser>(config_path);
  parser->set_node(node);
  std::string verbosity = "INFO";
  parser->parse_config("verbosity", verbosity);
  ov_core::Printer::setPrintLevel(verbosity);

  VioManagerOptions params;
  params.print_and_load(parser);
  params.use_multi_threading_subs = false; // camera updates complete inside callback_inertial
  params.use_multi_threading_pubs = false;
  auto sys = std::make_shared<VioManager>(params);
  auto viz = std::make_shared<SerialVisualizer>(node, sys);

  // Topics as setup_subscribers resolves them (kalibr chain files, ROS parameter override).
  std::string topic_imu = "/imu0", topic_cam0 = "/cam0/image_raw", topic_cam1 = "/cam1/image_raw";
  node->get_parameter("topic_imu", topic_imu);
  node->get_parameter("topic_camera0", topic_cam0);
  node->get_parameter("topic_camera1", topic_cam1);
  parser->parse_external("relative_config_imu", "imu0", "rostopic", topic_imu);
  parser->parse_external("relative_config_imucam", "cam0", "rostopic", topic_cam0);
  parser->parse_external("relative_config_imucam", "cam1", "rostopic", topic_cam1);
  if (!parser->successful()) {
    PRINT_ERROR(RED "unable to parse all parameters, please fix\n" RESET);
    return EXIT_FAILURE;
  }
  if (params.state_options.num_cameras != 2) {
    PRINT_ERROR(RED "ros2_serial_msckf handles the stereo case only (num_cameras %d)\n" RESET, params.state_options.num_cameras);
    return EXIT_FAILURE;
  }

  rosbag2_cpp::Reader reader;
  reader.open(bag_path);
  rosbag2_storage::StorageFilter filter;
  filter.topics = {topic_imu, topic_cam0, topic_cam1};
  reader.set_filter(filter);

  std::ofstream odom_csv(out_dir + "/odomimu.csv"), upd_csv(out_dir + "/updates.csv");
  odom_csv << "t,qx,qy,qz,qw,px,py,pz,vx,vy,vz,wx,wy,wz,var_vx,var_vy,var_vz\n";
  upd_csv << "t,wall_ms,initialized,msckf_feats,slam_feats,cam_imu_dt\n";

  rclcpp::Serialization<sensor_msgs::msg::Imu> ser_imu;
  rclcpp::Serialization<sensor_msgs::msg::Image> ser_img;
  // Both eyes carry the same stamp (stereo_cam cuts them from one buffer): pair by stamp.
  std::map<int64_t, sensor_msgs::msg::Image::SharedPtr> pending[2];
  size_t n_imu = 0, n_img[2] = {0, 0}, n_pairs = 0, n_unpaired = 0, n_updates = 0, n_odom = 0, n_dropped = 0;
  double t_first = -1.0, t_last = -1.0, wall_updates = 0.0;
  auto wall0 = std::chrono::steady_clock::now();

  while (rclcpp::ok() && reader.has_next()) {
    auto bm = reader.read_next();
    rclcpp::SerializedMessage sm(*bm->serialized_data);
    if (bm->topic_name == topic_imu) {
      auto imu = std::make_shared<sensor_msgs::msg::Imu>();
      ser_imu.deserialize_message(&sm, imu.get());
      double t = stamp_of(imu->header.stamp);
      if (t_first < 0)
        t_first = t;
      t_last = t;
      n_imu++;
      std::vector<double> before = viz->queued_stamps();
      auto w0 = std::chrono::steady_clock::now();
      viz->callback_inertial(imu);
      double wall_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - w0).count();
      size_t done = before.size() - std::min(before.size(), viz->queued());
      for (size_t k = 0; k < done; k++) {
        n_updates++;
        wall_updates += wall_ms / done;
        char buf[256];
        std::snprintf(buf, sizeof(buf), "%.9f,%.3f,%d,%zu,%zu,%.6f\n", before[k], wall_ms / done, sys->initialized() ? 1 : 0,
                      sys->get_good_features_MSCKF().size(), sys->get_features_SLAM().size(),
                      sys->get_state()->_calib_dt_CAMtoIMU->value()(0));
        upd_csv << buf;
      }
      if (write_odom_row(odom_csv, sys, t, *imu))
        n_odom++;
      continue;
    }
    int cam = (bm->topic_name == topic_cam0) ? 0 : 1;
    auto img = std::make_shared<sensor_msgs::msg::Image>();
    ser_img.deserialize_message(&sm, img.get());
    n_img[cam]++;
    int64_t key = stamp_ns(img->header.stamp);
    auto other = pending[1 - cam].find(key);
    if (other == pending[1 - cam].end()) {
      pending[cam][key] = img;
    } else {
      auto left = cam == 0 ? img : other->second;
      auto right = cam == 0 ? other->second : img;
      pending[1 - cam].erase(other);
      size_t q = viz->queued();
      viz->callback_stereo(left, right, 0, 1);
      n_pairs++;
      if (viz->queued() == q)
        n_dropped++;
    }
    // An eye that has waited 1 s for its partner never gets one.
    for (auto &p : pending) {
      while (!p.empty() && p.begin()->first < key - 1000000000LL) {
        p.erase(p.begin());
        n_unpaired++;
      }
    }
  }
  double wall_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall0).count();
  viz->visualize_final();
  std::printf("ros2_serial_msckf: %zu IMU, %zu/%zu images, %zu stereo pairs (%zu unpaired, %zu dropped by track_frequency), "
              "%zu processed, %zu odom rows\n",
              n_imu, n_img[0], n_img[1], n_pairs, n_unpaired, n_dropped, n_updates, n_odom);
  std::printf("ros2_serial_msckf: bag span %.1f s, wall %.1f s (%.1fx real time), updates %.1f ms mean, initialized %s",
              t_last - t_first, wall_s, wall_s > 0 ? (t_last - t_first) / wall_s : 0.0, n_updates ? wall_updates / n_updates : 0.0,
              sys->initialized() ? "yes" : "no");
  if (sys->initialized())
    std::printf(" at +%.2f s", sys->initialized_time() - t_first);
  std::printf("\n");
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
