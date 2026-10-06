// mower_coverage_node: /coverage/plan service (geometry_msgs/Polygon boundary
// + holes + start/goal in, nav_msgs/Path out) backed by the pure-geometry
// planner in mower_coverage_core (Fields2Cover headland + swath).
//
// The plan carries no turn geometry: the mower is diff-drive and pivots in
// place between segments, so turns are the navigation stack's job.

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <mower_interfaces/srv/plan_coverage.hpp>

#include "mower_coverage/coverage_planning.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace {

// Sentinel defaults applied when a request field is left at its "unset" value.
// The swath spacing default is cut_width_m - swath_overlap_m (ROS parameters,
// set from mower.launch.py); these are only the parameter defaults.
// cut_width_m: URDF cutter disc radius 0.10 m -> 0.20 m. TODO: measure the blade.
constexpr double kDefaultCutWidth = 0.20;         // blade cut width [m]
constexpr double kDefaultSwathOverlap = 0.02;     // overlap between swaths [m]
constexpr double kDefaultHeadlandWidth = 0.20;    // desired headland band [m]
constexpr double kDefaultMinSwathLength = 0.15;   // sliver-drop threshold [m]
constexpr double kDuplicatePoseTol = 1e-6;        // dedupe consecutive poses

double distance(const mower_coverage::Point2D& a, const mower_coverage::Point2D& b) {
  return std::hypot(a.first - b.first, a.second - b.second);
}

std::vector<mower_coverage::Point2D> polygonToPoints(
    const geometry_msgs::msg::Polygon& poly) {
  std::vector<mower_coverage::Point2D> pts;
  pts.reserve(poly.points.size());
  for (const auto& p : poly.points) {
    pts.emplace_back(p.x, p.y);
  }
  return pts;
}

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

}  // namespace

class MowerCoverageNode : public rclcpp::Node {
 public:
  MowerCoverageNode() : rclcpp::Node("mower_coverage_node") {
    const double cut_width = declare_parameter<double>("cut_width_m", kDefaultCutWidth);
    const double overlap =
        declare_parameter<double>("swath_overlap_m", kDefaultSwathOverlap);
    default_operation_width_ = cut_width - overlap;
    if (!(cut_width > 0.0) || overlap < 0.0 || !(default_operation_width_ > 0.0)) {
      RCLCPP_ERROR(get_logger(),
                   "invalid cut_width_m=%.3f / swath_overlap_m=%.3f; using %.3f - %.3f",
                   cut_width, overlap, kDefaultCutWidth, kDefaultSwathOverlap);
      default_operation_width_ = kDefaultCutWidth - kDefaultSwathOverlap;
    }
    service_ = create_service<mower_interfaces::srv::PlanCoverage>(
        "/coverage/plan",
        std::bind(&MowerCoverageNode::handlePlan, this, std::placeholders::_1,
                  std::placeholders::_2));
    RCLCPP_INFO(get_logger(),
                "mower_coverage_node ready: service /coverage/plan "
                "(Fields2Cover headland + swath, default swath spacing %.3f m)",
                default_operation_width_);
  }

 private:
  void handlePlan(
      const std::shared_ptr<mower_interfaces::srv::PlanCoverage::Request> request,
      std::shared_ptr<mower_interfaces::srv::PlanCoverage::Response> response) {
    const auto t0 = std::chrono::steady_clock::now();

    const double op_width = request->operation_width > 0.0
                                ? request->operation_width
                                : default_operation_width_;
    const double headland_width = request->headland_width > 0.0
                                      ? request->headland_width
                                      : kDefaultHeadlandWidth;
    const double min_swath_length = request->min_swath_length > 0.0
                                        ? request->min_swath_length
                                        : kDefaultMinSwathLength;
    const double mow_angle_rad =
        request->mow_angle_deg >= 0.0
            ? request->mow_angle_deg * M_PI / 180.0
            : -1.0;  // < 0 = auto (fewest swaths)
    const std::string frame_id =
        request->frame_id.empty() ? "map" : request->frame_id;

    if (request->boundary.points.size() < 3) {
      response->success = false;
      response->message = "boundary needs at least 3 points";
      return;
    }

    std::vector<std::vector<mower_coverage::Point2D>> holes;
    holes.reserve(request->holes.size());
    for (const auto& h : request->holes) {
      holes.push_back(polygonToPoints(h));
    }

    auto field = mower_coverage::makeFieldCell(
        polygonToPoints(request->boundary), holes);

    mower_coverage::PathMode mode = mower_coverage::PathMode::kZigzag;
    if (!mower_coverage::parsePathMode(request->path_mode, &mode)) {
      response->success = false;
      response->message = "unknown path_mode '" + request->path_mode +
                          "' (zigzag | spiral | contour_only)";
      return;
    }
    // headland_rings (per-area perimeter_laps): -1 = keep headland_passes,
    // 0 = no rings, > 0 = exactly that many.
    const int headland_passes =
        request->headland_rings < 0
            ? request->headland_passes
            : (request->headland_rings == 0 ? -1 : request->headland_rings);

    const mower_coverage::CoveragePlan plan =
        mower_coverage::planCoverage(field, op_width, headland_width,
                                     headland_passes,
                                     /*border_inset=*/0.0,
                                     mow_angle_rad, min_swath_length, mode,
                                     request->edge_first);

    // ---- Assemble nav_msgs/Path: transit-in, rings, swaths, transit-out ----
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = frame_id;

    auto addPose = [&](const mower_coverage::Point2D& p, double yaw) {
      if (!path.poses.empty() &&
          distance({path.poses.back().pose.position.x,
                    path.poses.back().pose.position.y},
                   p) < kDuplicatePoseTol) {
        return;  // zero-length edge — drop
      }
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = p.first;
      pose.pose.position.y = p.second;
      pose.pose.position.z = 0.0;
      pose.pose.orientation = yawToQuaternion(yaw);
      path.poses.push_back(pose);
    };

    // The path pieces in drive order: rings (closed loops) then swaths, or
    // swaths then rings when the plan is swaths-first (edge_first = false).
    std::vector<std::vector<mower_coverage::Point2D>> segments;
    segments.reserve(plan.rings.size() + plan.swaths.size());
    auto addRings = [&]() {
      for (const auto& ring : plan.rings) {
        segments.push_back(ring);
      }
    };
    auto addSwaths = [&]() {
      for (const auto& swath : plan.swaths) {
        segments.push_back({swath.first, swath.second});
      }
    };
    if (plan.swaths_first) {
      addSwaths();
      addRings();
    } else {
      addRings();
      addSwaths();
    }

    // Optional transit from the mower's start position to the first pose.
    const bool has_start =
        request->has_start &&
        !segments.empty() &&
        distance({request->start.x, request->start.y}, segments.front().front()) >
            kDuplicatePoseTol;
    if (has_start) {
      const auto start = mower_coverage::Point2D{request->start.x,
                                                 request->start.y};
      addPose(start, std::atan2(segments.front().front().second - start.second,
                                segments.front().front().first - start.first));
    }

    for (const auto& seg : segments) {
      for (size_t i = 0; i < seg.size(); ++i) {
        const auto& p = seg[i];
        const auto& q = seg[(i + 1) % seg.size()];
        addPose(p, std::atan2(q.second - p.second, q.first - p.first));
      }
    }

    // Optional transit from the last pose to the goal.
    if (request->has_goal && !path.poses.empty()) {
      const auto goal =
          mower_coverage::Point2D{request->goal.x, request->goal.y};
      const auto last = mower_coverage::Point2D{
          path.poses.back().pose.position.x, path.poses.back().pose.position.y};
      if (distance(last, goal) > kDuplicatePoseTol) {
        addPose(goal, std::atan2(goal.second - last.second,
                                 goal.first - last.first));
      }
    }

    // ---- Response ----
    double total_distance = 0.0;
    for (size_t i = 1; i < path.poses.size(); ++i) {
      total_distance += std::hypot(
          path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x,
          path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y);
    }

    const double planning_time_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
            .count();

    response->path = path;
    response->total_distance = total_distance;
    response->ring_count = static_cast<uint32_t>(plan.rings.size());
    response->swath_count = static_cast<uint32_t>(plan.swaths.size());
    response->planning_time_s = planning_time_s;
    response->success = !path.poses.empty();

    std::string msg = std::string(plan.swaths_first ? "swaths first, " : "") +
                      (request->path_mode.empty() ? std::string("zigzag")
                                                  : request->path_mode) +
                      ": " + std::to_string(plan.rings.size()) + " rings, " +
                      std::to_string(plan.swaths.size()) + " swaths, " +
                      std::to_string(static_cast<int>(total_distance * 100)) +
                      " cm, fraction " +
                      std::to_string(static_cast<int>(plan.planned_fraction * 1000) / 1000.0);
    for (const auto& d : plan.drops) {
      msg += "; " + d;
    }
    response->message = msg;

    if (!response->success) {
      RCLCPP_WARN(get_logger(), "coverage plan failed: %s", msg.c_str());
    }
  }

  rclcpp::Service<mower_interfaces::srv::PlanCoverage>::SharedPtr service_;
  double default_operation_width_ = kDefaultCutWidth - kDefaultSwathOverlap;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MowerCoverageNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
