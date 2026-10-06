// mower_coverage_node: /coverage/plan service (geometry_msgs/Polygon boundary
// + holes + start/goal in, nav_msgs/Path out) backed by the pure-geometry
// planner in mower_coverage_core (Fields2Cover headland + swath).
//
// Swath-to-swath turns (route_order / min_turn_radius_m / turn_type) are
// emitted as extra poses before the swath they lead into and flagged in
// pose_flags (POSE_TURN / POSE_TURN_REVERSE), so the bridge can strip them
// for its structural ring/swath split and re-insert them into the sub-paths.

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <mower_interfaces/srv/plan_coverage.hpp>

#include "mower_coverage/coverage_planning.hpp"

#include <algorithm>
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
// Extra pull-back of the OUTER ring beyond half a swath [m]. The outermost
// ring's centreline sits op_width/2 + boundary_inset_m inside the recorded
// boundary (blade edge boundary_inset_m inside the line). 0.15: the Tron's
// tracking error put it 9 cm outside the boundary in a field test.
constexpr double kDefaultBoundaryInset = 0.15;
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
    // Chassis pull-back of the OUTERMOST driven pass inside the recorded boundary [m].
    // 0 = the outer ring centreline runs on the recorded line. Read per request, so
    // `ros2 param set` applies to the next plan. See docs/analysis/2026-10-06_ring_drift.md.
    declare_parameter<double>("border_inset_m", 0.0);
    boundary_inset_ =
        declare_parameter<double>("boundary_inset_m", kDefaultBoundaryInset);
    if (!(boundary_inset_ >= 0.0)) {
      RCLCPP_ERROR(get_logger(), "invalid boundary_inset_m=%.3f; using %.3f",
                   boundary_inset_, kDefaultBoundaryInset);
      boundary_inset_ = kDefaultBoundaryInset;
    }
    // 0 = pivot in place between swaths (diff-drive Tron): straight
    // connectors. > 0 = F2C v3 Dubins turns of this radius between swaths.
    // Read per request (request min_turn_radius_m < 0 = this parameter).
    declare_parameter<double>("min_turn_radius_m", 0.0);
    service_ = create_service<mower_interfaces::srv::PlanCoverage>(
        "/coverage/plan",
        std::bind(&MowerCoverageNode::handlePlan, this, std::placeholders::_1,
                  std::placeholders::_2));
    RCLCPP_INFO(get_logger(),
                "mower_coverage_node ready: service /coverage/plan "
                "(Fields2Cover %s headland + swath, default swath spacing %.3f m, "
                "boundary_inset_m %.3f, min_turn_radius_m %.3f)",
                mower_coverage::builtWithF2CV3() ? "v3" : "2.1",
                default_operation_width_, boundary_inset_,
                get_parameter("min_turn_radius_m").as_double());
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
    // Per-area edge margin (request boundary_inset_m >= 0) overrides the node param.
    const double boundary_inset = request->boundary_inset_m >= 0.0
                                      ? request->boundary_inset_m
                                      : boundary_inset_;
    // Outer ring centreline inset from the recorded boundary: the larger of the
    // absolute border_inset_m and blade-edge boundary_inset + half a swath.
    const double border_inset = std::max(
        std::max(0.0, get_parameter("border_inset_m").as_double()),
        op_width / 2.0 + boundary_inset);
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

    mower_coverage::RouteOptions route;
    if (!mower_coverage::parseRouteOrder(request->route_order, &route.order)) {
      response->success = false;
      response->message = "unknown route_order '" + request->route_order +
                          "' (boustrophedon | snake | spiral)";
      return;
    }
    if (!mower_coverage::parseTurnType(request->turn_type, &route.turn_type)) {
      response->success = false;
      response->message = "unknown turn_type '" + request->turn_type +
                          "' (auto | loop | reverse | pivot)";
      return;
    }
    route.spiral_size = std::max(2, static_cast<int>(request->route_spiral_size));
    route.min_turn_radius = request->min_turn_radius_m >= 0.0
                                ? request->min_turn_radius_m
                                : get_parameter("min_turn_radius_m").as_double();

    route.fill_gaps = true;
    route.min_gap_area_m2 = 0.01;
    route.target_inset = boundary_inset;  // blade edge; holes keep the ring keep-out
    const mower_coverage::CoveragePlan plan =
        mower_coverage::planCoverage(field, op_width, headland_width,
                                     headland_passes,
                                     border_inset,
                                     mow_angle_rad, min_swath_length, mode,
                                     request->edge_first, route);

    // ---- Assemble nav_msgs/Path: transit-in, rings, swaths, transit-out ----
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = frame_id;

    std::vector<uint8_t> flags;
    auto addPose = [&](const mower_coverage::Point2D& p, double yaw, uint8_t flag = 0) {
      if (!path.poses.empty() &&
          distance({path.poses.back().pose.position.x,
                    path.poses.back().pose.position.y},
                   p) < kDuplicatePoseTol) {
        return;  // zero-length edge — drop
      }
      flags.push_back(flag);
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
    // seg_flags parallel to segments: per point 0 mow, 1 turn, 2 reverse turn.
    std::vector<std::vector<mower_coverage::Point2D>> segments;
    std::vector<std::vector<uint8_t>> seg_flags;
    segments.reserve(plan.rings.size() + plan.swaths.size());
    auto addRings = [&]() {
      for (const auto& ring : plan.rings) {
        segments.push_back(ring);
        seg_flags.emplace_back(ring.size(), 0);
      }
    };
    auto addSwaths = [&]() {
      for (size_t i = 0; i < plan.swaths.size(); ++i) {
        const auto& swath = plan.swaths[i];
        std::vector<mower_coverage::Point2D> seg;
        std::vector<uint8_t> fl;
        if (i < plan.turns.size()) {
          for (const auto& tp : plan.turns[i].poses) {
            seg.push_back(tp.p);
            fl.push_back(tp.reverse ? 2 : 1);
          }
        }
        seg.push_back(swath.first);
        seg.push_back(swath.second);
        fl.push_back(0);
        fl.push_back(0);
        segments.push_back(seg);
        seg_flags.push_back(fl);
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

    for (size_t s = 0; s < segments.size(); ++s) {
      const auto& seg = segments[s];
      for (size_t i = 0; i < seg.size(); ++i) {
        const auto& p = seg[i];
        const auto& q = seg[(i + 1) % seg.size()];
        double yaw = std::atan2(q.second - p.second, q.first - p.first);
        if (seg_flags[s][i] == 2) {
          yaw += M_PI;  // reverse: the robot faces against its travel
        }
        addPose(p, yaw, seg_flags[s][i]);
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
    response->pose_flags = flags;
    response->total_distance = total_distance;
    response->ring_count = static_cast<uint32_t>(plan.rings.size());
    response->swath_count = static_cast<uint32_t>(plan.swaths.size());
    response->planning_time_s = planning_time_s;
    response->success = !path.poses.empty();
    response->coverage_fraction = plan.coverage_fraction;
    response->coverage_fraction_before_fill = plan.coverage_fraction_before_fill;
    response->gap_count = static_cast<uint32_t>(plan.gap_count);
    response->gap_area_m2 = plan.gap_area_m2;
    response->fill_swath_count = static_cast<uint32_t>(plan.fill_swaths);
    for (const auto& g : plan.gaps) {
      geometry_msgs::msg::Polygon poly;
      for (size_t i = 0; i + 1 < g.size(); ++i) {  // open ring
        geometry_msgs::msg::Point32 pt;
        pt.x = static_cast<float>(g[i].first);
        pt.y = static_cast<float>(g[i].second);
        poly.points.push_back(pt);
      }
      response->gaps.push_back(poly);
    }

    std::string msg = std::string(plan.swaths_first ? "swaths first, " : "") +
                      (request->path_mode.empty() ? std::string("zigzag")
                                                  : request->path_mode) +
                      (plan.turns.empty()
                           ? std::string()
                           : " (route " + (request->route_order.empty()
                                               ? std::string("boustrophedon")
                                               : request->route_order) +
                                 ", turns r=" +
                                 std::to_string(static_cast<int>(route.min_turn_radius * 100)) +
                                 "cm: " + std::to_string(plan.loop_turns) + " loop, " +
                                 std::to_string(plan.reverse_turns) + " reverse-curve, " +
                                 std::to_string(plan.pivot_turns) + " pivot)") +
                      ": " + std::to_string(plan.rings.size()) + " rings, " +
                      std::to_string(plan.swaths.size()) + " swaths, " +
                      std::to_string(static_cast<int>(total_distance * 100)) +
                      " cm, fraction " +
                      std::to_string(static_cast<int>(plan.planned_fraction * 1000) / 1000.0) +
                      ", coverage " +
                      std::to_string(static_cast<int>(plan.coverage_fraction * 10000) / 10000.0) +
                      " (" + std::to_string(plan.gap_count) + " gaps)";
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
  double boundary_inset_ = kDefaultBoundaryInset;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MowerCoverageNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
