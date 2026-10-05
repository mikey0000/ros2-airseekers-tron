// coverage_cli: tiny standalone CLI test for the mower_coverage planner core.
// Plans a square field (optionally with a centered square hole) and prints
// the resulting coverage path. No ROS runtime needed — links only
// mower_coverage_core, so it runs on any machine with the F2C apt package.
//
//   coverage_cli [--size M] [--op-width M] [--headland N] [--angle DEG]
//                [--hole M] [--min-swath M] [--csv]
//
// Example:
//   coverage_cli --size 4 --op-width 0.5 --headland 1 --csv

#include "mower_coverage/coverage_planning.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace {

struct Options {
  double size = 2.0;          // square field side [m]
  double op_width = 0.18;     // swath spacing [m]
  int headland = 0;           // -1 none, 0 auto, >0 forced
  double angle_deg = -1.0;    // < 0 = auto
  double hole = 0.0;          // centered square hole side [m], 0 = none
  double min_swath = 0.15;    // sliver-drop threshold [m]
  bool csv = false;           // print path as CSV (x,y,yaw)
};

void usage() {
  std::printf(
      "usage: coverage_cli [--size M] [--op-width M] [--headland N] "
      "[--angle DEG] [--hole M] [--min-swath M] [--csv]\n");
}

}  // namespace

int main(int argc, char** argv) {
  Options opt;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const bool has_value = (i + 1 < argc);
    if (arg == "--size" && has_value) {
      opt.size = std::atof(argv[++i]);
    } else if (arg == "--op-width" && has_value) {
      opt.op_width = std::atof(argv[++i]);
    } else if (arg == "--headland" && has_value) {
      opt.headland = std::atoi(argv[++i]);
    } else if (arg == "--angle" && has_value) {
      opt.angle_deg = std::atof(argv[++i]);
    } else if (arg == "--hole" && has_value) {
      opt.hole = std::atof(argv[++i]);
    } else if (arg == "--min-swath" && has_value) {
      opt.min_swath = std::atof(argv[++i]);
    } else if (arg == "--csv") {
      opt.csv = true;
    } else {
      usage();
      return 1;
    }
  }
  if (opt.size <= 0.0 || opt.op_width <= 0.0 || opt.hole < 0.0 ||
      opt.hole >= opt.size) {
    std::fprintf(stderr,
                 "invalid geometry: need size > 0, op_width > 0, "
                 "0 <= hole < size\n");
    return 1;
  }

  // CCW square boundary.
  std::vector<mower_coverage::Point2D> boundary = {
      {0.0, 0.0}, {opt.size, 0.0}, {opt.size, opt.size}, {0.0, opt.size}};

  std::vector<std::vector<mower_coverage::Point2D>> holes;
  if (opt.hole > 0.0) {
    const double c = (opt.size - opt.hole) / 2.0;
    holes.push_back({{c, c},
                     {c + opt.hole, c},
                     {c + opt.hole, c + opt.hole},
                     {c, c + opt.hole}});
  }

  auto field = mower_coverage::makeFieldCell(boundary, holes);
  const double angle_rad =
      opt.angle_deg >= 0.0 ? opt.angle_deg * M_PI / 180.0 : -1.0;
  auto plan = mower_coverage::planBoustrophedon(
      field, opt.op_width, /*headland_width=*/0.20, opt.headland,
      /*border_inset=*/0.0, angle_rad, opt.min_swath);

  double swath_len = 0.0;
  for (const auto& s : plan.swaths) {
    swath_len += std::hypot(s.second.first - s.first.first,
                            s.second.second - s.first.second);
  }
  double ring_len = 0.0;
  for (const auto& ring : plan.rings) {
    for (size_t i = 0; i + 1 < ring.size(); ++i) {
      ring_len += std::hypot(ring[i + 1].first - ring[i].first,
                             ring[i + 1].second - ring[i].second);
    }
  }

  std::printf("field: %.2f x %.2f m, hole: %.2f m, op_width: %.3f m, "
              "headland: %d, angle: %.1f deg\n",
              opt.size, opt.size, opt.hole, opt.op_width, opt.headland,
              opt.angle_deg);
  std::printf("plan: %zu rings (%.2f m), %zu swaths (%.2f m), "
              "swath_angle %.2f deg, fraction %.3f\n",
              plan.rings.size(), ring_len, plan.swaths.size(), swath_len,
              plan.swath_angle_rad * 180.0 / M_PI, plan.planned_fraction);
  for (const auto& d : plan.drops) {
    std::printf("  %s\n", d.c_str());
  }

  if (opt.csv) {
    // Same assembly the node does: rings (closed loops) then swaths.
    std::printf("x,y,yaw\n");
    auto emit = [&](const std::vector<mower_coverage::Point2D>& seg) {
      for (size_t i = 0; i < seg.size(); ++i) {
        const auto& p = seg[i];
        const auto& q = seg[(i + 1) % seg.size()];
        std::printf("%.6f,%.6f,%.6f\n", p.first, p.second,
                    std::atan2(q.second - p.second, q.first - p.first));
      }
    };
    for (const auto& ring : plan.rings) {
      emit(ring);
    }
    for (const auto& s : plan.swaths) {
      emit({s.first, s.second});
    }
  }
  return plan.swaths.empty() && plan.rings.empty() ? 2 : 0;
}
