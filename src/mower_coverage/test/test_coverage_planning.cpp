// Pure-geometry gtests for the mower_coverage planner core, run against the
// REAL Fields2Cover 2.1.0 library — no robot, no ROS node. The point is to
// catch a broken plan (empty / out-of-bounds / non-serpentine / hole-crossing)
// in CI instead of on the robot.
//
// Expectations are pinned to the empirically verified F2C 2.1.0 runtime
// behavior (probed against ros-humble-fields2cover): generateHeadlands with a
// negative width DILATES the cell, the outermost driven pass lands on the
// recorded line, and BoustrophedonOrder chains consecutive swaths.

#include <gtest/gtest.h>

#include "mower_coverage/coverage_planning.hpp"

#include <cmath>
#include <string>
#include <vector>

using mower_coverage::CoveragePlan;
using mower_coverage::Point2D;

namespace {

constexpr double kOpWidth = 0.5;
constexpr double kHeadlandWidth = 0.20;
constexpr double kMinSwath = 0.15;

std::vector<Point2D> square(double x0, double y0, double side) {
  return {{x0, y0}, {x0 + side, y0}, {x0 + side, y0 + side}, {x0, y0 + side}};
}

f2c::types::Cell field(const std::vector<Point2D>& boundary) {
  return mower_coverage::makeFieldCell(boundary, {});
}

CoveragePlan planSquare(double side, int headland, double angle_deg = -1.0,
                        double op_width = kOpWidth) {
  const double angle_rad = angle_deg >= 0.0 ? angle_deg * M_PI / 180.0 : -1.0;
  return mower_coverage::planBoustrophedon(
      field(square(0.0, 0.0, side)), op_width, kHeadlandWidth, headland,
      /*border_inset=*/0.0, angle_rad, kMinSwath);
}

double swathLength(const std::pair<Point2D, Point2D>& s) {
  return std::hypot(s.second.first - s.first.first,
                    s.second.second - s.first.second);
}

// Minimum distance from any swath endpoint to the field boundary (the axis-
// aligned square [0, side]^2).
double minEndpointDistToBoundary(const CoveragePlan& plan, double side) {
  double best = std::numeric_limits<double>::max();
  for (const auto& s : plan.swaths) {
    for (const auto& p : {s.first, s.second}) {
      best = std::min(best, std::min({std::abs(p.first - 0.0),
                                      std::abs(p.first - side),
                                      std::abs(p.second - 0.0),
                                      std::abs(p.second - side)}));
    }
  }
  return best;
}

// Ray-casting point-in-polygon; on-edge counts as inside (tolerance tol).
bool pointInPolygon(const Point2D& p, const std::vector<Point2D>& poly,
                    double tol = 1e-6) {
  bool inside = false;
  for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
    const auto& a = poly[i];
    const auto& b = poly[j];
    // on-edge?
    const double dx = b.first - a.first;
    const double dy = b.second - a.second;
    const double len2 = dx * dx + dy * dy;
    double t = 0.0;
    if (len2 > 1e-18) {
      t = ((p.first - a.first) * dx + (p.second - a.second) * dy) / len2;
      t = std::max(0.0, std::min(1.0, t));
    }
    const double d = std::hypot(p.first - (a.first + t * dx),
                                p.second - (a.second + t * dy));
    if (d < tol) {
      return true;
    }
    if ((a.second > p.second) != (b.second > p.second) &&
        p.first < (b.first - a.first) * (p.second - a.second) / (b.second - a.second) + a.first) {
      inside = !inside;
    }
  }
  return inside;
}

}  // namespace

// A 2x2 field at 0.5 m spacing with headland disabled: the -op_width/2
// boundary offset dilates the planning cell to [-0.25, 2.25]^2, so F2C places
// 5 swaths at y = 0.0, 0.5, 1.0, 1.5, 2.0. F2C clips them to the DILATED cell
// (length 2.5, ends 0.25 m outside the field); the planner clips them back to
// the recorded field, so each is exactly 2.0 long with ends on x = 0 / x = 2.
// The outermost swath lands exactly ON the recorded boundary.
TEST(CoveragePlanning, SquareCoversField) {
  auto plan = planSquare(2.0, /*headland=*/-1);
  ASSERT_EQ(plan.swaths.size(), 5u);
  for (const auto& s : plan.swaths) {
    EXPECT_NEAR(swathLength(s), 2.0, 1e-6);
    for (const auto& p : {s.first, s.second}) {
      EXPECT_GE(p.first, -1e-9);
      EXPECT_LE(p.first, 2.0 + 1e-9);
    }
  }
  // Swath lines are horizontal (auto angle converged to 0 for a square).
  for (const auto& s : plan.swaths) {
    EXPECT_NEAR(s.first.second, s.second.second, 1e-9);
  }
}

TEST(CoveragePlanning, SquareSwathsAreSerpentine) {
  auto plan = planSquare(2.0, /*headland=*/-1);
  ASSERT_EQ(plan.swaths.size(), 5u);
  for (size_t i = 1; i < plan.swaths.size(); ++i) {
    const auto& prev = plan.swaths[i - 1];
    const auto& cur = plan.swaths[i];
    // Each swath starts where the previous one ended (chained at the same x).
    const double chain = std::hypot(cur.first.first - prev.second.first,
                                    cur.first.second - prev.second.second);
    EXPECT_LE(chain, kOpWidth + 1e-9);
    // Directions alternate.
    const double dx1 = prev.second.first - prev.first.first;
    const double dy1 = prev.second.second - prev.first.second;
    const double dx2 = cur.second.first - cur.first.first;
    const double dy2 = cur.second.second - cur.first.second;
    EXPECT_LT(dx1 * dx2 + dy1 * dy2, 0.0);
  }
}

TEST(CoveragePlanning, FixedAngleIsHonoredAndDeterministic) {
  auto a = planSquare(2.0, /*headland=*/-1, /*angle_deg=*/0.0);
  auto b = planSquare(2.0, /*headland=*/-1, /*angle_deg=*/0.0);
  ASSERT_EQ(a.swaths.size(), b.swaths.size());
  for (const auto& s : a.swaths) {
    EXPECT_NEAR(s.first.second, s.second.second, 1e-9);  // horizontal
  }
  for (size_t i = 0; i < a.swaths.size(); ++i) {
    EXPECT_NEAR(a.swaths[i].first.first, b.swaths[i].first.first, 1e-9);
    EXPECT_NEAR(a.swaths[i].first.second, b.swaths[i].first.second, 1e-9);
    EXPECT_NEAR(a.swaths[i].second.first, b.swaths[i].second.first, 1e-9);
    EXPECT_NEAR(a.swaths[i].second.second, b.swaths[i].second.second, 1e-9);
  }
}

// 3x3 field with a 1x1 centered hole, default headland (auto -> 1 ring).
// The mainland hole is GROWN by the headland offset, so no swath may come
// near the recorded hole.
TEST(CoveragePlanning, HoleIsNotCrossed) {
  std::vector<Point2D> boundary = square(0.0, 0.0, 3.0);
  std::vector<std::vector<Point2D>> holes = {square(1.0, 1.0, 1.0)};
  auto cell = mower_coverage::makeFieldCell(boundary, holes);
  auto plan = mower_coverage::planBoustrophedon(
      cell, kOpWidth, kHeadlandWidth, /*headland_passes=*/0,
      /*border_inset=*/0.0, /*mow_angle_rad=*/-1.0, kMinSwath);
  ASSERT_FALSE(plan.swaths.empty());
  for (const auto& s : plan.swaths) {
    for (int k = 0; k <= 20; ++k) {
      const double t = k / 20.0;
      const Point2D p = {s.first.first + t * (s.second.first - s.first.first),
                         s.first.second + t * (s.second.second - s.first.second)};
      // Recorded hole is [1,2]^2; require a 0.1 m margin.
      EXPECT_FALSE(p.first > 0.9 && p.first < 2.1 && p.second > 0.9 &&
                   p.second < 2.1)
          << "swath sample (" << p.first << ", " << p.second
          << ") is inside the recorded hole";
    }
  }
}

// L-shaped (concave) field: the plan is non-empty and every swath endpoint
// stays inside or on the field — in particular no swath end overshoots into
// the notch or past the outer edges (F2C alone would leave them op_width/2
// outside, since it clips to the dilated planning cell).
TEST(CoveragePlanning, ConcaveFieldIsCovered) {
  std::vector<Point2D> lshape = {{0, 0}, {3, 0}, {3, 1}, {1, 1}, {1, 3}, {0, 3}};
  auto plan = mower_coverage::planBoustrophedon(
      field(lshape), kOpWidth, kHeadlandWidth, /*headland_passes=*/-1,
      /*border_inset=*/0.0, /*mow_angle_rad=*/-1.0, kMinSwath);
  ASSERT_FALSE(plan.swaths.empty());
  EXPECT_GT(plan.planned_fraction, 0.0);
  for (const auto& s : plan.swaths) {
    EXPECT_TRUE(pointInPolygon(s.first, lshape))
        << "start (" << s.first.first << ", " << s.first.second << ") outside L";
    EXPECT_TRUE(pointInPolygon(s.second, lshape))
        << "end (" << s.second.first << ", " << s.second.second << ") outside L";
  }
}

// A field smaller than one swath still plans: the -op_width/2 boundary
// offset dilates the tiny cell, and F2C returns a single swath. (The plan is
// never empty for a non-degenerate field — the dilation grows the cell.)
TEST(CoveragePlanning, SmallFieldStillPlans) {
  auto plan = planSquare(0.2, /*headland=*/-1);
  ASSERT_EQ(plan.swaths.size(), 1u);
  EXPECT_GT(swathLength(plan.swaths.front()), 0.0);
}

// Degenerate (zero-area, collinear) input must not crash or hang. F2C's
// buffer turns the collapsed ring into a thin strip and still returns a
// swath; the planner reports it rather than dropping the field.
TEST(CoveragePlanning, DegenerateRingDoesNotCrash) {
  auto cell = mower_coverage::makeFieldCell({{0, 0}, {1, 0}, {2, 0}}, {});
  auto plan = mower_coverage::planBoustrophedon(
      cell, kOpWidth, kHeadlandWidth, /*headland_passes=*/-1,
      /*border_inset=*/0.0, /*mow_angle_rad=*/-1.0, kMinSwath);
  SUCCEED();  // no crash, no hang
}

// A doubled leading vertex (the common case from OpenMower exports / GUI
// polygons) is dropped by the 1 cm dedup gate: the plan is identical to the
// clean ring's.
TEST(CoveragePlanning, RingDedupDropsDoubledLeadingVertex) {
  std::vector<Point2D> doubled = {{0, 0}, {0, 0}, {2, 0}, {2, 2}, {0, 2}};
  auto clean = planSquare(2.0, /*headland=*/-1);
  auto dirty = mower_coverage::planBoustrophedon(
      field(doubled), kOpWidth, kHeadlandWidth, /*headland_passes=*/-1,
      /*border_inset=*/0.0, /*mow_angle_rad=*/-1.0, kMinSwath);
  ASSERT_EQ(clean.swaths.size(), dirty.swaths.size());
  for (size_t i = 0; i < clean.swaths.size(); ++i) {
    EXPECT_NEAR(clean.swaths[i].first.first, dirty.swaths[i].first.first, 1e-9);
    EXPECT_NEAR(clean.swaths[i].first.second, dirty.swaths[i].first.second, 1e-9);
    EXPECT_NEAR(clean.swaths[i].second.first, dirty.swaths[i].second.first, 1e-9);
    EXPECT_NEAR(clean.swaths[i].second.second, dirty.swaths[i].second.second, 1e-9);
  }
}

// A 1 mm out-and-back spike on an edge is dropped by the 5 mm chord gate.
TEST(CoveragePlanning, RingSpikeIsDropped) {
  std::vector<Point2D> spiked = {{0, 0}, {2, 0}, {2, 2}, {1, 2.001}, {0, 2}};
  auto clean = planSquare(2.0, /*headland=*/-1);
  auto dirty = mower_coverage::planBoustrophedon(
      field(spiked), kOpWidth, kHeadlandWidth, /*headland_passes=*/-1,
      /*border_inset=*/0.0, /*mow_angle_rad=*/-1.0, kMinSwath);
  ASSERT_EQ(clean.swaths.size(), dirty.swaths.size());
  for (size_t i = 0; i < clean.swaths.size(); ++i) {
    EXPECT_NEAR(swathLength(clean.swaths[i]), swathLength(dirty.swaths[i]), 1e-6);
  }
}

// Pins the -op_width/2 compensation: with headland disabled, the outermost
// driven pass lands exactly ON the recorded boundary (distance 0), because
// F2C places it half a swath inside the (dilated) planning cell.
TEST(CoveragePlanning, OutermostSwathLandsOnTheRecordedLine) {
  auto plan = planSquare(2.0, /*headland=*/-1);
  ASSERT_FALSE(plan.swaths.empty());
  EXPECT_NEAR(minEndpointDistToBoundary(plan, 2.0), 0.0, 1e-9);
}

// "Mow to the edge" (headland -1) reaches strictly closer to the boundary
// than the default rings-on plan, whose mainland is eroded by one ring.
TEST(CoveragePlanning, HeadlandDisabledReachesCloserToTheBoundary) {
  auto off = planSquare(3.0, /*headland=*/-1);
  auto on = planSquare(3.0, /*headland=*/0);
  ASSERT_FALSE(off.swaths.empty());
  ASSERT_FALSE(on.swaths.empty());
  EXPECT_LT(minEndpointDistToBoundary(off, 3.0),
            minEndpointDistToBoundary(on, 3.0));
}

// Forced headland passes produce one closed ring per pass, outermost first,
// with the ring on the recorded boundary; mainland swaths sit inside it.
TEST(CoveragePlanning, HeadlandRingsAreProduced) {
  auto plan = planSquare(2.0, /*headland=*/1);
  ASSERT_EQ(plan.rings.size(), 1u);
  const auto& ring = plan.rings.front();
  ASSERT_GE(ring.size(), 4u);
  EXPECT_NEAR(ring.front().first, ring.back().first, 1e-9);  // closed loop
  EXPECT_NEAR(ring.front().second, ring.back().second, 1e-9);
  // Ring lies on the recorded [0,2]^2 boundary.
  for (const auto& p : ring) {
    const double d = std::min({std::abs(p.first - 0.0), std::abs(p.first - 2.0),
                               std::abs(p.second - 0.0), std::abs(p.second - 2.0)});
    EXPECT_NEAR(d, 0.0, 1e-6);
  }
  // Mainland swaths are inside the ring.
  for (const auto& s : plan.swaths) {
    EXPECT_GT(s.first.first, 0.0);
    EXPECT_LT(s.first.first, 2.0);
  }
}

TEST(CoveragePlanning, DiagnosticsReportPlannedFraction) {
  auto plan = planSquare(2.0, /*headland=*/-1);
  EXPECT_GT(plan.planned_fraction, 0.0);
  EXPECT_LE(plan.planned_fraction, 1.0);
}

// Auto-angle search is O(area); above the 400 m^2 gate the planner uses the
// boundary's longest-edge angle. For an axis-aligned square that is 0.
TEST(CoveragePlanning, LargeFieldUsesLongestEdgeAngleFallback) {
  auto plan = planSquare(25.0, /*headland=*/-1);
  ASSERT_FALSE(plan.swaths.empty());
  EXPECT_NEAR(std::sin(plan.swath_angle_rad), 0.0, 1e-9);
}

// A negative longest-edge angle is used as-is (never falling back to the
// exhaustive auto search): a rectangle rotated -20 deg plans at -20 deg
// (mod pi, since swath lines are undirected). 40x20 = 800 m^2 so the field is
// above the 400 m^2 auto-search gate and actually takes the longest-edge path.
TEST(CoveragePlanning, NegativeLongestEdgeNeverFallsBackToAutoSearch) {
  const double ang = -20.0 * M_PI / 180.0;
  const double c = std::cos(ang);
  const double s = std::sin(ang);
  auto rot = [&](double x, double y) {
    return Point2D{x * c - y * s, x * s + y * c};
  };
  // 40x20 rectangle, first long edge at -20 deg.
  std::vector<Point2D> boundary = {rot(-20, 10), rot(20, 10), rot(20, -10),
                                  rot(-20, -10)};
  auto plan = mower_coverage::planBoustrophedon(
      field(boundary), kOpWidth, kHeadlandWidth, /*headland_passes=*/-1,
      /*border_inset=*/0.0, /*mow_angle_rad=*/-1.0, kMinSwath);
  ASSERT_FALSE(plan.swaths.empty());
  EXPECT_NEAR(std::sin(plan.swath_angle_rad - ang), 0.0, 1e-9);
}
