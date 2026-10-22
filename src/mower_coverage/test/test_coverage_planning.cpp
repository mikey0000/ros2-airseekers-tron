// Pure-geometry gtests for the mower_coverage planner core, run against the
// REAL Fields2Cover library (v3 by default, apt 2.1.0 with F2C_V3=OFF) — no
// robot, no ROS node. The point is to
// catch a broken plan (empty / out-of-bounds / non-serpentine / hole-crossing)
// in CI instead of on the robot.
//
// Expectations are pinned to the empirically verified F2C 2.1.0 runtime
// behavior (probed against ros-jazzy-fields2cover): generateHeadlands with a
// negative width DILATES the cell, the outermost driven pass lands on the
// recorded line, and BoustrophedonOrder chains consecutive swaths.

#include <gtest/gtest.h>

#include "mower_coverage/coverage_planning.hpp"

#include <algorithm>
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

// ---------------------------------------------------------------------------
// Drive order: transits, hole headlands, ring -> swath hand-over.
// ---------------------------------------------------------------------------

namespace {

// Node / service defaults (mower_coverage_node): 0.18 m swaths, 0.20 m
// headland band -> auto = 2 rings, 0.15 m sliver drop.
constexpr double kNodeOpWidth = 0.18;
constexpr double kNodeHeadlandWidth = 0.20;
// The bridge / mission layer's join threshold (kSegmentTransitGapM).
constexpr double kTransitGap = 0.6;

const std::vector<Point2D> kLShape = {{0, 0}, {10, 0}, {10, 3}, {4, 3}, {4, 8}, {0, 8}};
const std::vector<Point2D> kRect10x6 = {{0, 0}, {10, 0}, {10, 6}, {0, 6}};
const std::vector<Point2D> kHole2x2 = {{4, 2}, {6, 2}, {6, 4}, {4, 4}};

CoveragePlan planNode(const std::vector<Point2D>& boundary,
                      const std::vector<std::vector<Point2D>>& holes,
                      double angle_deg, int headland_passes = 0,
                      double border_inset = 0.0) {
  const double angle_rad = angle_deg >= 0.0 ? angle_deg * M_PI / 180.0 : -1.0;
  return mower_coverage::planBoustrophedon(
      mower_coverage::makeFieldCell(boundary, holes), kNodeOpWidth,
      kNodeHeadlandWidth, headland_passes, border_inset, angle_rad, kMinSwath);
}

// The path pieces in the node's drive order: rings (closed loops), then
// swaths (two poses each).
std::vector<std::vector<Point2D>> driveSegments(const CoveragePlan& plan) {
  std::vector<std::vector<Point2D>> segs;
  if (!plan.swaths_first) {
    segs.assign(plan.rings.begin(), plan.rings.end());
  }
  for (const auto& s : plan.swaths) {
    segs.push_back({s.first, s.second});
  }
  if (plan.swaths_first) {
    segs.insert(segs.end(), plan.rings.begin(), plan.rings.end());
  }
  return segs;
}

double ptDist(const Point2D& a, const Point2D& b) {
  return std::hypot(a.first - b.first, a.second - b.second);
}

// Transits as the bridge counts them (splitter.join_subpaths): a gap above
// 0.6 m from one segment's end to the next one's start.
int countTransits(const CoveragePlan& plan, double* total_m = nullptr) {
  const auto segs = driveSegments(plan);
  int n = 0;
  double total = 0.0;
  for (size_t i = 1; i < segs.size(); ++i) {
    const double g = ptDist(segs[i - 1].back(), segs[i].front());
    if (g > kTransitGap) {
      ++n;
      total += g;
    }
  }
  if (total_m != nullptr) {
    *total_m = total;
  }
  return n;
}

double pointSegDist(const Point2D& p, const Point2D& a, const Point2D& b) {
  const double dx = b.first - a.first;
  const double dy = b.second - a.second;
  const double len2 = dx * dx + dy * dy;
  double t = 0.0;
  if (len2 > 1e-18) {
    t = std::max(0.0, std::min(1.0, ((p.first - a.first) * dx +
                                     (p.second - a.second) * dy) / len2));
  }
  return std::hypot(p.first - (a.first + t * dx), p.second - (a.second + t * dy));
}

// Distance from p to the boundary of an open polygon vertex list.
double distToPolygonEdge(const Point2D& p, const std::vector<Point2D>& poly) {
  double best = std::numeric_limits<double>::max();
  for (size_t i = 0; i < poly.size(); ++i) {
    best = std::min(best, pointSegDist(p, poly[i], poly[(i + 1) % poly.size()]));
  }
  return best;
}

// Signed: > 0 outside the polygon, < 0 strictly inside.
double signedDistToPolygon(const Point2D& p, const std::vector<Point2D>& poly) {
  const double d = distToPolygonEdge(p, poly);
  return (d > 1e-9 && pointInPolygon(p, poly)) ? -d : d;
}

// Rings that run around `hole` (every vertex within `reach` of it).
std::vector<std::vector<Point2D>> ringsAroundHole(const CoveragePlan& plan,
                                                  const std::vector<Point2D>& hole,
                                                  double reach) {
  std::vector<std::vector<Point2D>> out;
  for (const auto& ring : plan.rings) {
    bool near = true;
    for (const auto& p : ring) {
      near = near && distToPolygonEdge(p, hole) < reach;
    }
    if (near) {
      out.push_back(ring);
    }
  }
  return out;
}

}  // namespace

// F2C's BoustrophedonOrder alternates between the two pieces of every sweep
// line that crosses the notch (old plan: 40 transits, 263 m on this field).
// Cell ordering mows each arm of the L in one serpentine.
TEST(CoveragePlanning, LShapeAt135DegNeedsAtMostThreeTransits) {
  auto plan = planNode(kLShape, {}, 135.0);
  ASSERT_EQ(plan.rings.size(), 2u);
  ASSERT_FALSE(plan.swaths.empty());
  double total = 0.0;
  const int transits = countTransits(plan, &total);
  EXPECT_LE(transits, 3) << "transit length " << total << " m";
  for (const auto& s : plan.swaths) {
    EXPECT_TRUE(pointInPolygon(s.first, kLShape));
    EXPECT_TRUE(pointInPolygon(s.second, kLShape));
  }
}

// Same for a hole (old plan: 29 transits, 175 m).
TEST(CoveragePlanning, RectangleWithHoleNeedsAtMostThreeTransits) {
  auto plan = planNode(kRect10x6, {kHole2x2}, -1.0);
  ASSERT_FALSE(plan.swaths.empty());
  double total = 0.0;
  const int transits = countTransits(plan, &total);
  EXPECT_LE(transits, 3) << "transit length " << total << " m";
}

// Re-ordering never changes WHAT is mown: the plan still has one swath per
// clipped piece and every swath stays out of the hole and its ring band.
// With 2 hole rings (at op_width/2 and 3/2 op_width outside the hole), the
// swaths keep 2 * op_width clear of the recorded hole.
TEST(CoveragePlanning, HoleRingBandIsAKeepOutForSwaths) {
  for (double inset : {0.0, 0.1}) {
    auto plan = planNode(kRect10x6, {kHole2x2}, -1.0, /*headland_passes=*/2, inset);
    ASSERT_FALSE(plan.swaths.empty());
    const double clear = inset + 2.0 * kNodeOpWidth;
    for (const auto& s : plan.swaths) {
      for (int k = 0; k <= 50; ++k) {
        const double t = k / 50.0;
        const Point2D p = {s.first.first + t * (s.second.first - s.first.first),
                           s.first.second + t * (s.second.second - s.first.second)};
        EXPECT_GE(signedDistToPolygon(p, kHole2x2), clear - 1e-6)
            << "inset " << inset << " swath sample (" << p.first << ", "
            << p.second << ")";
      }
    }
  }
}

// Holes get as many headland rings as the outer boundary: ring k (0-based)
// at border_inset + op_width/2 + k * op_width OUTSIDE the recorded hole (the
// blade edge touches the obstacle but never overhangs it), enclosing it,
// driven hole edge first.
TEST(CoveragePlanning, HoleGetsHeadlandRingsOutsideTheHole) {
  for (double inset : {0.0, 0.1}) {
    auto plan = planNode(kRect10x6, {kHole2x2}, -1.0, /*headland_passes=*/2, inset);
    const auto hole_rings = ringsAroundHole(plan, kHole2x2, 1.0);
    ASSERT_EQ(hole_rings.size(), 2u) << "inset " << inset;
    ASSERT_EQ(plan.rings.size(), 4u);  // 2 outer + 2 around the hole
    for (size_t k = 0; k < hole_rings.size(); ++k) {
      const double expected = inset + 0.5 * kNodeOpWidth + k * kNodeOpWidth;
      const auto& ring = hole_rings[k];
      ASSERT_GE(ring.size(), 5u);
      EXPECT_NEAR(ptDist(ring.front(), ring.back()), 0.0, 1e-9);  // closed
      // Sample along the edges: buffered corners sit further out (mitre),
      // the straight runs at exactly the offset.
      double min_d = std::numeric_limits<double>::max();
      for (size_t i = 0; i + 1 < ring.size(); ++i) {
        for (int j = 0; j < 20; ++j) {
          const double t = j / 20.0;
          const Point2D p = {ring[i].first + t * (ring[i + 1].first - ring[i].first),
                             ring[i].second + t * (ring[i + 1].second - ring[i].second)};
          const double d = signedDistToPolygon(p, kHole2x2);
          EXPECT_GE(d, expected - 1e-3) << "ring " << k << " inset " << inset;
          min_d = std::min(min_d, d);
        }
      }
      EXPECT_NEAR(min_d, expected, 1e-3) << "ring " << k << " inset " << inset;
      // The ring encloses the hole: every hole vertex is inside it.
      for (const auto& h : kHole2x2) {
        EXPECT_TRUE(pointInPolygon(h, ring, 0.0));
      }
    }
  }
}

// F2C restarts every ring at the same polygon corner, so the last outer ring
// used to end 4-7 m from the first swath (one transit on a plain
// rectangle). The ring starts are now chained back from the first swath.
TEST(CoveragePlanning, LastRingEndsNextToTheFirstSwath) {
  auto plan = planNode(kRect10x6, {}, -1.0);
  ASSERT_EQ(plan.rings.size(), 2u);
  ASSERT_FALSE(plan.swaths.empty());
  EXPECT_LT(ptDist(plan.rings.back().back(), plan.swaths.front().first), 1.0);
  EXPECT_EQ(countTransits(plan), 0);
  // Concentric rings hand over with one op_width sideways step.
  EXPECT_NEAR(ptDist(plan.rings[0].back(), plan.rings[1].front()), kNodeOpWidth, 1e-6);

  auto l = planNode(kLShape, {}, 135.0);
  ASSERT_FALSE(l.rings.empty());
  ASSERT_FALSE(l.swaths.empty());
  EXPECT_LT(ptDist(l.rings.back().back(), l.swaths.front().first), 1.0);
}

// The bridge's splitter relies on the node's pose layout: every ring is a
// closed loop whose start pose appears nowhere else in the ring (it ends a
// ring at the FIRST later pose within 1 mm of its start), swaths are two
// distinct poses, and no segment starts exactly where the previous one ended
// (the node drops a pose within 1e-6 m of the previous one).
TEST(CoveragePlanning, PathLayoutStaysBridgeCompatible) {
  for (const auto& plan :
       {planNode(kLShape, {}, 135.0), planNode(kRect10x6, {kHole2x2}, -1.0),
        planNode(kRect10x6, {}, -1.0), planSquare(3.0, /*headland=*/1)}) {
    for (const auto& ring : plan.rings) {
      ASSERT_GE(ring.size(), 4u);
      EXPECT_LT(ptDist(ring.front(), ring.back()), 1e-9);
      for (size_t i = 1; i + 1 < ring.size(); ++i) {
        EXPECT_GT(ptDist(ring[i], ring.front()), 1e-3);
      }
    }
    for (const auto& s : plan.swaths) {
      EXPECT_GT(ptDist(s.first, s.second), 1e-6);
    }
    const auto segs = driveSegments(plan);
    for (size_t i = 1; i < segs.size(); ++i) {
      EXPECT_GT(ptDist(segs[i - 1].back(), segs[i].front()), 1e-6);
    }
  }
}

// Many cells (more than the exact ordering handles) fall back to greedy
// ordering and still emit every swath of a serpentine: consecutive swaths in
// one cell alternate direction.
TEST(CoveragePlanning, ManyHolesStillPlanAndStayOutOfHoles) {
  std::vector<std::vector<Point2D>> holes;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 2; ++j) {
      holes.push_back(square(2.0 + 4.0 * i, 2.0 + 5.0 * j, 1.0));
    }
  }
  auto plan = planNode({{0, 0}, {18, 0}, {18, 10}, {0, 10}}, holes, 0.0);
  ASSERT_FALSE(plan.swaths.empty());
  EXPECT_EQ(plan.rings.size(), 2u + 2u * holes.size());
  for (const auto& s : plan.swaths) {
    for (const auto& h : holes) {
      const Point2D mid = {(s.first.first + s.second.first) / 2.0,
                           (s.first.second + s.second.second) / 2.0};
      EXPECT_GT(signedDistToPolygon(s.first, h), 0.0);
      EXPECT_GT(signedDistToPolygon(s.second, h), 0.0);
      EXPECT_GT(signedDistToPolygon(mid, h), 0.0);
    }
  }
  double total = 0.0;
  const int transits = countTransits(plan, &total);
  // Report, not a hard bound beyond "far fewer than one per hole crossing".
  RecordProperty("transits", transits);
  EXPECT_LT(transits, 30) << total << " m";
}

// ---------------------------------------------------------------------------
// Path modes (per-area settings): spiral, contour_only, edge_first = false.
// ---------------------------------------------------------------------------

namespace {

CoveragePlan planMode(const std::vector<Point2D>& boundary,
                      const std::vector<std::vector<Point2D>>& holes,
                      mower_coverage::PathMode mode, int headland_passes = 0,
                      bool edge_first = true, double angle_deg = -1.0) {
  const double angle_rad = angle_deg >= 0.0 ? angle_deg * M_PI / 180.0 : -1.0;
  return mower_coverage::planCoverage(
      mower_coverage::makeFieldCell(boundary, holes), kNodeOpWidth,
      kNodeHeadlandWidth, headland_passes, /*border_inset=*/0.0, angle_rad,
      kMinSwath, mode, edge_first);
}

// Fraction of the field (boundary minus holes, sampled on a `step` grid)
// within `reach` of the driven segments (each segment as a polyline; the
// connectors between segments are not counted).
double coveredFraction(const CoveragePlan& plan, const std::vector<Point2D>& boundary,
                       const std::vector<std::vector<Point2D>>& holes, double reach,
                       double step = 0.05) {
  double x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9;
  for (const auto& p : boundary) {
    x0 = std::min(x0, p.first);
    y0 = std::min(y0, p.second);
    x1 = std::max(x1, p.first);
    y1 = std::max(y1, p.second);
  }
  std::vector<std::pair<Point2D, Point2D>> edges;
  for (const auto& seg : driveSegments(plan)) {
    for (size_t i = 0; i + 1 < seg.size(); ++i) {
      edges.push_back({seg[i], seg[i + 1]});
    }
  }
  size_t total = 0;
  size_t covered = 0;
  for (double x = x0 + step / 2; x < x1; x += step) {
    for (double y = y0 + step / 2; y < y1; y += step) {
      const Point2D p{x, y};
      if (!pointInPolygon(p, boundary, 0.0)) {
        continue;
      }
      bool in_hole = false;
      for (const auto& h : holes) {
        in_hole = in_hole || pointInPolygon(p, h, 0.0);
      }
      if (in_hole) {
        continue;
      }
      ++total;
      for (const auto& e : edges) {
        if (pointSegDist(p, e.first, e.second) <= reach) {
          ++covered;
          break;
        }
      }
    }
  }
  return total > 0 ? static_cast<double>(covered) / static_cast<double>(total) : 0.0;
}

// Largest distance any driven pose (ring vertex / swath end) or edge sample
// lies OUTSIDE the boundary or INSIDE a hole band (signed, > 0 = violation).
double worstOvershoot(const CoveragePlan& plan, const std::vector<Point2D>& boundary,
                      const std::vector<std::vector<Point2D>>& holes, double hole_clear) {
  double worst = -1e9;
  for (const auto& seg : driveSegments(plan)) {
    for (size_t i = 0; i + 1 < seg.size(); ++i) {
      for (int k = 0; k <= 10; ++k) {
        const double t = k / 10.0;
        const Point2D p{seg[i].first + t * (seg[i + 1].first - seg[i].first),
                        seg[i].second + t * (seg[i + 1].second - seg[i].second)};
        worst = std::max(worst, signedDistToPolygon(p, boundary));
        for (const auto& h : holes) {
          worst = std::max(worst, hole_clear - signedDistToPolygon(p, h));
        }
      }
    }
  }
  return worst;
}

}  // namespace

TEST(CoveragePlanning, ParsePathMode) {
  using mower_coverage::PathMode;
  PathMode m = PathMode::kSpiral;
  EXPECT_TRUE(mower_coverage::parsePathMode("", &m));
  EXPECT_EQ(m, PathMode::kZigzag);
  for (const char* z : {"zigzag", "cross", "alternate"}) {
    EXPECT_TRUE(mower_coverage::parsePathMode(z, &m));
    EXPECT_EQ(m, PathMode::kZigzag);
  }
  EXPECT_TRUE(mower_coverage::parsePathMode("spiral", &m));
  EXPECT_EQ(m, PathMode::kSpiral);
  EXPECT_TRUE(mower_coverage::parsePathMode("contour_only", &m));
  EXPECT_EQ(m, PathMode::kContourOnly);
  EXPECT_FALSE(mower_coverage::parsePathMode("diagonal", &m));
}

// planCoverage(kZigzag, edge_first) is exactly planBoustrophedon.
TEST(CoveragePlanning, ZigzagModeMatchesBoustrophedon) {
  const auto a = planNode(kLShape, {}, 135.0);
  const auto b = planMode(kLShape, {}, mower_coverage::PathMode::kZigzag, 0, true, 135.0);
  ASSERT_EQ(a.rings.size(), b.rings.size());
  ASSERT_EQ(a.swaths.size(), b.swaths.size());
  for (size_t i = 0; i < a.swaths.size(); ++i) {
    EXPECT_NEAR(ptDist(a.swaths[i].first, b.swaths[i].first), 0.0, 1e-9);
    EXPECT_NEAR(ptDist(a.swaths[i].second, b.swaths[i].second), 0.0, 1e-9);
  }
  EXPECT_FALSE(b.swaths_first);
}

// Spiral on a rectangle: rings only, outermost on the recorded line, each
// next ring one op_width inside, chained without a transit, covering the
// whole field including the centre strip, never leaving the field.
TEST(CoveragePlanning, SpiralRectangleCoversFieldWithoutTransits) {
  const auto plan = planMode(kRect10x6, {}, mower_coverage::PathMode::kSpiral);
  ASSERT_TRUE(plan.swaths.empty());
  // 6 m wide / 0.18 m = 16.7 rings to the centre line.
  EXPECT_GE(plan.rings.size(), 16u);
  EXPECT_LE(plan.rings.size(), 18u);
  // Outside-in: ring k is (about) k * op_width + 0 inside the boundary.
  for (size_t k = 0; k + 1 < plan.rings.size(); ++k) {
    double min_d = 1e9;
    for (const auto& p : plan.rings[k]) {
      min_d = std::min(min_d, distToPolygonEdge(p, kRect10x6));
    }
    EXPECT_NEAR(min_d, k * kNodeOpWidth, 1e-3) << "ring " << k;
  }
  EXPECT_EQ(countTransits(plan), 0);
  EXPECT_LT(worstOvershoot(plan, kRect10x6, {}, 0.0), 1e-6);
  EXPECT_GE(coveredFraction(plan, kRect10x6, {}, 0.5 * kNodeOpWidth + 1e-3), 0.995);
  EXPECT_GE(plan.planned_fraction, 0.95);
}

// Spiral around a hole: no pass enters the hole or its half-swath band, the
// field around it is still covered, and the drive stays mostly continuous.
TEST(CoveragePlanning, SpiralRespectsHoles) {
  const auto plan = planMode(kRect10x6, {kHole2x2}, mower_coverage::PathMode::kSpiral);
  ASSERT_TRUE(plan.swaths.empty());
  ASSERT_FALSE(plan.rings.empty());
  EXPECT_LT(worstOvershoot(plan, kRect10x6, {kHole2x2}, 0.5 * kNodeOpWidth), 1e-3);
  EXPECT_GE(coveredFraction(plan, kRect10x6, {kHole2x2}, 0.5 * kNodeOpWidth + 1e-3), 0.97);
  double total = 0.0;
  const int transits = countTransits(plan, &total);
  RecordProperty("transits", transits);
  EXPECT_LE(transits, 6) << total << " m";
}

TEST(CoveragePlanning, SpiralLShapeCoversBothArms) {
  const auto plan = planMode(kLShape, {}, mower_coverage::PathMode::kSpiral);
  ASSERT_TRUE(plan.swaths.empty());
  EXPECT_LT(worstOvershoot(plan, kLShape, {}, 0.0), 1e-6);
  EXPECT_GE(coveredFraction(plan, kLShape, {}, 0.5 * kNodeOpWidth + 1e-3), 0.98);
  EXPECT_LE(countTransits(plan), 3);
}

// A field narrower than one swath still gets one ring.
TEST(CoveragePlanning, SpiralNarrowFieldStillPlans) {
  const std::vector<Point2D> strip = {{0, 0}, {5, 0}, {5, 0.15}, {0, 0.15}};
  const auto plan = planMode(strip, {}, mower_coverage::PathMode::kSpiral);
  ASSERT_FALSE(plan.rings.empty());
  EXPECT_LT(worstOvershoot(plan, strip, {}, 0.0), 1e-6);
}

// contour_only: exactly headland_passes rings (at least one), no swaths,
// the vendor's "edge only" mode.
TEST(CoveragePlanning, ContourOnlyDrivesOnlyThePerimeterLaps) {
  for (int laps : {1, 3}) {
    const auto plan =
        planMode(kRect10x6, {}, mower_coverage::PathMode::kContourOnly, laps);
    EXPECT_TRUE(plan.swaths.empty());
    ASSERT_EQ(plan.rings.size(), static_cast<size_t>(laps));
    EXPECT_EQ(countTransits(plan), 0);
    EXPECT_LT(worstOvershoot(plan, kRect10x6, {}, 0.0), 1e-6);
    // The outer lap runs ON the line, so the covered band along the edge is
    // (laps - 1/2) * op_width wide; nothing inside it is mown.
    const double frac = coveredFraction(plan, kRect10x6, {}, 0.5 * kNodeOpWidth + 1e-3);
    const double b = (laps - 0.5) * kNodeOpWidth;
    const double band = 1.0 - (10.0 - 2 * b) * (6.0 - 2 * b) / 60.0;
    EXPECT_NEAR(frac, band, 0.02) << laps << " laps";
  }
  // headland_passes -1 (no rings) still gives the one contour lap.
  const auto one = planMode(kRect10x6, {}, mower_coverage::PathMode::kContourOnly, -1);
  EXPECT_EQ(one.rings.size(), 1u);
  // Holes get their laps too.
  const auto holed =
      planMode(kRect10x6, {kHole2x2}, mower_coverage::PathMode::kContourOnly, 2);
  EXPECT_EQ(holed.rings.size(), 4u);
  EXPECT_LT(worstOvershoot(holed, kRect10x6, {kHole2x2}, 0.5 * kNodeOpWidth), 1e-3);
}

// edge_first = false: the same swaths and rings, swaths driven first, then
// the rings innermost first, each chained from the previous end — no extra
// transit compared to edge first.
TEST(CoveragePlanning, EdgeLastDrivesSwathsThenRingsInsideOut) {
  const auto first = planMode(kRect10x6, {}, mower_coverage::PathMode::kZigzag, 2, true);
  const auto last = planMode(kRect10x6, {}, mower_coverage::PathMode::kZigzag, 2, false);
  EXPECT_FALSE(first.swaths_first);
  EXPECT_TRUE(last.swaths_first);
  ASSERT_EQ(last.rings.size(), 2u);
  EXPECT_EQ(last.swaths.size(), first.swaths.size());
  // Innermost ring first.
  auto edge_d = [&](const std::vector<Point2D>& ring) {
    double d = 1e9;
    for (const auto& p : ring) {
      d = std::min(d, distToPolygonEdge(p, kRect10x6));
    }
    return d;
  };
  EXPECT_NEAR(edge_d(last.rings[0]), kNodeOpWidth, 1e-3);
  EXPECT_NEAR(edge_d(last.rings[1]), 0.0, 1e-3);
  // Hand-over: last swath -> first ring and ring -> ring are short steps.
  EXPECT_LT(ptDist(last.swaths.back().second, last.rings.front().front()), kTransitGap);
  EXPECT_NEAR(ptDist(last.rings[0].back(), last.rings[1].front()), kNodeOpWidth, 1e-6);
  EXPECT_EQ(countTransits(last), countTransits(first));
  EXPECT_LT(worstOvershoot(last, kRect10x6, {}, 0.0), 1e-6);
  EXPECT_NEAR(coveredFraction(last, kRect10x6, {}, 0.5 * kNodeOpWidth + 1e-3),
              coveredFraction(first, kRect10x6, {}, 0.5 * kNodeOpWidth + 1e-3), 1e-9);

  // With a hole: still the same pieces, swaths first, hole rings included.
  const auto holed =
      planMode(kRect10x6, {kHole2x2}, mower_coverage::PathMode::kZigzag, 2, false);
  EXPECT_TRUE(holed.swaths_first);
  EXPECT_EQ(holed.rings.size(), 4u);
  EXPECT_LE(countTransits(holed), 4);
}

// Every mode keeps the bridge's pose-layout contract (closed rings whose
// start appears nowhere else, distinct segment joints).
TEST(CoveragePlanning, PathModesStayBridgeCompatible) {
  using mower_coverage::PathMode;
  for (const auto& plan :
       {planMode(kRect10x6, {}, PathMode::kSpiral),
        planMode(kRect10x6, {kHole2x2}, PathMode::kSpiral),
        planMode(kLShape, {}, PathMode::kSpiral),
        planMode(kRect10x6, {kHole2x2}, PathMode::kContourOnly, 2),
        planMode(kRect10x6, {kHole2x2}, PathMode::kZigzag, 2, false),
        planMode(kLShape, {}, PathMode::kZigzag, 2, false, 135.0)}) {
    for (const auto& ring : plan.rings) {
      ASSERT_GE(ring.size(), 4u);
      EXPECT_LT(ptDist(ring.front(), ring.back()), 1e-9);
      for (size_t i = 1; i + 1 < ring.size(); ++i) {
        EXPECT_GT(ptDist(ring[i], ring.front()), 1e-3);
      }
    }
    const auto segs = driveSegments(plan);
    for (size_t i = 1; i < segs.size(); ++i) {
      EXPECT_GT(ptDist(segs[i - 1].back(), segs[i].front()), 1e-6);
    }
  }
}

// The node plans with border_inset = op_width / 2 + boundary_inset_m (default
// 0.15): every OUTER ring point and every swath end must then sit at least
// that far inside the recorded boundary, so tracking error does not put the
// chassis outside the line.
TEST(CoveragePlanning, NodeBoundaryInsetKeepsOuterRingInside) {
  const double op = kNodeOpWidth;
  const double inset = op / 2.0 + 0.15;
  const auto plan = mower_coverage::planCoverage(
      mower_coverage::makeFieldCell(kRect10x6, {}), op, kNodeHeadlandWidth,
      /*headland_passes=*/2, inset, /*mow_angle_rad=*/-1.0, kMinSwath,
      mower_coverage::PathMode::kZigzag, /*edge_first=*/true);
  ASSERT_FALSE(plan.rings.empty());
  ASSERT_FALSE(plan.swaths.empty());
  auto edgeDist = [](const Point2D& p) {
    return std::min({p.first, 10.0 - p.first, p.second, 6.0 - p.second});
  };
  double min_ring = 1e9;
  for (const auto& p : plan.rings.front()) min_ring = std::min(min_ring, edgeDist(p));
  EXPECT_NEAR(min_ring, inset, 0.01);
  for (const auto& r : plan.rings)
    for (const auto& p : r) EXPECT_GE(edgeDist(p), inset - 0.01);
  for (const auto& s : plan.swaths) {
    EXPECT_GE(edgeDist(s.first), inset - 0.01);
    EXPECT_GE(edgeDist(s.second), inset - 0.01);
  }
}

// min_turn_radius 0 (diff-drive pivot) inserts nothing; > 0 on F2C v3 yields
// a Dubins connector whose points stay near the two swath ends.
TEST(CoveragePlanning, TurnPlannerPivotAndDubins) {
  EXPECT_TRUE(mower_coverage::planTurn({0, 0}, 0.0, {0, 0.5}, M_PI, 0.18, 0.0).empty());
  const auto turn =
      mower_coverage::planTurn({0, 0}, 0.0, {0, 0.5}, M_PI, 0.18, 0.25);
  if (!mower_coverage::builtWithF2CV3()) {
    EXPECT_TRUE(turn.empty());
    return;
  }
  ASSERT_GE(turn.size(), 3u);
  for (const auto& p : turn) {
    EXPECT_LT(std::hypot(p.first, p.second - 0.25), 1.0);
  }
}

// ---------------------------------------------------------------------------
// Route order (F2C v3 SnakeOrder / SpiralOrder) and swath-to-swath turns.
// ---------------------------------------------------------------------------

namespace {

mower_coverage::CoveragePlan planRoute(const mower_coverage::RouteOptions& route,
                                       double side_x = 6.0, double side_y = 3.0,
                                       int rings = 3) {
  std::vector<Point2D> rect = {{0, 0}, {side_x, 0}, {side_x, side_y}, {0, side_y}};
  return mower_coverage::planCoverage(field(rect), 0.18, 0.2, rings, /*border_inset=*/0.24,
                                      /*mow_angle=*/0.0, 0.15,
                                      mower_coverage::PathMode::kZigzag, true, route);
}

double turnLen(const Point2D& a, const mower_coverage::SwathTurn& t, const Point2D& b) {
  double len = 0.0;
  Point2D prev = a;
  for (const auto& tp : t.poses) {
    len += std::hypot(tp.p.first - prev.first, tp.p.second - prev.second);
    prev = tp.p;
  }
  return len + std::hypot(b.first - prev.first, b.second - prev.second);
}

}  // namespace

TEST(CoverageRoute, ParseRouteOrderAndTurnType) {
  mower_coverage::RouteOrder o;
  EXPECT_TRUE(mower_coverage::parseRouteOrder("", &o));
  EXPECT_EQ(o, mower_coverage::RouteOrder::kBoustrophedon);
  EXPECT_TRUE(mower_coverage::parseRouteOrder("snake", &o));
  EXPECT_EQ(o, mower_coverage::RouteOrder::kSnake);
  EXPECT_TRUE(mower_coverage::parseRouteOrder("spiral", &o));
  EXPECT_FALSE(mower_coverage::parseRouteOrder("zigzag", &o));
  mower_coverage::TurnType t;
  EXPECT_TRUE(mower_coverage::parseTurnType("", &t));
  EXPECT_EQ(t, mower_coverage::TurnType::kAuto);
  EXPECT_TRUE(mower_coverage::parseTurnType("reverse", &t));
  EXPECT_EQ(t, mower_coverage::TurnType::kReverse);
  EXPECT_FALSE(mower_coverage::parseTurnType("omega", &t));
}

TEST(CoverageRoute, SnakePermutationSkipsRows) {
  if (!mower_coverage::builtWithF2CV3()) GTEST_SKIP() << "F2C v3 only";
  using R = mower_coverage::RouteOrder;
  EXPECT_EQ(mower_coverage::routeOrderPermutation(6, R::kSnake, 0),
            (std::vector<size_t>{0, 2, 4, 5, 3, 1}));
  EXPECT_EQ(mower_coverage::routeOrderPermutation(7, R::kSnake, 0),
            (std::vector<size_t>{0, 2, 4, 6, 5, 3, 1}));
  EXPECT_EQ(mower_coverage::routeOrderPermutation(2, R::kSnake, 0),
            (std::vector<size_t>{0, 1}));
  EXPECT_EQ(mower_coverage::routeOrderPermutation(4, R::kBoustrophedon, 0),
            (std::vector<size_t>{0, 1, 2, 3}));
  // Spiral: every index exactly once.
  auto sp = mower_coverage::routeOrderPermutation(12, R::kSpiral, 6);
  ASSERT_EQ(sp.size(), 12u);
  std::vector<size_t> sorted = sp;
  std::sort(sorted.begin(), sorted.end());
  for (size_t i = 0; i < 12; ++i) EXPECT_EQ(sorted[i], i);
  EXPECT_NE(sp, mower_coverage::routeOrderPermutation(12, R::kBoustrophedon, 6));
}

// Snake plan on a rectangle: consecutive swaths are two op_widths apart
// (except the one hand-over in the middle), antiparallel, and every swath of
// the boustrophedon plan is still driven.
TEST(CoverageRoute, SnakePlanVisitsEveryOtherSwath) {
  if (!mower_coverage::builtWithF2CV3()) GTEST_SKIP() << "F2C v3 only";
  mower_coverage::RouteOptions b, s;
  s.order = mower_coverage::RouteOrder::kSnake;
  const auto pb = planRoute(b);
  const auto ps = planRoute(s);
  ASSERT_EQ(pb.swaths.size(), ps.swaths.size());
  ASSERT_GE(ps.swaths.size(), 6u);
  size_t wide = 0;
  for (size_t i = 1; i < ps.swaths.size(); ++i) {
    const double lateral = std::abs(ps.swaths[i].first.second - ps.swaths[i - 1].second.second);
    EXPECT_LT(lateral, 0.18 * 2 + 1e-3);
    wide += lateral > 0.18 * 1.5 ? 1 : 0;
    // antiparallel
    const double dx0 = ps.swaths[i - 1].second.first - ps.swaths[i - 1].first.first;
    const double dx1 = ps.swaths[i].second.first - ps.swaths[i].first.first;
    EXPECT_LT(dx0 * dx1, 0.0);
  }
  EXPECT_GE(wide, ps.swaths.size() - 2);
  EXPECT_TRUE(ps.turns.empty());  // no turn radius: pivots, no turn poses
}

// Turns: a wide headland (3 rings) leaves room for a 0.5 m loop on snake
// rows; every turn pose stays inside the inset field; turns go only between
// close swaths.
TEST(CoverageRoute, LoopTurnsStayInsideTheInsetField) {
  if (!mower_coverage::builtWithF2CV3()) GTEST_SKIP() << "F2C v3 only";
  mower_coverage::RouteOptions r;
  r.order = mower_coverage::RouteOrder::kSnake;
  r.min_turn_radius = 0.5;
  r.turn_type = mower_coverage::TurnType::kAuto;
  const auto p = planRoute(r, 8.0, 4.0, 4);
  ASSERT_EQ(p.turns.size(), p.swaths.size());
  EXPECT_EQ(p.turns[0].kind, mower_coverage::TurnKind::kNone);
  EXPECT_GT(p.loop_turns + p.reverse_turns, 0u);
  for (const auto& t : p.turns) {
    for (const auto& tp : t.poses) {
      EXPECT_GE(tp.p.first, 0.24 - 0.02);
      EXPECT_LE(tp.p.first, 8.0 - 0.24 + 0.02);
      EXPECT_GE(tp.p.second, 0.24 - 0.02);
      EXPECT_LE(tp.p.second, 4.0 - 0.24 + 0.02);
    }
  }
}

// No headland room (no rings, swaths end on the inset line): every loop would
// bulge out of the field, so loop-only falls back to pivots everywhere.
TEST(CoverageRoute, TurnWithoutRoomFallsBackToPivot) {
  if (!mower_coverage::builtWithF2CV3()) GTEST_SKIP() << "F2C v3 only";
  mower_coverage::RouteOptions r;
  r.order = mower_coverage::RouteOrder::kSnake;
  r.min_turn_radius = 0.5;
  r.turn_type = mower_coverage::TurnType::kLoop;
  const auto p = planRoute(r, 6.0, 3.0, /*rings=*/-1);
  ASSERT_GT(p.swaths.size(), 3u);
  EXPECT_EQ(p.loop_turns, 0u);
  EXPECT_EQ(p.pivot_turns, p.swaths.size() - 1);
  for (const auto& t : p.turns) EXPECT_TRUE(t.poses.empty());
}

// planSwathTurn: loop when unconstrained; inside a tight corridor the loop
// does not fit and auto falls back to a reverse-then-curve turn (some poses
// driven backwards); pivot type never produces poses.
TEST(CoverageRoute, AutoTurnFallsBackLoopReversePivot) {
  if (!mower_coverage::builtWithF2CV3()) GTEST_SKIP() << "F2C v3 only";
  using mower_coverage::TurnKind;
  using mower_coverage::TurnType;
  // swath k ends at (0,0) heading +x; swath k+2 starts at (0,0.36) heading -x.
  const Point2D a{0.0, 0.0}, b{0.0, 0.36};
  const auto free_turn =
      mower_coverage::planSwathTurn(a, 0.0, b, M_PI, 0.18, 0.5, TurnType::kAuto, nullptr);
  EXPECT_EQ(free_turn.kind, TurnKind::kLoop);
  ASSERT_FALSE(free_turn.poses.empty());
  for (const auto& tp : free_turn.poses) EXPECT_FALSE(tp.reverse);

  // Corridor: 0.6 m beyond the swath ends, generous sideways.
  const auto region = mower_coverage::makeFieldCell(
      {{-5.0, -2.0}, {0.6, -2.0}, {0.6, 2.4}, {-5.0, 2.4}}, {});
  f2c::types::Cells cells;
  cells.addGeometry(region);
  const auto loop_only =
      mower_coverage::planSwathTurn(a, 0.0, b, M_PI, 0.18, 0.5, TurnType::kLoop, &cells);
  const auto autot =
      mower_coverage::planSwathTurn(a, 0.0, b, M_PI, 0.18, 0.5, TurnType::kAuto, &cells);
  if (loop_only.kind == TurnKind::kPivot) {
    EXPECT_NE(autot.kind, TurnKind::kLoop);
  }
  if (autot.kind == TurnKind::kReverseCurve) {
    bool any_rev = false;
    for (const auto& tp : autot.poses) {
      any_rev |= tp.reverse;
      EXPECT_LE(tp.p.first, 0.6 + 0.011);
    }
    EXPECT_TRUE(any_rev);
  }
  // Nothing fits in a 5 cm corridor.
  const auto tiny = mower_coverage::makeFieldCell(
      {{-5.0, -0.02}, {0.02, -0.02}, {0.02, 0.38}, {-5.0, 0.38}}, {});
  f2c::types::Cells tcells;
  tcells.addGeometry(tiny);
  const auto none =
      mower_coverage::planSwathTurn(a, 0.0, b, M_PI, 0.18, 0.5, TurnType::kAuto, &tcells);
  EXPECT_EQ(none.kind, TurnKind::kPivot);
  EXPECT_TRUE(none.poses.empty());
  EXPECT_EQ(mower_coverage::planSwathTurn(a, 0.0, b, M_PI, 0.18, 0.5, TurnType::kPivot,
                                          nullptr).kind,
            TurnKind::kPivot);
  (void)turnLen;
}

// 2026-10-13 swath-end turns: a forward loop between snake rows (0.36 m apart) needs
// ~1.06 m (r 0.44) of room past the swath ends; with 2 headland rings (op 0.18, edge
// margin 0.05 -> border_inset 0.14) a rectangle leaves 0.27 m, so auto never picks a
// loop there; 8 rings (1.44 m band) do fit r 0.5 loops and then nothing reverses.
TEST(CoverageRoute, LoopTurnNeedsAWideHeadland) {
  if (!mower_coverage::builtWithF2CV3()) GTEST_SKIP() << "F2C v3 only";
  using mower_coverage::TurnKind;
  using mower_coverage::TurnType;
  const std::vector<Point2D> rect = {{0, 0}, {8, 0}, {8, 5}, {0, 5}};
  auto plan = [&](int rings) {
    mower_coverage::RouteOptions r;
    r.order = mower_coverage::RouteOrder::kSnake;
    r.min_turn_radius = 0.5;
    r.turn_type = TurnType::kAuto;
    return mower_coverage::planCoverage(field(rect), 0.18, 0.2, rings, 0.14, 0.0, 0.15,
                                        mower_coverage::PathMode::kZigzag, true, r);
  };
  const auto two = plan(2);
  ASSERT_GT(two.swaths.size(), 10u);
  EXPECT_EQ(two.loop_turns, 0u);
  double max_x = 0.0;
  for (const auto& sw : two.swaths) max_x = std::max({max_x, sw.first.first, sw.second.first});
  EXPECT_NEAR(8.0 - 0.14 - max_x, 0.27, 0.01);       // room past the swath ends
  const auto eight = plan(8);
  EXPECT_GT(eight.loop_turns, 0u);
  EXPECT_EQ(eight.reverse_turns, 0u);
  EXPECT_EQ(eight.pivot_turns, 0u);

  // Room beyond a swath end (corridor depth) for one snake turn, r 0.44 (the tightest
  // both-wheel arc): a reverse turn from ~0.31 m, a forward loop only from ~1.06 m.
  const Point2D a{0.0, 0.0}, b{0.0, 0.36};
  auto kind_at = [&](double room) {
    const auto region = mower_coverage::makeFieldCell(
        {{-5.0, -2.0}, {room, -2.0}, {room, 2.4}, {-5.0, 2.4}}, {});
    f2c::types::Cells cells;
    cells.addGeometry(region);
    return mower_coverage::planSwathTurn(a, 0.0, b, M_PI, 0.18, 0.44, TurnType::kAuto,
                                         &cells).kind;
  };
  EXPECT_EQ(kind_at(0.27), TurnKind::kPivot);
  EXPECT_EQ(kind_at(0.45), TurnKind::kReverseCurve);
  EXPECT_EQ(kind_at(1.00), TurnKind::kReverseCurve);
  EXPECT_EQ(kind_at(1.10), TurnKind::kLoop);
}

// A turn radius below the tightest drivable arc is raised to it (and said so):
// the plan equals one requested at that radius.
TEST(CoverageRoute, TurnRadiusRaisedToTheDrivableMinimum) {
  if (!mower_coverage::builtWithF2CV3()) GTEST_SKIP() << "F2C v3 only";
  mower_coverage::RouteOptions tight;
  tight.order = mower_coverage::RouteOrder::kSnake;
  tight.min_turn_radius = 0.25;
  tight.min_drivable_turn_radius = 0.44;
  mower_coverage::RouteOptions ref = tight;
  ref.min_turn_radius = 0.44;
  const auto p = planRoute(tight, 8.0, 4.0, 4);
  const auto q = planRoute(ref, 8.0, 4.0, 4);
  ASSERT_EQ(p.turns.size(), q.turns.size());
  EXPECT_GT(p.loop_turns + p.reverse_turns, 0u);
  EXPECT_EQ(p.loop_turns, q.loop_turns);
  EXPECT_EQ(p.reverse_turns, q.reverse_turns);
  for (size_t i = 0; i < p.turns.size(); ++i) {
    EXPECT_EQ(p.turns[i].poses.size(), q.turns[i].poses.size()) << i;
  }
  ASSERT_FALSE(p.drops.empty());
  EXPECT_NE(p.drops.front().find("raised to 0.44"), std::string::npos) << p.drops.front();
  // not below the minimum, or no turns (pivots only): unchanged, no note
  for (const auto& d : q.drops) EXPECT_EQ(d.find("raised to"), std::string::npos);
  mower_coverage::RouteOptions pivots = tight;
  pivots.min_turn_radius = 0.0;
  const auto z = planRoute(pivots, 8.0, 4.0, 4);
  EXPECT_TRUE(z.turns.empty());
  for (const auto& d : z.drops) EXPECT_EQ(d.find("raised to"), std::string::npos);
}

// Race track ("lands"): rows >= 2r apart inside each block, so with enough
// headland the in-block turns are forward U-turns and nothing pivots.
TEST(CoverageRoute, RacetrackGivesForwardUTurns) {
  EXPECT_EQ(mower_coverage::racetrackHalfBlock(0.18, 0.5), 7);
  EXPECT_EQ(mower_coverage::racetrackHalfBlock(0.18, 0.0), 2);
  using R = mower_coverage::RouteOrder;
  EXPECT_EQ(mower_coverage::routeOrderPermutation(6, R::kRacetrack, 3),
            (std::vector<size_t>{0, 3, 1, 4, 2, 5}));
  // tail block of 3 after a block of 4 (k = 2): k' = 2
  EXPECT_EQ(mower_coverage::routeOrderPermutation(7, R::kRacetrack, 2),
            (std::vector<size_t>{0, 2, 1, 3, 4, 6, 5}));
  if (!mower_coverage::builtWithF2CV3()) GTEST_SKIP() << "F2C v3 only";
  mower_coverage::RouteOptions r;
  r.order = R::kRacetrack;
  r.min_turn_radius = 0.5;
  const auto p = planRoute(r, 10.0, 6.0, 4);
  EXPECT_GT(p.loop_turns, p.swaths.size() / 2);
  EXPECT_EQ(p.pivot_turns, 0u);
}

// Racetrack without headland room would turn its 1.26 m row gaps into
// transits; the planner falls back to snake (short pivots, no transits).
TEST(CoverageRoute, RacetrackFallsBackToSnakeWithoutHeadlandRoom) {
  if (!mower_coverage::builtWithF2CV3()) GTEST_SKIP() << "F2C v3 only";
  mower_coverage::RouteOptions r;
  r.order = mower_coverage::RouteOrder::kRacetrack;
  r.min_turn_radius = 0.5;
  const auto p = planRoute(r, 10.0, 6.0, /*rings=*/-1);
  EXPECT_EQ(p.wide_pivot_turns, 0u);
  bool said = false;
  for (const auto& d : p.drops) said |= d.find("used snake") != std::string::npos;
  EXPECT_TRUE(said);
}

// ---------------------------------------------------------------------------
// Coverage verification + gap filling (owner requirement: no gaps).
// op_width 0.18, blade-edge inset 0.05 -> outer ring centreline at 0.14.
// ---------------------------------------------------------------------------
namespace {

CoveragePlan planFilled(const f2c::types::Cell& cell, int rings, bool fill,
                        mower_coverage::RouteOrder order = mower_coverage::RouteOrder::kBoustrophedon,
                        double radius = 0.0) {
  mower_coverage::RouteOptions r;
  r.order = order;
  r.min_turn_radius = radius;
  r.fill_gaps = fill;
  r.min_gap_area_m2 = 0.01;
  r.target_inset = 0.05;
  return mower_coverage::planCoverage(cell, 0.18, 0.2, rings, 0.09 + 0.05, -1.0, 0.15,
                                      mower_coverage::PathMode::kZigzag, true, r);
}

void expectFilled(const char* name, const f2c::types::Cell& cell, int rings, double min_frac,
                  mower_coverage::RouteOrder order = mower_coverage::RouteOrder::kBoustrophedon,
                  double radius = 0.0) {
  const CoveragePlan before = planFilled(cell, rings, false, order, radius);
  const CoveragePlan after = planFilled(cell, rings, true, order, radius);
  std::printf("[coverage] %-28s before %.4f (%zu gaps, %.4f m2)  after %.4f (%zu gaps, %.4f m2, "
              "+%zu fill passes)\n",
              name, before.coverage_fraction, before.gap_count, before.gap_area_m2,
              after.coverage_fraction, after.gap_count, after.gap_area_m2, after.fill_swaths);
  EXPECT_GT(after.target_area_m2, 0.0);
  EXPECT_DOUBLE_EQ(after.coverage_fraction_before_fill, before.coverage_fraction);
  EXPECT_GE(after.coverage_fraction, min_frac) << name;
  EXPECT_GE(after.coverage_fraction, before.coverage_fraction - 1e-9);
  EXPECT_EQ(after.swaths.size(), before.swaths.size() + after.fill_swaths);
  if (!after.turns.empty()) {
    EXPECT_EQ(after.turns.size(), after.swaths.size());
  }
}

const std::vector<Point2D> kArea1 = {
    {0.389398, 1.76668}, {0.329541, 2.23991}, {0.141112, 2.43469}, {-0.869764, 2.28123},
    {-1.55917, 2.01274}, {-2.7828, 1.01557},  {-3.006, 0.657267},  {-3.07431, 0.315402},
    {-2.90828, -0.904132}, {-2.70292, -1.68169}, {-2.48501, -1.75286}, {-1.37076, -1.00271},
    {-0.969795, -0.462225}, {-0.657529, 0.252227}, {0.146422, 1.03497}};

}  // namespace

TEST(CoverageGaps, Rectangle305) {
  expectFilled("rect 3.05x4", field({{0, 0}, {3.05, 0}, {3.05, 4.0}, {0, 4.0}}), 2, 0.995);
}

TEST(CoverageGaps, LShape) {
  expectFilled("L-shape", field({{0, 0}, {4, 0}, {4, 1.7}, {1.7, 1.7}, {1.7, 4}, {0, 4}}), 2,
               0.995);
}

TEST(CoverageGaps, PolygonWithHole) {
  expectFilled("rect with hole",
               mower_coverage::makeFieldCell(square(0, 0, 4), {square(1.5, 1.5, 0.8)}), 2,
               0.995);
}

TEST(CoverageGaps, Area1TwoRings) {
  expectFilled("Area 1, 2 rings", field(kArea1), 2, 0.99);
}

TEST(CoverageGaps, Area1RacetrackThreeRings) {
  expectFilled("Area 1, racetrack 3 rings r0.3", field(kArea1), 3,
               0.99, mower_coverage::RouteOrder::kRacetrack, 0.3);
}

TEST(CoverageGaps, VerifyReportsGapsWithoutFill) {
  const CoveragePlan p = planFilled(field({{0, 0}, {3.05, 0}, {3.05, 4.0}, {0, 4.0}}), 2, false);
  EXPECT_EQ(p.fill_swaths, 0u);
  EXPECT_LE(p.coverage_fraction, 1.0);
  EXPECT_EQ(p.gap_count, p.gaps.size());
}

// ---- Stock-style edge layout (owner 2026-10-21) -------------------------------------

namespace {

// Signed distance of p inside the axis-aligned rectangle [0, w] x [0, h] (> 0 inside).
double rectInset(const Point2D& p, double w, double h) {
  return std::min(std::min(p.first, w - p.first), std::min(p.second, h - p.second));
}

CoveragePlan planLayout(const std::string& layout, double w, double h,
                        const std::string& order = "racetrack") {
  mower_coverage::RouteOptions route;
  EXPECT_TRUE(mower_coverage::parseRouteOrder(order, &route.order));
  route.min_turn_radius = 0.5;
  route.min_drivable_turn_radius = 0.40;
  route.fill_gaps = true;
  EXPECT_TRUE(mower_coverage::applyEdgeLayout(layout, 0.18, &route));
  return mower_coverage::planCoverage(
      field({{0, 0}, {w, 0}, {w, h}, {0, h}}), 0.18, 0.20, 2, 0.14, 0.0, 0.15,
      mower_coverage::PathMode::kZigzag, true, route);
}

// Every ring vertex of ring k at the expected inset (the rectangle's offsets are rectangles).
// Ring k at its inset along the edges (rectInset, the minimum, is that inset everywhere but
// in the rounded convex corners, where it is larger: corner radius 0.7).
void expectRingInsets(const CoveragePlan& plan, const std::vector<double>& insets, double w,
                      double h) {
  ASSERT_EQ(plan.rings.size(), insets.size());
  for (size_t k = 0; k < insets.size(); ++k) {
    double lo = 1e9;
    for (const auto& p : plan.rings[k]) {
      EXPECT_GE(rectInset(p, w, h), insets[k] - 0.01) << "ring " << k;
      EXPECT_LE(rectInset(p, w, h), insets[k] + 0.7 * (1.0 - std::sqrt(0.5)) + 0.01);
      lo = std::min(lo, rectInset(p, w, h));
    }
    EXPECT_NEAR(lo, insets[k], 0.01) << "ring " << k;
  }
}

double minInset(const CoveragePlan& plan, double w, double h, bool swaths, bool turns) {
  double m = 1e9;
  if (swaths) {
    for (const auto& s : plan.swaths) {
      m = std::min(m, std::min(rectInset(s.first, w, h), rectInset(s.second, w, h)));
    }
  }
  if (turns) {
    for (const auto& t : plan.turns) {
      for (const auto& tp : t.poses) m = std::min(m, rectInset(tp.p, w, h));
    }
  }
  return m;
}

}  // namespace

TEST(EdgeLayout, UnknownNameIsRejectedLegacyIsANoOp) {
  mower_coverage::RouteOptions r;
  EXPECT_FALSE(mower_coverage::applyEdgeLayout("wobbly", 0.18, &r));
  EXPECT_TRUE(mower_coverage::applyEdgeLayout("", 0.18, &r));
  EXPECT_TRUE(mower_coverage::applyEdgeLayout("legacy", 0.18, &r));
  EXPECT_TRUE(r.lap_offsets.empty());
}

TEST(EdgeLayout, SoftLapsOnTheLineThenStockInsetsRowsAt045) {
  const double w = 8.0, h = 6.0;
  const CoveragePlan plan = planLayout("soft", w, h);
  expectRingInsets(plan, {0.0, 0.15, 0.30}, w, h);
  ASSERT_FALSE(plan.swaths.empty());
  // row ends (and fill passes) never closer than 0.45 to the line; the first row at 0.45
  EXPECT_GE(minInset(plan, w, h, true, false), 0.45 - 0.01);
  EXPECT_LE(minInset(plan, w, h, true, false), 0.45 + 0.01);
  // swath-to-swath turns stay >= 0.45 inside (no arcs out to the edge)
  if (!plan.turns.empty()) EXPECT_GE(minInset(plan, w, h, false, true), 0.45 - 0.02);
  // the blade (0.18 + overlap) reaches the whole field: lap 0 covers to the line
  EXPECT_GT(plan.coverage_fraction, 0.995);
}

TEST(EdgeLayout, HardKeepsTheBodyInsideWithMargin) {
  const double w = 8.0, h = 6.0;
  const CoveragePlan plan = planLayout("hard", w, h);
  expectRingInsets(plan, {0.45, 0.63, 0.81, 0.99}, w, h);
  ASSERT_FALSE(plan.swaths.empty());
  EXPECT_GE(minInset(plan, w, h, true, false), 1.08 - 0.01);   // rows laterally from 1.10
  EXPECT_GT(plan.coverage_fraction, 0.995);   // of the target (blade of lap 0.45 inwards)
  // the row ends: 1.08 from the edge ahead (pivot room)
  for (const auto& s : plan.swaths) {
    const bool horizontal = std::abs(s.first.second - s.second.second) < 1e-6;
    for (const auto& p : {s.first, s.second}) {
      const double ahead = horizontal ? std::min(p.first, w - p.first)
                                      : std::min(p.second, h - p.second);
      EXPECT_GE(ahead, 1.08 - 0.01);
    }
  }
  // turns: base_link >= 0.745 in (pivot swing 0.645 + 0.10 margin)
  if (!plan.turns.empty()) EXPECT_GE(minInset(plan, w, h, false, true), 0.745 - 0.02);
}

TEST(EdgeLayout, SoftHoleIsClearedByTheBodySides) {
  const double w = 8.0, h = 6.0;
  mower_coverage::RouteOptions route;
  ASSERT_TRUE(mower_coverage::applyEdgeLayout("soft", 0.18, &route));
  const std::vector<Point2D> hole = {{3, 2}, {4, 2}, {4, 3}, {3, 3}};
  const CoveragePlan plan = mower_coverage::planCoverage(
      mower_coverage::makeFieldCell({{0, 0}, {w, 0}, {w, h}, {0, h}}, {hole}), 0.18, 0.20, 2,
      0.14, 0.0, 0.15, mower_coverage::PathMode::kZigzag, true, route);
  // every driven pose >= 0.40 from the hole (body half width 0.235 + 0.10, and the 0.40 arc)
  auto holeDist = [](const Point2D& p) {
    const double dx = std::max({3.0 - p.first, 0.0, p.first - 4.0});
    const double dy = std::max({2.0 - p.second, 0.0, p.second - 3.0});
    return std::hypot(dx, dy);
  };
  for (const auto& r : plan.rings) for (const auto& p : r) EXPECT_GE(holeDist(p), 0.40 - 0.01);
  for (const auto& s : plan.swaths) {
    EXPECT_GE(holeDist(s.first), 0.85 - 0.01);   // row ends: pivot sweep 0.645 + 0.10 and more
    EXPECT_GE(holeDist(s.second), 0.85 - 0.01);
  }
}

// Round 2 (2026-10-21), real_run5 Area 1: every outer lap runs the same way round (an
// unbuffered lap 0 keeps the recorded CCW orientation: a lap 0 -> lap 1 hand-off was a 180
// deg turn), lap 0 keeps the field's corners (no r 0.7 arc ON the line: the outer
// front corner rides 0.41 past it there), and no gap-fill pass is a short / low-yield sliver
// pass (they came in stacks 0.02 m apart, driven as 180 deg pivots).
TEST(EdgeLayout, SoftLapsOneDirectionLap0SharpNoSliverFills) {
  const std::vector<Point2D> area = {
      {-1.61954, 1.81719}, {-0.891224, 2.25271}, {-0.684406, 2.6065}, {-0.475308, 4.42287},
      {-0.335033, 5.52028}, {-1.14671, 5.26496}, {-3.34799, 4.33405}, {-4.36569, 3.80114},
      {-4.75099, 3.25008}, {-5.48879, 2.17825}, {-6.0423, 1.4301}, {-5.59106, 0.665662},
      {-5.36252, 0.178298}, {-5.15124, -0.0268646}, {-5.38696, -0.585309}, {-4.74587, -1.38861},
      {-3.28978, 0.250287}, {-1.92367, 1.30651}};
  mower_coverage::RouteOptions route;
  ASSERT_TRUE(mower_coverage::parseRouteOrder("racetrack", &route.order));
  route.min_turn_radius = 0.5;
  route.min_drivable_turn_radius = 0.40;
  route.fill_gaps = true;
  route.min_gap_area_m2 = 0.01;
  route.target_inset = 0.05;
  ASSERT_TRUE(mower_coverage::applyEdgeLayout("soft", 0.18, &route));
  const CoveragePlan plan = mower_coverage::planCoverage(
      field(area), 0.18, 0.18, 2, 0.14, -1.0, 0.15, mower_coverage::PathMode::kZigzag, true,
      route);
  ASSERT_EQ(plan.rings.size(), 3u);
  int sign = 0;
  for (const auto& r : plan.rings) {
    double a2 = 0.0;
    for (size_t j = 0; j + 1 < r.size(); ++j) {
      a2 += r[j].first * r[j + 1].second - r[j + 1].first * r[j].second;
    }
    const int s = a2 > 0.0 ? 1 : -1;
    if (sign == 0) sign = s;
    EXPECT_EQ(s, sign) << "laps run different ways round";
  }
  // lap 0: every field vertex that turns (> 5 deg; near-straight ones are cleaned away) is a
  // lap vertex, on the line
  for (size_t i = 0; i < area.size(); ++i) {
    const auto& u = area[(i + area.size() - 1) % area.size()];
    const auto& v = area[i];
    const auto& w = area[(i + 1) % area.size()];
    const double t = std::remainder(std::atan2(w.second - v.second, w.first - v.first) -
                                        std::atan2(v.second - u.second, v.first - u.first),
                                    2.0 * M_PI);
    if (std::abs(t) < 5.0 * M_PI / 180.0) continue;
    double best = 1e9;
    for (const auto& p : plan.rings[0]) {
      best = std::min(best, std::hypot(p.first - v.first, p.second - v.second));
    }
    EXPECT_LT(best, 0.01) << "lap 0 off the corner at " << v.first << ", " << v.second;
  }
  for (size_t i = plan.swaths.size() - plan.fill_swaths; i < plan.swaths.size(); ++i) {
    const auto& s = plan.swaths[i];
    EXPECT_GE(std::hypot(s.second.first - s.first.first, s.second.second - s.first.second),
              0.30 - 1e-6);
  }
  EXPECT_LE(plan.fill_swaths, 6u);
  EXPECT_GT(plan.coverage_fraction, 0.94);   // round 4: rows < 1.2 m dropped (-0.5 %)
}

// Round 2: the lap -> lap hand-offs (every lap's start) sit on a straight stretch, >= 0.6 m
// from the lap's vertices (real_run5: lap 0 started on a convex vertex, a 119 deg pivot there).
TEST(EdgeLayout, LapHandOffsClearOfCorners) {
  const double w = 8.0, h = 6.0;
  for (const char* order : {"racetrack", "boustrophedon"}) {
    const CoveragePlan plan = planLayout("soft", w, h, order);
    ASSERT_EQ(plan.rings.size(), 3u);
    for (const auto& r : plan.rings) {
      const Point2D s = r.front();
      for (size_t i = 1; i + 1 < r.size(); ++i) {
        EXPECT_GE(std::hypot(r[i].first - s.first, r[i].second - s.second), 0.6 - 1e-6)
            << order << " lap start " << s.first << ", " << s.second;
      }
    }
  }
}

// edge_laps (owner 2026-10-22): the number of edge laps, the rows moving with them. Soft: laps
// at 0 / 0.15 / ..., rows 0.15 x edge_laps in (0: rows from the line); hard: laps from 0.45
// at 0.18 plus the band-closing lap, rows behind them (hard 0 = 1: rows are clipped >= 0.745).
TEST(EdgeLayout, EdgeLapsMoveTheRows) {
  const double w = 8.0, h = 6.0;
  struct Case { const char* mode; int laps; std::vector<double> rings; double side, end; };
  const std::vector<Case> cases = {
      {"soft", 0, {}, 0.0, 0.0},
      {"soft", 1, {0.0}, 0.15, 0.15},
      {"soft", 3, {0.0, 0.15, 0.30}, 0.45, 0.45},
      {"hard", 0, {0.45, 0.63}, 0.74, 0.745},     // hard needs a lap: 0 = 1
      {"hard", 1, {0.45, 0.63}, 0.74, 0.745},
      {"hard", 3, {0.45, 0.63, 0.81, 0.99}, 1.10, 1.08},
  };
  for (const auto& c : cases) {
    mower_coverage::RouteOptions route;
    ASSERT_TRUE(mower_coverage::parseRouteOrder("racetrack", &route.order));
    route.min_turn_radius = 0.5;
    route.min_drivable_turn_radius = 0.40;
    route.fill_gaps = true;
    ASSERT_TRUE(mower_coverage::applyEdgeLayout(c.mode, 0.18, &route, c.laps));
    EXPECT_TRUE(route.edge_layout);
    EXPECT_NEAR(route.row_side_inset, c.side, 1e-9) << c.mode << " " << c.laps;
    EXPECT_NEAR(route.row_end_inset, c.end, 1e-9) << c.mode << " " << c.laps;
    const CoveragePlan plan = mower_coverage::planCoverage(
        field({{0, 0}, {w, 0}, {w, h}, {0, h}}), 0.18, 0.20, 2, 0.14, 0.0, 0.15,
        mower_coverage::PathMode::kZigzag, true, route);
    SCOPED_TRACE(std::string(c.mode) + " " + std::to_string(c.laps) + " laps");
    expectRingInsets(plan, c.rings, w, h);
    ASSERT_FALSE(plan.swaths.empty());
    // the outermost rows sit where the laps leave off (row ends, sides: the nearer one)
    const double lo = std::min(c.side, c.end);
    EXPECT_GE(minInset(plan, w, h, true, false), lo - 0.01);
    EXPECT_LE(minInset(plan, w, h, true, false), std::max(lo, 0.09) + 0.03);
    EXPECT_GT(plan.coverage_fraction, 0.95);
  }
  // the default is the stock 3 laps; negative counts are refused
  mower_coverage::RouteOptions a, b;
  ASSERT_TRUE(mower_coverage::applyEdgeLayout("soft", 0.18, &a));
  ASSERT_TRUE(mower_coverage::applyEdgeLayout("soft", 0.18, &b, 3));
  EXPECT_EQ(a.lap_offsets, b.lap_offsets);
  EXPECT_FALSE(mower_coverage::applyEdgeLayout("soft", 0.18, &a, -1));
}

// Round 3 (owner option (a), 2026-10-22): lap starts / hand-offs on a stretch with <= 10 deg of
// total turning within +-1.2 m of arc length (a bend of many small vertices counts too).
namespace {
double turnWithin(const std::vector<Point2D>& loop, const Point2D& p, double w) {
  std::vector<Point2D> pts(loop);
  const size_t n = pts.size();
  std::vector<double> cum(n + 1, 0.0);
  double sp = 0.0, bestd = 1e9;
  for (size_t i = 0; i < n; ++i) {
    const Point2D& a = pts[i];
    const Point2D& b = pts[(i + 1) % n];
    const double L = std::hypot(b.first - a.first, b.second - a.second);
    const double dx = b.first - a.first, dy = b.second - a.second;
    double u = L > 0 ? ((p.first - a.first) * dx + (p.second - a.second) * dy) / (L * L) : 0;
    u = std::max(0.0, std::min(1.0, u));
    const double d = std::hypot(a.first + u * dx - p.first, a.second + u * dy - p.second);
    if (d < bestd) { bestd = d; sp = cum[i] + u * L; }
    cum[i + 1] = cum[i] + L;
  }
  double tot = 0.0;
  for (size_t i = 0; i < n; ++i) {
    const Point2D& a = pts[(i + n - 1) % n];
    const Point2D& v = pts[i];
    const Point2D& b = pts[(i + 1) % n];
    double d = std::fabs(cum[i] - sp);
    d = std::min(d, cum[n] - d);
    if (d > w) continue;
    const double h0 = std::atan2(v.second - a.second, v.first - a.first);
    const double h1 = std::atan2(b.second - v.second, b.first - v.first);
    tot += std::fabs(std::remainder(h1 - h0, 2.0 * M_PI));
  }
  return tot * 180.0 / M_PI;
}
}  // namespace

TEST(EdgeLayout, LapStartOnAStraightStretchRun5) {
  // real_run5 Area 1 outline; the round-2 lap 0 start (-0.61, 3.23) sat 0.63 m after the
  // (-0.891, 2.253) / (-0.684, 2.606) bend (-23 / -29 deg): the run 3 gates' worst body excess
  const std::vector<Point2D> loop = {
      {-1.620, 1.817}, {-0.891, 2.253}, {-0.684, 2.606}, {-0.475, 4.423}, {-0.335, 5.520},
      {-1.147, 5.265}, {-3.348, 4.334}, {-4.366, 3.801}, {-4.751, 3.250}, {-5.489, 2.178},
      {-6.042, 1.430}, {-5.591, 0.666}, {-5.363, 0.178}, {-5.151, -0.027}, {-5.387, -0.585},
      {-4.746, -1.389}, {-3.290, 0.250}, {-1.924, 1.307}};
  const Point2D p = mower_coverage::lapStartPoint(loop, {-0.61, 3.23}, 1.2, 10.0);
  EXPECT_LE(turnWithin(loop, p, 1.2), 10.0) << p.first << ", " << p.second;
  EXPECT_GE(std::hypot(p.first + 0.684, p.second - 2.606), 1.2 - 1e-6);
  EXPECT_LT(std::hypot(p.first + 0.61, p.second - 3.23), 1.5);   // stays on that edge
}

TEST(EdgeLayout, LapStartCountsBendsOfSmallVertices) {
  // a 10 x 4 rectangle whose bottom edge carries a 50 deg bend made of 5 x 10 deg vertices
  // 0.1 m apart at x 4.0 .. 4.4; target 0.5 m after it
  std::vector<Point2D> loop = {{0, 0}};
  double x = 4.0, y = 0.0, h = 0.0;
  loop.push_back({x, y});
  for (int k = 0; k < 5; ++k) {
    h += 10.0 * M_PI / 180.0 * (k < 3 ? 1 : -1.5);
    x += 0.1 * std::cos(h);
    y += 0.1 * std::sin(h);
    loop.push_back({x, y});
  }
  loop.push_back({10, y});
  loop.push_back({10, 4});
  loop.push_back({0, 4});
  const Point2D p = mower_coverage::lapStartPoint(loop, {x + 0.5, y}, 1.2, 10.0);
  EXPECT_LE(turnWithin(loop, p, 1.2), 10.0);
  EXPECT_GE(p.first, x + 1.1 - 1e-6);   // 1.2 m past the last turning vertex (0.1 m before x)
}

TEST(EdgeLayout, LapStartFallsBackToTheStraightestPoint) {
  std::vector<Point2D> circle;
  for (int k = 0; k < 72; ++k)
    circle.push_back({std::cos(k * M_PI / 36), std::sin(k * M_PI / 36)});
  const Point2D p = mower_coverage::lapStartPoint(circle, {1.2, 0.0}, 1.2, 10.0);
  EXPECT_NEAR(std::hypot(p.first, p.second), 1.0, 0.01);          // on the loop
  EXPECT_LT(std::hypot(p.first - 1.0, p.second), 0.1);            // nearest of the equals
}

// Obstacles (holes) are hard keep-outs, 2026-10-23: a manual obstacle 0.39 m inside Area 7's
// west line. The soft layout grew it 0.30 with F2C's mitred buffer, kept it as a hole ring
// 0.30 from the pole (body side clearance 0.065) squeezed 0.08 inside the line, and the inner
// laps' mitred opening spiked 0.55 m outside the line. Now: grown 0.40 with round joins,
// filleted 0.40 where it meets the line; an obstacle nearer the line than 0.40 merges with
// the outside (lap 0 bends inward around it, the gap stays uncut).
namespace {

double segDist(const Point2D& p, const Point2D& a, const Point2D& b) {
  const double dx = b.first - a.first, dy = b.second - a.second, l = dx * dx + dy * dy;
  const double u = l > 0 ? std::clamp(((p.first - a.first) * dx + (p.second - a.second) * dy) / l,
                                      0.0, 1.0) : 0.0;
  return std::hypot(p.first - a.first - u * dx, p.second - a.second - u * dy);
}

bool inPoly(const Point2D& p, const std::vector<Point2D>& pg) {
  bool c = false;
  for (size_t i = 0, j = pg.size() - 1; i < pg.size(); j = i++) {
    const auto& a = pg[j];
    const auto& b = pg[i];
    if ((a.second > p.second) != (b.second > p.second) &&
        p.first < a.first + (p.second - a.second) * (b.first - a.first) / (b.second - a.second)) {
      c = !c;
    }
  }
  return c;
}

// Signed distance to a polygon's outline (> 0 inside).
double polyInset(const Point2D& p, const std::vector<Point2D>& pg) {
  double d = 1e9;
  for (size_t i = 0, j = pg.size() - 1; i < pg.size(); j = i++) d = std::min(d, segDist(p, pg[j], pg[i]));
  return inPoly(p, pg) ? d : -d;
}

// Every driven polyline of the plan (rings, swaths, turn poses, and the short swath-to-swath
// connectors a pivot drives; longer ones are bridge transits), sampled every 2 cm.
std::vector<Point2D> drivenSamples(const CoveragePlan& plan) {
  std::vector<std::vector<Point2D>> lines(plan.rings.begin(), plan.rings.end());
  for (size_t i = 0; i < plan.swaths.size(); ++i) {
    std::vector<Point2D> l;
    if (i > 0) {
      l.push_back(plan.swaths[i - 1].second);
      if (i < plan.turns.size()) for (const auto& tp : plan.turns[i].poses) l.push_back(tp.p);
      if (l.size() > 1 || std::hypot(plan.swaths[i].first.first - l[0].first,
                                     plan.swaths[i].first.second - l[0].second) <= 0.6) {
        l.push_back(plan.swaths[i].first);
        lines.push_back(l);
      }
    }
    lines.push_back({plan.swaths[i].first, plan.swaths[i].second});
  }
  std::vector<Point2D> out;
  for (const auto& l : lines) {
    for (size_t k = 0; k + 1 < l.size(); ++k) {
      const int n = std::max(1, static_cast<int>(std::hypot(l[k + 1].first - l[k].first,
                                                            l[k + 1].second - l[k].second) / 0.02));
      for (int j = 0; j <= n; ++j) {
        out.push_back({l[k].first + (l[k + 1].first - l[k].first) * j / n,
                       l[k].second + (l[k + 1].second - l[k].second) * j / n});
      }
    }
  }
  return out;
}

CoveragePlan planSoft(const std::vector<Point2D>& outer, const std::vector<Point2D>& hole,
                      double angle_rad) {
  mower_coverage::RouteOptions route;
  route.min_turn_radius = 0.5;
  route.min_drivable_turn_radius = 0.40;
  route.fill_gaps = true;
  route.target_inset = 0.15;
  EXPECT_TRUE(mower_coverage::applyEdgeLayout("soft", 0.18, &route));
  return mower_coverage::planCoverage(mower_coverage::makeFieldCell(outer, {hole}), 0.18, 0.20,
                                      0, 0.24, angle_rad, 0.15,
                                      mower_coverage::PathMode::kZigzag, true, route);
}

// Nothing driven leaves the line (lap 0 ON it: 1 cm), nothing comes within 0.40 of the
// obstacle (round-buffer chord: 1 cm), no driven path touches the obstacle.
void expectHardHole(const CoveragePlan& plan, const std::vector<Point2D>& outer,
                    const std::vector<Point2D>& hole) {
  ASSERT_FALSE(plan.rings.empty());
  ASSERT_FALSE(plan.swaths.empty());
  double out = -1e9, near = 1e9;
  for (const auto& p : drivenSamples(plan)) {
    out = std::max(out, -polyInset(p, outer));
    near = std::min(near, -polyInset(p, hole));
  }
  EXPECT_LE(out, 0.01) << "a driven point outside the line";
  EXPECT_GE(near, 0.40 - 0.01) << "a driven point too close to (or over) the obstacle";
}

const std::vector<Point2D> kRect = {{0, 0}, {8, 0}, {8, 6}, {0, 6}};

std::vector<Point2D> obstacleSquare(double x, double y, double s) {
  return {{x, y}, {x + s, y}, {x + s, y + s}, {x, y + s}};
}

}  // namespace

TEST(EdgeLayoutHoles, ObstacleNearTheEdgeMergesWithTheOutside) {
  const auto hole = obstacleSquare(0.36, 2.9, 0.25);  // 0.36 from the west line, under 0.40
  for (double a : {0.0, M_PI / 2.0, M_PI / 4.0}) {
    const CoveragePlan plan = planSoft(kRect, hole, a);
    expectHardHole(plan, kRect, hole);
    // no ring of its own around the obstacle: lap 0 is one outline that bends inward around it
    for (const auto& r : plan.rings) {
      bool all_near = true;
      for (const auto& p : r) all_near = all_near && -polyInset(p, hole) < 1.0;
      EXPECT_FALSE(all_near) << "a hole ring";
    }
    bool behind = false;  // lap 0 passes east of the obstacle, the gap west of it uncut
    for (const auto& p : plan.rings.front()) {
      EXPECT_FALSE(p.first < 0.36 - 0.01 && p.second > 2.9 && p.second < 3.15) << "gap driven";
      behind = behind || (p.first > 0.61 + 0.39 && p.second > 2.85 && p.second < 3.2);
    }
    EXPECT_TRUE(behind);
  }
}

TEST(EdgeLayoutHoles, ObstacleInACorner) {
  const auto hole = obstacleSquare(0.36, 0.36, 0.25);
  for (double a : {0.0, M_PI / 4.0}) expectHardHole(planSoft(kRect, hole, a), kRect, hole);
}

TEST(EdgeLayoutHoles, ObstacleWellInsideGetsItsOwnRings) {
  const auto hole = obstacleSquare(3.9, 2.9, 0.25);
  const CoveragePlan plan = planSoft(kRect, hole, 0.0);
  expectHardHole(plan, kRect, hole);
  int hole_rings = 0;
  for (const auto& r : plan.rings) {
    bool all_near = true;
    for (const auto& p : r) all_near = all_near && -polyInset(p, hole) < 1.0;
    hole_rings += all_near ? 1 : 0;
  }
  EXPECT_EQ(hole_rings, 3);  // laps 0.40 / 0.55 / 0.70 around it
  // the outer laps keep their sharp corners and insets (the fillets stay near the hole)
  EXPECT_NEAR(rectInset(plan.rings.front()[0], 8, 6), 0.0, 0.01);
}

// The field geometry of 2026-10-23 (areas.dat area_5 "Area 7" and its manual obstacle).
TEST(EdgeLayoutHoles, Area7PoleNearTheWestLine) {
  const std::vector<Point2D> outer = {
      {-2.42777,0.603669},
      {-3.68007,-0.64144},
      {-4.13443,-1.25334},
      {-5.4586,-2.45428},
      {-5.64323,-2.60242},
      {-5.81183,-2.6159},
      {-6.10666,-2.26859},
      {-6.40751,-1.56303},
      {-6.49084,-0.694381},
      {-6.6368,-0.497456},
      {-6.60865,-0.32109},
      {-6.81943,0.200973},
      {-6.7096,0.67657},
      {-5.97887,1.77966},
      {-4.41611,3.63844},
      {-3.71662,3.98655},
      {-2.75692,4.28974},
      {-1.80164,4.76187},
      {-0.601567,4.99453},
      {-0.307907,5.13676},
      {-0.367525,5.16016},
      {-0.0265033,5.25821},
      {0.0592182,4.98217},
      {-0.213657,2.26314},
      {-1.35556,1.21173},
      {-2.2729,0.579132}};
  const std::vector<Point2D> hole = {
      {-6.17014,-0.449989},
      {-6.10125,-0.690233},
      {-5.87919,-0.611264},
      {-5.93394,-0.366968}};
  for (double a : {-1.0, M_PI / 4.0, 3.0 * M_PI / 4.0}) {
    expectHardHole(planSoft(outer, hole, a), outer, hole);
  }
}
