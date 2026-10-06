// Pure-geometry gtests for the mower_coverage planner core, run against the
// REAL Fields2Cover library (v3 by default, apt 2.1.0 with F2C_V3=OFF) — no
// robot, no ROS node. The point is to
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
