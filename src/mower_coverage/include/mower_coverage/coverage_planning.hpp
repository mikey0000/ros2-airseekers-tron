// Pure-geometry coverage planning on Fields2Cover (v3 by default, apt 2.1.0
// with -DF2C_V3=OFF; headland + swath).
//
// No ROS types appear in this header: the planner is unit-testable against the
// real F2C library with no robot and no ROS node. The node
// (mower_coverage_node) converts geometry_msgs/Polygon in and nav_msgs/Path
// out; everything between is f2c::types.
//
// Replaces the vendor's GPL polygon_coverage stack (CGAL decomposition +
// Mono/GkMa GTSP solver). F2C does the geometry (ConstHL headlands, BruteForce
// swaths); the drive order is our own: clipped swath pieces are grouped into
// boustrophedon cells, the cells ordered for the fewest transits, and the
// ring starts chained back from the first swath. No turn planning — the
// mower is diff-drive and pivots in place between segments.

#pragma once

#include <fields2cover.h>

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace mower_coverage {

using Point2D = std::pair<double, double>;

// Swath visiting order inside each boustrophedon cell (per-area setting
// `route_order`), via the Fields2Cover v3 route planners.
enum class RouteOrder {
  kBoustrophedon,  // 0,1,2,3,...: neighbour after neighbour (hairpin turns)
  kSnake,          // f2c::rp::SnakeOrder: 0,2,4,...,5,3,1 (turns 2 swaths wide)
  kSpiral,         // f2c::rp::SpiralOrder(route_spiral_size)
  kRacetrack,      // blocks of 2k: 0,k,1,k+1,..,k-1,2k-1 with (k-1)*op_width >= 2r,
                   // so every in-block turn is a plain forward U-turn (tractor "lands")
};

// Swath-to-swath turn policy (per-area setting `turn_type`). Every candidate
// turn must stay inside the drivable region (field pulled in by border_inset,
// holes grown), else the next candidate is tried; a pivot is the last resort.
enum class TurnType {
  kAuto,     // forward loop (Dubins) -> reverse-then-curve (Reeds-Shepp) -> pivot
  kLoop,     // forward loop -> pivot
  kReverse,  // reverse-then-curve -> pivot
  kPivot,    // pivot in place (straight connector), the pre-turn behaviour
};

enum class TurnKind { kNone, kPivot, kLoop, kReverseCurve };

struct TurnPose {
  Point2D p;
  bool reverse = false;  // reached by driving backwards
};

// The turn INTO a swath: poses strictly between the previous swath's end and
// this swath's start. kPivot / kNone carry no poses (straight connector).
struct SwathTurn {
  TurnKind kind = TurnKind::kNone;
  std::vector<TurnPose> poses;
};

struct RouteOptions {
  RouteOrder order = RouteOrder::kBoustrophedon;
  int spiral_size = 6;
  double min_turn_radius = 0.0;  // <= 0: pivots only
  TurnType turn_type = TurnType::kAuto;
  // Coverage verification / gap filling (see CoveragePlan::coverage_fraction).
  // fill_gaps: add extra fill passes (ordinary swaths) for uncovered pieces
  // larger than min_gap_area_m2, up to kFillIterations rounds.
  bool fill_gaps = false;
  double min_gap_area_m2 = 0.01;
  // Target = field shrunk by target_inset (blade edge) minus holes grown by
  // hole_margin. < 0: derived from border_inset (border_inset - op_width/2,
  // resp. border_inset: what the rings are built for).
  double target_inset = -1.0;
  double hole_margin = -1.0;
};

bool parseRouteOrder(const std::string& name, RouteOrder* order);  // "" = boustrophedon
// kRacetrack half-block k for a turn radius: smallest k with (k-1)*w >= 2r, >= 2.
int racetrackHalfBlock(double op_width, double min_turn_radius);
bool parseTurnType(const std::string& name, TurnType* type);       // "" = auto

// Drive-sequence permutation of n parallel swaths (indices in sweep order).
// spiral_size: SpiralOrder block size, or k (half block) for kRacetrack.
std::vector<size_t> routeOrderPermutation(size_t n, RouteOrder order, int spiral_size);

struct CoveragePlan {
  // Closed loops (first == last; polygon corners plus the inserted start
  // point), in drive order: the outer-boundary run (outermost pass first),
  // then one run per hole (ring on the hole first, then outward). The last
  // ring starts and ends at its point closest to the first swath.
  std::vector<std::vector<Point2D>> rings;
  // {start, end} pairs in drive order: serpentine within each boustrophedon
  // cell, cells ordered for the fewest gaps > 0.6 m.
  std::vector<std::pair<Point2D, Point2D>> swaths;
  // Swath heading actually used (radians); for auto plans, the angle chosen.
  double swath_angle_rad = 0.0;
  // Human-readable coverage-loss notes, e.g. "dropped swath len=0.1200<0.1500".
  std::vector<std::string> drops;
  // Strip area / field area. Visibility diagnostic, not a coverage guarantee.
  double planned_fraction = 0.0;
  // Drive order of the two blocks: false = rings, then swaths (edge first,
  // the default); true = swaths, then rings (innermost ring first).
  bool swaths_first = false;
  // Parallel to swaths: turns[i] is the turn from swath i-1 into swath i
  // (turns[0] is always kNone). Empty when no turn planning was requested.
  std::vector<SwathTurn> turns;
  size_t loop_turns = 0, reverse_turns = 0, pivot_turns = 0;
  // Pivots across a gap > 0.6 m: the bridge makes those blade-off transits.
  size_t wide_pivot_turns = 0;
  // Coverage verification: the swept footprint (every ring / swath / turn /
  // short connector sweeps op_width) against the target region. Exact OGR
  // (GEOS) geometry, no raster.
  double target_area_m2 = 0.0;
  double coverage_fraction = 0.0;         // covered / target area (after fill)
  double coverage_fraction_before_fill = 0.0;
  double gap_area_m2 = 0.0;               // all uncovered area
  size_t gap_count = 0;                   // pieces >= min_gap_area_m2
  std::vector<std::vector<Point2D>> gaps; // their exterior rings (closed)
  size_t fill_swaths = 0;                 // extra passes appended to `swaths`
};

// Recompute the coverage fields of `plan` (no filling).
void verifyCoverage(const f2c::types::Cell& field, double op_width, double target_inset,
                    double hole_margin, double min_gap_area_m2, CoveragePlan* plan);

// Path pattern of one plan (per-area mowing setting `path_mode`). The
// mission-level modes `cross` (two zigzag plans 90 deg apart) and `alternate`
// (zigzag, angle rotated between runs) are plain kZigzag plans here.
enum class PathMode {
  kZigzag,       // headland rings + boustrophedon swaths (the original planner)
  kSpiral,       // concentric inward rings until the field collapses, no swaths
  kContourOnly,  // only the headland rings (vendor cut_mode 2), no swaths
};

// "zigzag" / "cross" / "alternate" / "" -> kZigzag, "spiral", "contour_only".
// Returns false (and kZigzag) for an unknown name.
bool parsePathMode(const std::string& name, PathMode* mode);

// Build a clean f2c ring from raw boundary vertices: drops vertices within
// 1 cm of the previous kept one, drops near-collinear spikes, re-closes
// (first == last), then repairs self-touch via OGRPolygon::Buffer(0.0).
// F2C/boost rejects a zero-length edge and silently drops the area, so this
// gate is the highest-value sanity check in the planner.
f2c::types::LinearRing makeCleanRing(const std::vector<Point2D>& raw);

// Build the field cell: exterior ring = boundary, one interior ring per hole.
f2c::types::Cell makeFieldCell(const std::vector<Point2D>& boundary,
                               const std::vector<std::vector<Point2D>>& holes);

// Plan boustrophedon coverage of `field`.
//
// op_width          swath spacing [m] (the "border width"); must be > 0
// headland_width    desired headland band [m]; only feeds the auto ring count
// headland_passes   -1 = no rings (swaths mow to the boundary),
//                    0 = auto: max(1, ceil(headland_width / op_width)),
//                    >0 = exactly that many rings
// border_inset      chassis pull-back inside the recorded line [m]
// mow_angle_rad     swath heading; < 0 = auto (fewest swaths, or the
//                    boundary's longest-edge angle above the area gate)
// min_swath_length  drop swaths shorter than this [m]
CoveragePlan planBoustrophedon(const f2c::types::Cell& field,
                               double op_width,
                               double headland_width,
                               int headland_passes,
                               double border_inset,
                               double mow_angle_rad,
                               double min_swath_length);

// planBoustrophedon with a path pattern and the ring/swath drive order.
//
// kZigzag      identical to planBoustrophedon (edge_first = true).
// kSpiral      rings at op_width spacing, outermost first, until the field
//              (minus the hole bands) collapses; a last ring near the medial
//              axis covers the centre strip. Holes get rings like headlands.
//              headland_passes is ignored. No swaths.
// kContourOnly only the headland rings (headland_passes as above, at least 1).
// edge_first   false: swaths first, then the rings innermost first, chained
//              forwards from the last swath end. Only matters for kZigzag.
CoveragePlan planCoverage(const f2c::types::Cell& field,
                          double op_width,
                          double headland_width,
                          int headland_passes,
                          double border_inset,
                          double mow_angle_rad,
                          double min_swath_length,
                          PathMode mode,
                          bool edge_first,
                          const RouteOptions& route = RouteOptions{});

// kRacetrack falls back to kSnake for the whole plan when that gives fewer
// wide pivots (racetrack rows are >= 2r apart, so a turn that does not fit in
// the headland becomes a transit instead of a short pivot); `drops` says so.

// One swath-to-swath turn under `type` (see TurnType). `region` (nullable)
// is the area the whole turn must stay inside (1 cm tolerance); nullptr skips
// the containment check. Returns kPivot with no poses when nothing fits.
SwathTurn planSwathTurn(const Point2D& from, double from_yaw,
                        const Point2D& to, double to_yaw,
                        double robot_width, double min_turn_radius, TurnType type,
                        const f2c::types::Cells* region);

// Forward turn between the end of one swath (pose `from`, heading from_yaw)
// and the start of the next (pose `to`, heading to_yaw) with the F2C v3
// Dubins turn planner, discretised to ~5 cm. Interior points only (from/to
// excluded). Returns empty when min_turn_radius <= 0 (the default: the Tron
// is diff-drive and pivots in place, so a straight connector IS its turn),
// when built against F2C 2.1 (F2C_V3=OFF), or when the planner fails.
std::vector<Point2D> planTurn(const Point2D& from, double from_yaw,
                              const Point2D& to, double to_yaw,
                              double robot_width, double min_turn_radius);

// True when compiled against Fields2Cover v3.
bool builtWithF2CV3();

}  // namespace mower_coverage
