// Pure-geometry coverage planning on Fields2Cover 2.1.0 (headland + swath).
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
};

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

}  // namespace mower_coverage
