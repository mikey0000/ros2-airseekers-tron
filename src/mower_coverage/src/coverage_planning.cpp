#include "mower_coverage/coverage_planning.hpp"

#include <ogr_geometry.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

namespace mower_coverage {
namespace {

// Ring sanitization tolerances. Operator-recorded field boundaries carry
// mm-scale geometric degeneracies (a real 252 m^2 field had a 1.2 mm
// near-duplicate closure vertex and several 1-10 mm sliver edges). These are
// exact enough that boost::geometry accepts the points yet degenerate enough
// that F2C's headland offset / swath clip silently produce PARTIAL coverage.
// Both tolerances sit safely below op_width, so no real corner is removed.
constexpr double kRingDedupTolM = 0.01;   // drop a vertex within 1 cm of the last kept one
constexpr double kRingSpikeTolM = 0.005;  // drop a vertex within 5 mm of its neighbours' chord

// AUTO swath-angle field-size gate. BruteForce::generateBestSwaths sweeps 180
// candidate angles (1 deg step) and regenerates the full swath set at each, so
// its cost scales ~linearly with field area: a 100x100 m field takes ~24 s,
// far past any action timeout. Above this area we skip the exhaustive search
// and use the boundary's longest-edge orientation (the angle the search
// almost always converges to on a rectangular lawn anyway), which is ~3
// orders of magnitude cheaper and equally deterministic across re-plans.
constexpr double kAutoAngleMaxAreaM2 = 400.0;  // ~20 x 20 m

// AUTO swath-angle search resolution. BruteForce defaults to a 1 deg step,
// which sweeps the FULL 2*pi and regenerates the entire swath set 360 times
// per plan. The swath-count objective (NSwath) is near-flat within a few
// degrees of the optimum, so a 5 deg step cuts that cost ~5x while leaving
// the chosen angle — and thus the whole plan — essentially unchanged.
constexpr double kAutoAngleStepRad = 5.0 * M_PI / 180.0;

double dist(const Point2D& a, const Point2D& b) {
  return std::hypot(a.first - b.first, a.second - b.second);
}

// Distance from point p to the segment a-b.
double pointToSegmentDist(const Point2D& p, const Point2D& a, const Point2D& b) {
  const double dx = b.first - a.first;
  const double dy = b.second - a.second;
  const double len2 = dx * dx + dy * dy;
  if (len2 < 1e-18) {
    return dist(p, a);
  }
  double t = ((p.first - a.first) * dx + (p.second - a.second) * dy) / len2;
  t = std::max(0.0, std::min(1.0, t));
  return std::hypot(p.first - (a.first + t * dx), p.second - (a.second + t * dy));
}

// Format a "dropped <what> <val><cmp><thresh>" diagnostics line without
// pulling in <sstream>. Values are short distances/areas, so 4 decimals is
// plenty of resolution for a log line.
std::string fmtDrop(const char* what, double value, const char* cmp, double thresh) {
  char buf[128];
  std::snprintf(buf, sizeof(buf), "dropped %s%.4f%s%.4f", what, value, cmp, thresh);
  return std::string(buf);
}

// Metric dedup + near-collinear spike drop + explicit re-close. A zero-length
// edge makes the ring non-simple and boost::geometry (under F2C) rejects it,
// silently dropping the area — the single highest-value ingest gate.
std::vector<Point2D> dedupClosedRing(const std::vector<Point2D>& raw) {
  std::vector<Point2D> pts;
  pts.reserve(raw.size());
  for (const auto& p : raw) {
    if (!pts.empty() && dist(p, pts.back()) < kRingDedupTolM) {
      continue;  // near-zero-length edge
    }
    // Drop a tail vertex that sits within kRingSpikeTolM of its neighbours'
    // chord: that is either a redundant collinear point or the tip of an
    // out-and-back spike. Retry p against the new tail so chains of
    // degenerate vertices collapse.
    while (pts.size() >= 2 &&
           pointToSegmentDist(pts.back(), pts[pts.size() - 2], p) <
               kRingSpikeTolM) {
      pts.pop_back();
    }
    pts.push_back(p);
  }
  // Drop a near-duplicate closing vertex, then re-close explicitly so the
  // open vertex list carries first == last.
  while (pts.size() >= 2 && dist(pts.front(), pts.back()) < kRingDedupTolM) {
    pts.pop_back();
  }
  if (pts.size() >= 2) {
    pts.push_back(pts.front());
  }
  return pts;
}

// Repair a ring into an OGR-valid one via Buffer(0.0) — the standard
// self-intersection fix. Returns the valid equivalent of a self-touching /
// sliver-laden ring. A buffer-by-zero can split a figure-8 ring into several
// parts; keep the largest. Any degeneracy (null buffer, empty result,
// collapsed ring) returns the input unchanged — the planner must NEVER drop
// the field.
std::vector<Point2D> repairSelfTouch(const std::vector<Point2D>& ring) {
  if (ring.size() < 4) {
    return ring;
  }
  OGRPolygon poly;
  auto* exterior = new OGRLinearRing();
  for (const auto& p : ring) {
    exterior->addPoint(p.first, p.second);
  }
  exterior->closeRings();
  poly.addRing(exterior);  // OGRPolygon takes ownership
  std::unique_ptr<OGRGeometry> fixed(poly.Buffer(0.0));
  const OGRPolygon* fixed_poly = dynamic_cast<const OGRPolygon*>(fixed.get());
  if (fixed_poly == nullptr) {
    // Buffer split the ring into several parts; keep the largest.
    const OGRMultiPolygon* mp = dynamic_cast<const OGRMultiPolygon*>(fixed.get());
    if (mp != nullptr) {
      double best_area = -1.0;
      for (int i = 0; i < mp->getNumGeometries(); ++i) {
        const OGRPolygon* part =
            dynamic_cast<const OGRPolygon*>(mp->getGeometryRef(i));
        if (part == nullptr) {
          continue;
        }
        const double a = part->getArea();
        if (a > best_area) {
          best_area = a;
          fixed_poly = part;
        }
      }
    }
  }
  if (fixed_poly == nullptr || fixed_poly->getExteriorRing() == nullptr ||
      fixed_poly->getExteriorRing()->getNumPoints() < 4) {
    return ring;
  }
  const OGRLinearRing* r = fixed_poly->getExteriorRing();
  std::vector<Point2D> out;
  out.reserve(static_cast<size_t>(r->getNumPoints()));
  for (int i = 0; i < r->getNumPoints(); ++i) {
    out.emplace_back(r->getX(i), r->getY(i));
  }
  return out;
}

std::vector<Point2D> ringToLoop(const f2c::types::LinearRing& ring) {
  std::vector<Point2D> loop;
  loop.reserve(ring.size());
  for (size_t i = 0; i < ring.size(); ++i) {
    const f2c::types::Point p = ring.getGeometry(i);
    loop.emplace_back(p.getX(), p.getY());
  }
  return loop;
}

// F2C closes every concentric ring at the SAME polygon corner, so the
// ring->ring junction demands a ~90-112 deg heading change across an
// op_width gap. Rotate each loop to start mid-longest-edge before filling:
// the junction then happens mid-straight, which a diff-drive pivot tracks
// cleanly.
std::vector<Point2D> rotateToLongestEdgeMid(const std::vector<Point2D>& loop) {
  if (loop.size() < 4) {
    return loop;
  }
  size_t best = 0;
  double best_len = -1.0;
  for (size_t i = 0; i + 1 < loop.size(); ++i) {
    const double len = dist(loop[i], loop[i + 1]);
    if (len > best_len) {
      best_len = len;
      best = i;
    }
  }
  const Point2D mid = {(loop[best].first + loop[best + 1].first) / 2.0,
                       (loop[best].second + loop[best + 1].second) / 2.0};
  std::vector<Point2D> out;
  out.reserve(loop.size());
  out.push_back(mid);
  for (size_t i = best + 1; i + 1 < loop.size(); ++i) {
    out.push_back(loop[i]);
  }
  for (size_t i = 0; i <= best; ++i) {
    out.push_back(loop[i]);
  }
  out.push_back(mid);  // re-close
  return out;
}

double ringLength(const std::vector<Point2D>& loop) {
  double len = 0.0;
  for (size_t i = 0; i + 1 < loop.size(); ++i) {
    len += dist(loop[i], loop[i + 1]);
  }
  return len;
}

// Orientation (radians) of the longest edge of a cell's outer ring. A cheap,
// deterministic AUTO swath angle for large fields. Falls back to 0 (sweep
// along +x) for a degenerate ring.
double longestEdgeAngle(const f2c::types::Cell& cell) {
  const f2c::types::LinearRing ring = cell.getGeometry(0);
  double best_len = -1.0;
  double best_angle = 0.0;
  for (size_t i = 0; i < ring.size(); ++i) {
    const f2c::types::Point p = ring.getGeometry(i);
    const f2c::types::Point q = ring.getGeometry((i + 1) % ring.size());
    const double dx = q.getX() - p.getX();
    const double dy = q.getY() - p.getY();
    const double len = std::hypot(dx, dy);
    if (len > best_len) {
      best_len = len;
      best_angle = std::atan2(dy, dx);
    }
  }
  return best_angle;
}

// Signed area (shoelace) of a closed loop.
double signedArea(const std::vector<Point2D>& loop) {
  double a = 0.0;
  for (size_t i = 0; i + 1 < loop.size(); ++i) {
    a += loop[i].first * loop[i + 1].second - loop[i + 1].first * loop[i].second;
  }
  return 0.5 * a;
}

}  // namespace

f2c::types::LinearRing makeCleanRing(const std::vector<Point2D>& raw) {
  std::vector<Point2D> pts = repairSelfTouch(dedupClosedRing(raw));
  f2c::types::LinearRing ring;
  for (const auto& p : pts) {
    ring.addPoint(p.first, p.second);
  }
  return ring;
}

f2c::types::Cell makeFieldCell(const std::vector<Point2D>& boundary,
                               const std::vector<std::vector<Point2D>>& holes) {
  f2c::types::Cell cell(makeCleanRing(boundary));
  for (const auto& hole : holes) {
    if (hole.size() >= 3) {
      cell.addRing(makeCleanRing(hole));
    }
  }
  return cell;
}

CoveragePlan planBoustrophedon(const f2c::types::Cell& field,
                               double op_width,
                               double headland_width,
                               int headland_passes,
                               double border_inset,
                               double mow_angle_rad,
                               double min_swath_length) {
  CoveragePlan plan;
  if (op_width <= 0.0) {
    plan.drops.push_back("op_width <= 0");
    return plan;
  }

  // Field area from the clean rings (exterior minus holes) — the denominator
  // of planned_fraction.
  double field_area = 0.0;
  {
    const f2c::types::LinearRing exterior = field.getGeometry(0);
    field_area = std::abs(signedArea(ringToLoop(exterior)));
    for (size_t i = 1; i < field.size(); ++i) {
      field_area -= std::abs(signedArea(ringToLoop(field.getGeometry(i))));
    }
  }

  // Headland ring count, resolved first: the boundary offset, the headland
  // rings and the mainland all depend on it. Three-way contract on
  // headland_passes: <0 none (swaths mow to the boundary), ==0 auto
  // (ceil(headland_width / op_width), floored at 1), >0 forced.
  const int n_rings =
      (headland_passes < 0)
          ? 0
          : ((headland_passes > 0)
                 ? headland_passes
                 : std::max(1, static_cast<int>(
                               std::ceil(headland_width / op_width - 1e-9))));

  f2c::hg::ConstHL hl;
  f2c::sg::BruteForce bf;
  bf.setStepAngle(kAutoAngleStepRad);
  f2c::rp::BoustrophedonOrder order;

  f2c::types::Cells field_cells;
  field_cells.addGeometry(field);

  // Boundary offset so the OUTERMOST DRIVEN PASS's centerline sits
  // `border_inset` inside the recorded line. The -op_width/2 is not cosmetic:
  // F2C 2.1.0 places the outermost driven pass half a swath inside the
  // planning cell in BOTH generators, so without the term every pass sits
  // op_width/2 too deep and "mow to the edge" undercuts by ~8 cm at shipped
  // defaults.
  f2c::types::Cells safe_cells =
      hl.generateHeadlands(field_cells, border_inset - op_width / 2.0);

  // Headland rings, outermost pass first (dir_out2in). v2.1.0 returns
  // std::vector<F2CCells> — one Cells per pass, rings as its cells — so the
  // rings come back directly (v3 returns chained 2-point segments that must
  // be re-stitched; do not port that).
  if (n_rings > 0 && safe_cells.size() > 0) {
    std::vector<f2c::types::Cells> passes =
        hl.generateHeadlandSwaths(safe_cells, op_width, n_rings,
                                  /*dir_out2in=*/true);
    for (const auto& pass : passes) {
      for (size_t i = 0; i < pass.size(); ++i) {
        const f2c::types::LinearRing ring =
            pass.getGeometry(i).getGeometry(0);  // cell's exterior ring
        plan.rings.push_back(rotateToLongestEdgeMid(ringToLoop(ring)));
      }
    }
  }

  // Mainland: the field left inside the headland rings. Skipped entirely when
  // n_rings == 0 — never call buffer(-0.0): upstream that is a real buffer
  // pass, not a no-op, and it can re-node the polygon and drop marginal
  // parts.
  f2c::types::Cells mainland;
  if (n_rings > 0 && safe_cells.size() > 0) {
    mainland = hl.generateHeadlands(safe_cells, n_rings * op_width);
  } else {
    mainland = safe_cells;
  }

  // Straight serpentine swaths per mainland cell.
  double swath_strip_area = 0.0;
  for (size_t i = 0; i < mainland.size(); ++i) {
    const f2c::types::Cell cell = mainland.getGeometry(i);
    f2c::types::Swaths sw;
    if (mow_angle_rad >= 0.0) {
      plan.swath_angle_rad = mow_angle_rad;
      sw = bf.generateSwaths(mow_angle_rad, op_width, cell);
    } else if (std::abs(cell.area()) > kAutoAngleMaxAreaM2) {
      plan.swath_angle_rad = longestEdgeAngle(cell);
      sw = bf.generateSwaths(plan.swath_angle_rad, op_width, cell);
    } else {
      sw = bf.generateBestSwaths(f2c::obj::NSwath(), op_width, cell);
    }
    f2c::types::Swaths sorted = order.genSortedSwaths(sw);
    for (size_t s = 0; s < sorted.size(); ++s) {
      const f2c::types::Swath& swath = sorted[s];
      const f2c::types::LineString line = swath.getPath();
      if (line.size() < 2) {
        plan.drops.push_back("dropped swath with < 2 points");
        continue;
      }
      const f2c::types::Point p0 = line.getGeometry(0);
      const f2c::types::Point p1 = line.getGeometry(line.size() - 1);
      const double len =
          std::hypot(p1.getX() - p0.getX(), p1.getY() - p0.getY());
      if (len < min_swath_length) {
        plan.drops.push_back(fmtDrop("swath len=", len, "<", min_swath_length));
        continue;
      }
      plan.swaths.push_back({{p0.getX(), p0.getY()}, {p1.getX(), p1.getY()}});
      swath_strip_area += len * op_width;
    }
  }

  // For auto plans, report the angle actually used (from the first swath).
  if (mow_angle_rad < 0.0 && !plan.swaths.empty()) {
    const auto& s = plan.swaths.front();
    plan.swath_angle_rad =
        std::atan2(s.second.second - s.second.first, s.second.first - s.first.first);
  }

  // Report. Silent coverage loss is the failure mode worth designing
  // against, so the fraction and the drop list always ship.
  double ring_strip_area = 0.0;
  for (const auto& ring : plan.rings) {
    ring_strip_area += ringLength(ring);
  }
  ring_strip_area *= op_width;
  if (field_area > 0.0) {
    plan.planned_fraction =
        std::min(1.0, (swath_strip_area + ring_strip_area) / field_area);
  }
  if (plan.swaths.empty() && plan.rings.empty()) {
    plan.drops.push_back("empty plan");
  }
  return plan;
}

}  // namespace mower_coverage
