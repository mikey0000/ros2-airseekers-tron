#include "mower_coverage/coverage_planning.hpp"

#include <ogr_geometry.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
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

// Swath clip pieces closer than this along the swath axis are one segment.
constexpr double kSwathMergeTolM = 1e-6;

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
  // An already-valid ring is returned untouched: Buffer(0.0) would re-orient
  // and re-start it (GEOS normalises to clockwise from a different vertex),
  // discarding the operator's recorded vertex order, which the longest-edge
  // AUTO angle relies on for a deterministic tie-break.
  if (poly.IsValid()) {
    return ring;
  }
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
        const double a = part->get_Area();
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
// op_width gap. With no swaths to aim at, start the first loop mid-longest-
// edge: the junction then happens mid-straight, which a diff-drive pivot
// tracks cleanly. (With swaths, rotateLoopToPoint chains the ring starts back
// from the first swath instead.)
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
// along +x) for a degenerate ring. Call it on the RECORDED field, not on a
// buffered planning cell: GEOS re-orients and re-starts buffered rings (a
// dilated square comes back clockwise starting on a vertical edge), so on a
// tie the "longest" edge would be a GEOS artifact rather than the operator's
// first edge. Ties keep the first edge in recorded order.
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

// Clip one swath (p0 -> p1) to `drivable`, returning the inside pieces in
// drive order (projected along p0 -> p1), each oriented p0 -> p1. Pieces that
// touch end-to-end (GEOS splits a line lying ON a boundary edge at every
// vertex) are merged back into one segment. If the clip comes back empty —
// numerical noise on a swath that lies exactly on the recorded line — the
// swath is returned unclipped rather than silently dropping coverage.
std::vector<std::pair<Point2D, Point2D>> clipSwathToDrivable(
    const f2c::types::Cells& drivable, const Point2D& p0, const Point2D& p1) {
  const double dx = p1.first - p0.first;
  const double dy = p1.second - p0.second;
  const double len = std::hypot(dx, dy);
  if (len < 1e-12) {
    return {};
  }
  const double ux = dx / len;
  const double uy = dy / len;
  auto proj = [&](double x, double y) {
    return (x - p0.first) * ux + (y - p0.second) * uy;
  };

  f2c::types::LineString line;
  line.addPoint(p0.first, p0.second);
  line.addPoint(p1.first, p1.second);
  const f2c::types::MultiLineString inside = drivable.getLinesInside(line);

  // Each piece as a [t_start, t_end] interval along the swath axis.
  std::vector<std::pair<double, double>> spans;
  for (size_t i = 0; i < inside.size(); ++i) {
    const f2c::types::LineString piece = inside.getGeometry(i);
    if (piece.size() < 2) {
      continue;
    }
    double t_lo = std::numeric_limits<double>::max();
    double t_hi = std::numeric_limits<double>::lowest();
    for (size_t k = 0; k < piece.size(); ++k) {
      const f2c::types::Point q = piece.getGeometry(k);
      const double t = proj(q.getX(), q.getY());
      t_lo = std::min(t_lo, t);
      t_hi = std::max(t_hi, t);
    }
    spans.emplace_back(std::max(0.0, t_lo), std::min(len, t_hi));
  }
  if (spans.empty()) {
    return {{p0, p1}};
  }
  std::sort(spans.begin(), spans.end());
  std::vector<std::pair<double, double>> merged;
  for (const auto& sp : spans) {
    if (!merged.empty() && sp.first <= merged.back().second + kSwathMergeTolM) {
      merged.back().second = std::max(merged.back().second, sp.second);
    } else {
      merged.push_back(sp);
    }
  }
  std::vector<std::pair<Point2D, Point2D>> out;
  out.reserve(merged.size());
  for (const auto& sp : merged) {
    if (sp.second - sp.first < 1e-9) {
      continue;
    }
    out.push_back({{p0.first + sp.first * ux, p0.second + sp.first * uy},
                   {p0.first + sp.second * ux, p0.second + sp.second * uy}});
  }
  return out;
}

// Signed area (shoelace) of a closed loop.
double signedArea(const std::vector<Point2D>& loop) {
  double a = 0.0;
  for (size_t i = 0; i + 1 < loop.size(); ++i) {
    a += loop[i].first * loop[i + 1].second - loop[i + 1].first * loop[i].second;
  }
  return 0.5 * a;
}

// ---------------------------------------------------------------------------
// Drive-order optimisation: headland rings and swath pieces.
// ---------------------------------------------------------------------------

// Mirrors the bridge / mission layer's join threshold
// (mowgli_interfaces::coverage_geometry::kSegmentTransitGapM, inclusive): a
// gap above it between one segment's end and the next one's start becomes a
// blade-off Nav2 transit.
constexpr double kTransitGapM = 0.6;
// Cost added per transit when ordering, so the order first minimises the
// NUMBER of transits and only then the metres driven between segments.
constexpr double kTransitPenaltyM = 1000.0;
// A sweep-line piece continues the boustrophedon cell of the single piece it
// overlaps on the previous sweep line only if neither end moves further than
// this along the swath axis. A bigger jump (an L-shaped notch reached
// side-on) would put a long connector into the serpentine; cutting the cell
// there lets the cell ordering pick entries that avoid it.
constexpr double kCellJumpM = 1.0;
// Exact (Held-Karp) cell ordering up to this many cells, greedy above. 10
// cells x 4 entries is ~1.6 M transitions: a few ms.
constexpr size_t kMaxExactCells = 10;
// Sweep lines whose lateral offsets differ by less than this are one line.
constexpr double kSweepLineTolM = 1e-3;
// Snap a ring start onto an existing vertex within this distance, so no
// other pose of the ring sits within the bridge's 1 mm closure tolerance of
// the start (that would end the ring early in the splitter).
constexpr double kRingSnapTolM = 0.005;

using Seg = std::pair<Point2D, Point2D>;

double gapCost(double d) { return d + (d > kTransitGapM ? kTransitPenaltyM : 0.0); }

struct LoopHit {
  size_t edge = 0;  // edge index i: loop[i] -> loop[i + 1]
  double t = 0.0;   // position along the edge, [0, 1]
  Point2D p{0.0, 0.0};
  double d = std::numeric_limits<double>::max();
};

// Closest point on a closed loop (first == last) to q.
LoopHit closestOnLoop(const std::vector<Point2D>& loop, const Point2D& q) {
  LoopHit best;
  for (size_t i = 0; i + 1 < loop.size(); ++i) {
    const Point2D& a = loop[i];
    const Point2D& b = loop[i + 1];
    const double dx = b.first - a.first;
    const double dy = b.second - a.second;
    const double len2 = dx * dx + dy * dy;
    double t = 0.0;
    if (len2 > 1e-18) {
      t = ((q.first - a.first) * dx + (q.second - a.second) * dy) / len2;
      t = std::max(0.0, std::min(1.0, t));
    }
    const Point2D p{a.first + t * dx, a.second + t * dy};
    const double d = dist(p, q);
    if (d < best.d) {
      best = {i, t, p, d};
    }
  }
  return best;
}

double loopToLoopDist(const std::vector<Point2D>& a, const std::vector<Point2D>& b) {
  double best = std::numeric_limits<double>::max();
  for (const auto& p : a) {
    best = std::min(best, closestOnLoop(b, p).d);
  }
  for (const auto& p : b) {
    best = std::min(best, closestOnLoop(a, p).d);
  }
  return best;
}

// Re-start a closed loop (first == last) at its point closest to `target`.
// The new start is inserted on its edge unless it lies within kRingSnapTolM
// of a vertex, in which case that vertex becomes the start. Re-closed.
std::vector<Point2D> rotateLoopToPoint(const std::vector<Point2D>& loop,
                                       const Point2D& target) {
  if (loop.size() < 4) {
    return loop;
  }
  const size_t n = loop.size() - 1;  // open vertex count
  const LoopHit hit = closestOnLoop(loop, target);
  std::vector<Point2D> out;
  out.reserve(loop.size() + 1);
  const Point2D& a = loop[hit.edge];
  const Point2D& b = loop[hit.edge + 1];
  if (dist(hit.p, a) < kRingSnapTolM || dist(hit.p, b) < kRingSnapTolM) {
    const size_t s = (dist(hit.p, a) <= dist(hit.p, b)) ? hit.edge : (hit.edge + 1) % n;
    for (size_t k = 0; k <= n; ++k) {
      out.push_back(loop[(s + k) % n]);
    }
    return out;
  }
  out.push_back(hit.p);
  for (size_t k = 1; k <= n; ++k) {
    out.push_back(loop[(hit.edge + k) % n]);
  }
  out.push_back(hit.p);
  return out;
}

// Ray-casting point-in-polygon on a closed loop.
bool pointInLoop(const Point2D& p, const std::vector<Point2D>& loop) {
  bool inside = false;
  for (size_t i = 0; i + 1 < loop.size(); ++i) {
    const Point2D& a = loop[i];
    const Point2D& b = loop[i + 1];
    if ((a.second > p.second) != (b.second > p.second)) {
      const double x =
          a.first + (p.second - a.second) * (b.first - a.first) / (b.second - a.second);
      if (x > p.first) {
        inside = !inside;
      }
    }
  }
  return inside;
}

// One boustrophedon cell: consecutive sweep lines, one piece each, every
// piece overlapping exactly the piece before it (no split / merge event in
// between). A serpentine over such a cell only ever steps one op_width
// sideways between swaths.
struct SweepCell {
  std::vector<Seg> lines;  // {lo end, hi end} along the swath axis, sweep order
  // Per entry option (bit 0: sweep the lines in reverse, bit 1: start the
  // first line at its hi end): entry / exit point and the cost of the
  // in-cell connectors.
  Point2D entry[4];
  Point2D exit[4];
  double internal[4] = {0.0, 0.0, 0.0, 0.0};
};

std::vector<Seg> cellSwaths(const SweepCell& cell, int opt) {
  const bool reverse = (opt & 1) != 0;
  const bool hi_start = (opt & 2) != 0;
  const size_t k = cell.lines.size();
  std::vector<Seg> out;
  out.reserve(k);
  for (size_t j = 0; j < k; ++j) {
    const Seg& line = cell.lines[reverse ? k - 1 - j : j];
    const bool from_lo = ((j % 2) == 0) != hi_start;
    out.push_back(from_lo ? Seg{line.first, line.second} : Seg{line.second, line.first});
  }
  return out;
}

void finalizeCell(SweepCell& cell) {
  for (int opt = 0; opt < 4; ++opt) {
    const std::vector<Seg> sw = cellSwaths(cell, opt);
    cell.entry[opt] = sw.front().first;
    cell.exit[opt] = sw.back().second;
    double c = 0.0;
    for (size_t j = 1; j < sw.size(); ++j) {
      c += gapCost(dist(sw[j - 1].second, sw[j].first));
    }
    cell.internal[opt] = c;
  }
}

// Group the clipped swath pieces of ONE swath set (one angle) into
// boustrophedon cells. This is the boustrophedon cell decomposition done on
// the pieces themselves: a new cell starts wherever the sweep splits (one
// piece overlaps two on the next line, e.g. at a hole or a notch), merges, or
// jumps (kCellJumpM). Working on the actual pieces, not on a polygon
// decomposition, keeps the swath geometry (spacing, clipping) exactly as
// generated across cell borders.
std::vector<SweepCell> buildSweepCells(const std::vector<Seg>& pieces, double op_width) {
  std::vector<SweepCell> cells;
  if (pieces.empty()) {
    return cells;
  }
  const Seg& ref = pieces.front();
  const double rl = dist(ref.first, ref.second);
  const double ux = (ref.second.first - ref.first.first) / rl;
  const double uy = (ref.second.second - ref.first.second) / rl;
  struct P {
    Seg seg;  // lo -> hi
    double n, tlo, thi;
  };
  std::vector<P> ps;
  ps.reserve(pieces.size());
  for (const auto& s : pieces) {
    const double ta = s.first.first * ux + s.first.second * uy;
    const double tb = s.second.first * ux + s.second.second * uy;
    const double na = -s.first.first * uy + s.first.second * ux;
    const double nb = -s.second.first * uy + s.second.second * ux;
    P p;
    p.n = 0.5 * (na + nb);
    if (ta <= tb) {
      p.seg = s;
      p.tlo = ta;
      p.thi = tb;
    } else {
      p.seg = {s.second, s.first};
      p.tlo = tb;
      p.thi = ta;
    }
    ps.push_back(p);
  }
  std::sort(ps.begin(), ps.end(), [](const P& a, const P& b) {
    return a.n != b.n ? a.n < b.n : a.tlo < b.tlo;
  });
  // Sweep lines (ranks) by lateral offset.
  std::vector<std::vector<size_t>> ranks;
  std::vector<double> rank_n;
  for (size_t i = 0; i < ps.size(); ++i) {
    if (rank_n.empty() || ps[i].n - rank_n.back() > kSweepLineTolM) {
      ranks.emplace_back();
      rank_n.push_back(ps[i].n);
    }
    ranks.back().push_back(i);
  }
  auto overlaps = [&](size_t a, size_t b) {
    return std::min(ps[a].thi, ps[b].thi) - std::max(ps[a].tlo, ps[b].tlo) > 1e-6;
  };
  std::vector<size_t> cell_of(ps.size(), 0);
  for (size_t r = 0; r < ranks.size(); ++r) {
    const bool adjacent = r > 0 && rank_n[r] - rank_n[r - 1] <= 1.5 * op_width;
    for (size_t q : ranks[r]) {
      size_t pred = 0;
      size_t n_pred = 0;
      if (adjacent) {
        for (size_t p : ranks[r - 1]) {
          if (overlaps(p, q)) {
            pred = p;
            ++n_pred;
          }
        }
      }
      bool join = false;
      if (n_pred == 1) {
        size_t n_succ = 0;
        for (size_t q2 : ranks[r]) {
          n_succ += overlaps(pred, q2) ? 1 : 0;
        }
        join = n_succ == 1 && std::abs(ps[q].tlo - ps[pred].tlo) <= kCellJumpM &&
               std::abs(ps[q].thi - ps[pred].thi) <= kCellJumpM;
      }
      if (join) {
        cell_of[q] = cell_of[pred];
      } else {
        cell_of[q] = cells.size();
        cells.emplace_back();
      }
      cells[cell_of[q]].lines.push_back(ps[q].seg);
    }
  }
  for (auto& c : cells) {
    finalizeCell(c);
  }
  return cells;
}

struct CellOrder {
  std::vector<std::pair<size_t, int>> seq;  // (cell, entry option) in drive order
  double cost = 0.0;
};

// Order the cells and pick each one's entry so the whole swath sequence has
// the fewest transits, then the shortest gaps. `start_loop` (the last
// headland ring, nullable) is free to start anywhere, so the first entry is
// charged its distance to the loop. Exact Held-Karp DP for <= kMaxExactCells
// cells; nearest-neighbour above that.
CellOrder orderCells(const std::vector<SweepCell>& cells,
                     const std::vector<Point2D>* start_loop) {
  CellOrder best;
  const size_t m = cells.size();
  if (m == 0) {
    return best;
  }
  const size_t S = 4 * m;
  std::vector<double> start(S, 0.0);
  std::vector<double> internal(S, 0.0);
  for (size_t s = 0; s < S; ++s) {
    const SweepCell& c = cells[s / 4];
    const int opt = static_cast<int>(s % 4);
    internal[s] = c.internal[opt];
    if (start_loop != nullptr) {
      start[s] = gapCost(closestOnLoop(*start_loop, c.entry[opt]).d);
    }
  }
  auto trans = [&](size_t a, size_t b) {
    return gapCost(dist(cells[a / 4].exit[a % 4], cells[b / 4].entry[b % 4]));
  };
  const double inf = std::numeric_limits<double>::infinity();

  if (m <= kMaxExactCells) {
    const size_t full = (size_t{1} << m) - 1;
    std::vector<double> dp((full + 1) * S, inf);
    std::vector<int> parent((full + 1) * S, -1);
    for (size_t s = 0; s < S; ++s) {
      dp[(size_t{1} << (s / 4)) * S + s] = start[s] + internal[s];
    }
    for (size_t mask = 1; mask <= full; ++mask) {
      for (size_t s = 0; s < S; ++s) {
        const double cur = dp[mask * S + s];
        if (cur == inf || !(mask & (size_t{1} << (s / 4)))) {
          continue;
        }
        for (size_t s2 = 0; s2 < S; ++s2) {
          const size_t bit = size_t{1} << (s2 / 4);
          if (mask & bit) {
            continue;
          }
          const double nd = cur + trans(s, s2) + internal[s2];
          const size_t idx = (mask | bit) * S + s2;
          if (nd < dp[idx]) {
            dp[idx] = nd;
            parent[idx] = static_cast<int>(s);
          }
        }
      }
    }
    size_t last = 0;
    double best_cost = inf;
    for (size_t s = 0; s < S; ++s) {
      if (dp[full * S + s] < best_cost) {
        best_cost = dp[full * S + s];
        last = s;
      }
    }
    best.cost = best_cost;
    size_t mask = full;
    int s = static_cast<int>(last);
    while (s >= 0) {
      best.seq.emplace_back(static_cast<size_t>(s) / 4, s % 4);
      const int p = parent[mask * S + static_cast<size_t>(s)];
      mask &= ~(size_t{1} << (static_cast<size_t>(s) / 4));
      s = p;
    }
    std::reverse(best.seq.begin(), best.seq.end());
    return best;
  }

  // Greedy nearest neighbour.
  std::vector<bool> used(m, false);
  int prev = -1;
  for (size_t step = 0; step < m; ++step) {
    size_t pick = 0;
    double pick_cost = inf;
    for (size_t s = 0; s < S; ++s) {
      if (used[s / 4]) {
        continue;
      }
      const double c = (prev < 0 ? start[s] : trans(static_cast<size_t>(prev), s)) + internal[s];
      if (c < pick_cost) {
        pick_cost = c;
        pick = s;
      }
    }
    used[pick / 4] = true;
    best.cost += pick_cost;
    best.seq.emplace_back(pick / 4, static_cast<int>(pick % 4));
    prev = static_cast<int>(pick);
  }
  return best;
}

// A run of concentric headland rings driven back to back: around the outer
// boundary (outermost first) or around one hole (hole edge first, outward).
struct RingGroup {
  std::vector<std::vector<Point2D>> loops;
  bool hole = false;
  size_t last_pass = 0;  // pass index of loops.back()
};

// Spiral: hard cap on the ring count (a 100 m wide field at 0.18 m is ~280).
constexpr int kMaxSpiralRings = 2000;
// Spiral: a cell of an offset pass smaller than this is "collapsed".
constexpr double kCollapsedAreaM2 = 1e-6;
// Spiral: bisection steps for the collapse offset of the last ring.
constexpr int kCollapseBisectSteps = 30;

bool cellsEmpty(const f2c::types::Cells& c) {
  return c.size() == 0 || std::abs(c.area()) < kCollapsedAreaM2;
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

bool parsePathMode(const std::string& name, PathMode* mode) {
  if (name.empty() || name == "zigzag" || name == "cross" || name == "alternate") {
    *mode = PathMode::kZigzag;
    return true;
  }
  if (name == "spiral") {
    *mode = PathMode::kSpiral;
    return true;
  }
  if (name == "contour_only") {
    *mode = PathMode::kContourOnly;
    return true;
  }
  *mode = PathMode::kZigzag;
  return false;
}

CoveragePlan planBoustrophedon(const f2c::types::Cell& field,
                               double op_width,
                               double headland_width,
                               int headland_passes,
                               double border_inset,
                               double mow_angle_rad,
                               double min_swath_length) {
  return planCoverage(field, op_width, headland_width, headland_passes, border_inset,
                      mow_angle_rad, min_swath_length, PathMode::kZigzag,
                      /*edge_first=*/true);
}

CoveragePlan planCoverage(const f2c::types::Cell& field,
                          double op_width,
                          double headland_width,
                          int headland_passes,
                          double border_inset,
                          double mow_angle_rad,
                          double min_swath_length,
                          PathMode mode,
                          bool edge_first) {
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
  int n_rings =
      (headland_passes < 0)
          ? 0
          : ((headland_passes > 0)
                 ? headland_passes
                 : std::max(1, static_cast<int>(
                               std::ceil(headland_width / op_width - 1e-9))));
  if (mode == PathMode::kContourOnly) {
    n_rings = std::max(1, n_rings);  // a contour plan with no contour is empty
  }
  const bool want_swaths = mode == PathMode::kZigzag;

  f2c::hg::ConstHL hl;
  f2c::sg::BruteForce bf;
  bf.setStepAngle(kAutoAngleStepRad);

  f2c::types::Cells field_cells;
  field_cells.addGeometry(field);

  // Boundary offset so the OUTERMOST DRIVEN PASS's centerline sits
  // `border_inset` inside the recorded line. The -op_width/2 is not cosmetic:
  // F2C 2.1.0 places the outermost driven pass half a swath inside the
  // planning cell in BOTH generators, so without the term every pass sits
  // op_width/2 too deep and "mow to the edge" undercuts by ~8 cm at shipped
  // defaults.
  //
  // Holes are obstacles (trees, beds, the dock), so they do NOT get the "mow
  // onto the line" treatment: grow each hole by op_width/2 first, which
  // cancels the -op_width/2 dilation for the holes only. The first ring
  // around a hole then runs border_inset + op_width/2 outside it — the blade
  // edge touches the obstacle edge but never overhangs it — and everything
  // inward (hole rings, mainland, swaths) moves out with it.
  f2c::types::Cells hl_field = field_cells;
  if (field.size() > 1) {
    f2c::types::Cells grown;
    for (size_t i = 1; i < field.size(); ++i) {
      grown.addGeometry(f2c::types::Cell::buffer(
          f2c::types::Cell(field.getGeometry(i)), op_width / 2.0));
    }
    hl_field = f2c::types::Cells(f2c::types::Cell(field.getGeometry(0)))
                   .difference(grown.unionCascaded());
  }
  f2c::types::Cells safe_cells =
      hl.generateHeadlands(hl_field, border_inset - op_width / 2.0);

  // Headland rings: one Cells per pass, outermost pass first, each pass's
  // rings as its cells (see the pass loop below for why we do not call
  // generateHeadlandSwaths). A pass cell's EXTERIOR ring runs along the field
  // boundary; its INTERIOR rings run around the holes: pass k sits
  // border_inset + op_width/2 + (k - 1) * op_width OUTSIDE the recorded hole
  // (the outer pass k sits border_inset + (k - 1) * op_width inside the
  // recorded boundary; see the hole growth above). Rings are
  // grouped into concentric runs (outer: outside-in; hole: hole edge first,
  // outward, so the run ends next to the mainland) by containment against
  // the previous pass, which survives a pass splitting into several cells or
  // a hole ring merging with its neighbour.
  std::vector<RingGroup> groups;
  std::vector<f2c::types::Cells> passes;
  if (mode == PathMode::kSpiral && safe_cells.size() > 0) {
    // Concentric inward rings: pass k is the boundary of the planning cell
    // shrunk by (k + 1/2) * op_width — the same offsets generateHeadlandSwaths
    // uses for headland rings, continued until the cell collapses.
    double prev_d = -0.5 * op_width;
    for (int k = 0; k < kMaxSpiralRings; ++k) {
      const double d = (k + 0.5) * op_width;
      f2c::types::Cells c = hl.generateHeadlands(safe_cells, d);
      if (!cellsEmpty(c)) {
        passes.push_back(c);
        prev_d = d;
        continue;
      }
      // Collapsed between prev_d and d. The centre strip (width up to 2 *
      // (collapse - prev_d) - op_width) is left uncovered by the last ring;
      // add one more ring half a swath outside the collapse offset (its
      // inside is then within reach of the blade) unless the gap is tiny.
      double lo = std::max(prev_d, 0.0);
      double hi = d;
      for (int it = 0; it < kCollapseBisectSteps; ++it) {
        const double mid = 0.5 * (lo + hi);
        if (cellsEmpty(hl.generateHeadlands(safe_cells, mid))) {
          hi = mid;
        } else {
          lo = mid;
        }
      }
      if (lo - prev_d > 0.5 * op_width + 0.02) {
        double f = std::max(lo - 0.5 * op_width, prev_d + 0.1 * op_width);
        f = std::max(1e-3, std::min(f, lo - 0.005));
        f2c::types::Cells last = hl.generateHeadlands(safe_cells, f);
        if (!cellsEmpty(last)) {
          passes.push_back(last);
        }
      }
      break;
    }
  } else if (n_rings > 0 && safe_cells.size() > 0) {
    // Pass k = the planning cell shrunk by (k + 1/2) * op_width, outermost
    // first. This is exactly ConstHL::generateHeadlandSwaths(dir_out2in) in
    // both F2C 2.1 and v3, but v3 returns each pass as loose 2-point line
    // sections (MultiLineString) instead of Cells, which loses the
    // exterior/hole ring split the grouping below needs. Building the passes
    // from generateHeadlands keeps the rings as polygons on both versions.
    for (int k = 0; k < n_rings; ++k) {
      f2c::types::Cells c = hl.generateHeadlands(safe_cells, (k + 0.5) * op_width);
      if (cellsEmpty(c)) {
        break;
      }
      passes.push_back(c);
    }
  }
  {
    for (size_t k = 0; k < passes.size(); ++k) {
      const f2c::types::Cells& pass = passes[k];
      for (size_t i = 0; i < pass.size(); ++i) {
        const f2c::types::Cell pc = pass.getGeometry(i);
        for (size_t r = 0; r < pc.size(); ++r) {
          std::vector<Point2D> loop = ringToLoop(pc.getGeometry(r));
          if (loop.size() < 4) {
            continue;
          }
          const bool hole = r > 0;
          RingGroup* target = nullptr;
          for (auto& g : groups) {
            // Continue a run only from the previous pass (a run that started
            // late, after a split, continues too).
            if (k == 0 || g.hole != hole || g.last_pass != k - 1) {
              continue;
            }
            // Outer pass k lies inside outer pass k-1; hole pass k encloses
            // hole pass k-1.
            const bool nested = hole ? pointInLoop(g.loops.back().front(), loop)
                                     : pointInLoop(loop.front(), g.loops.back());
            if (nested) {
              target = &g;
              break;
            }
          }
          if (target == nullptr) {
            groups.push_back(RingGroup{{}, hole, k});
            target = &groups.back();
          }
          target->loops.push_back(std::move(loop));
          target->last_pass = k;
        }
      }
    }
  }

  // Mainland: the field left inside the headland rings. Skipped entirely when
  // n_rings == 0 — never call buffer(-0.0): upstream that is a real buffer
  // pass, not a no-op, and it can re-node the polygon and drop marginal
  // parts. Holes grow by the same band, so the hole rings' strip is a
  // keep-out for the swaths.
  f2c::types::Cells mainland;
  if (!want_swaths) {
    // spiral / contour_only: rings only, no mainland swaths.
  } else if (n_rings > 0 && safe_cells.size() > 0) {
    mainland = hl.generateHeadlands(safe_cells, n_rings * op_width);
  } else {
    mainland = safe_cells;
  }

  // Drivable region for swath centerlines: the recorded field pulled in by
  // border_inset. BruteForce clips each sweep line to the (dilated) planning
  // cell, so with no headland rings every swath end overshoots the recorded
  // line by op_width/2 - border_inset along the swath axis — and in a concave
  // field it runs that far into the notch. The dilation is only there to put
  // the outermost pass laterally ON the line; the ends must not leave the
  // field, so clip every swath back to it. With rings the mainland is already
  // well inside this region and the clip is a no-op. No buffer when
  // border_inset <= 0 (never call buffer(-0.0), see the mainland note).
  // A degenerate (zero-area / invalid) region cannot be clipped against —
  // GEOS raises a TopologyException and F2C then dereferences a null result —
  // so such a field keeps its unclipped swaths and says so in `drops`.
  const f2c::types::Cells drivable =
      (border_inset > 0.0) ? hl.generateHeadlands(field_cells, border_inset)
                           : field_cells;
  const bool clip_swaths = drivable.size() > 0 && drivable.area() > 1e-9 &&
                           drivable.get() != nullptr && drivable.get()->IsValid();
  if (!clip_swaths) {
    plan.drops.push_back("drivable region degenerate; swath ends not clipped");
  }

  // Straight swaths per mainland cell, clipped to the drivable region, then
  // grouped into boustrophedon cells (one swath set = one angle per mainland
  // cell, so cells never mix angles).
  double swath_strip_area = 0.0;
  std::vector<SweepCell> sweep_cells;
  for (size_t i = 0; i < mainland.size(); ++i) {
    const f2c::types::Cell cell = mainland.getGeometry(i);
    f2c::types::Swaths sw;
    if (mow_angle_rad >= 0.0) {
      plan.swath_angle_rad = mow_angle_rad;
      sw = bf.generateSwaths(mow_angle_rad, op_width, cell);
    } else if (std::abs(cell.area()) > kAutoAngleMaxAreaM2) {
      plan.swath_angle_rad = longestEdgeAngle(field);
      sw = bf.generateSwaths(plan.swath_angle_rad, op_width, cell);
    } else {
      f2c::obj::NSwath n_swath_objective;  // v2.x: non-const ref; v3: const ref
      sw = bf.generateBestSwaths(n_swath_objective, op_width, cell);
    }
    std::vector<Seg> pieces;
    for (size_t s = 0; s < sw.size(); ++s) {
      const f2c::types::LineString line = sw[s].getPath();
      if (line.size() < 2) {
        plan.drops.push_back("dropped swath with < 2 points");
        continue;
      }
      const f2c::types::Point p0 = line.getGeometry(0);
      const f2c::types::Point p1 = line.getGeometry(line.size() - 1);
      const Point2D a{p0.getX(), p0.getY()};
      const Point2D b{p1.getX(), p1.getY()};
      const std::vector<Seg> clipped =
          clip_swaths ? clipSwathToDrivable(drivable, a, b) : std::vector<Seg>{{a, b}};
      for (const auto& piece : clipped) {
        const double len = dist(piece.first, piece.second);
        if (len < min_swath_length) {
          plan.drops.push_back(fmtDrop("swath len=", len, "<", min_swath_length));
          continue;
        }
        pieces.push_back(piece);
        swath_strip_area += len * op_width;
      }
    }
    std::vector<SweepCell> cs = buildSweepCells(pieces, op_width);
    sweep_cells.insert(sweep_cells.end(), cs.begin(), cs.end());
  }

  // Drive order. F2C 2.1's BoustrophedonOrder sorts ALL pieces of a swath
  // set by sweep line, so on a concave field or around a hole it alternates
  // between the pieces of every line and each step is a transit. Instead:
  //   1. order the boustrophedon cells and pick each one's entry corner
  //      (orderCells: fewest transits, then shortest gaps);
  //   2. serpentine inside each cell;
  //   3. the outermost outer ring run goes first; when there are hole ring
  //      runs, the one whose last ring lets the swaths start closest goes
  //      last, the others in nearest-neighbour order in between;
  //   4. chain the ring starts BACKWARDS from the first swath: the last ring
  //      starts (and so ends) at its point closest to the first swath start,
  //      each earlier ring at its point closest to the next ring's start.
  //      Concentric rings then hand over with a one-op_width sideways step
  //      and the last ring hands over to the first swath without a transit.
  std::vector<size_t> group_order;
  CellOrder swath_order;
  if (groups.empty()) {
    swath_order = orderCells(sweep_cells, nullptr);
  } else {
    size_t last = 0;
    if (groups.size() == 1) {
      swath_order = orderCells(sweep_cells, &groups[0].loops.back());
    } else {
      double best = std::numeric_limits<double>::infinity();
      for (size_t g = 1; g < groups.size(); ++g) {
        CellOrder co = orderCells(sweep_cells, &groups[g].loops.back());
        if (co.cost < best) {
          best = co.cost;
          swath_order = std::move(co);
          last = g;
        }
      }
    }
    group_order.push_back(0);
    std::vector<bool> used(groups.size(), false);
    used[0] = true;
    used[last] = true;
    for (size_t step = 1; step + 1 < groups.size(); ++step) {
      const std::vector<Point2D>& from = groups[group_order.back()].loops.back();
      size_t pick = 0;
      double pick_d = std::numeric_limits<double>::infinity();
      for (size_t g = 0; g < groups.size(); ++g) {
        if (used[g]) {
          continue;
        }
        const double d = loopToLoopDist(from, groups[g].loops.front());
        if (d < pick_d) {
          pick_d = d;
          pick = g;
        }
      }
      used[pick] = true;
      group_order.push_back(pick);
    }
    if (last != 0) {
      group_order.push_back(last);
    }
  }

  for (const auto& [c, opt] : swath_order.seq) {
    for (const Seg& s : cellSwaths(sweep_cells[c], opt)) {
      plan.swaths.push_back(s);
    }
  }

  for (size_t g : group_order) {
    for (const auto& loop : groups[g].loops) {
      plan.rings.push_back(loop);
    }
  }
  if (!plan.rings.empty() && !plan.swaths.empty() && !edge_first) {
    // Swaths first, then the rings innermost first: the innermost ring
    // borders the mainland, so it starts next to the last swath end and each
    // later ring next to the previous ring's end (one op_width step out).
    std::reverse(plan.rings.begin(), plan.rings.end());
    plan.swaths_first = true;
    Point2D target = plan.swaths.back().second;
    for (auto& ring : plan.rings) {
      ring = rotateLoopToPoint(ring, target);
      target = ring.back();
    }
  } else if (!plan.rings.empty()) {
    if (!plan.swaths.empty()) {
      Point2D target = plan.swaths.front().first;
      for (size_t r = plan.rings.size(); r-- > 0;) {
        plan.rings[r] = rotateLoopToPoint(plan.rings[r], target);
        target = plan.rings[r].front();
      }
    } else {
      // Rings only: start mid-longest-edge (a ring->ring junction mid-
      // straight, not at a corner), then chain forwards.
      plan.rings[0] = rotateToLongestEdgeMid(plan.rings[0]);
      for (size_t r = 1; r < plan.rings.size(); ++r) {
        plan.rings[r] = rotateLoopToPoint(plan.rings[r], plan.rings[r - 1].back());
      }
    }
  }

  // For auto plans, report the angle actually used (from the first swath).
  if (mow_angle_rad < 0.0 && !plan.swaths.empty()) {
    const auto& s = plan.swaths.front();
    plan.swath_angle_rad =
        std::atan2(s.second.second - s.first.second, s.second.first - s.first.first);
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

bool builtWithF2CV3() {
#ifdef MOWER_COVERAGE_F2C_V3
  return true;
#else
  return false;
#endif
}

std::vector<Point2D> planTurn(const Point2D& from, double from_yaw,
                              const Point2D& to, double to_yaw,
                              double robot_width, double min_turn_radius) {
  std::vector<Point2D> out;
#ifdef MOWER_COVERAGE_F2C_V3
  if (!(min_turn_radius > 0.0) || !(robot_width > 0.0)) {
    return out;
  }
  try {
    f2c::types::Robot robot(robot_width, robot_width);
    robot.setMinTurningRadius(min_turn_radius);
    f2c::pp::DubinsCurves dubins;
    f2c::types::Path turn = dubins.createTurn(
        robot, f2c::types::Point(from.first, from.second), from_yaw,
        f2c::types::Point(to.first, to.second), to_yaw);
    if (turn.size() == 0) {
      return out;
    }
    turn.discretize(0.05);
    for (const auto& st : turn.getStates()) {
      const Point2D p{st.point.getX(), st.point.getY()};
      if (dist(p, from) < 1e-3 || dist(p, to) < 1e-3) {
        continue;
      }
      out.push_back(p);
    }
  } catch (const std::exception&) {
    out.clear();
  }
#else
  (void)from; (void)from_yaw; (void)to; (void)to_yaw;
  (void)robot_width; (void)min_turn_radius;
#endif
  return out;
}

}  // namespace mower_coverage
