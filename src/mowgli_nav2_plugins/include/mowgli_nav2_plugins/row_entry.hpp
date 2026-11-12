// Row entry speed hold for RowEntryCritic (2026-11-12 row straightness review,
// docs/analysis/2026-11-12_row_straightness.md).
//
// Field 2026-11-12 (area 4, 63 rows): the first 1.5 m of a row is 3.3 cm rms against 1.65
// settled. The coverage pivot hands over with the robot 8.3 deg rms off the row (mean +5.3 deg
// on one direction, -2.1 on the other: the pivot ends short / long by turn direction, and its
// join heading aims at a point 0.4 m ahead on the row, so a few cm of lateral offset after the
// pivot already mean several degrees), and MPPI is at 0.20 m/s within a few decimetres. The
// heading error then becomes a lateral swing that takes ~3 m to settle.
//
// The hold: when a new path (a new goal) starts with a straight row of at least row_min_m
// ahead, forward speed is capped at entry_speed until the robot heading is within heading_tol
// of the row; then the cap rises at ramp_accel until release_speed (then no cap). It ends
// anyway after max_entry_m from where the path started. A path whose start is already
// aligned, a bend, a short turn leg: no cap. The hold acts once per path: once released it
// does not come back.
#ifndef MOWGLI_NAV2_PLUGINS__ROW_ENTRY_HPP_
#define MOWGLI_NAV2_PLUGINS__ROW_ENTRY_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace mowgli_nav2_plugins::row_entry
{

struct Params
{
  float entry_speed{0.10f};        // m/s while the heading is off the row
  float heading_tol{0.045f};       // rad (2.6 deg): aligned
  float ramp_accel{0.10f};         // m/s^2: cap rise once aligned
  float release_speed{0.30f};      // m/s: cap at or above this = hold over
  float max_entry_m{3.0f};         // m from the path start: hold over regardless
  float row_min_m{1.0f};           // m of straight path ahead that make a row (< the
                                   // controller's prune_distance: MPPI's path ends there)
  float row_straight_tol{0.10f};   // rad: max segment heading off the row chord
  float new_goal_m{0.25f};         // m: goal jumped this far in one cycle = a new path
};

enum class Phase { kNone, kHold, kRamp };

struct State
{
  Phase phase{Phase::kNone};
  bool have_goal{false};
  float gx{0.0f}, gy{0.0f};        // goal at the previous cycle
  float sx{0.0f}, sy{0.0f};        // robot position when it started
  double t_aligned{0.0};
};

inline float wrap(float a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

// Heading of the straight row the path (first n points) starts with: the chord from point 0
// to the point row_min_m along it. False when the path is shorter than row_min_m or any
// segment of that stretch (>= 1 cm) is more than row_straight_tol off the chord.
template<class Arr>
bool rowHeading(const Arr & px, const Arr & py, size_t n, const Params & p, float & heading)
{
  if (n < 2) {return false;}
  float s = 0.0f;
  size_t end = 0;
  for (size_t i = 1; i < n; ++i) {
    s += std::hypot(px(i) - px(i - 1), py(i) - py(i - 1));
    if (s >= p.row_min_m) {end = i; break;}
  }
  if (end == 0) {return false;}
  const float h = std::atan2(py(end) - py(0), px(end) - px(0));
  for (size_t i = 1; i <= end; ++i) {
    const float dx = px(i) - px(i - 1), dy = py(i) - py(i - 1);
    if (dx * dx + dy * dy < 1e-4f) {continue;}
    if (std::fabs(wrap(std::atan2(dy, dx) - h)) > p.row_straight_tol) {return false;}
  }
  heading = h;
  return true;
}

// One control cycle at time now (s) for the robot at (x, y, yaw) following a path to goal
// (gx, gy); row / row_heading from rowHeading on that path. Returns the forward speed cap
// (m/s), or a negative value for no cap.
inline float update(
  State & s, const Params & p, double now, float x, float y, float yaw, float gx, float gy,
  bool row, float row_heading)
{
  // A new path is a goal JUMP between consecutive cycles: MPPI hands the goal over in the
  // odom frame, so it creeps with map -> odom (sim 2026-11-12: 5 cm in under a second on the
  // -d -l runs) and a fixed reference would start a "new path" mid-row.
  const bool jump = !s.have_goal || std::hypot(gx - s.gx, gy - s.gy) > p.new_goal_m;
  s.have_goal = true;
  s.gx = gx;
  s.gy = gy;
  if (jump) {
    s.sx = x;
    s.sy = y;
    // already on the row heading (a mid-row re-send, a good pivot): no hold, no ramp
    s.phase = row && std::fabs(wrap(yaw - row_heading)) > p.heading_tol ?
      Phase::kHold : Phase::kNone;
  }
  if (s.phase == Phase::kHold) {
    if (!row || std::hypot(x - s.sx, y - s.sy) > p.max_entry_m) {
      s.phase = Phase::kNone;
    } else if (std::fabs(wrap(yaw - row_heading)) <= p.heading_tol) {
      s.phase = Phase::kRamp;
      s.t_aligned = now;
    } else {
      return p.entry_speed;
    }
  }
  if (s.phase == Phase::kRamp) {
    const float cap = p.entry_speed +
      p.ramp_accel * static_cast<float>(std::max(0.0, now - s.t_aligned));
    if (cap < p.release_speed) {return cap;}
    s.phase = Phase::kNone;
  }
  return -1.0f;
}

// Adds weight * (mean over the horizon of the forward speed above the cap)^power to each
// sample's cost. The cap of step j is cap + ramp * j * dt (ramp = ramp_accel while ramping,
// 0 while holding). cap < 0: nothing. Returns the number of (sample, step) cells over it.
template<class Arr2, class Arr1>
size_t scoreBatch(
  const Arr2 & cvx, Arr1 & costs, float cap, float ramp, float dt, float weight,
  unsigned int power)
{
  if (cap < 0.0f) {return 0;}
  const size_t batch = std::min(static_cast<size_t>(cvx.shape(0)),
      static_cast<size_t>(costs.shape(0)));
  const size_t steps = cvx.shape(1);
  if (steps == 0) {return 0;}
  size_t over = 0;
  for (size_t i = 0; i < batch; ++i) {
    float sum = 0.0f;
    for (size_t j = 0; j < steps; ++j) {
      const float excess = cvx(i, j) - (cap + ramp * dt * static_cast<float>(j));
      if (excess > 0.0f) {
        sum += excess;
        ++over;
      }
    }
    const float c = weight * sum / static_cast<float>(steps);
    costs(i) += power > 1u ? std::pow(c, static_cast<float>(power)) : c;
  }
  return over;
}

}  // namespace mowgli_nav2_plugins::row_entry

#endif  // MOWGLI_NAV2_PLUGINS__ROW_ENTRY_HPP_
