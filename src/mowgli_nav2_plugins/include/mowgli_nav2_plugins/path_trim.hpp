// Path trim for PathAlignAheadCritic (2026-11-03, slope sim): MPPI has no integral action,
// so a constant lateral disturbance (downhill creep on a side slope, 1 cm/s) becomes a
// constant cross-track offset. The trim shifts the alignment target sideways (upslope):
//
//   * integral: a world-frame offset o, updated each control cycle by
//     o -= e_n * dt / trim_tau_s, where e_n is the robot's signed perpendicular distance
//     from the (unshifted) path; |o| <= trim_max_m. Steady state: e_n = 0. It only winds
//     while the robot is on the path (|e_n| <= trim_max_err_m) and moving
//     (|v| >= trim_min_speed); it leaks with trim_leak_tau_s. A weave averages out over
//     tau, a persistent offset does not. World frame, so the next swath (driven the other
//     way) keeps the same upslope direction.
//   * feed-forward: ff = k_roll x sin(roll) x |v| toward the robot's left (REP-103 roll > 0
//     = right side down = downhill to the right), 0 without a fresh roll.
//
// The applied shift of each path point is the component of (o + ff) along that point's
// path normal. Flat ground: roll 0, no persistent error -> o ~ 0, the critic is unchanged.
// The caller may wind with another time constant than tau_s (PathAlignAheadCritic's side-slope
// values, path_align_ahead_critic.hpp tuningAt).
#ifndef MOWGLI_NAV2_PLUGINS__PATH_TRIM_HPP_
#define MOWGLI_NAV2_PLUGINS__PATH_TRIM_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace mowgli_nav2_plugins::path_trim
{

struct Params
{
  bool enabled{false};
  float tau_s{5.0f};
  float max_m{0.10f};
  float max_err_m{0.30f};
  float min_speed{0.05f};
  float leak_tau_s{60.0f};
  float k_roll{0.0f};
};

struct State
{
  float ox{0.0f};
  float oy{0.0f};
};

struct CrossTrack
{
  bool ok{false};
  float ex{0.0f};    // robot - foot point, perpendicular to the path (world frame)
  float ey{0.0f};
  float signed_m{0.0f};   // + = left of the path
  float tx{1.0f};    // unit tangent at the foot point
  float ty{0.0f};
};

// Perpendicular offset of (x, y) from the polyline px/py (first n points).
template<class Arr>
CrossTrack crossTrack(const Arr & px, const Arr & py, size_t n, float x, float y)
{
  CrossTrack best;
  float best_d2 = 1e30f;
  for (size_t i = 1; i < n; ++i) {
    const float ax = px(i - 1), ay = py(i - 1);
    const float dx = px(i) - ax, dy = py(i) - ay;
    const float l2 = dx * dx + dy * dy;
    if (l2 < 1e-8f) {continue;}
    const float u = std::clamp(((x - ax) * dx + (y - ay) * dy) / l2, 0.0f, 1.0f);
    const float fx = ax + u * dx, fy = ay + u * dy;
    const float d2 = (x - fx) * (x - fx) + (y - fy) * (y - fy);
    if (d2 < best_d2) {
      best_d2 = d2;
      const float l = std::sqrt(l2);
      best.ok = true;
      best.tx = dx / l;
      best.ty = dy / l;
      // the perpendicular part only (the clamped ends add an along-track part)
      const float rx = x - fx, ry = y - fy;
      best.signed_m = -best.ty * rx + best.tx * ry;
      best.ex = -best.ty * best.signed_m;
      best.ey = best.tx * best.signed_m;
    }
  }
  return best;
}

// One control cycle of the integral, winding with time constant tau_s.
inline void update(
  State & s, const CrossTrack & ct, float v, float dt, const Params & p, float tau_s)
{
  if (!p.enabled || dt <= 0.0f) {return;}
  if (p.leak_tau_s > 0.0f) {
    const float k = std::exp(-dt / p.leak_tau_s);
    s.ox *= k;
    s.oy *= k;
  }
  if (ct.ok && std::fabs(ct.signed_m) <= p.max_err_m && std::fabs(v) >= p.min_speed &&
    tau_s > 0.0f)
  {
    const float g = dt / tau_s;
    s.ox -= ct.ex * g;
    s.oy -= ct.ey * g;
  }
  const float m = std::hypot(s.ox, s.oy);
  if (m > p.max_m && m > 0.0f) {
    s.ox *= p.max_m / m;
    s.oy *= p.max_m / m;
  }
}

// One control cycle of the integral with tau_s.
inline void update(State & s, const CrossTrack & ct, float v, float dt, const Params & p)
{
  update(s, ct, v, dt, p, p.tau_s);
}

// Feed-forward toward the robot's left (world frame) for roll (rad), yaw (rad), speed v.
inline void feedForward(float roll, float yaw, float v, const Params & p, float & fx, float & fy)
{
  const float b = p.enabled ? p.k_roll * std::sin(roll) * std::fabs(v) : 0.0f;
  fx = -std::sin(yaw) * b;
  fy = std::cos(yaw) * b;
}

// Shift each of the first n points by the component of (sx, sy) along its path normal.
template<class Arr>
void shiftPath(Arr & px, Arr & py, size_t n, float sx, float sy)
{
  if (n < 2 || (sx == 0.0f && sy == 0.0f)) {return;}
  std::vector<float> dx(n), dy(n);
  for (size_t i = 0; i < n; ++i) {
    const size_t a = i == 0 ? 0 : i - 1;
    const size_t b = i + 1 < n ? i + 1 : n - 1;
    float tx = px(b) - px(a), ty = py(b) - py(a);
    const float l = std::hypot(tx, ty);
    if (l < 1e-6f) {dx[i] = dy[i] = 0.0f; continue;}
    tx /= l;
    ty /= l;
    const float k = -ty * sx + tx * sy;     // along the left normal (-ty, tx)
    dx[i] = -ty * k;
    dy[i] = tx * k;
  }
  for (size_t i = 0; i < n; ++i) {
    px(i) += dx[i];
    py(i) += dy[i];
  }
}

}  // namespace mowgli_nav2_plugins::path_trim

#endif  // MOWGLI_NAV2_PLUGINS__PATH_TRIM_HPP_
