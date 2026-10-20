// Wheel-floor feasibility of a diff-drive (v, w) command (Airseekers Tron, 2026-10-15).
//
// Pure math, no ROS: shared by the MPPI WheelFloorCritic and its unit tests.
//
// The drive (docs/DESIGN_path_conditioning_2026-10.md section 2): track b = 0.40 m (the MCU's
// wheel split; physical tyre track 0.39, 2026-10-20; was 0.48), a wheel
// asked for less than ~0.06 m/s while the robot turns does not move (MCU / turf floor). With
// h = b / 2 and inner = |v| - |w| h (the slower wheel, signed in the direction of travel):
//
//   * straight (|w| <= straight_wz): both wheels equal, any speed drives;
//   * arc:   inner >= +floor (both wheels forward, the inner one above the floor);
//   * pivot: inner <= -floor (both wheels counter-rotate above the floor; at v = 0 this is
//            |w| >= floor / h = 0.3 rad/s, = the MCU clamp);
//   * anything in between (0 < |inner| < floor) leaves one wheel under the floor: the robot
//     does not do what was asked, and cmd_vel_slew's turn shaper rewrites the command
//     (widens it, or snaps it to a 0.3 rad/s pivot).
//
// violation() is the distance (m/s of wheel speed) from the nearest drivable band, 0 when
// drivable. It mirrors cmd_vel_slew.both_wheels_legal(target=True) so that a command with
// zero violation passes the shaper unchanged.
#ifndef MOWGLI_NAV2_PLUGINS__WHEEL_FLOOR_HPP_
#define MOWGLI_NAV2_PLUGINS__WHEEL_FLOOR_HPP_

#include <algorithm>
#include <cmath>

namespace mowgli_nav2_plugins::wheel_floor
{

struct Params
{
  double half_track{0.20};       // b / 2, m
  double floor{0.06};            // m/s, slowest wheel speed that moves while turning
  double straight_wz{0.0};       // rad/s, |w| at or below this counts as straight
  double stop_speed{0.02};       // m/s, both wheels below this = a stop, not penalised
  bool allow_pivots{true};       // false: counter-rotation is penalised too (arcs only)
};

// Distance from the drivable set in wheel speed (m/s); 0 = drivable.
inline double violation(double v, double w, const Params & p)
{
  const double av = std::fabs(v);
  const double aw = std::fabs(w);
  if (aw <= p.straight_wz) {
    return 0.0;                                    // straight
  }
  if (av + aw * p.half_track < p.stop_speed) {
    return 0.0;                                    // (nearly) stopped
  }
  const double inner = av - aw * p.half_track;
  if (inner >= p.floor) {
    return 0.0;                                    // arc, inner wheel above the floor
  }
  if (!p.allow_pivots) {
    return p.floor - inner;                        // only arcs count as drivable
  }
  if (inner <= -p.floor) {
    return 0.0;                                    // proper counter-rotation / pivot
  }
  return std::min(p.floor - inner, inner + p.floor);
}

inline bool drivable(double v, double w, const Params & p)
{
  return violation(v, w, p) <= 1e-9;
}

}  // namespace mowgli_nav2_plugins::wheel_floor

#endif  // MOWGLI_NAV2_PLUGINS__WHEEL_FLOOR_HPP_
