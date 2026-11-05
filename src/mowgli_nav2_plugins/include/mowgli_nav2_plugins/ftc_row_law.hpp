// FTC ("follow the carrot") row control law for FTCRowController (2026-11-05, row weave study,
// docs/analysis/2026-11-05_row_weave_ftc.md).
//
// Ported from OpenMower's ftc_local_planner by Clemens Elflein
// (github.com/ClemensElflein/open_mower_ros, src/lib/ftc_local_planner/src/ftc_planner.cpp,
// commit a69d144): FTCPlanner::update_control_point() and calculate_velocity_commands().
// open_mower_ros is distributed under the GNU General Public License v3; this file is part of
// this GPLv3 project and is distributed under the same licence (see the repository LICENSE).
//
// The law, as upstream:
//   * a carrot (control point) moves along the path at the target speed, accelerating at
//     `acceleration`;
//   * its pose in the robot frame gives lon = x, lat = y, ang = yaw;
//   * v = kp_lon * lon (forward only), w = kp_ang * ang + kd_ang * d(ang)/dt
//     + kp_lat * lat + ki_lat * integral(lat) (clamped to +-ki_lat_max) + kd_lat * d(lat)/dt.
//
// Port changes (why: section 2 of the analysis note):
//   * Rows only. The carrot runs on a straight row; the controller hands bends, path ends and
//     blocked rows to MPPI (FTCRowController). So there is no PRE_ROTATE / POST_ROTATE /
//     WAITING_FOR_GOAL_APPROACH state, no speed_angular carrot limit, no speed_fast /
//     speed_slow choice (the target speed is the coverage MPPI's vx_max, i.e. the cut speed),
//     no lateral_priority gain and no oscillation recovery.
//   * The carrot is kept between the robot's projection and `max_lead` ahead of it (upstream
//     lets it run away and declares a crash at max_follow_distance; here the carrot waits
//     when the robot is slowed downstream, e.g. by the collision monitor's slowdown zone).
//   * Path tangents come from the path geometry, not the pose orientations.
//   * Tron gains: kp_lat 2.5, kp_ang 1.2, kp_lon 1.2 (a 0.25 m carrot lead at 0.3 m/s).
//     Upstream's kp_lat 25 / kp_ang 3 / kd_lat 1.23 go unstable in the row model with this
//     drive's lags (0.2 s command delay, 0.3 rad/s clamp, 0.055 m track lag, 0.1 s GNSS
//     latency, RTK noise; analysis note section 2).
#ifndef MOWGLI_NAV2_PLUGINS__FTC_ROW_LAW_HPP_
#define MOWGLI_NAV2_PLUGINS__FTC_ROW_LAW_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace mowgli_nav2_plugins::ftc_row
{

struct Gains
{
  double kp_lon{1.2};
  double kp_lat{2.5};
  double ki_lat{0.0};
  double ki_lat_max{0.1};
  double kd_lat{0.0};
  double kp_ang{1.2};
  double kd_ang{0.0};
  double acceleration{0.3};   // carrot speed change, m/s^2
};

struct Command
{
  double v{0.0};
  double w{0.0};
};

inline double wrapAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

// A path in the robot pose frame with cumulative arc length.
struct Path
{
  std::vector<double> x, y, s;

  void clear()
  {
    x.clear();
    y.clear();
    s.clear();
  }

  void push(double px, double py)
  {
    if (!x.empty() && std::hypot(px - x.back(), py - y.back()) < 1e-6) {
      return;   // duplicate pose (upstream skips them while moving the carrot)
    }
    s.push_back(x.empty() ? 0.0 : s.back() + std::hypot(px - x.back(), py - y.back()));
    x.push_back(px);
    y.push_back(py);
  }

  size_t size() const {return x.size();}
  double length() const {return s.empty() ? 0.0 : s.back();}
};

struct PathPoint
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};   // tangent
  size_t seg{0};     // segment index (seg -> seg + 1)
};

// Point and tangent at arc length s (clamped to the path).
inline PathPoint pointAt(const Path & p, double s)
{
  PathPoint r;
  if (p.size() < 2) {
    if (p.size() == 1) {
      r.x = p.x[0];
      r.y = p.y[0];
    }
    return r;
  }
  s = std::clamp(s, 0.0, p.length());
  size_t i = static_cast<size_t>(std::upper_bound(p.s.begin(), p.s.end(), s) - p.s.begin());
  i = std::clamp<size_t>(i, 1, p.size() - 1) - 1;
  const double l = p.s[i + 1] - p.s[i];
  const double u = l > 0.0 ? (s - p.s[i]) / l : 0.0;
  r.x = p.x[i] + u * (p.x[i + 1] - p.x[i]);
  r.y = p.y[i] + u * (p.y[i + 1] - p.y[i]);
  r.yaw = std::atan2(p.y[i + 1] - p.y[i], p.x[i + 1] - p.x[i]);
  r.seg = i;
  return r;
}

struct Projection
{
  bool ok{false};
  size_t seg{0};
  double s{0.0};        // arc length of the foot point
  double lateral{0.0};  // robot left of the path (+)
  double heading{0.0};  // robot yaw - path tangent
};

// Foot point of (x, y) on the segments from `seg0` forward, up to `window_m` of arc length
// past the start of seg0. Searching forward only keeps the projection on the current row:
// the neighbouring row lies 0.18 m away but metres further along the path.
inline Projection project(const Path & p, double x, double y, double yaw, size_t seg0,
  double window_m)
{
  Projection best;
  if (p.size() < 2) {
    return best;
  }
  seg0 = std::min(seg0, p.size() - 2);
  double best_d2 = 1e300;
  for (size_t i = seg0; i + 1 < p.size(); ++i) {
    if (p.s[i] - p.s[seg0] > window_m) {
      break;
    }
    const double dx = p.x[i + 1] - p.x[i], dy = p.y[i + 1] - p.y[i];
    const double l2 = dx * dx + dy * dy;
    if (l2 <= 0.0) {
      continue;
    }
    const double u = std::clamp(((x - p.x[i]) * dx + (y - p.y[i]) * dy) / l2, 0.0, 1.0);
    const double fx = p.x[i] + u * dx, fy = p.y[i] + u * dy;
    const double d2 = (x - fx) * (x - fx) + (y - fy) * (y - fy);
    if (d2 < best_d2) {
      best_d2 = d2;
      const double l = std::sqrt(l2);
      best.ok = true;
      best.seg = i;
      best.s = p.s[i] + u * l;
      best.lateral = (-(dy / l) * (x - fx) + (dx / l) * (y - fy));
      best.heading = wrapAngle(yaw - std::atan2(dy, dx));
    }
  }
  return best;
}

// True when the path from arc length s0 to s0 + length is one straight line: every vertex in
// between lies within `tol` of the chord. False when the path ends before s0 + length.
inline bool straightAhead(const Path & p, double s0, double length, double tol)
{
  if (p.size() < 2 || s0 + length > p.length() + 1e-6) {
    return false;
  }
  const PathPoint a = pointAt(p, s0);
  const PathPoint b = pointAt(p, s0 + length);
  const double cx = b.x - a.x, cy = b.y - a.y;
  const double cl = std::hypot(cx, cy);
  if (cl < 0.5 * length) {
    return false;
  }
  for (size_t i = a.seg + 1; i <= b.seg; ++i) {
    const double d = std::fabs(cx * (p.y[i] - a.y) - cy * (p.x[i] - a.x)) / cl;
    if (d > tol) {
      return false;
    }
  }
  return true;
}

// One row-following episode: carrot state, PID memory.
class Law
{
public:
  // Start (or restart) on the row: the carrot at the steady-state lead for the current speed.
  void start(double s_robot, double v_now, const Gains & g)
  {
    speed_ = std::max(0.0, v_now);
    s_carrot_ = s_robot + (g.kp_lon > 0.0 ? speed_ / g.kp_lon : 0.0);
    i_lat_ = 0.0;
    have_last_ = false;
  }

  // Move the carrot (upstream update_control_point, FOLLOWING): its speed ramps towards
  // v_target at g.acceleration; it stays within [s_robot, s_robot + max_lead].
  void advance(double s_robot, double v_target, double max_lead, double dt, const Gains & g)
  {
    if (speed_ < v_target) {
      speed_ = std::min(v_target, speed_ + g.acceleration * dt);
    } else {
      speed_ = std::max(v_target, speed_ - g.acceleration * dt);
    }
    s_carrot_ = std::clamp(s_carrot_ + speed_ * dt, s_robot, s_robot + std::max(0.0, max_lead));
  }

  // Upstream calculate_velocity_commands (FOLLOWING) for the carrot pose in the robot frame.
  Command command(double lon, double lat, double ang, double dt, double v_max, double w_max,
    const Gains & g)
  {
    i_lat_ = std::clamp(i_lat_ + lat * dt, -g.ki_lat_max, g.ki_lat_max);
    double d_lat = 0.0, d_ang = 0.0;
    if (have_last_ && dt > 0.0) {
      d_lat = (lat - last_lat_) / dt;
      d_ang = (ang - last_ang_) / dt;
    }
    last_lat_ = lat;
    last_ang_ = ang;
    have_last_ = true;
    Command c;
    c.v = std::clamp(g.kp_lon * lon, 0.0, std::max(0.0, v_max));   // forward_only
    c.w = std::clamp(g.kp_ang * ang + g.kd_ang * d_ang + g.kp_lat * lat + g.ki_lat * i_lat_ +
        g.kd_lat * d_lat, -w_max, w_max);
    return c;
  }

  // Full step: carrot pose on `path` against the robot pose (x, y, yaw).
  Command step(const Path & path, double x, double y, double yaw, double dt, double v_max,
    double w_max, const Gains & g)
  {
    const PathPoint c = pointAt(path, s_carrot_);
    const double dx = c.x - x, dy = c.y - y;
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    lon_ = cy * dx + sy * dy;
    lat_ = -sy * dx + cy * dy;
    ang_ = wrapAngle(c.yaw - yaw);
    return command(lon_, lat_, ang_, dt, v_max, w_max, g);
  }

  double carrot() const {return s_carrot_;}
  double speed() const {return speed_;}
  double lon() const {return lon_;}
  double lat() const {return lat_;}
  double ang() const {return ang_;}
  double integral() const {return i_lat_;}

private:
  double s_carrot_{0.0};
  double speed_{0.0};
  double i_lat_{0.0};
  double last_lat_{0.0};
  double last_ang_{0.0};
  bool have_last_{false};
  double lon_{0.0};
  double lat_{0.0};
  double ang_{0.0};
};

// When the row controller may drive: on a straight that runs at least `need_m` past the
// robot, close to the line. Entry uses half the exit limits (hysteresis).
struct Gate
{
  double straight_tol_m{0.01};
  double handoff_m{0.4};       // MPPI takes over this far before the row's end / a bend
  double max_lateral_m{0.15};
  double max_angle_rad{0.35};
};

inline bool mayEnter(const Path & p, const Projection & pr, double lead_max, const Gate & g)
{
  return pr.ok && std::fabs(pr.lateral) <= 0.5 * g.max_lateral_m &&
         std::fabs(pr.heading) <= 0.5 * g.max_angle_rad &&
         straightAhead(p, pr.s, g.handoff_m + 2.0 * lead_max, g.straight_tol_m);
}

inline bool mayStay(const Path & p, const Projection & pr, double s_carrot, const Gate & g)
{
  return pr.ok && std::fabs(pr.lateral) <= g.max_lateral_m &&
         std::fabs(pr.heading) <= g.max_angle_rad &&
         straightAhead(p, pr.s, std::max(0.0, s_carrot - pr.s) + g.handoff_m, g.straight_tol_m);
}

}  // namespace mowgli_nav2_plugins::ftc_row

#endif  // MOWGLI_NAV2_PLUGINS__FTC_ROW_LAW_HPP_
