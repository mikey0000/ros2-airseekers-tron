// MPPI critic: Nav2's PathAlignCritic (Jazzy 1.3.13), scored at a control point AHEAD of
// base_link (B13-1, docs/analysis/2026-10-29_localisation_wander.md section 6).
//
// Why: the straight-line weave is the controller's lightly damped lateral mode. PathAlign
// scores each sampled trajectory pose's distance to the path point at the same arc length;
// on the Tron that gives w ~ -4.9 lat - 0.77 yaw (field 10-28), i.e. it effectively tracks a
// point only 0.16 m ahead. The drive's own lag (the ground track lags the heading, the turn
// centre sits ahead of base_link) and the ~0.1 s GNSS fix latency move what the controller
// sees BEHIND the rotation centre, which takes most of that heading damping away. Scoring a
// point control_point_x ahead of each pose (along the trajectory's yaw) against the path
// point control_point_x further along the path adds control_point_x x heading to every
// lateral error: the classic look-ahead control point, which restores the damping.
//
// control_point_x 0.0 is exactly PathAlignCritic (same parameters, same gating: goal
// tolerance, offset_from_furthest, max_path_occupancy_ratio, invalid path points). With
// control_point_x > 0 the path is used up to control_point_x beyond the furthest reached
// point (so the ahead points of the horizon's last poses are not clamped to it).
//
// Jazzy loads critics as "mppi::critics::" + <name in the critics list>; parameters are
// <controller>.PathAlignAheadCritic.*: cost_weight (10.0), cost_power (1),
// max_path_occupancy_ratio (0.07), offset_from_furthest (20), trajectory_point_step (4),
// threshold_to_consider (0.5), use_path_orientations (false), control_point_x (0.0, m).
// Revert: put "PathAlignCritic" back in the critics list (same parameters, without
// control_point_x).
#ifndef MOWGLI_NAV2_PLUGINS__PATH_ALIGN_AHEAD_CRITIC_HPP_
#define MOWGLI_NAV2_PLUGINS__PATH_ALIGN_AHEAD_CRITIC_HPP_

#include <cmath>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
#pragma GCC diagnostic ignored "-Wstringop-overflow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>
#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"
#pragma GCC diagnostic pop

namespace mowgli_nav2_plugins::path_align_ahead
{

struct Params
{
  int trajectory_point_step{4};
  bool use_path_orientations{false};
  float control_point_x{0.0f};
};

// Number of path points the scoring uses: the furthest reached point (PathAlignCritic), plus
// enough points to cover control_point_x beyond it, at most the points with a validity flag.
inline size_t pathPointsUsed(
  const xt::xtensor<float, 1> & px, const xt::xtensor<float, 1> & py, size_t furthest,
  size_t n_valid, float control_point_x)
{
  size_t n = std::min(furthest, n_valid);
  if (control_point_x <= 0.0f) {
    return n;
  }
  float extra = 0.0f;
  while (n < n_valid && n >= 1 && extra < control_point_x) {
    const float dx = px(n) - px(n - 1);
    const float dy = py(n) - py(n - 1);
    extra += std::sqrt(dx * dx + dy * dy);
    ++n;
  }
  return n;
}

// Per-sample cost (before weight / power): the mean distance between each strided trajectory
// pose's control point and the path point at its arc length (+ control_point_x). This is
// PathAlignCritic::score's loop with the control point; costs is resized to the batch.
inline void scoreBatch(
  const xt::xtensor<float, 2> & tx, const xt::xtensor<float, 2> & ty,
  const xt::xtensor<float, 2> & tyaw, const xt::xtensor<float, 1> & px,
  const xt::xtensor<float, 1> & py, const xt::xtensor<float, 1> & pyaw,
  const std::vector<bool> & valid, size_t n_path, const Params & p,
  std::vector<float> & costs)
{
  const size_t batch = tx.shape(0);
  const size_t steps = tx.shape(1);
  costs.assign(batch, 0.0f);
  if (n_path == 0 || steps == 0) {
    return;
  }
  std::vector<float> s(n_path, 0.0f);
  for (size_t i = 1; i < n_path; ++i) {
    const float dx = px(i) - px(i - 1);
    const float dy = py(i) - py(i - 1);
    s[i] = s[i - 1] + std::sqrt(dx * dx + dy * dy);
  }
  const size_t stride = static_cast<size_t>(std::max(1, p.trajectory_point_step));
  const float d = p.control_point_x;
  for (size_t t = 0; t < batch; ++t) {
    float sum = 0.0f;
    unsigned int n = 0u;
    unsigned int path_pt = 0u;
    float traj_s = 0.0f;
    float xm1 = tx(t, 0);
    float ym1 = ty(t, 0);
    for (size_t j = stride; j < steps; j += stride) {
      const float x = tx(t, j);
      const float y = ty(t, j);
      float dx = x - xm1;
      float dy = y - ym1;
      xm1 = x;
      ym1 = y;
      traj_s += std::sqrt(dx * dx + dy * dy);
      path_pt = mppi::utils::findClosestPathPt(s, traj_s + d, path_pt);
      if (!valid[path_pt]) {
        continue;
      }
      const float yaw = tyaw(t, j);
      const float cx = d != 0.0f ? x + d * std::cos(yaw) : x;
      const float cy = d != 0.0f ? y + d * std::sin(yaw) : y;
      dx = px(path_pt) - cx;
      dy = py(path_pt) - cy;
      ++n;
      if (p.use_path_orientations) {
        const float dyaw = static_cast<float>(
          angles::shortest_angular_distance(pyaw(path_pt), yaw));
        sum += std::sqrt(dx * dx + dy * dy + dyaw * dyaw);
      } else {
        sum += std::sqrt(dx * dx + dy * dy);
      }
    }
    costs[t] = n > 0u ? sum / static_cast<float>(n) : 0.0f;
  }
}

}  // namespace mowgli_nav2_plugins::path_align_ahead

namespace mppi::critics
{

class PathAlignAheadCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(CriticData & data) override;

protected:
  mowgli_nav2_plugins::path_align_ahead::Params params_;
  size_t offset_from_furthest_{0};
  float threshold_to_consider_{0};
  float max_path_occupancy_ratio_{0};
  unsigned int power_{0};
  float weight_{0};
  std::vector<float> cost_;
};

}  // namespace mppi::critics

#endif  // MOWGLI_NAV2_PLUGINS__PATH_ALIGN_AHEAD_CRITIC_HPP_
