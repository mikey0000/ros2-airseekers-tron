// PathAlignAheadCritic scoring: control_point_x 0 reproduces Nav2 PathAlignCritic's loop
// (Jazzy 1.3.13, copied below as the reference), and the control point adds heading to the
// lateral error.
#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

#include "mowgli_nav2_plugins/path_align_ahead_critic.hpp"

namespace paa = mowgli_nav2_plugins::path_align_ahead;

namespace
{

// PathAlignCritic::score's per-sample loop (nav2_mppi_controller 1.3.13), unweighted.
std::vector<float> reference(
  const xt::xtensor<float, 2> & tx, const xt::xtensor<float, 2> & ty,
  const xt::xtensor<float, 2> & tyaw, const xt::xtensor<float, 1> & px,
  const xt::xtensor<float, 1> & py, const xt::xtensor<float, 1> & pyaw,
  const std::vector<bool> & valid, size_t count, int step, bool orient)
{
  std::vector<float> s(count, 0.0f);
  for (size_t i = 1; i != count; i++) {
    const float dx = px(i) - px(i - 1), dy = py(i) - py(i - 1);
    s[i] = s[i - 1] + sqrtf(dx * dx + dy * dy);
  }
  std::vector<float> out(tx.shape(0), 0.0f);
  std::vector<size_t> idx;
  for (size_t j = 0; j < tx.shape(1); j += step) {idx.push_back(j);}
  for (size_t t = 0; t < tx.shape(0); ++t) {
    float sum = 0.0f, ts = 0.0f;
    unsigned int n = 0u, pp = 0u;
    float xm = tx(t, 0), ym = ty(t, 0);
    for (size_t k = 1; k < idx.size(); k++) {
      const float x = tx(t, idx[k]), y = ty(t, idx[k]);
      float dx = x - xm, dy = y - ym;
      xm = x;
      ym = y;
      ts += sqrtf(dx * dx + dy * dy);
      pp = mppi::utils::findClosestPathPt(s, ts, pp);
      if (valid[pp]) {
        dx = px(pp) - x;
        dy = py(pp) - y;
        n++;
        if (orient) {
          const float dyaw = angles::shortest_angular_distance(pyaw(pp), tyaw(t, idx[k]));
          sum += sqrtf(dx * dx + dy * dy + dyaw * dyaw);
        } else {
          sum += sqrtf(dx * dx + dy * dy);
        }
      }
    }
    out[t] = n > 0u ? sum / static_cast<float>(n) : 0.0f;
  }
  return out;
}

// A straight path along +x from the origin.
void straightPath(size_t n, float spacing, xt::xtensor<float, 1> & px,
  xt::xtensor<float, 1> & py, xt::xtensor<float, 1> & pyaw)
{
  px = xt::zeros<float>({n});
  py = xt::zeros<float>({n});
  pyaw = xt::zeros<float>({n});
  for (size_t i = 0; i < n; ++i) {px(i) = spacing * static_cast<float>(i);}
}

// One unicycle rollout per (y0, yaw0), v 0.2 m/s, w 0, dt 0.05, 40 steps.
void rollouts(const std::vector<std::pair<float, float>> & init, xt::xtensor<float, 2> & tx,
  xt::xtensor<float, 2> & ty, xt::xtensor<float, 2> & tyaw)
{
  const size_t steps = 40;
  tx = xt::zeros<float>({init.size(), steps});
  ty = xt::zeros<float>({init.size(), steps});
  tyaw = xt::zeros<float>({init.size(), steps});
  for (size_t b = 0; b < init.size(); ++b) {
    for (size_t j = 0; j < steps; ++j) {
      const float d = 0.01f * static_cast<float>(j);
      tx(b, j) = d * std::cos(init[b].second);
      ty(b, j) = init[b].first + d * std::sin(init[b].second);
      tyaw(b, j) = init[b].second;
    }
  }
}

}  // namespace

TEST(PathAlignAhead, ZeroControlPointIsPathAlignCritic)
{
  std::mt19937 rng(7);
  std::uniform_real_distribution<float> u(-1.0f, 1.0f);
  const size_t batch = 64, steps = 40, npath = 60;
  // a curved path and random-walk trajectories around it
  xt::xtensor<float, 1> px = xt::zeros<float>({npath}), py = px, pyaw = px;
  for (size_t i = 0; i < npath; ++i) {
    const float a = 0.02f * static_cast<float>(i);
    px(i) = 0.6f * std::sin(a * 2.0f);
    py(i) = 0.6f * (1.0f - std::cos(a * 2.0f));
    pyaw(i) = 2.0f * a;
  }
  xt::xtensor<float, 2> tx = xt::zeros<float>({batch, steps}), ty = tx, tyaw = tx;
  for (size_t b = 0; b < batch; ++b) {
    float x = 0.02f * u(rng), y = 0.02f * u(rng), th = 0.2f * u(rng);
    for (size_t j = 0; j < steps; ++j) {
      tx(b, j) = x;
      ty(b, j) = y;
      tyaw(b, j) = th;
      const float v = 0.2f + 0.1f * u(rng), w = 0.6f + 0.3f * u(rng);
      x += 0.05f * v * std::cos(th);
      y += 0.05f * v * std::sin(th);
      th += 0.05f * w;
    }
  }
  std::vector<bool> valid(npath - 1, true);
  valid[17] = false;
  for (int step : {1, 2, 4}) {
    for (bool orient : {false, true}) {
      for (size_t furthest : {size_t(25), size_t(40)}) {
        paa::Params p;
        p.trajectory_point_step = step;
        p.use_path_orientations = orient;
        const size_t n = paa::pathPointsUsed(px, py, furthest, valid.size(), 0.0f);
        ASSERT_EQ(n, furthest);
        std::vector<float> got;
        paa::scoreBatch(tx, ty, tyaw, px, py, pyaw, valid, n, p, got);
        const auto ref = reference(tx, ty, tyaw, px, py, pyaw, valid, furthest, step, orient);
        ASSERT_EQ(got.size(), ref.size());
        for (size_t b = 0; b < batch; ++b) {
          EXPECT_NEAR(got[b], ref[b], 1e-6f) << "step " << step << " sample " << b;
        }
      }
    }
  }
}

TEST(PathAlignAhead, PathExtendsByTheControlPoint)
{
  xt::xtensor<float, 1> px, py, pyaw;
  straightPath(100, 0.05f, px, py, pyaw);
  EXPECT_EQ(paa::pathPointsUsed(px, py, 30, 99, 0.0f), 30u);
  const size_t n = paa::pathPointsUsed(px, py, 30, 99, 0.25f);  // 5 x 0.05 m (+ rounding)
  EXPECT_GE(n, 35u);
  EXPECT_LE(n, 36u);
  EXPECT_EQ(paa::pathPointsUsed(px, py, 97, 99, 0.25f), 99u);   // capped at the path
}

TEST(PathAlignAhead, LateralOffsetCostsTheSameHeadingCostsMore)
{
  xt::xtensor<float, 1> px, py, pyaw;
  straightPath(200, 0.01f, px, py, pyaw);
  std::vector<bool> valid(199, true);
  xt::xtensor<float, 2> tx, ty, tyaw;
  // parallel 3 cm off the line, and on the line heading 0.1 rad away
  rollouts({{0.03f, 0.0f}, {0.0f, 0.1f}}, tx, ty, tyaw);
  paa::Params p;
  p.trajectory_point_step = 2;
  std::vector<float> c0, c1;
  paa::scoreBatch(tx, ty, tyaw, px, py, pyaw, valid, paa::pathPointsUsed(px, py, 40, 199, 0.0f),
    p, c0);
  p.control_point_x = 0.25f;
  paa::scoreBatch(tx, ty, tyaw, px, py, pyaw, valid, paa::pathPointsUsed(px, py, 40, 199, 0.25f),
    p, c1);
  // a pure lateral offset: the same at both control points (1 cm path spacing)
  EXPECT_NEAR(c0[0], 0.03f, 0.003f);
  EXPECT_NEAR(c1[0], 0.03f, 0.003f);
  // a heading error: the control point adds about 0.25 m x 0.1 rad to every pose's error
  EXPECT_GT(c1[1], c0[1] + 0.02f);
}
