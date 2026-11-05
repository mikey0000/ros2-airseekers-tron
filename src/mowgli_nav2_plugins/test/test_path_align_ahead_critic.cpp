// PathAlignAheadCritic scoring: control_point_x 0 reproduces Nav2 PathAlignCritic's loop
// (Jazzy 1.3.13, copied below as the reference, use_path_orientations false), the control
// point adds heading to the lateral error, and the latency projection scores a rollout as if
// it started from the projected pose.
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
  const xt::xtensor<float, 2> & /*tyaw*/, const xt::xtensor<float, 1> & px,
  const xt::xtensor<float, 1> & py, const std::vector<bool> & valid, size_t count, int step)
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
        sum += sqrtf(dx * dx + dy * dy);
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
    for (size_t furthest : {size_t(25), size_t(40)}) {
      paa::Params p;
      p.trajectory_point_step = step;
      const size_t n = paa::pathPointsUsed(px, py, furthest, valid.size(), 0.0f);
      ASSERT_EQ(n, furthest);
      std::vector<float> got, off;
      paa::scoreBatch(tx, ty, tyaw, px, py, valid, n, p, got);
      // latency 0: the identity projection changes nothing
      paa::scoreBatch(tx, ty, tyaw, px, py, valid, n, p, off,
        paa::projection(0.01f, -0.02f, 0.1f, 0.2f, 0.3f, 0.0f));
      const auto ref = reference(tx, ty, tyaw, px, py, valid, furthest, step);
      ASSERT_EQ(got.size(), ref.size());
      for (size_t b = 0; b < batch; ++b) {
        EXPECT_NEAR(got[b], ref[b], 1e-6f) << "step " << step << " sample " << b;
        EXPECT_EQ(off[b], got[b]) << "step " << step << " sample " << b;
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
  paa::scoreBatch(tx, ty, tyaw, px, py, valid, paa::pathPointsUsed(px, py, 40, 199, 0.0f), p,
    c0);
  p.control_point_x = 0.25f;
  paa::scoreBatch(tx, ty, tyaw, px, py, valid, paa::pathPointsUsed(px, py, 40, 199, 0.25f), p,
    c1);
  // a pure lateral offset: the same at both control points (1 cm path spacing)
  EXPECT_NEAR(c0[0], 0.03f, 0.003f);
  EXPECT_NEAR(c1[0], 0.03f, 0.003f);
  // a heading error: the control point adds about 0.25 m x 0.1 rad to every pose's error
  EXPECT_GT(c1[1], c0[1] + 0.02f);
}

TEST(PathAlignAhead, ProjectionDrivesThePoseAlongItsArc)
{
  // latency 0 or standing still: no move
  EXPECT_FALSE(paa::projection(1.0f, 2.0f, 0.3f, 0.2f, 0.1f, 0.0f).active());
  EXPECT_FALSE(paa::projection(1.0f, 2.0f, 0.3f, 0.0f, 0.0f, 0.35f).active());
  // straight: v x latency along the heading
  const auto s = paa::projection(1.0f, 2.0f, 0.5f, 0.2f, 0.0f, 0.35f);
  EXPECT_NEAR(s.qx, 1.0f + 0.07f * std::cos(0.5f), 1e-6f);
  EXPECT_NEAR(s.qy, 2.0f + 0.07f * std::sin(0.5f), 1e-6f);
  EXPECT_FLOAT_EQ(s.dyaw, 0.0f);
  EXPECT_NEAR(s.arc, 0.07f, 1e-6f);
  // arc: R = v / w = 1 m about the centre left of the pose; the end stays on the circle
  const auto a = paa::projection(0.0f, 0.0f, 0.0f, 0.3f, 0.3f, 1.0f);
  EXPECT_NEAR(a.qx, std::sin(0.3f), 1e-5f);
  EXPECT_NEAR(a.qy, 1.0f - std::cos(0.3f), 1e-5f);
  EXPECT_NEAR(a.dyaw, 0.3f, 1e-6f);
  EXPECT_NEAR(a.arc, 0.3f, 1e-6f);
  // pivot: turns in place
  const auto p = paa::projection(1.0f, 1.0f, 0.0f, 0.0f, 0.3f, 0.35f);
  EXPECT_FLOAT_EQ(p.qx, 1.0f);
  EXPECT_FLOAT_EQ(p.qy, 1.0f);
  EXPECT_NEAR(p.dyaw, 0.105f, 1e-6f);
}

TEST(PathAlignAhead, ProjectedRolloutScoresAsARolloutFromTheProjectedPose)
{
  // a curved path; rollouts with random controls from the real pose and, with the same
  // controls, from the projected pose
  const size_t batch = 32, steps = 40, npath = 120;
  xt::xtensor<float, 1> px = xt::zeros<float>({npath}), py = px, pyaw = px;
  for (size_t i = 0; i < npath; ++i) {
    const float a = 0.0125f * static_cast<float>(i);    // 1.5 rad of a 2 m radius arc
    px(i) = 2.0f * std::sin(a);
    py(i) = 2.0f * (1.0f - std::cos(a));
  }
  std::mt19937 rng(11);
  std::uniform_real_distribution<float> u(-1.0f, 1.0f);
  const float x0 = 0.01f, y0 = -0.02f, th0 = 0.05f, v0 = 0.2f, w0 = 0.12f, lat = 0.35f;
  const auto proj = paa::projection(x0, y0, th0, v0, w0, lat);
  ASSERT_TRUE(proj.active());
  xt::xtensor<float, 2> ax = xt::zeros<float>({batch, steps}), ay = ax, ayaw = ax;
  xt::xtensor<float, 2> bx = ax, by = ax, byaw = ax;
  for (size_t b = 0; b < batch; ++b) {
    float xa = x0, ya = y0, ta = th0, xb = proj.qx, yb = proj.qy, tb = th0 + proj.dyaw;
    for (size_t j = 0; j < steps; ++j) {
      const float v = 0.2f + 0.08f * u(rng), w = 0.1f + 0.2f * u(rng);
      ta += 0.05f * w;
      tb += 0.05f * w;
      xa += 0.05f * v * std::cos(ta);
      ya += 0.05f * v * std::sin(ta);
      xb += 0.05f * v * std::cos(tb);
      yb += 0.05f * v * std::sin(tb);
      ax(b, j) = xa; ay(b, j) = ya; ayaw(b, j) = ta;
      bx(b, j) = xb; by(b, j) = yb; byaw(b, j) = tb;
    }
  }
  std::vector<bool> valid(npath - 1, true);
  paa::Params p;
  p.trajectory_point_step = 2;
  p.control_point_x = 0.25f;
  const size_t n = paa::pathPointsUsed(px, py, 30, valid.size(), p.control_point_x + proj.arc);
  paa::Projection arc_only;    // the reference rollouts already start at the projected pose
  arc_only.arc = proj.arc;
  std::vector<float> got, ref, unprojected;
  paa::scoreBatch(ax, ay, ayaw, px, py, valid, n, p, got, proj);
  paa::scoreBatch(bx, by, byaw, px, py, valid, n, p, ref, arc_only);
  paa::scoreBatch(ax, ay, ayaw, px, py, valid, n, p, unprojected);
  float diff = 0.0f;
  for (size_t b = 0; b < batch; ++b) {
    EXPECT_NEAR(got[b], ref[b], 2e-4f) << "sample " << b;
    diff += std::fabs(got[b] - unprojected[b]);
  }
  EXPECT_GT(diff / static_cast<float>(batch), 1e-3f);    // and the projection does matter
}

TEST(PathAlignAhead, LatencyProjectionAddsHeadingNotAlongTrackError)
{
  xt::xtensor<float, 1> px, py, pyaw;
  straightPath(200, 0.01f, px, py, pyaw);
  std::vector<bool> valid(199, true);
  xt::xtensor<float, 2> tx, ty, tyaw;
  // on the line, 3 cm off parallel, and on the line heading 0.1 rad away; v 0.2 m/s
  rollouts({{0.0f, 0.0f}, {0.03f, 0.0f}, {0.0f, 0.1f}}, tx, ty, tyaw);
  paa::Params p;
  p.trajectory_point_step = 2;
  p.control_point_x = 0.25f;
  std::vector<float> c0, c1;
  const auto none = paa::projection(0.0f, 0.0f, 0.0f, 0.2f, 0.0f, 0.0f);
  paa::scoreBatch(tx, ty, tyaw, px, py, valid, paa::pathPointsUsed(px, py, 40, 199, 0.25f), p,
    c0, none);
  // each rollout's own start pose projected 0.35 s at 0.2 m/s: 0.07 m along its heading
  std::vector<float> c1b(3);
  for (size_t b = 0; b < 3; ++b) {
    const auto pr = paa::projection(tx(b, 0) - 0.01f * std::cos(tyaw(b, 0)),
      ty(b, 0) - 0.01f * std::sin(tyaw(b, 0)), tyaw(b, 0), 0.2f, 0.0f, 0.35f);
    paa::scoreBatch(tx, ty, tyaw, px, py, valid,
      paa::pathPointsUsed(px, py, 40, 199, 0.25f + pr.arc), p, c1, pr);
    c1b[b] = c1[b];
  }
  EXPECT_NEAR(c1b[0], c0[0], 0.003f);   // on the line: the path is matched further along too
  EXPECT_NEAR(c1b[1], c0[1], 0.003f);   // a lateral offset: unchanged
  EXPECT_GT(c1b[2], c0[2] + 0.005f);    // a heading error: + about 0.07 m x 0.1 rad
}

TEST(PathAlignAhead, SlopeUsesTheSlopeValues)
{
  paa::Tuning flat;
  flat.control_point_x = 0.30f;
  flat.latency_s = 0.2f;
  flat.trim_tau_s = 20.0f;
  paa::SlopeSchedule slope;
  slope.control_point_x = 0.25f;
  slope.trim_tau_s = 5.0f;
  // schedule off (tilt_rad 0): flat on every tilt
  auto t = paa::tuningAt(flat, slope, 0.2f, true);
  EXPECT_EQ(t.control_point_x, 0.30f);
  EXPECT_EQ(t.latency_s, 0.2f);
  EXPECT_EQ(t.trim_tau_s, 20.0f);
  slope.tilt_rad = static_cast<float>(8.0 * M_PI / 180.0);
  for (float tilt : {0.0f, 0.10f}) {              // flat, 5.7 deg
    t = paa::tuningAt(flat, slope, tilt, true);
    EXPECT_EQ(t.control_point_x, 0.30f) << tilt;
    EXPECT_EQ(t.latency_s, 0.2f) << tilt;
    EXPECT_EQ(t.trim_tau_s, 20.0f) << tilt;
  }
  t = paa::tuningAt(flat, slope, 0.17f, true);    // 9.7 deg
  EXPECT_EQ(t.control_point_x, 0.25f);
  EXPECT_EQ(t.latency_s, 0.0f);                   // no projection on a slope
  EXPECT_EQ(t.trim_tau_s, 5.0f);
  t = paa::tuningAt(flat, slope, 0.17f, false);   // stale IMU: flat
  EXPECT_EQ(t.control_point_x, 0.30f);
  EXPECT_EQ(t.latency_s, 0.2f);
}

TEST(PathAlignAhead, TiltCountsRollAndPitch)
{
  auto quat = [](double roll, double pitch, double yaw, double & qx, double & qy) {
      const double cr = std::cos(roll / 2), sr = std::sin(roll / 2);
      const double cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
      const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
      qx = sr * cp * cy - cr * sp * sy;
      qy = cr * sp * cy + sr * cp * sy;
    };
  double qx, qy;
  const double d10 = 10.0 * M_PI / 180.0;
  quat(0.0, 0.0, 1.0, qx, qy);
  EXPECT_NEAR(paa::tiltAngle(qx, qy), 0.0f, 1e-6f);           // level, any heading
  quat(d10, 0.0, 0.7, qx, qy);
  EXPECT_NEAR(paa::tiltAngle(qx, qy), d10, 1e-5f);            // side slope
  quat(0.0, -d10, -2.0, qx, qy);
  EXPECT_NEAR(paa::tiltAngle(qx, qy), d10, 1e-5f);            // along the fall line
  // 10 deg slope at 45 deg to the heading: roll and pitch about 7 deg each, tilt 10 deg
  const double r = std::atan(std::tan(d10) * std::sin(M_PI / 4));
  const double p = std::asin(std::sin(d10) * std::cos(M_PI / 4));
  quat(r, p, 0.3, qx, qy);
  EXPECT_NEAR(paa::tiltAngle(qx, qy), d10, 2e-3f);
}
