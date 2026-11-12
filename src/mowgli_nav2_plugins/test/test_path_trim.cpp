// Path trim math (path_trim.hpp): integral of the cross-track error, roll feed-forward,
// path normal shift.
#include <gtest/gtest.h>

#include <cmath>

#include <xtensor/xtensor.hpp>

#include "mowgli_nav2_plugins/path_trim.hpp"

namespace pt = mowgli_nav2_plugins::path_trim;

namespace
{
xt::xtensor<float, 1> arr(std::initializer_list<float> v)
{
  xt::xtensor<float, 1> a = xt::zeros<float>({v.size()});
  size_t i = 0;
  for (float f : v) {a(i++) = f;}
  return a;
}

pt::Params on()
{
  pt::Params p;
  p.enabled = true;
  return p;
}
}  // namespace

TEST(PathTrim, CrossTrackSignedPerpendicular)
{
  auto px = arr({0.0f, 1.0f, 2.0f});
  auto py = arr({0.0f, 0.0f, 0.0f});
  auto ct = pt::crossTrack(px, py, 3, 0.5f, 0.04f);
  ASSERT_TRUE(ct.ok);
  EXPECT_NEAR(ct.signed_m, 0.04f, 1e-6f);
  EXPECT_NEAR(ct.ex, 0.0f, 1e-6f);
  EXPECT_NEAR(ct.ey, 0.04f, 1e-6f);
  ct = pt::crossTrack(px, py, 3, 1.5f, -0.03f);
  EXPECT_NEAR(ct.signed_m, -0.03f, 1e-6f);
  // beyond the end: only the perpendicular part counts
  ct = pt::crossTrack(px, py, 3, 2.5f, -0.03f);
  EXPECT_NEAR(ct.ex, 0.0f, 1e-6f);
  EXPECT_NEAR(ct.ey, -0.03f, 1e-6f);
}

TEST(PathTrim, DisabledOrStillDoesNothing)
{
  pt::State s;
  pt::CrossTrack ct;
  ct.ok = true;
  ct.ey = ct.signed_m = -0.05f;
  pt::Params off;
  pt::update(s, ct, 0.2f, 0.05f, off);
  EXPECT_EQ(s.oy, 0.0f);
  pt::update(s, ct, 0.0f, 0.05f, on());      // standing still: no wind-up
  EXPECT_EQ(s.oy, 0.0f);
  float fx = 1.0f, fy = 1.0f;
  pt::Params p = on();
  p.k_roll = 2.0f;
  pt::feedForward(0.0f, 0.3f, 0.2f, p, fx, fy);   // flat: no feed-forward
  EXPECT_EQ(fx, 0.0f);
  EXPECT_NEAR(fy, 0.0f, 1e-9f);
}

TEST(PathTrim, PersistentErrorWindsWithTauAndClamps)
{
  pt::State s;
  pt::CrossTrack ct;
  ct.ok = true;
  ct.ey = ct.signed_m = -0.05f;          // 5 cm right of the path (downhill)
  pt::Params p = on();
  p.leak_tau_s = 0.0f;
  for (int k = 0; k < 100; ++k) {pt::update(s, ct, 0.2f, 0.05f, p);}   // 5 s = tau
  EXPECT_NEAR(s.oy, 0.05f, 1e-4f);       // integral of e / tau over tau = e: shift left
  EXPECT_NEAR(s.ox, 0.0f, 1e-6f);
  for (int k = 0; k < 1000; ++k) {pt::update(s, ct, 0.2f, 0.05f, p);}
  EXPECT_NEAR(s.oy, 0.10f, 1e-5f);       // clamp trim_max_m
  ct.ey = ct.signed_m = -0.5f;           // off the path (a turn): frozen
  const float o = s.oy;
  pt::update(s, ct, 0.2f, 0.05f, p);
  EXPECT_EQ(s.oy, o);
}

TEST(PathTrim, ZeroMeanWeaveAveragesOut)
{
  pt::State s;
  pt::Params p = on();
  for (int k = 0; k < 2000; ++k) {        // 0.1 Hz weave, +-5 cm, 100 s
    pt::CrossTrack ct;
    ct.ok = true;
    ct.ey = ct.signed_m = 0.05f * std::sin(2.0f * static_cast<float>(M_PI) * 0.1f * k * 0.05f);
    pt::update(s, ct, 0.2f, 0.05f, p);
  }
  EXPECT_LT(std::hypot(s.ox, s.oy), 0.03f);
}

TEST(PathTrim, LeakDecays)
{
  pt::State s{0.0f, 0.08f};
  pt::Params p = on();
  pt::CrossTrack none;
  for (int k = 0; k < 1200; ++k) {pt::update(s, none, 0.2f, 0.05f, p);}  // 60 s
  EXPECT_NEAR(s.oy, 0.08f * std::exp(-1.0f), 1e-3f);
}

TEST(PathTrim, RollFeedForwardPointsUpslope)
{
  pt::Params p = on();
  p.k_roll = 2.0f;
  float fx, fy;
  // heading +x, roll +10 deg (right side down, downhill = -y): shift toward +y
  pt::feedForward(static_cast<float>(10.0 * M_PI / 180.0), 0.0f, 0.2f, p, fx, fy);
  EXPECT_NEAR(fx, 0.0f, 1e-6f);
  EXPECT_NEAR(fy, 2.0f * std::sin(10.0f * static_cast<float>(M_PI) / 180.0f) * 0.2f, 1e-6f);
  // the return swath (heading -x) on the same slope sees roll -10: same world direction
  pt::feedForward(static_cast<float>(-10.0 * M_PI / 180.0), static_cast<float>(M_PI), 0.2f, p,
    fx, fy);
  EXPECT_GT(fy, 0.0f);
}

TEST(PathTrim, ShiftMovesOnlyAlongTheNormal)
{
  auto px = arr({0.0f, 1.0f, 2.0f, 2.0f});
  auto py = arr({0.0f, 0.0f, 0.0f, 1.0f});
  pt::shiftPath(px, py, 2, 0.03f, 0.05f);     // straight +x part: only y moves
  EXPECT_NEAR(px(0), 0.0f, 1e-6f);
  EXPECT_NEAR(py(0), 0.05f, 1e-6f);
  EXPECT_NEAR(px(1), 1.0f, 1e-6f);
  EXPECT_NEAR(py(1), 0.05f, 1e-6f);
  EXPECT_NEAR(px(2), 2.0f, 1e-6f);           // beyond n: untouched
  EXPECT_NEAR(py(2), 0.0f, 1e-6f);
}

TEST(PathTrim, RollFilterLowPassesTheWobble)
{
  // 2026-11-12: a 0.5 Hz, 0.6 deg roll wobble on a 5 deg side slope, 100 Hz samples
  pt::RollFilter f;
  const float slope = static_cast<float>(5.0 * M_PI / 180.0);
  const float wob = static_cast<float>(0.6 * M_PI / 180.0);
  EXPECT_FLOAT_EQ(f.update(0.0, slope + wob, 3.0f, 1.0f), slope + wob);   // starts at the sample
  float lo = 1e9f, hi = -1e9f;
  for (int k = 1; k <= 3000; ++k) {
    const double t = 0.01 * k;
    const float x = slope + wob * static_cast<float>(std::sin(2.0 * M_PI * 0.5 * t));
    const float y = f.update(t, x, 3.0f, 1.0f);
    if (t > 20.0) {lo = std::min(lo, y); hi = std::max(hi, y);}
  }
  EXPECT_NEAR(0.5f * (lo + hi), slope, 0.01f * slope);     // the steady slope passes
  // first order at 0.5 Hz, tau 3 s: |H| = 1 / sqrt(1 + (2 pi 0.5 3)^2) ~ 0.106
  EXPECT_NEAR(0.5f * (hi - lo) / wob, 0.106f, 0.01f);
}

TEST(PathTrim, RollFilterStepResponseAndReset)
{
  pt::RollFilter f;
  f.update(0.0, 0.0f, 3.0f, 1.0f);
  float y = 0.0f;
  for (int k = 1; k <= 300; ++k) {y = f.update(0.01 * k, 0.1f, 3.0f, 1.0f);}
  EXPECT_NEAR(y, 0.1f * (1.0f - std::exp(-1.0f)), 1e-4f);   // one time constant: 63 %
  // a gap longer than reset_after (stale roll): restart at the new sample
  EXPECT_FLOAT_EQ(f.update(10.0, -0.05f, 3.0f, 1.0f), -0.05f);
  // tau 0: raw roll, as before 2026-11-12
  pt::RollFilter raw;
  raw.update(0.0, 0.0f, 0.0f, 1.0f);
  EXPECT_FLOAT_EQ(raw.update(0.01, 0.2f, 0.0f, 1.0f), 0.2f);
}
