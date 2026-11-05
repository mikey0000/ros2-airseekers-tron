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

TEST(PathTrim, SlopeScheduleWindsFasterOnASideSlope)
{
  pt::Params p = on();
  p.tau_s = 20.0f;
  // no schedule: tau_s on every roll
  EXPECT_EQ(pt::integralTau(p, 0.2f, true), 20.0f);
  p.slope_tau_s = 5.0f;
  p.slope_roll_rad = static_cast<float>(8.0 * M_PI / 180.0);
  EXPECT_EQ(pt::integralTau(p, 0.0f, true), 20.0f);                   // flat
  EXPECT_EQ(pt::integralTau(p, 0.10f, true), 20.0f);                  // 5.7 deg
  EXPECT_EQ(pt::integralTau(p, 0.17f, true), 5.0f);                   // 9.7 deg, right down
  EXPECT_EQ(pt::integralTau(p, -0.17f, true), 5.0f);                  // left down
  EXPECT_EQ(pt::integralTau(p, 0.17f, false), 20.0f);                 // stale roll: flat value
  // the same persistent offset winds 4x faster at the slope tau
  pt::CrossTrack ct;
  ct.ok = true;
  ct.ey = ct.signed_m = -0.02f;
  p.leak_tau_s = 0.0f;
  pt::State flat, slope;
  for (int k = 0; k < 40; ++k) {   // 2 s
    pt::update(flat, ct, 0.2f, 0.05f, p, pt::integralTau(p, 0.0f, true));
    pt::update(slope, ct, 0.2f, 0.05f, p, pt::integralTau(p, 0.17f, true));
  }
  EXPECT_NEAR(flat.oy, 0.02f * 2.0f / 20.0f, 1e-5f);
  EXPECT_NEAR(slope.oy, 0.02f * 2.0f / 5.0f, 1e-5f);
  // the 5-argument update is the flat-ground tau_s
  pt::State s5;
  for (int k = 0; k < 40; ++k) {pt::update(s5, ct, 0.2f, 0.05f, p);}
  EXPECT_EQ(s5.oy, flat.oy);
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
