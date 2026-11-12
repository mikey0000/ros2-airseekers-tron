// Row entry speed hold (row_entry.hpp, 2026-11-12): row detection, the hold / ramp / release
// state machine and the batch cost.
#include <gtest/gtest.h>

#include <cmath>

#include <xtensor/xtensor.hpp>

#include "mowgli_nav2_plugins/row_entry.hpp"

namespace re = mowgli_nav2_plugins::row_entry;

namespace
{
constexpr float kDeg = static_cast<float>(M_PI / 180.0);

// A straight path along heading h from (x0, y0): n points 0.05 m apart.
void line(xt::xtensor<float, 1> & px, xt::xtensor<float, 1> & py, size_t n, float h,
  float x0 = 0.0f, float y0 = 0.0f)
{
  px = xt::zeros<float>({n});
  py = xt::zeros<float>({n});
  for (size_t i = 0; i < n; ++i) {
    px(i) = x0 + 0.05f * static_cast<float>(i) * std::cos(h);
    py(i) = y0 + 0.05f * static_cast<float>(i) * std::sin(h);
  }
}
}  // namespace

TEST(RowEntry, RowHeadingNeedsAStraightRowAhead)
{
  re::Params p;
  xt::xtensor<float, 1> px, py;
  float h = 0.0f;
  line(px, py, 60, 105.0f * kDeg);          // 2.95 m
  ASSERT_TRUE(re::rowHeading(px, py, 60, p, h));
  EXPECT_NEAR(h, 105.0f * kDeg, 1e-5f);
  EXPECT_FALSE(re::rowHeading(px, py, 20, p, h));   // 0.95 m: a turn leg, not a row
  // a 90 deg bend 0.5 m ahead: not a row
  for (size_t i = 10; i < 60; ++i) {
    px(i) = px(10) + 0.05f * static_cast<float>(i - 10) * std::cos(15.0f * kDeg);
    py(i) = py(10) + 0.05f * static_cast<float>(i - 10) * std::sin(15.0f * kDeg);
  }
  EXPECT_FALSE(re::rowHeading(px, py, 60, p, h));
}

TEST(RowEntry, HoldsUntilAlignedThenRampsAndReleases)
{
  re::Params p;
  re::State s;
  const float row = 0.0f;
  // a new path, 8 deg off: hold at the entry speed
  EXPECT_FLOAT_EQ(re::update(s, p, 0.0, 0.0f, 0.0f, 8.0f * kDeg, 10.0f, 0.0f, true, row),
    p.entry_speed);
  EXPECT_EQ(s.phase, re::Phase::kHold);
  EXPECT_FLOAT_EQ(re::update(s, p, 0.5, 0.05f, 0.0f, 4.0f * kDeg, 10.0f, 0.0f, true, row),
    p.entry_speed);
  // within the tolerance: ramp from the entry speed at ramp_accel
  EXPECT_FLOAT_EQ(re::update(s, p, 1.0, 0.10f, 0.0f, 2.0f * kDeg, 10.0f, 0.0f, true, row),
    p.entry_speed);
  EXPECT_EQ(s.phase, re::Phase::kRamp);
  EXPECT_NEAR(re::update(s, p, 2.0, 0.25f, 0.0f, 3.5f * kDeg, 10.0f, 0.0f, true, row),
    p.entry_speed + p.ramp_accel, 1e-6f);     // no fall-back to the hold once aligned
  // at the release speed: no cap any more, and it stays off for this path
  EXPECT_LT(re::update(s, p, 3.1, 0.6f, 0.0f, 0.0f, 10.0f, 0.0f, true, row), 0.0f);
  EXPECT_EQ(s.phase, re::Phase::kNone);
  EXPECT_LT(re::update(s, p, 4.0, 0.8f, 0.0f, 9.0f * kDeg, 10.0f, 0.0f, true, row), 0.0f);
}

TEST(RowEntry, NoHoldWhenAlignedNotARowOrTooFar)
{
  re::Params p;
  re::State s;
  // a new path that starts aligned (good pivot, mid-row re-send): never capped
  EXPECT_LT(re::update(s, p, 0.0, 0.0f, 0.0f, 1.0f * kDeg, 10.0f, 0.0f, true, 0.0f), 0.0f);
  EXPECT_EQ(s.phase, re::Phase::kNone);
  // a new path that is no row (turn leg / bend): never capped
  EXPECT_LT(re::update(s, p, 1.0, 0.0f, 0.0f, 30.0f * kDeg, 0.0f, 0.2f, false, 0.0f), 0.0f);
  // a row entered 8 deg off that never aligns: released after max_entry_m
  EXPECT_GT(re::update(s, p, 2.0, 0.0f, 0.0f, 8.0f * kDeg, -10.0f, 0.0f, true, 0.0f), 0.0f);
  EXPECT_GT(re::update(s, p, 3.0, 2.9f, 0.0f, 8.0f * kDeg, -10.0f, 0.0f, true, 0.0f), 0.0f);
  EXPECT_LT(re::update(s, p, 4.0, 3.1f, 0.0f, 8.0f * kDeg, -10.0f, 0.0f, true, 0.0f), 0.0f);
  // a goal that moves by less than new_goal_m is the same path
  EXPECT_LT(re::update(s, p, 5.0, 3.2f, 0.0f, 8.0f * kDeg, -10.02f, 0.0f, true, 0.0f), 0.0f);
  // the next row (new goal) holds again
  EXPECT_FLOAT_EQ(
    re::update(s, p, 6.0, 3.2f, 0.2f, 186.0f * kDeg, 13.2f, 0.2f, true, 0.0f), p.entry_speed);
}

TEST(RowEntry, BatchCostPenalisesSpeedOverTheCap)
{
  xt::xtensor<float, 2> cvx = xt::zeros<float>({3, 4});
  for (size_t j = 0; j < 4; ++j) {
    cvx(0, j) = 0.08f;    // under the cap
    cvx(1, j) = 0.20f;    // 0.1 over
    cvx(2, j) = 0.12f;    // under the ramped cap from step 1 on
  }
  xt::xtensor<float, 1> costs = xt::zeros<float>({3});
  EXPECT_EQ(re::scoreBatch(cvx, costs, -1.0f, 0.0f, 0.05f, 30.0f, 1u), 0u);   // no cap
  EXPECT_FLOAT_EQ(costs(1), 0.0f);
  EXPECT_EQ(re::scoreBatch(cvx, costs, 0.10f, 0.0f, 0.05f, 30.0f, 1u), 8u);
  EXPECT_FLOAT_EQ(costs(0), 0.0f);
  EXPECT_NEAR(costs(1), 30.0f * 0.10f, 1e-5f);
  EXPECT_NEAR(costs(2), 30.0f * 0.02f, 1e-5f);
  // ramping 0.4 m/s^2 over 0.05 s steps: caps 0.10, 0.12, 0.14, 0.16
  costs = xt::zeros<float>({3});
  re::scoreBatch(cvx, costs, 0.10f, 0.4f, 0.05f, 30.0f, 1u);
  EXPECT_NEAR(costs(2), 30.0f * 0.02f / 4.0f, 1e-5f);
  EXPECT_NEAR(costs(1), 30.0f * (0.10f + 0.08f + 0.06f + 0.04f) / 4.0f, 1e-5f);
}
