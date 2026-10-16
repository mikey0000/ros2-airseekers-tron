// WheelFloorCritic: the drivable-set rule (mirrors cmd_vel_slew.both_wheels_legal) and the
// batch scoring the critic adds to MPPI's sample costs.
#include <gtest/gtest.h>

#include "mowgli_nav2_plugins/wheel_floor_critic.hpp"

using mowgli_nav2_plugins::wheel_floor::Params;
using mowgli_nav2_plugins::wheel_floor::drivable;
using mowgli_nav2_plugins::wheel_floor::scoreBatch;
using mowgli_nav2_plugins::wheel_floor::violation;

TEST(WheelFloor, StraightAndStopAreDrivable)
{
  Params p;
  EXPECT_TRUE(drivable(0.3, 0.0, p));
  EXPECT_TRUE(drivable(0.03, 0.0, p));     // slow straight: the floor only bites in turns
  EXPECT_TRUE(drivable(-0.1, 0.0, p));
  EXPECT_TRUE(drivable(0.0, 0.0, p));
  EXPECT_TRUE(drivable(0.005, 0.02, p));   // both wheels < stop_speed: a stop
}

TEST(WheelFloor, ArcsNeedTheInnerWheelAboveTheFloor)
{
  Params p;
  // r 0.44 m at the MCU's 0.3 rad/s: inner = 0.132 - 0.072 = 0.06 -> drivable
  EXPECT_TRUE(drivable(0.132, 0.3, p));
  EXPECT_TRUE(drivable(0.3, 0.3, p));      // r 1.0 m at full speed
  EXPECT_TRUE(drivable(-0.15, 0.3, p));    // reversing arc, inner 0.078
  // the undock donut: |v| <= 0.03 with |w| 0.05-0.3
  EXPECT_FALSE(drivable(0.03, 0.1, p));    // inner 0.006: one-wheel-ish creep
  EXPECT_FALSE(drivable(0.02, 0.05, p));
  // the run 3 corners: v 0.02-0.05, w 0.1-0.2 (r 0.2-0.3 m)
  EXPECT_FALSE(drivable(0.05, 0.2, p));
  EXPECT_NEAR(violation(0.05, 0.1, p), 0.06 - (0.05 - 0.024), 1e-9);
}

TEST(WheelFloor, PivotsNeedQuarterRadianPerSecond)
{
  Params p;
  EXPECT_TRUE(drivable(0.0, 0.25, p));     // both wheels 0.06 m/s counter-rotating
  EXPECT_TRUE(drivable(0.0, -0.3, p));
  EXPECT_FALSE(drivable(0.0, 0.15, p));    // 0.036 m/s per wheel: does not turn
  EXPECT_NEAR(violation(0.0, 0.15, p), 0.06 - 0.036, 1e-9);
  // one wheel backwards slower than the floor, the other forward
  EXPECT_FALSE(drivable(0.04, 0.3, p));    // inner -0.032
  p.allow_pivots = false;
  EXPECT_FALSE(drivable(0.0, 0.3, p));
  EXPECT_NEAR(violation(0.0, 0.3, p), 0.06 + 0.072, 1e-9);
  EXPECT_TRUE(drivable(0.2, 0.3, p));
}

TEST(WheelFloor, ViolationIsDistanceToTheNearestBand)
{
  Params p;
  // inner just under the arc band -> small; just above the pivot band -> small
  EXPECT_NEAR(violation(0.1, 0.1, p), 0.0, 1e-9);            // inner 0.076
  EXPECT_NEAR(violation(0.08, 0.1, p), 0.004, 1e-9);         // inner 0.056
  EXPECT_NEAR(violation(0.0, 0.24, p), 0.06 - 0.0576, 1e-9);  // inner -0.0576
  EXPECT_GT(violation(0.03, 0.12, p), 0.0);                   // inner 0.0012: worst case
  EXPECT_NEAR(violation(0.03, 0.125, p), 0.06, 1e-9);         // inner 0
}

TEST(WheelFloor, StraightThresholdIsOptional)
{
  Params p;
  EXPECT_FALSE(drivable(0.03, 0.005, p));  // default: any turn rate is a turn (= the shaper)
  p.straight_wz = 0.01;
  EXPECT_TRUE(drivable(0.03, 0.005, p));
}

TEST(WheelFloorCritic, ScoreBatchPenalisesOnlyInfeasibleSamples)
{
  Params p;
  // 3 samples x 4 steps: drivable arc, undock-donut creep, half/half
  xt::xtensor<float, 2> vx = {{0.2f, 0.2f, 0.2f, 0.2f},
    {0.03f, 0.03f, 0.03f, 0.03f},
    {0.2f, 0.2f, 0.03f, 0.03f}};
  xt::xtensor<float, 2> wz = {{0.2f, 0.2f, 0.2f, 0.2f},
    {0.1f, 0.1f, 0.1f, 0.1f},
    {0.2f, 0.2f, 0.1f, 0.1f}};
  xt::xtensor<float, 1> costs = {1.0f, 1.0f, 1.0f};
  const size_t bad = scoreBatch(vx, wz, costs, p, 10.0, 1);
  EXPECT_EQ(bad, 6u);
  EXPECT_FLOAT_EQ(costs(0), 1.0f);                      // unchanged
  const double x = violation(0.03f, 0.1f, p) / 0.06;    // ~0.9
  EXPECT_NEAR(costs(1), 1.0 + 10.0 * x, 1e-4);
  EXPECT_NEAR(costs(2), 1.0 + 10.0 * x * 0.5, 1e-4);   // mean over the 4 steps
  EXPECT_GT(costs(1), costs(2));
}

TEST(WheelFloorCritic, PowerSharpensLargeViolations)
{
  Params p;
  xt::xtensor<float, 2> vx = {{0.03f}, {0.09f}};
  xt::xtensor<float, 2> wz = {{0.1f}, {0.15f}};   // x ~0.9 and x ~0.5
  xt::xtensor<float, 1> c1 = {0.0f, 0.0f};
  xt::xtensor<float, 1> c2 = {0.0f, 0.0f};
  scoreBatch(vx, wz, c1, p, 1.0, 1);
  scoreBatch(vx, wz, c2, p, 1.0, 2);
  EXPECT_GT(c1(0) / c1(1), 1.0f);
  EXPECT_GT(c2(0) / c2(1), c1(0) / c1(1));
}

TEST(WheelFloorCritic, EmptyAndDisabledFloorDoNothing)
{
  Params p;
  xt::xtensor<float, 2> vx = xt::zeros<float>({2, 0});
  xt::xtensor<float, 2> wz = xt::zeros<float>({2, 0});
  xt::xtensor<float, 1> costs = {0.0f, 0.0f};
  EXPECT_EQ(scoreBatch(vx, wz, costs, p, 10.0, 1), 0u);
  p.floor = 0.0;
  xt::xtensor<float, 2> v2 = {{0.03f}};
  xt::xtensor<float, 2> w2 = {{0.1f}};
  xt::xtensor<float, 1> c2 = {0.0f};
  EXPECT_EQ(scoreBatch(v2, w2, c2, p, 10.0, 1), 0u);
  EXPECT_FLOAT_EQ(c2(0), 0.0f);
}
