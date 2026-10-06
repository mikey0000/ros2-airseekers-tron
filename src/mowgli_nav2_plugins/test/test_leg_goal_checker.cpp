// LegProgress (LegGoalChecker core): a short turn leg whose start already lies
// inside a loose xy tolerance must still be driven.
#include <gtest/gtest.h>

#include <cmath>

#include "mowgli_nav2_plugins/leg_goal_checker.hpp"

using mowgli_nav2_plugins::LegProgress;

TEST(LegProgress, NotReachedAtStartOfShortLeg)
{
  LegProgress lp;
  // 0.3 m reverse leg along -x, robot facing +x (yaw 0), goal yaw 0.
  EXPECT_FALSE(lp.update(0.0, 0.0, 0.0, -0.3, 0.0, 0.0));
  EXPECT_FALSE(lp.update(0.0, 0.0, 0.0, -0.3, 0.0, 0.0));
}

TEST(LegProgress, ReachedAfterDrivingTheLeg)
{
  LegProgress lp;
  bool done = false;
  int steps = 0;
  for (double x = 0.0; x >= -0.31 && !done; x -= 0.01, ++steps)
  {
    done = lp.update(x, 0.0, 0.0, -0.3, 0.0, 0.0);
  }
  EXPECT_TRUE(done);
  EXPECT_GE(lp.travelled, 0.21);
  EXPECT_GT(steps, 20);
}

TEST(LegProgress, YawGate)
{
  LegProgress lp;
  lp.update(0.0, 0.0, 0.0, 0.5, 0.0, 0.0);
  for (double x = 0.0; x <= 0.5; x += 0.01)
  {
    lp.update(x, 0.0, 0.0, 0.5, 0.0, 0.0);
  }
  // At the goal but turned 0.8 rad away: not reached by the strict rule.
  EXPECT_FALSE(lp.update(0.5, 0.0, 0.8, 0.5, 0.0, 0.0));
}

TEST(LegProgress, OvershootCompletes)
{
  LegProgress lp;
  // Passes 0.12 m beside the goal (never within 0.08), then moves away.
  bool done = false;
  for (double x = 0.0; x <= 0.8 && !done; x += 0.01)
  {
    done = lp.update(x, 0.12, 0.0, 0.5, 0.0, 0.0);
  }
  EXPECT_TRUE(done);
}

TEST(LegProgress, ResetOnNewGoalPose)
{
  LegProgress lp;
  for (double x = 0.0; x <= 0.5; x += 0.01)
  {
    lp.update(x, 0.0, 0.0, 0.5, 0.0, 0.0);
  }
  // New goal 0.3 m further, no reset() call: travel restarts from zero.
  EXPECT_FALSE(lp.update(0.5, 0.0, 0.0, 0.8, 0.0, 0.0));
  EXPECT_NEAR(lp.travelled, 0.0, 1e-9);
}

TEST(LegProgress, IgnoresPoseJumps)
{
  LegProgress lp;
  lp.update(0.0, 0.0, 0.0, 0.8, 0.0, 0.0);
  // Relocalisation jump (> max_step) straight onto the goal: no travel credited.
  EXPECT_FALSE(lp.update(0.8, 0.0, 0.0, 0.8, 0.0, 0.0));
  EXPECT_NEAR(lp.travelled, 0.0, 1e-9);
}
