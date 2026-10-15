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

// ---- along_path mode: a leg into a pivot vertex is done when the vertex is reached along
// the path, whatever the yaw and without a position disc
static LegProgress alongPath()
{
  LegProgress lp;
  lp.along_path = true;
  lp.progress_fraction = 0.5;
  lp.xy_tol = 0.02;
  lp.lateral_tol = 0.15;
  return lp;
}

TEST(LegProgressAlongPath, NotReachedBeforeTheVertexPlane)
{
  LegProgress lp = alongPath();
  // 0.18 m connector along +y, goal yaw +pi/2
  EXPECT_FALSE(lp.update(0.0, 0.0, M_PI / 2, 0.0, 0.18, M_PI / 2));
  for (double y = 0.0; y < 0.15; y += 0.01)
  {
    EXPECT_FALSE(lp.update(0.0, y, M_PI / 2, 0.0, 0.18, M_PI / 2));
  }
}

TEST(LegProgressAlongPath, ReachedOnCrossingTheVertexPlaneOffsetAndTurned)
{
  LegProgress lp = alongPath();
  lp.update(0.0, 0.0, M_PI / 2, 0.0, 0.5, M_PI / 2);
  bool done = false;
  double y = 0.0;
  // 0.10 m beside the line (outside any 0.08 m disc) and 0.6 rad off the goal yaw
  for (; y <= 0.6 && !done; y += 0.01)
  {
    done = lp.update(0.10, y, M_PI / 2 + 0.6, 0.0, 0.5, M_PI / 2);
  }
  EXPECT_TRUE(done);
  EXPECT_NEAR(y, 0.49, 0.025);
}

TEST(LegProgressAlongPath, TooFarBesideTheLineIsNotReached)
{
  LegProgress lp = alongPath();
  lp.update(0.0, 0.0, 0.0, 0.5, 0.0, 0.0);
  for (double x = 0.0; x <= 0.8; x += 0.01)
  {
    EXPECT_FALSE(lp.update(x, 0.3, 0.0, 0.5, 0.0, 0.0));
  }
}

TEST(LegProgressAlongPath, TravelGateStillApplies)
{
  LegProgress lp = alongPath();
  // starts 0.7 m before the vertex; a relocalisation jump (> max_step) past it credits
  // no travel
  lp.update(-0.3, 0.0, 0.0, 0.4, 0.0, 0.0);
  EXPECT_FALSE(lp.update(0.45, 0.0, 0.0, 0.4, 0.0, 0.0));
}

TEST(LegProgressAlongPath, DefaultModeUnchanged)
{
  LegProgress lp;          // along_path false: the 0.08 m disc + yaw rule as before
  lp.update(0.0, 0.0, 0.0, 0.5, 0.0, 0.0);
  for (double x = 0.0; x <= 0.5; x += 0.01)
  {
    lp.update(x, 0.0, 0.0, 0.5, 0.0, 0.0);
  }
  EXPECT_FALSE(lp.update(0.5, 0.0, 0.8, 0.5, 0.0, 0.0));
}

TEST(LegProgressAlongPath, AFarStretchOnTheGoalLineIsNotTheGoal)
{
  LegProgress lp = alongPath();
  lp.overshoot_xy_tol = 0.20;
  // goal (0, 0) heading +x; the leg first passes 1.4 m beyond it on the goal line
  lp.update(-2.0, 1.0, 0.0, 0.0, 0.0, 0.0);
  for (double x = -2.0; x <= 1.4; x += 0.01)
  {
    lp.update(x, 1.0, 0.0, 0.0, 0.0, 0.0);
  }
  EXPECT_FALSE(lp.update(1.4, 0.05, -M_PI / 2, 0.0, 0.0, 0.0));
  EXPECT_TRUE(lp.update(0.1, 0.05, M_PI, 0.0, 0.0, 0.0));
}
