// LegProgress (LegGoalChecker core): a short turn leg whose start already lies
// inside a loose xy tolerance must still be driven.
#include <gtest/gtest.h>

#include <cmath>
#include <random>

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
  lp.update(0.0, 0.6, 0.0, 0.5, 0.0, 0.0);
  for (double x = 0.0; x <= 0.8; x += 0.01)
  {
    // 0.6 m beside: outside the lateral band and never within pass_radius (0.5)
    EXPECT_FALSE(lp.update(x, 0.6, 0.0, 0.5, 0.0, 0.0));
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

TEST(LegProgressAlongPath, PassedWideAndMovingAwayIsReached)
{
  LegProgress lp = alongPath();
  // goal (0, 0) heading +x; the robot runs 0.3 m beside it (outside the lateral band)
  lp.update(-1.0, 0.3, 0.0, 0.0, 0.0, 0.0);
  bool done = false;
  double x = -1.0;
  for (; x <= 1.0 && !done; x += 0.01)
  {
    done = lp.update(x, 0.3, 0.0, 0.0, 0.0, 0.0);
  }
  EXPECT_TRUE(done);
  EXPECT_GT(x, 0.0);
  EXPECT_LT(x, 0.3);
}

// ---- goal drift (2026-10-16 field run 6): controller_server transforms the map-frame goal
// into odom every cycle, so a fixed vertex random-walks in odom with map -> odom. The same
// leg must still end at the vertex.

// Drives the robot (odom) 0.02 m/step along +x through a vertex at (2, 0) map, the goal seen
// in odom random-walking sigma_step per axis per call (seeded). Returns the robot's
// along-track position relative to the drifted vertex when the checker fired (2.0 = on the
// vertex), or NaN when it never fired within 3 m.
static double driveThroughDriftingVertex(LegProgress lp, unsigned seed, double sigma_step)
{
  std::mt19937 rng(seed);
  std::normal_distribution<double> n(0.0, sigma_step);
  double dx = 0.0, dy = 0.0;
  for (double x = 0.0; x <= 3.0; x += 0.02)
  {
    dx += n(rng);
    dy += n(rng);
    if (lp.update(x, 0.0, 0.0, 2.0 + dx, dy, 0.0))
    {
      return x - dx;
    }
  }
  return std::nan("");
}

TEST(LegProgressDrift, AlongPathEndsAtTheVertexWhileTheGoalDrifts)
{
  for (unsigned seed = 1; seed <= 20; ++seed)
  {
    LegProgress lp = alongPath();
    const double x = driveThroughDriftingVertex(lp, seed, 0.01);
    ASSERT_FALSE(std::isnan(x)) << "seed " << seed << ": the leg never ended";
    // fired at the vertex plane: at most the 0.02 m tolerance (+ one step / drift step)
    // short, and not past the 0.20 m overshoot box
    EXPECT_GE(x, 2.0 - 0.02 - 0.05) << "seed " << seed;
    EXPECT_LE(x, 2.0 + 0.20 + 0.05) << "seed " << seed;
  }
}

TEST(LegProgressDrift, DiscModeEndsAtTheGoalWhileTheGoalDrifts)
{
  for (unsigned seed = 1; seed <= 20; ++seed)
  {
    LegProgress lp;    // 0.08 m disc + yaw, overshoot 0.20 m
    const double x = driveThroughDriftingVertex(lp, seed, 0.01);
    ASSERT_FALSE(std::isnan(x)) << "seed " << seed << ": the leg never ended";
    EXPECT_GE(x, 2.0 - 0.08 - 0.05) << "seed " << seed;
    EXPECT_LE(x, 2.0 + 0.20 + 0.05) << "seed " << seed;
  }
}

TEST(LegProgressDrift, TheOldOneMillimetreRuleNeverEndsTheLeg)
{
  // documents the field failure: with the pre-fix 1 mm new-goal test every drift step
  // restarted the travel gate and the leg ran through the vertex
  int ended = 0;
  for (unsigned seed = 1; seed <= 20; ++seed)
  {
    LegProgress lp = alongPath();
    lp.new_goal_jump = 1e-3;
    ended += std::isnan(driveThroughDriftingVertex(lp, seed, 0.01)) ? 0 : 1;
  }
  EXPECT_LE(ended, 2);
}

TEST(LegProgressDrift, CumulativeDriftOnALongLegIsTheSameLeg)
{
  // a 10 m leg with a steady 2 mm/step map -> odom drift (1 m in total, more than the
  // 0.78 m worst 60 s drift of run 6): never a new leg, the travel gate keeps counting
  LegProgress lp = alongPath();
  bool done = false;
  double x = 0.0;
  for (int i = 0; x <= 12.0 && !done; ++i, x += 0.02)
  {
    done = lp.update(x, 0.0, 0.0, 10.0 + 0.002 * i, 0.0, 0.0);
  }
  EXPECT_TRUE(done);
  EXPECT_GT(lp.travelled, 10.0);
}

TEST(LegProgressDrift, AGoalJumpWithoutResetIsANewLeg)
{
  LegProgress lp = alongPath();
  for (double x = 0.0; x <= 0.5; x += 0.01)
  {
    lp.update(x, 0.0, 0.0, 1.0, 0.0, 0.0);
  }
  // 0.2 m step (field max map -> odom step 0.16 m): same leg, travel kept
  lp.update(0.5, 0.0, 0.0, 1.2, 0.0, 0.0);
  EXPECT_GT(lp.travelled, 0.45);
  // 0.3 m jump: a different goal (reset() was not called): travel restarts
  lp.update(0.5, 0.0, 0.0, 1.5, 0.0, 0.0);
  EXPECT_NEAR(lp.travelled, 0.0, 1e-9);
}
