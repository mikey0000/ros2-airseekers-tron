// FTC row law (ftc_row_law.hpp): path geometry, the upstream PID terms, the carrot, the row
// gate, and closed-loop row tracking on a unicycle with the Tron's command delay.
#include <gtest/gtest.h>

#include <cmath>
#include <deque>

#include "mowgli_nav2_plugins/ftc_row_law.hpp"

namespace fr = mowgli_nav2_plugins::ftc_row;

namespace
{
// Straight row along +x from 0 to `len`, 5 cm spacing, then (optionally) a left U-turn arc of
// radius r and the neighbouring row 2r to the left, driven back along -x.
fr::Path rowWithTurn(double len, double r, bool turn)
{
  fr::Path p;
  for (double s = 0.0; s <= len + 1e-9; s += 0.05) {
    p.push(s, 0.0);
  }
  if (!turn) {
    return p;
  }
  for (double a = 0.05 / r; a <= M_PI + 1e-9; a += 0.05 / r) {
    p.push(len + r * std::sin(a), r - r * std::cos(a));
  }
  for (double s = 0.05; s <= len + 1e-9; s += 0.05) {
    p.push(len - s, 2.0 * r);
  }
  return p;
}
}  // namespace

TEST(FtcRowLaw, PathPointAtAndDuplicates)
{
  fr::Path p;
  p.push(0.0, 0.0);
  p.push(0.0, 0.0);    // duplicate dropped
  p.push(1.0, 0.0);
  p.push(1.0, 1.0);
  ASSERT_EQ(p.size(), 3u);
  EXPECT_DOUBLE_EQ(p.length(), 2.0);
  auto a = fr::pointAt(p, 0.25);
  EXPECT_NEAR(a.x, 0.25, 1e-12);
  EXPECT_NEAR(a.yaw, 0.0, 1e-12);
  auto b = fr::pointAt(p, 1.5);
  EXPECT_NEAR(b.x, 1.0, 1e-12);
  EXPECT_NEAR(b.y, 0.5, 1e-12);
  EXPECT_NEAR(b.yaw, M_PI / 2, 1e-12);
  auto c = fr::pointAt(p, 9.0);   // clamped to the end
  EXPECT_NEAR(c.y, 1.0, 1e-12);
}

TEST(FtcRowLaw, ProjectionSignsAndForwardWindowStaysOnTheRow)
{
  const fr::Path p = rowWithTurn(3.0, 0.5, true);
  // 4 cm left of the first row, heading 0.1 rad left
  auto pr = fr::project(p, 1.0, 0.04, 0.1, 0, 2.0);
  ASSERT_TRUE(pr.ok);
  EXPECT_NEAR(pr.s, 1.0, 1e-9);
  EXPECT_NEAR(pr.lateral, 0.04, 1e-9);
  EXPECT_NEAR(pr.heading, 0.1, 1e-9);
  // 5 cm from the neighbouring row (1.0 m to the left): a forward window from row 1 keeps
  // the foot point on row 1, an unrestricted search jumps to the neighbour.
  const size_t seg = pr.seg;
  pr = fr::project(p, 1.0, 0.95, 0.0, seg, 1.0);
  ASSERT_TRUE(pr.ok);
  EXPECT_NEAR(pr.s, 1.0, 1e-9);
  EXPECT_NEAR(pr.lateral, 0.95, 1e-9);
  auto far = fr::project(p, 1.0, 0.95, 0.0, 0, 100.0);
  EXPECT_GT(far.s, 3.0);
}

TEST(FtcRowLaw, StraightAheadStopsAtTheBendAndThePathEnd)
{
  const fr::Path p = rowWithTurn(3.0, 0.5, true);
  EXPECT_TRUE(fr::straightAhead(p, 1.0, 1.9, 0.01));
  EXPECT_TRUE(fr::straightAhead(p, 1.0, 2.0, 0.01));
  // 0.2 m into the arc: the vertices leave the chord by more than 1 cm
  EXPECT_FALSE(fr::straightAhead(p, 1.0, 2.2, 0.01));
  const fr::Path q = rowWithTurn(3.0, 0.5, false);
  EXPECT_TRUE(fr::straightAhead(q, 1.0, 2.0, 0.01));
  EXPECT_FALSE(fr::straightAhead(q, 1.0, 2.05, 0.01));   // beyond the end
}

TEST(FtcRowLaw, CommandIsTheUpstreamPid)
{
  fr::Gains g;
  g.kp_lon = 1.5;
  g.kp_lat = 2.0;
  g.ki_lat = 0.5;
  g.ki_lat_max = 0.03;
  g.kd_lat = 0.3;
  g.kp_ang = 1.1;
  g.kd_ang = 0.2;
  fr::Law law;
  law.start(0.0, 0.0, g);
  // first cycle: no derivative yet
  auto c = law.command(0.1, 0.02, 0.05, 0.05, 0.3, 0.3, g);
  EXPECT_NEAR(c.v, 0.15, 1e-12);
  EXPECT_NEAR(c.w, 1.1 * 0.05 + 2.0 * 0.02 + 0.5 * 0.02 * 0.05, 1e-12);
  // second cycle: derivatives by difference, integral accumulates
  c = law.command(0.1, 0.03, 0.04, 0.05, 0.3, 0.3, g);
  const double i = 0.02 * 0.05 + 0.03 * 0.05;
  EXPECT_NEAR(c.w, 1.1 * 0.04 + 0.2 * (-0.01 / 0.05) + 2.0 * 0.03 + 0.5 * i +
    0.3 * (0.01 / 0.05), 1e-12);
  // integral windup clamp (upstream ki_lat_max on the integral itself)
  for (int k = 0; k < 1000; ++k) {
    law.command(0.1, 0.1, 0.0, 0.05, 0.3, 0.3, g);
  }
  EXPECT_NEAR(law.integral(), 0.03, 1e-12);
  // forward only, and the speed / turn-rate limits
  c = law.command(-0.2, 0.0, 0.0, 0.05, 0.3, 0.3, g);
  EXPECT_EQ(c.v, 0.0);
  c = law.command(1.0, 1.0, 1.0, 0.05, 0.25, 0.3, g);
  EXPECT_NEAR(c.v, 0.25, 1e-12);
  EXPECT_NEAR(c.w, 0.3, 1e-12);
}

TEST(FtcRowLaw, CarrotRampsAndStaysBetweenRobotAndMaxLead)
{
  fr::Gains g;
  g.kp_lon = 1.2;
  g.acceleration = 0.3;
  fr::Law law;
  law.start(1.0, 0.12, g);
  EXPECT_NEAR(law.carrot(), 1.0 + 0.12 / 1.2, 1e-12);
  law.advance(1.0, 0.3, 10.0, 0.1, g);
  EXPECT_NEAR(law.speed(), 0.15, 1e-12);          // +0.3 m/s^2 x 0.1 s
  EXPECT_NEAR(law.carrot(), 1.1 + 0.015, 1e-12);
  // the robot is held back (downstream slowdown): the carrot waits at max_lead
  for (int k = 0; k < 100; ++k) {
    law.advance(1.0, 0.3, 0.375, 0.05, g);
  }
  EXPECT_NEAR(law.carrot(), 1.375, 1e-12);
  // the robot jumps ahead of the carrot (new foot point): the carrot is never behind it
  law.advance(2.0, 0.3, 0.375, 0.05, g);
  EXPECT_GE(law.carrot(), 2.0);
}

TEST(FtcRowLaw, GateHysteresisAndHandoffBeforeTheBend)
{
  const fr::Path p = rowWithTurn(3.0, 0.5, true);
  fr::Gate g;   // tol 0.01, handoff 0.4, lateral 0.15, angle 0.35
  const double lead = 0.375;
  auto pr = fr::project(p, 1.0, 0.0, 0.0, 0, 2.0);
  EXPECT_TRUE(fr::mayEnter(p, pr, lead, g));
  pr = fr::project(p, 1.0, 0.1, 0.0, 0, 2.0);       // 0.10 > 0.075: no entry ...
  EXPECT_FALSE(fr::mayEnter(p, pr, lead, g));
  EXPECT_TRUE(fr::mayStay(p, pr, pr.s + 0.25, g));   // ... but a running row continues
  pr = fr::project(p, 1.0, 0.0, 0.3, 0, 2.0);
  EXPECT_FALSE(fr::mayEnter(p, pr, lead, g));
  EXPECT_TRUE(fr::mayStay(p, pr, pr.s + 0.25, g));
  pr = fr::project(p, 1.0, 0.0, 0.4, 0, 2.0);
  EXPECT_FALSE(fr::mayStay(p, pr, pr.s + 0.25, g));
  // entry needs handoff + 2 x lead = 1.15 m of straight; at s 2.0 only 1.0 m is left
  pr = fr::project(p, 2.0, 0.0, 0.0, 0, 3.0);
  EXPECT_FALSE(fr::mayEnter(p, pr, lead, g));
  // a running row hands off when the bend comes within handoff of the carrot
  EXPECT_TRUE(fr::mayStay(p, pr, 2.5, g));    // carrot 2.5 + 0.4 < 3.0
  // 0.2 m into the r 0.5 arc the vertices are 4 cm off the chord (at 0.1 m only 1 cm: the
  // 1 cm tolerance sees an r 0.5 bend about 0.1-0.2 m after it starts)
  EXPECT_FALSE(fr::mayStay(p, pr, 2.8, g));
}

// Closed loop on a unicycle with the Tron's command delay (0.2 s) and wheel lag (0.1 s),
// 20 Hz control, the 0.3 rad/s turn clamp: from 8 cm / 0.15 rad off the row the robot settles
// on the line without a growing oscillation, and holds the target speed.
TEST(FtcRowLaw, ClosedLoopSettlesOnTheRow)
{
  fr::Path p;
  for (double s = 0.0; s <= 30.0 + 1e-9; s += 0.05) {
    p.push(s, 0.0);
  }
  fr::Gains g;   // the Tron defaults
  fr::Law law;
  double x = 0.5, y = 0.08, yaw = -0.15, v = 0.2, w = 0.0;
  const double dt = 0.005;
  std::deque<fr::Command> delay(static_cast<size_t>(0.2 / dt), fr::Command{0.2, 0.0});
  fr::Command cmd{0.2, 0.0};
  size_t seg = 0;
  auto pr = fr::project(p, x, y, yaw, seg, 2.0);
  law.start(pr.s, v, g);
  double peak_late = 0.0, peak_early = 0.0;
  for (int k = 0; k < static_cast<int>(60.0 / dt); ++k) {
    if (k % 10 == 0) {
      pr = fr::project(p, x, y, yaw, seg, 1.0);
      seg = pr.seg;
      law.advance(pr.s, 0.3, 1.5 * 0.3 / g.kp_lon, 0.05, g);
      cmd = law.step(p, x, y, yaw, 0.05, 0.3, 0.3, g);
    }
    delay.push_back(cmd);
    const fr::Command a = delay.front();
    delay.pop_front();
    v += (a.v - v) * dt / 0.1;
    w += (a.w - w) * dt / 0.1;
    yaw += w * dt;
    x += v * std::cos(yaw) * dt;
    y += v * std::sin(yaw) * dt;
    const double t = k * dt;
    if (t > 5.0 && t < 15.0) {
      peak_early = std::max(peak_early, std::fabs(y));
    }
    if (t > 30.0) {
      peak_late = std::max(peak_late, std::fabs(y));
    }
  }
  EXPECT_LT(peak_early, 0.03);
  EXPECT_LT(peak_late, 0.002);
  EXPECT_NEAR(v, 0.3, 0.01);
  // steady lead v / kp_lon (measured between two carrot updates: within one 0.05 s step)
  EXPECT_NEAR(law.carrot() - fr::project(p, x, y, yaw, seg, 1.0).s, 0.3 / g.kp_lon, 0.02);
}
