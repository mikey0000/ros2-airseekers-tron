// Pure decision-table tests for the docking-aware bumper manoeuvre (no ROS).
#include <gtest/gtest.h>

#include "bumper_controller/bumper_decision.h"

using namespace mower_controller;

TEST(DockPhase, MapsDockingStates) {
    EXPECT_EQ(dockPhaseFromState("NAV_TO_APPROACH"), DockPhase::APPROACH);
    EXPECT_EQ(dockPhaseFromState("ALIGNING"), DockPhase::APPROACH);
    EXPECT_EQ(dockPhaseFromState("SEARCHING"), DockPhase::APPROACH);
    EXPECT_EQ(dockPhaseFromState("DOCKING"), DockPhase::REVERSE);
    EXPECT_EQ(dockPhaseFromState("FINAL_DOCKING"), DockPhase::REVERSE);
    EXPECT_EQ(dockPhaseFromState("RETRY"), DockPhase::REVERSE);
    for (const char* s : {"", "CHARGING", "SUCCEEDED", "FAILED", "MOWING", "garbage"})
        EXPECT_EQ(dockPhaseFromState(s), DockPhase::NONE) << s;
}

TEST(Decision, NotDockingIsLegacy) {
    DecisionConfig c;
    auto p = decideManoeuvre(DockPhase::NONE, true, false, c);
    EXPECT_DOUBLE_EQ(p.distance, c.back_distance);
    EXPECT_LT(p.speed, 0.0);
    EXPECT_TRUE(p.rotate);
    EXPECT_DOUBLE_EQ(p.hold_s, 0.0);
}

TEST(Decision, ReversePhaseFrontHitHoldsNeverMovesTowardDock) {
    DecisionConfig c;
    auto p = decideManoeuvre(DockPhase::REVERSE, true, false, c);
    EXPECT_DOUBLE_EQ(p.distance, 0.0);
    EXPECT_FALSE(p.rotate);
    EXPECT_DOUBLE_EQ(p.hold_s, c.dock_hold_s);
    // both bumpers: hold too
    p = decideManoeuvre(DockPhase::REVERSE, true, true, c);
    EXPECT_DOUBLE_EQ(p.distance, 0.0);
    EXPECT_FALSE(p.rotate);
}

TEST(Decision, ReversePhaseRearHitBacksOffForward) {
    DecisionConfig c;
    auto p = decideManoeuvre(DockPhase::REVERSE, false, true, c);
    EXPECT_DOUBLE_EQ(p.distance, 0.15);
    EXPECT_GT(p.speed, 0.0);
    EXPECT_FALSE(p.rotate);
    EXPECT_DOUBLE_EQ(p.hold_s, c.dock_hold_s);
}

TEST(Decision, ApproachPhaseShortReverseNoTurn) {
    DecisionConfig c;
    auto p = decideManoeuvre(DockPhase::APPROACH, true, false, c);
    EXPECT_DOUBLE_EQ(p.distance, 0.10);
    EXPECT_LT(p.speed, 0.0);
    EXPECT_FALSE(p.rotate);
    EXPECT_DOUBLE_EQ(p.hold_s, c.dock_hold_s);
    p = decideManoeuvre(DockPhase::APPROACH, false, true, c);   // rear only: hold
    EXPECT_DOUBLE_EQ(p.distance, 0.0);
    EXPECT_FALSE(p.rotate);
}
