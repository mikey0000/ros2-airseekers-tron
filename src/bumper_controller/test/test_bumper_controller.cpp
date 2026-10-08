// Bumper controller safety tests: edge-triggered, time-bounded (<= 2 s), suppressed
// while docked / charging / estop, and silent after a short zero burst.
#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "bumper_controller/bumper_controller.h"

using mower_controller::BumperController;

class BumperTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override {
        static int n = 0;
        node_ = std::make_shared<rclcpp::Node>("bumper_test_" + std::to_string(n++));
        ctrl_ = std::make_unique<BumperController>(*node_);
        sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_bumper", 100,
            [this](geometry_msgs::msg::Twist::SharedPtr m) { msgs_.push_back(*m); });
        exec_.add_node(node_);
    }
    void TearDown() override { exec_.remove_node(node_); }

    // One 20 Hz tick; returns the number of twists published in it.
    size_t tick(bool bumper, bool suppressed = false, bool routing = true) {
        ctrl_->update(bumper, bumper, false, routing, 0.05, suppressed);
        ctrl_->step(0.05);
        const size_t before = msgs_.size();
        for (int i = 0; i < 5; ++i) exec_.spin_some(std::chrono::milliseconds(5));
        return msgs_.size() - before;
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<BumperController> ctrl_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
    rclcpp::executors::SingleThreadedExecutor exec_;
    std::vector<geometry_msgs::msg::Twist> msgs_;
};

TEST_F(BumperTest, TimeBoundedEvenWhenRobotCannotTurn) {
    // No odom ever moves: the rotate phase can never converge (the docked bug).
    tick(true);
    int active = 0;
    for (int i = 0; i < 200; ++i) {
        tick(false);
        if (ctrl_->state() == BumperController::State::BACKING_UP) active++;
    }
    EXPECT_EQ(ctrl_->state(), BumperController::State::IDLE);
    EXPECT_LE(active * 0.05, 2.0 + 1e-9);
    // Silent afterwards.
    EXPECT_FALSE(ctrl_->publishing());
    EXPECT_EQ(tick(false), 0u);
    // The last messages are zeros.
    ASSERT_FALSE(msgs_.empty());
    EXPECT_DOUBLE_EQ(msgs_.back().linear.x, 0.0);
    EXPECT_DOUBLE_EQ(msgs_.back().angular.z, 0.0);
}

TEST_F(BumperTest, HeldBumperTriggersOnce) {
    for (int i = 0; i < 200; ++i) tick(true);   // stuck contact for 10 s
    EXPECT_EQ(ctrl_->state(), BumperController::State::IDLE);
    EXPECT_EQ(tick(true), 0u);
    // A fresh edge triggers again.
    tick(false);
    tick(true);
    EXPECT_EQ(ctrl_->state(), BumperController::State::BACKING_UP);
}

TEST_F(BumperTest, SuppressedWhileDockedOrEstop) {
    for (int i = 0; i < 20; ++i) {
        EXPECT_EQ(tick(i % 2 == 0, /*suppressed=*/true), 0u);
        EXPECT_EQ(ctrl_->state(), BumperController::State::IDLE);
    }
    ctrl_->injectBumper();
    EXPECT_EQ(ctrl_->state(), BumperController::State::IDLE);
}

TEST_F(BumperTest, SuppressionAbortsManoeuvreAndGoesSilent) {
    tick(true);
    tick(true);
    ASSERT_EQ(ctrl_->state(), BumperController::State::BACKING_UP);
    tick(true, /*suppressed=*/true);
    EXPECT_EQ(ctrl_->state(), BumperController::State::IDLE);
    for (int i = 0; i < 5; ++i) tick(false, true);
    EXPECT_EQ(tick(false, true), 0u);
    EXPECT_DOUBLE_EQ(msgs_.back().linear.x, 0.0);
    EXPECT_DOUBLE_EQ(msgs_.back().angular.z, 0.0);
}

TEST_F(BumperTest, RoutingDisabledNeverMoves) {
    EXPECT_EQ(tick(true, false, /*routing=*/false), 0u);
    EXPECT_EQ(ctrl_->state(), BumperController::State::IDLE);
}

TEST_F(BumperTest, DockReversePhaseFrontHitHoldsZero) {
    ctrl_->setDockPhase(mower_controller::DockPhase::REVERSE);
    tick(true);
    int active = 0;
    for (int i = 0; i < 100; ++i) {
        tick(false);
        if (ctrl_->state() == BumperController::State::BACKING_UP) active++;
    }
    EXPECT_EQ(ctrl_->state(), BumperController::State::IDLE);
    EXPECT_GE(active * 0.05, 0.9);       // held ~dock_hold_s
    EXPECT_LE(active * 0.05, 2.0 + 1e-9);
    ASSERT_FALSE(msgs_.empty());
    for (const auto& m : msgs_) {
        EXPECT_DOUBLE_EQ(m.linear.x, 0.0);
        EXPECT_DOUBLE_EQ(m.angular.z, 0.0);
    }
}

TEST_F(BumperTest, DockApproachPhaseShortReverseNoTurn) {
    ctrl_->setDockPhase(mower_controller::DockPhase::APPROACH);
    tick(true);
    for (int i = 0; i < 100; ++i) tick(false);
    EXPECT_EQ(ctrl_->state(), BumperController::State::IDLE);
    double dist = 0.0;
    for (const auto& m : msgs_) {
        EXPECT_LE(m.linear.x, 0.0);
        EXPECT_DOUBLE_EQ(m.angular.z, 0.0);
        dist += -m.linear.x * 0.05;
    }
    EXPECT_NEAR(dist, 0.10, 0.03);
}

TEST_F(BumperTest, DockPhaseIsLatchedAtStart) {
    // Phase changes mid-manoeuvre do not alter the running plan.
    ctrl_->setDockPhase(mower_controller::DockPhase::REVERSE);
    tick(true);
    ctrl_->setDockPhase(mower_controller::DockPhase::NONE);
    for (int i = 0; i < 100; ++i) tick(false);
    for (const auto& m : msgs_) EXPECT_DOUBLE_EQ(m.linear.x, 0.0);
}
