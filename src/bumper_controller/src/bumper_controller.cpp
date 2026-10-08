// bumper_controller.cpp — BumperController state machine implementation (ported
// from the recovered libbumper_controller.so; heading PID inlined).
#include "bumper_controller/bumper_controller.h"

#include <algorithm>
#include <cmath>

namespace mower_controller {

namespace {
constexpr double kPi = 3.14159265358979323846;
}  // namespace

double BumperController::Pid::calc(double setpoint, double measured, double dt) {
    if (dt <= 0.0) return 0.0;
    const double error = setpoint - measured;
    integral += error * dt;
    const double deriv = (error - prev_error) / dt;
    double out = kp * error + ki * integral + kd * deriv;

    if (out > out_max)      out = out_max;
    else if (out < out_min) out = out_min;
    // (else: not saturated — keep the integral as-is)

    prev_error = error;
    return out;
}

double BumperController::yawFromQuaternion(double x, double y, double z, double w) {
    // yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

double BumperController::angularDiff(double a, double b) {
    return std::remainder(b - a, 2.0 * kPi);
}

BumperController::BumperController(rclcpp::Node& node)
    : BumperController(node, Config()) {}

BumperController::BumperController(rclcpp::Node& node, Config cfg)
    : node_(node), cfg_(cfg),
      cmd_vel_pub_(node_.create_publisher<geometry_msgs::msg::Twist>(
          node_.declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_bumper"), 10)),
      odom_sub_(node_.create_subscription<nav_msgs::msg::Odometry>(
          "/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr m) { onOdom(m); })) {
    // Heading PID tuning, overridable via node parameters (same names as the
    // recovered PidControllerROS).
    cfg_.pid_kp          = node_.declare_parameter<double>("pid_kp", cfg_.pid_kp);
    cfg_.pid_ki          = node_.declare_parameter<double>("pid_ki", cfg_.pid_ki);
    cfg_.pid_kd          = node_.declare_parameter<double>("pid_kd", cfg_.pid_kd);
    cfg_.pid_max_angular = node_.declare_parameter<double>("pid_max_angular", cfg_.pid_max_angular);
    cfg_.pid_tolerance   = node_.declare_parameter<double>("pid_tolerance", cfg_.pid_tolerance);
    cfg_.max_duration_s  = node_.declare_parameter<double>("max_duration_s", cfg_.max_duration_s);
    if (cfg_.max_duration_s > 2.0 || cfg_.max_duration_s <= 0.0) cfg_.max_duration_s = 2.0;
    cfg_.dock_backoff_distance = std::clamp(node_.declare_parameter<double>(
        "dock_backoff_distance", cfg_.dock_backoff_distance), 0.0, 0.3);
    cfg_.dock_rear_backoff_distance = std::clamp(node_.declare_parameter<double>(
        "dock_rear_backoff_distance", cfg_.dock_rear_backoff_distance), 0.0, 0.3);
    cfg_.dock_hold_s = std::clamp(node_.declare_parameter<double>(
        "dock_hold_s", cfg_.dock_hold_s), 0.0, 2.0);

    pid_.kp = cfg_.pid_kp;
    pid_.ki = cfg_.pid_ki;
    pid_.kd = cfg_.pid_kd;
    pid_.out_min = -cfg_.pid_max_angular;
    pid_.out_max =  cfg_.pid_max_angular;
}

void BumperController::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const auto& q = msg->pose.pose.orientation;
    current_yaw_ = yawFromQuaternion(q.x, q.y, q.z, q.w);
    have_yaw_ = true;
}

bool BumperController::update(bool bumper, bool bumper_l, bool bumper_r,
                              bool routing_enabled, double /*dt*/, bool suppressed,
                              bool bumper_rear) {
    const bool any = bumper || bumper_rear;
    const bool rising = any && !prev_bumper_;
    prev_bumper_ = any;
    bumper_ = bumper;
    bumper_l_ = bumper_l;
    bumper_r_ = bumper_r;
    routing_enabled_ = routing_enabled;
    suppressed_ = suppressed;

    if (suppressed_) {
        if (state_ == State::BACKING_UP) abortToIdle("suppressed (docked/charging/estop/lift)");
        return false;
    }
    if (rising && routing_enabled_ && state_ == State::IDLE) {
        enterBackingUp(bumper, bumper_rear);
        return true;
    }
    return false;
}

void BumperController::injectBumper() {
    if (routing_enabled_ && !suppressed_ && state_ == State::IDLE) {
        enterBackingUp(true, false);
    }
}

bool BumperController::spinOnce() {
    const auto now = node_.get_clock()->now();
    double dt = 0.05;   // nominal 20 Hz node timer when no prior tick exists
    if (last_spin_time_.nanoseconds() != 0) {
        dt = (now - last_spin_time_).seconds();
        if (dt <= 0.0 || dt > 1.0) dt = 0.05;
    }
    last_spin_time_ = now;
    return step(dt);
}

bool BumperController::step(double dt) {
    if (state_ != State::BACKING_UP) {
        // Trailing zero burst after a manoeuvre, then silence.
        if (zero_ticks_left_ > 0) {
            zero_ticks_left_--;
            publishTwist(0.0, 0.0);
        }
        return false;
    }
    if (suppressed_) {
        abortToIdle("suppressed");
        return false;
    }
    elapsed_ += dt;
    if (elapsed_ > cfg_.max_duration_s) {
        abortToIdle("time bound");
        return false;
    }

    if (rotate_counter_ == 0) {
        // Phase 1: straight leg (reverse, or forward on a rear hit while docking) for a
        // fixed duration, as chosen by decideManoeuvre() at the start.
        const double leg = (plan_.distance > 0.0 && plan_.speed != 0.0)
                               ? plan_.distance / std::fabs(plan_.speed) : 0.0;
        if (back_time_ < leg) {
            publishTwist(plan_.speed, 0.0);
            back_time_ += dt;
            return true;
        }
        if (plan_.rotate) {
            rotate_counter_ = cfg_.max_rotates;
            const double sign = (rotate_counter_ % 2 == 0) ? 1.0 : -1.0;
            startRotate(cfg_.rotate_angle * sign);
            publishTwist(0.0, 0.0);
            return true;
        }
        // Docking phases: hold still (zero on the priority lane) so the docking node's
        // bumper pause/resume re-plans; then go idle.
        if (hold_time_ < plan_.hold_s) {
            publishTwist(0.0, 0.0);
            hold_time_ += dt;
            return true;
        }
        finishToIdle();
        return false;
    }

    // Phase 2: rotating clear (heading PID); bounded by max_duration_s above, so a
    // robot that cannot turn (docked, estop, wheels blocked) no longer spins forever.
    double w = 0.0;
    const double err = spinRotate(dt, &w);
    publishTwist(0.0, w);
    if (std::fabs(err) < 1e-3 && !rotate_active_) {
        rotate_counter_--;
        if (rotate_counter_ <= 0) {
            finishToIdle();
            return false;
        }
        const double sign = (rotate_counter_ % 2 == 0) ? 1.0 : -1.0;
        startRotate(cfg_.rotate_angle * sign);
    }
    return true;
}

void BumperController::startRotate(double delta_yaw) {
    target_yaw_ = current_yaw_ + delta_yaw;   // relative turn
    rotate_active_ = have_yaw_;               // only start if we have a heading
    pid_.reset();
}

void BumperController::stopRotate() {
    rotate_active_ = false;
}

double BumperController::spinRotate(double dt, double* w) {
    *w = 0.0;
    if (!rotate_active_ || !have_yaw_) return 0.0;

    const double err = angularDiff(current_yaw_, target_yaw_);
    if (std::fabs(err) < cfg_.pid_tolerance) {
        rotate_active_ = false;
        return 0.0;
    }

    *w = pid_.calc(0.0, -err, dt);   // drive error -> 0 (rad/s)
    return err;
}

void BumperController::enterBackingUp(bool front, bool rear) {
    DecisionConfig dc;
    dc.back_distance = cfg_.back_distance;
    dc.back_speed = cfg_.back_speed;
    dc.dock_backoff_distance = cfg_.dock_backoff_distance;
    dc.dock_rear_backoff_distance = cfg_.dock_rear_backoff_distance;
    dc.dock_hold_s = cfg_.dock_hold_s;
    plan_ = decideManoeuvre(dock_phase_, front, rear, dc);
    hold_time_ = 0.0;
    if (dock_phase_ != DockPhase::NONE) {
        RCLCPP_WARN(node_.get_logger(),
                    "bumper in docking phase %d (front=%d rear=%d): %.2f m at %.2f m/s, hold %.1f s",
                    static_cast<int>(dock_phase_), front, rear, plan_.distance, plan_.speed,
                    plan_.hold_s);
    }
    state_ = State::BACKING_UP;
    back_time_ = 0.0;
    elapsed_ = 0.0;
    rotate_counter_ = 0;
    zero_ticks_left_ = 0;
}

void BumperController::finishToIdle() {
    state_ = State::IDLE;
    stopRotate();
    rotate_counter_ = 0;
    publishTwist(0.0, 0.0);
    zero_ticks_left_ = cfg_.zero_ticks > 0 ? cfg_.zero_ticks - 1 : 0;
}

void BumperController::abortToIdle(const char* why) {
    RCLCPP_WARN(node_.get_logger(), "bumper manoeuvre aborted: %s (%.2f s)", why, elapsed_);
    finishToIdle();
}

void BumperController::publishTwist(double linear, double angular) {
    geometry_msgs::msg::Twist t;
    t.linear.x = linear;
    t.angular.z = angular;
    cmd_vel_pub_->publish(t);
}

}  // namespace mower_controller
