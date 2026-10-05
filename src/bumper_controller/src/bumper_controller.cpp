// bumper_controller.cpp — BumperController state machine implementation (ported
// from the recovered libbumper_controller.so; heading PID inlined).
#include "bumper_controller/bumper_controller.h"

#include <cmath>

namespace mower_controller {

namespace {
constexpr double kPi = 3.14159265358979323846;
}  // namespace

double BumperController::Pid::calc(double setpoint, double measured, double dt) {
    if (dt <= 0.0) return out_min < 0.0 ? 0.0 : out_min;
    const double error = setpoint - measured;
    integral_ += error * dt;
    const double deriv = (error - prev_error_) / dt;
    double out = kp * error + ki * integral_ + kd * deriv;

    if (out > out_max_)      out = out_max_;
    else if (out < out_min_) out = out_min_;
    // (else: not saturated — keep the integral as-is)

    prev_error_ = error;
    return out;
}

double BumperController::yawFromQuaternion(double x, double y, double z, double w) {
    // yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

double BumperController::angularDiff(double a, double b) {
    return std::remainder(b - a, 2.0 * kPi);
}

BumperController::BumperController(rclcpp::Node& node, Config cfg)
    : node_(node), cfg_(cfg),
      cmd_vel_pub_(node_.create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10)),
      odom_sub_(node_.create_subscription<nav_msgs::msg::Odometry>(
          "/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr m) { onOdom(m); })) {
    // Heading PID tuning, overridable via node parameters (same names as the
    // recovered PidControllerROS).
    cfg_.pid_kp          = node_.declare_parameter<double>("pid_kp", cfg_.pid_kp);
    cfg_.pid_ki          = node_.declare_parameter<double>("pid_ki", cfg_.pid_ki);
    cfg_.pid_kd          = node_.declare_parameter<double>("pid_kd", cfg_.pid_kd);
    cfg_.pid_max_angular = node_.declare_parameter<double>("pid_max_angular", cfg_.pid_max_angular);
    cfg_.pid_tolerance   = node_.declare_parameter<double>("pid_tolerance", cfg_.pid_tolerance);

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
                              bool routing_enabled, double /*dt*/) {
    bumper_ = bumper;
    bumper_l_ = bumper_l;
    bumper_r_ = bumper_r;
    routing_enabled_ = routing_enabled;

    if (bumper_ && routing_enabled_ && state_ == State::IDLE) {
        enterBackingUp();
        return true;
    }
    return false;
}

void BumperController::injectBumper() {
    if (routing_enabled_ && state_ == State::IDLE) {
        enterBackingUp();
    }
}

bool BumperController::spinOnce() {
    if (state_ != State::BACKING_UP) return false;

    const auto now = node_.get_clock()->now();
    double dt = 0.02;   // assume a ~50 Hz control timer when no prior tick exists
    if (last_spin_time_.nanoseconds() != 0) {
        dt = (now - last_spin_time_).seconds();
        if (dt <= 0.0 || dt > 1.0) dt = 0.02;
    }
    last_spin_time_ = now;

    geometry_msgs::msg::Twist t;

    if (rotate_counter_ == 0) {
        // Phase 1: back up in reverse for a fixed duration.
        t.linear.x = cfg_.back_speed;
        cmd_vel_pub_->publish(t);
        back_time_ += dt;
        const double back_duration = cfg_.back_distance / std::fabs(cfg_.back_speed);
        if (back_time_ >= back_duration) {
            rotate_counter_ = cfg_.max_rotates;
            const double sign = (rotate_counter_ % 2 == 0) ? 1.0 : -1.0;
            startRotate(cfg_.rotate_angle * sign);
        }
        return true;
    }

    // Phase 2: rotating clear. The heading PID drives yaw; it self-deactivates on
    // convergence. A single /cmd_vel publish carries both the (zero) linear and the
    // computed angular command.
    double w = 0.0;
    const double err = spinRotate(dt, &w);
    geometry_msgs::msg::Twist rt;
    rt.linear.x = 0.0;
    rt.angular.z = w;
    cmd_vel_pub_->publish(rt);
    if (std::fabs(err) < 1e-3 && !rotate_active_) {
        rotate_counter_--;
        if (rotate_counter_ <= 0) {
            finishToIdle();
            return false;
        }
        // try a fresh rotation attempt (alternate direction)
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

void BumperController::enterBackingUp() {
    state_ = State::BACKING_UP;
    back_time_ = 0.0;
    rotate_counter_ = 0;
}

void BumperController::finishToIdle() {
    state_ = State::IDLE;
    stopRotate();
    geometry_msgs::msg::Twist t;
    t.linear.x = 0.0;
    t.angular.z = 0.0;
    cmd_vel_pub_->publish(t);
}

}  // namespace mower_controller
