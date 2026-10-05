// bumper_controller.h — bumper obstacle-avoidance safety controller.
//
// Reimplementation of libbumper_controller.so (namespace mower_controller), ported into
// the ROS 2 stack. The heading PID for the rotate-clear phase is inlined here (option b)
// so the package has no dependency on the deliberately-unported `pid_controller`.
//
// State machine (from mower_interfaces/MowerBaseDevStatus.bumper_routing_status):
//   IDLE        (0)  — no bumper contact, normal driving
//   BACKING_UP  (1)  — bumper hit: stop, back up, rotate clear, then return to IDLE
//
// The node reads bumper state from /mower_base/status and only routes while
// `bumper_routing_enabled` is set (owned by mower_base's /enable_bumper).
#pragma once
#include <cstdint>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <string>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace mower_controller {

class BumperController {
public:
    enum class State : uint8_t { IDLE = 0, BACKING_UP = 1 };

    struct Config {
        double back_distance = 0.30;   // meters to back up
        double back_speed    = -0.20;  // m/s (reverse)
        double rotate_angle  = 1.5708; // rad (~90 deg) first rotation step
        int    max_rotates   = 3;      // rotate-counter cap before surrender
        // Heading PID (rotate-clear), matching the recovered PidControllerROS defaults.
        double pid_kp          = 0.8;
        double pid_ki          = 0.0;
        double pid_kd          = 0.05;
        double pid_max_angular = 0.5;  // rad/s clamp
        double pid_tolerance   = 0.03; // rad convergence threshold
    };

    explicit BumperController(rclcpp::Node& node);
    BumperController(rclcpp::Node& node, Config cfg);

    // Call on every sensor update. Returns true when a state transition occurred.
    bool update(bool bumper, bool bumper_l, bool bumper_r, bool routing_enabled, double dt);

    // Inject a synthetic bumper hit (wired to /test_bumper_service).
    void injectBumper();

    // Service the routing loop (call from a timer). Returns true while actively
    // backing up / rotating.
    bool spinOnce();

    State state() const { return state_; }
    bool  routingEnabled() const { return routing_enabled_; }
    int   rotateCounter() const { return rotate_counter_; }

    // Accessors for the node to publish the bumper point cloud + routing status.
    bool bumperTriggered() const { return bumper_; }
    bool bumperLeft() const { return bumper_l_; }
    bool bumperRight() const { return bumper_r_; }

private:
    // --- inlined heading PID (replaces PidControllerROS) -------------------
    struct Pid {
        double kp = 1.0, ki = 0.0, kd = 0.0;
        double out_min = -1.0, out_max = 1.0;
        double prev_error = 0.0, integral = 0.0;
        void reset() { prev_error = 0.0; integral = 0.0; }
        // Positional PID with output clamp + conditional anti-windup.
        double calc(double setpoint, double measured, double dt);
    };

    void publishTwist(double linear, double angular);
    void enterBackingUp();
    void finishToIdle();
    void startRotate(double delta_yaw);   // relative yaw turn
    void stopRotate();
    double spinRotate(double dt, double* w);  // returns heading error; sets *w (rad/s)
    void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);

    static double yawFromQuaternion(double x, double y, double z, double w);
    static double angularDiff(double a, double b);

    rclcpp::Node& node_;
    Config cfg_;

    State state_ = State::IDLE;
    bool routing_enabled_ = false;
    bool bumper_ = false;
    bool bumper_l_ = false;
    bool bumper_r_ = false;
    int  rotate_counter_ = 0;

    // back-up progress: seconds elapsed in reverse (drives the fixed back duration)
    double back_time_ = 0.0;
    rclcpp::Time last_spin_time_;

    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> cmd_vel_pub_;
    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> odom_sub_;

    // heading state
    Pid pid_;
    double current_yaw_ = 0.0;
    bool have_yaw_ = false;
    double target_yaw_ = 0.0;
    bool rotate_active_ = false;
};

}  // namespace mower_controller
