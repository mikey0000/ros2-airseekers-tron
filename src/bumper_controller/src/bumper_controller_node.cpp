// bumper_controller_node.cpp — runs BumperController: bumper hit -> back up + rotate
// clear, publishing the bumper cloud and the routing state for mower_base to fold into
// MowerBaseDevStatus.bumper_routing_status.
//
// Trigger source: /mower_base/status (MowerBaseDevStatus) carries bumper_triggered,
// left/right_bumper_triggered and bumper_routing_enabled. The instant bumper safety
// cutoff is enforced by the MCU / mower_base; this node does the higher-level obstacle
// routing (IDLE <-> BACKING_UP).
#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_srvs/srv/empty.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "mower_interfaces/msg/mower_base_dev_status.hpp"

#include "bumper_controller/bumper_controller.h"

namespace {

// empty = the fields only, width 0: the between-contacts heartbeat (see onTick).
sensor_msgs::msg::PointCloud2 makeBumperCloud(bool left, bool right, const rclcpp::Time& stamp,
                                              bool empty = false) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = "base_link";
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.point_step = 16;
    cloud.fields.resize(3);
    const uint8_t f32 = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[0].name = "x"; cloud.fields[0].offset = 0; cloud.fields[0].datatype = f32; cloud.fields[0].count = 1;
    cloud.fields[1].name = "y"; cloud.fields[1].offset = 4; cloud.fields[1].datatype = f32; cloud.fields[1].count = 1;
    cloud.fields[2].name = "z"; cloud.fields[2].offset = 8; cloud.fields[2].datatype = f32; cloud.fields[2].count = 1;

    auto add = [&cloud](float x, float y, float z) {
        cloud.width++;
        cloud.data.resize(cloud.width * cloud.point_step);
        float* p = reinterpret_cast<float*>(cloud.data.data() + (cloud.width - 1) * cloud.point_step);
        p[0] = x; p[1] = y; p[2] = z; p[3] = 1.0f;
    };
    if (!empty) {
        if (left)  add(0.4f,  0.2f, 0.0f);
        if (right) add(0.4f, -0.2f, 0.0f);
        if (!left && !right) add(0.4f, 0.0f, 0.0f);
    }
    cloud.row_step = cloud.width * cloud.point_step;
    return cloud;
}

}  // namespace

class BumperControllerNode : public rclcpp::Node {
public:
    BumperControllerNode() : Node("bumper_controller_node"), ctrl_(*this) {
        using namespace std::chrono_literals;

        status_sub_ = create_subscription<mower_interfaces::msg::MowerBaseDevStatus>(
            "/mower_base/status", 10,
            [this](const mower_interfaces::msg::MowerBaseDevStatus::SharedPtr m) { onStatus(m); });

        // Docking FSM state (mower_docking republishes it every control tick while a dock
        // goal runs, '' when idle). Mission high_level_status only says "docking", not
        // which docking sub-phase, so this is the phase source. Stale -> NONE (legacy).
        dock_state_timeout_s_ = declare_parameter<double>("dock_state_timeout_s", 1.0);
        dock_state_sub_ = create_subscription<std_msgs::msg::String>(
            declare_parameter<std::string>("dock_state_topic", "/mower_docking/state"), 10,
            [this](const std_msgs::msg::String::SharedPtr m) {
                dock_state_ = m->data;
                dock_state_t_ = now();
            });

        test_bumper_srv_ = create_service<std_srvs::srv::Empty>(
            "/test_bumper_service",
            [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
                   std::shared_ptr<std_srvs::srv::Empty::Response>) { ctrl_.injectBumper(); });

        cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/bumper_cloud", 10);
        routing_pub_ = create_publisher<std_msgs::msg::UInt8>("/mower_base/bumper_routing_status", 10);

        timer_ = create_wall_timer(50ms, [this]() { onTick(); });
        RCLCPP_INFO(get_logger(), "bumper_controller_node up");
    }

private:
    void onStatus(const mower_interfaces::msg::MowerBaseDevStatus::SharedPtr m) {
        latest_ = *m;
    }

    void onTick() {
        if (!latest_) return;
        // Never manoeuvre on/at the dock, while charging, with the stop button /
        // estop latched or the mower lifted (dock contacts + latched interlock
        // previously left a rotate-clear spinning at -0.5 rad/s forever).
        const bool suppressed = latest_->is_docking_done || latest_->is_charging ||
                                latest_->stop_triggered || latest_->lift_triggered;
        mower_controller::DockPhase phase = mower_controller::DockPhase::NONE;
        if (dock_state_t_.nanoseconds() != 0 &&
            (now() - dock_state_t_).seconds() <= dock_state_timeout_s_) {
            phase = mower_controller::dockPhaseFromState(dock_state_);
        }
        ctrl_.setDockPhase(phase);
        ctrl_.update(latest_->bumper_triggered,
                     latest_->left_bumper_triggered,
                     latest_->right_bumper_triggered,
                     latest_->bumper_routing_enabled, 0.05, suppressed);
        ctrl_.spinOnce();

        // Publish the routing state + bumper cloud.
        std_msgs::msg::UInt8 routing;
        routing.data = (ctrl_.state() == mower_controller::BumperController::State::BACKING_UP) ? 1 : 0;
        routing_pub_->publish(routing);

        if (ctrl_.bumperTriggered()) {
            cloud_pub_->publish(makeBumperCloud(ctrl_.bumperLeft(), ctrl_.bumperRight(),
                                                get_clock()->now()));
            heartbeat_tick_ = 0;
        } else if (++heartbeat_tick_ >= 4) {
            // 2026-10-09: empty cloud at 5 Hz between contacts. The Nav2 collision_monitor
            // keeps the LAST message of each source and WARNs "Ignoring the source" on every
            // velocity command once it is older than source_timeout (1 s): after the first
            // bump that was a 20 Hz WARN for the rest of the run. Empty clouds mark nothing in
            // the costmaps (marking-only source, observation_persistence unchanged).
            heartbeat_tick_ = 0;
            cloud_pub_->publish(makeBumperCloud(false, false, get_clock()->now(), true));
        }
    }

    mower_controller::BumperController ctrl_;
    rclcpp::Subscription<mower_interfaces::msg::MowerBaseDevStatus>::SharedPtr status_sub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr test_bumper_srv_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dock_state_sub_;
    std::string dock_state_;
    rclcpp::Time dock_state_t_{0, 0, RCL_ROS_TIME};
    double dock_state_timeout_s_ = 1.0;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    int heartbeat_tick_ = 0;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr routing_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::optional<mower_interfaces::msg::MowerBaseDevStatus> latest_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BumperControllerNode>());
    rclcpp::shutdown();
    return 0;
}
