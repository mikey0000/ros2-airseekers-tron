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
#include "std_msgs/msg/u_int8.hpp"
#include "std_srvs/srv/empty.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "mower_interfaces/msg/mower_base_dev_status.hpp"

#include "bumper_controller/bumper_controller.h"

namespace {

sensor_msgs::msg::PointCloud2 makeBumperCloud(bool left, bool right, const rclcpp::Time& stamp) {
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
    if (left)  add(0.4f,  0.2f, 0.0f);
    if (right) add(0.4f, -0.2f, 0.0f);
    if (!left && !right) add(0.4f, 0.0f, 0.0f);
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
        ctrl_.update(latest_->bumper_triggered,
                     latest_->left_bumper_triggered,
                     latest_->right_bumper_triggered,
                     latest_->bumper_routing_enabled, 0.05);
        ctrl_.spinOnce();

        // Publish the routing state + bumper cloud.
        std_msgs::msg::UInt8 routing;
        routing.data = (ctrl_.state() == mower_controller::BumperController::State::BACKING_UP) ? 1 : 0;
        routing_pub_->publish(routing);

        if (ctrl_.bumperTriggered()) {
            cloud_pub_->publish(makeBumperCloud(ctrl_.bumperLeft(), ctrl_.bumperRight(),
                                                get_clock()->now()));
        }
    }

    mower_controller::BumperController ctrl_;
    rclcpp::Subscription<mower_interfaces::msg::MowerBaseDevStatus>::SharedPtr status_sub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr test_bumper_srv_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
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
