#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "cuda_slam_system/core/loop_closer.hpp"

namespace cuda_slam {

class LoopClosureNode : public rclcpp::Node {
public:
    LoopClosureNode() : Node("loop_closure_node") {
        RCLCPP_INFO(this->get_logger(), "Loop Closure Node initialized");
    }
};

} // namespace cuda_slam

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cuda_slam::LoopClosureNode>());
    rclcpp::shutdown();
    return 0;
}
