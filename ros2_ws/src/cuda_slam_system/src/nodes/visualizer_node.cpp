#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace cuda_slam {

class VisualizerNode : public rclcpp::Node {
public:
    VisualizerNode() : Node("visualizer_node") {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "slam_markers", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&VisualizerNode::publishMarkers, this));

        RCLCPP_INFO(this->get_logger(), "Visualizer Node initialized");
    }

private:
    void publishMarkers() {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        marker_pub_->publish(marker_array);
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace cuda_slam

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cuda_slam::VisualizerNode>());
    rclcpp::shutdown();
    return 0;
}
