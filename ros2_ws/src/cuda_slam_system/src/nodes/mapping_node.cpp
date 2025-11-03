#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include "cuda_slam_system/processing/point_cloud_processor.hpp"
#include "cuda_slam_system/processing/icp_aligner.hpp"

namespace cuda_slam {

class MappingNode : public rclcpp::Node {
public:
    MappingNode() : Node("mapping_node") {
        this->declare_parameter("voxel_leaf_size", 0.05);
        this->declare_parameter("use_cuda", true);

        voxel_size_ = this->get_parameter("voxel_leaf_size").as_double();

        pc_processor_ = std::make_shared<PointCloudProcessor>();
        icp_aligner_ = std::make_shared<ICPAligner>();

        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "points", 10,
            std::bind(&MappingNode::cloudCallback, this, std::placeholders::_1));

        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map", 10);

        global_map_ = std::make_shared<PointCloudT>();

        RCLCPP_INFO(this->get_logger(), "Mapping Node initialized");
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        PointCloudPtr cloud(new PointCloudT);
        pcl::fromROSMsg(*msg, *cloud);

        PointCloudPtr filtered(new PointCloudT);
        pc_processor_->voxelFilter(cloud, filtered, voxel_size_);

        *global_map_ += *filtered;

        sensor_msgs::msg::PointCloud2 map_msg;
        pcl::toROSMsg(*global_map_, map_msg);
        map_msg.header = msg->header;
        map_pub_->publish(map_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    std::shared_ptr<PointCloudProcessor> pc_processor_;
    std::shared_ptr<ICPAligner> icp_aligner_;
    PointCloudPtr global_map_;
    double voxel_size_;
};

} // namespace cuda_slam

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cuda_slam::MappingNode>());
    rclcpp::shutdown();
    return 0;
}
