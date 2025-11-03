#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include "cuda_slam_system/fusion/sensor_fusion.hpp"

namespace cuda_slam {

class FusionNode : public rclcpp::Node {
public:
    FusionNode() : Node("fusion_node") {
        // Declare parameters
        this->declare_parameter("imu_topic", "imu/data");
        this->declare_parameter("visual_odom_topic", "visual_odometry");
        this->declare_parameter("lidar_odom_topic", "lidar_odometry");
        this->declare_parameter("publish_rate", 100.0);
        this->declare_parameter("use_visual", true);
        this->declare_parameter("use_lidar", true);

        std::string imu_topic = this->get_parameter("imu_topic").as_string();
        std::string visual_topic = this->get_parameter("visual_odom_topic").as_string();
        std::string lidar_topic = this->get_parameter("lidar_odom_topic").as_string();
        double publish_rate = this->get_parameter("publish_rate").as_double();

        // Initialize sensor fusion
        sensor_fusion_ = std::make_shared<SensorFusion>();
        sensor_fusion_->setImuRate(100.0);

        // Subscribers
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 100,
            std::bind(&FusionNode::imuCallback, this, std::placeholders::_1));

        visual_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            visual_topic, 10,
            std::bind(&FusionNode::visualOdomCallback, this, std::placeholders::_1));

        lidar_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            lidar_topic, 10,
            std::bind(&FusionNode::lidarOdomCallback, this, std::placeholders::_1));

        // Publishers
        fused_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "fused_odometry", 10);

        fused_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "fused_pose", 10);

        // TF broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Timer for publishing
        auto timer_period = std::chrono::duration<double>(1.0 / publish_rate);
        timer_ = this->create_wall_timer(
            timer_period,
            std::bind(&FusionNode::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Sensor Fusion Node initialized");
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        ImuMeasurement imu;
        imu.timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        imu.acceleration << msg->linear_acceleration.x,
                            msg->linear_acceleration.y,
                            msg->linear_acceleration.z;

        imu.angular_velocity << msg->angular_velocity.x,
                                msg->angular_velocity.y,
                                msg->angular_velocity.z;

        sensor_fusion_->addImuMeasurement(imu);
        sensor_fusion_->processImuQueue();

        if (!initialized_ && imu_count_++ > 100) {
            // Initialize after collecting some IMU data
            Vector3d initial_position(0, 0, 0);
            Quaterniond initial_orientation = Quaterniond::Identity();

            // Estimate initial orientation from gravity
            Vector3d accel_avg = imu.acceleration; // Simplified
            Vector3d gravity_dir = -accel_avg.normalized();
            Vector3d z_axis(0, 0, 1);

            // Rotation from z-axis to gravity direction
            if ((gravity_dir - z_axis).norm() > 0.01) {
                Vector3d axis = z_axis.cross(gravity_dir).normalized();
                double angle = std::acos(z_axis.dot(gravity_dir));
                initial_orientation = Quaterniond(Eigen::AngleAxisd(angle, axis));
            }

            sensor_fusion_->initialize(initial_position, initial_orientation);
            initialized_ = true;

            RCLCPP_INFO(this->get_logger(), "Sensor fusion initialized");
        }
    }

    void visualOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!initialized_) return;

        SE3 pose = SE3::Identity();
        pose.translation() << msg->pose.pose.position.x,
                             msg->pose.pose.position.y,
                             msg->pose.pose.position.z;

        Quaterniond q(msg->pose.pose.orientation.w,
                     msg->pose.pose.orientation.x,
                     msg->pose.pose.orientation.y,
                     msg->pose.pose.orientation.z);

        pose.linear() = q.toRotationMatrix();

        double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        // Extract covariance
        Matrix4d covariance = Matrix4d::Identity() * 0.01;

        sensor_fusion_->updateWithVisualOdometry(pose, timestamp, covariance);

        RCLCPP_DEBUG(this->get_logger(), "Visual odometry update received");
    }

    void lidarOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!initialized_) return;

        SE3 pose = SE3::Identity();
        pose.translation() << msg->pose.pose.position.x,
                             msg->pose.pose.position.y,
                             msg->pose.pose.position.z;

        Quaterniond q(msg->pose.pose.orientation.w,
                     msg->pose.pose.orientation.x,
                     msg->pose.pose.orientation.y,
                     msg->pose.pose.orientation.z);

        pose.linear() = q.toRotationMatrix();

        double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        Matrix4d covariance = Matrix4d::Identity() * 0.01;

        sensor_fusion_->updateWithLidarOdometry(pose, timestamp, covariance);

        RCLCPP_DEBUG(this->get_logger(), "LiDAR odometry update received");
    }

    void timerCallback() {
        if (!initialized_) return;

        // Get fused pose
        SE3 fused_pose = sensor_fusion_->getFusedPose();
        Vector3d velocity = sensor_fusion_->getVelocity();

        // Create timestamp
        auto now = this->now();

        // Publish odometry
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        Vector3d position = fused_pose.translation();
        Quaterniond orientation(fused_pose.rotation());

        odom_msg.pose.pose.position.x = position.x();
        odom_msg.pose.pose.position.y = position.y();
        odom_msg.pose.pose.position.z = position.z();

        odom_msg.pose.pose.orientation.w = orientation.w();
        odom_msg.pose.pose.orientation.x = orientation.x();
        odom_msg.pose.pose.orientation.y = orientation.y();
        odom_msg.pose.pose.orientation.z = orientation.z();

        odom_msg.twist.twist.linear.x = velocity.x();
        odom_msg.twist.twist.linear.y = velocity.y();
        odom_msg.twist.twist.linear.z = velocity.z();

        fused_odom_pub_->publish(odom_msg);

        // Publish pose
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header = odom_msg.header;
        pose_msg.pose = odom_msg.pose.pose;
        fused_pose_pub_->publish(pose_msg);

        // Broadcast TF
        geometry_msgs::msg::TransformStamped transform;
        transform.header = odom_msg.header;
        transform.child_frame_id = "base_link";

        transform.transform.translation.x = position.x();
        transform.transform.translation.y = position.y();
        transform.transform.translation.z = position.z();

        transform.transform.rotation.w = orientation.w();
        transform.transform.rotation.x = orientation.x();
        transform.transform.rotation.y = orientation.y();
        transform.transform.rotation.z = orientation.z();

        tf_broadcaster_->sendTransform(transform);
    }

    // ROS communication
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr visual_odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr fused_pose_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Sensor fusion
    std::shared_ptr<SensorFusion> sensor_fusion_;

    // State
    bool initialized_ = false;
    int imu_count_ = 0;
};

} // namespace cuda_slam

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cuda_slam::FusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
