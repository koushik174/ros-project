#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include "cuda_slam_system/common.hpp"
#include "cuda_slam_system/core/frame.hpp"
#include "cuda_slam_system/core/orb_detector.hpp"
#include "cuda_slam_system/core/orb_matcher.hpp"

namespace cuda_slam {

class VisualOdometryNode : public rclcpp::Node {
public:
    VisualOdometryNode() : Node("visual_odometry_node") {
        // Declare parameters
        this->declare_parameter("use_cuda", true);
        this->declare_parameter("num_features", 2000);
        this->declare_parameter("publish_rate", 30.0);

        use_cuda_ = this->get_parameter("use_cuda").as_bool();
        int num_features = this->get_parameter("num_features").as_int();

        // Initialize ORB detector
        orb_detector_ = std::make_shared<OrbDetector>(num_features);
        orb_matcher_ = std::make_shared<OrbMatcher>();

        // Subscribers
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image_raw", 10,
            std::bind(&VisualOdometryNode::imageCallback, this, std::placeholders::_1));

        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera/camera_info", 10,
            std::bind(&VisualOdometryNode::cameraInfoCallback, this, std::placeholders::_1));

        // Publishers
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "visual_odometry", 10);

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "camera_pose", 10);

        // TF broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Initialize current pose
        current_pose_ = SE3::Identity();
        frame_id_ = 0;

        RCLCPP_INFO(this->get_logger(), "Visual Odometry Node initialized");
        RCLCPP_INFO(this->get_logger(), "CUDA acceleration: %s",
                    use_cuda_ ? "enabled" : "disabled");
    }

private:
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (camera_initialized_) return;

        camera_.fx = msg->k[0];
        camera_.fy = msg->k[4];
        camera_.cx = msg->k[2];
        camera_.cy = msg->k[5];
        camera_.width = msg->width;
        camera_.height = msg->height;

        if (msg->d.size() >= 5) {
            camera_.k1 = msg->d[0];
            camera_.k2 = msg->d[1];
            camera_.p1 = msg->d[2];
            camera_.p2 = msg->d[3];
            camera_.k3 = msg->d[4];
        }

        camera_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "Camera parameters received");
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!camera_initialized_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Waiting for camera info...");
            return;
        }

        // Convert ROS image to OpenCV
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Create frame
        double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        auto frame = std::make_shared<Frame>(frame_id_++, timestamp,
                                             cv_ptr->image, camera_);

        // Extract features
        frame->extractFeatures();

        // Track frame
        if (last_frame_) {
            trackFrame(frame);
        }

        // Publish odometry
        publishOdometry(frame, msg->header);

        // Update last frame
        last_frame_ = frame;
    }

    void trackFrame(FramePtr current_frame) {
        if (!last_frame_) return;

        // Match features
        std::vector<Match> matches;
        // Simplified matching (in production, use proper tracking)

        // Estimate motion (simplified)
        // In production: use PnP, essential matrix, or optical flow

        // For demonstration, we'll use identity motion
        // current_pose_ = current_pose_ * SE3::Identity();

        RCLCPP_DEBUG(this->get_logger(), "Tracked %zu features",
                    current_frame->getFeatures().size());
    }

    void publishOdometry(FramePtr frame, const std_msgs::msg::Header& header) {
        // Publish Odometry
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = header.stamp;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "camera";

        // Set pose
        Eigen::Vector3d position = current_pose_.translation();
        Eigen::Quaterniond orientation(current_pose_.rotation());

        odom_msg.pose.pose.position.x = position.x();
        odom_msg.pose.pose.position.y = position.y();
        odom_msg.pose.pose.position.z = position.z();

        odom_msg.pose.pose.orientation.w = orientation.w();
        odom_msg.pose.pose.orientation.x = orientation.x();
        odom_msg.pose.pose.orientation.y = orientation.y();
        odom_msg.pose.pose.orientation.z = orientation.z();

        odom_pub_->publish(odom_msg);

        // Publish PoseStamped
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header = odom_msg.header;
        pose_msg.pose = odom_msg.pose.pose;
        pose_pub_->publish(pose_msg);

        // Broadcast TF
        geometry_msgs::msg::TransformStamped transform;
        transform.header = header;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "camera";

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
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // SLAM components
    std::shared_ptr<OrbDetector> orb_detector_;
    std::shared_ptr<OrbMatcher> orb_matcher_;

    // State
    FramePtr last_frame_;
    SE3 current_pose_;
    CameraIntrinsics camera_;
    bool camera_initialized_ = false;
    uint64_t frame_id_;

    // Parameters
    bool use_cuda_;
};

} // namespace cuda_slam

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cuda_slam::VisualOdometryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
