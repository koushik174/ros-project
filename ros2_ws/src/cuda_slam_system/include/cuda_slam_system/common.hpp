#pragma once

#include <memory>
#include <vector>
#include <map>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace cuda_slam {

// Type definitions
using Matrix3d = Eigen::Matrix3d;
using Matrix4d = Eigen::Matrix4d;
using Vector3d = Eigen::Vector3d;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Quaterniond = Eigen::Quaterniond;
using SE3 = Eigen::Isometry3d;

// Forward declarations
class Frame;
class KeyFrame;
class MapPoint;
class Map;

// Shared pointers
using FramePtr = std::shared_ptr<Frame>;
using KeyFramePtr = std::shared_ptr<KeyFrame>;
using MapPointPtr = std::shared_ptr<MapPoint>;
using MapPtr = std::shared_ptr<Map>;

// Constants
constexpr int ORB_FEATURE_SIZE = 256;
constexpr int ORB_DESCRIPTOR_SIZE = 32; // 32 bytes = 256 bits
constexpr float ORB_MATCH_THRESHOLD = 50.0f;
constexpr int MAX_FEATURES_PER_FRAME = 2000;
constexpr int PYRAMID_LEVELS = 8;
constexpr float PYRAMID_SCALE = 1.2f;

// Camera intrinsics
struct CameraIntrinsics {
    float fx, fy, cx, cy;
    float k1, k2, p1, p2, k3;
    int width, height;

    CameraIntrinsics() : fx(0), fy(0), cx(0), cy(0),
                         k1(0), k2(0), p1(0), p2(0), k3(0),
                         width(0), height(0) {}
};

// IMU measurement
struct ImuMeasurement {
    double timestamp;
    Vector3d acceleration;
    Vector3d angular_velocity;

    ImuMeasurement() : timestamp(0.0) {
        acceleration.setZero();
        angular_velocity.setZero();
    }
};

// Feature point
struct Feature {
    cv::KeyPoint keypoint;
    cv::Mat descriptor;
    Vector3d bearing;
    MapPointPtr map_point;
    int pyramid_level;

    Feature() : pyramid_level(0), map_point(nullptr) {}
};

// Match result
struct Match {
    int query_idx;
    int train_idx;
    float distance;

    Match(int q, int t, float d) : query_idx(q), train_idx(t), distance(d) {}
};

// Loop closure detection result
struct LoopClosure {
    KeyFramePtr query_keyframe;
    KeyFramePtr matched_keyframe;
    SE3 relative_pose;
    float score;
    std::vector<Match> matches;

    LoopClosure() : score(0.0f) {}
};

} // namespace cuda_slam
