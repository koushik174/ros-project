#pragma once

#include "cuda_slam_system/common.hpp"
#include "cuda_slam_system/fusion/ekf.hpp"
#include <deque>
#include <memory>

namespace cuda_slam {

class SensorFusion {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SensorFusion();
    ~SensorFusion() = default;

    // IMU processing
    void addImuMeasurement(const ImuMeasurement& imu);
    void processImuQueue();

    // Visual odometry update
    void updateWithVisualOdometry(const SE3& pose, double timestamp,
                                 const Matrix4d& covariance);

    // LiDAR update
    void updateWithLidarOdometry(const SE3& pose, double timestamp,
                                const Matrix4d& covariance);

    // Map point update
    void updateWithMapPoints(const std::vector<Vector3d>& measured_points,
                            const std::vector<Vector3d>& predicted_points);

    // Get fused pose
    SE3 getFusedPose() const;
    Vector3d getVelocity() const;
    Vector3d getAccelBias() const;
    Vector3d getGyroBias() const;

    // Calibration
    void setImuToCamera(const SE3& T_cam_imu);
    void setImuToLidar(const SE3& T_lidar_imu);

    // Initialization
    void initialize(const Vector3d& position, const Quaterniond& orientation);
    bool isInitialized() const;

    // Configuration
    void setImuRate(double rate) { imu_rate_ = rate; }
    void setMaxImuQueueSize(size_t size) { max_imu_queue_size_ = size; }

    // Statistics
    double getLastUpdateTime() const { return last_update_time_; }
    int getImuQueueSize() const { return imu_queue_.size(); }

private:
    // EKF instance
    std::unique_ptr<ExtendedKalmanFilter> ekf_;

    // IMU buffer
    std::deque<ImuMeasurement> imu_queue_;
    double last_imu_time_;

    // Sensor calibration
    SE3 T_cam_imu_;     // Transform from IMU to camera
    SE3 T_lidar_imu_;   // Transform from IMU to LiDAR

    // Timing
    double last_update_time_;
    double imu_rate_;

    // Configuration
    size_t max_imu_queue_size_;
    bool use_visual_;
    bool use_lidar_;

    // Bias estimation
    bool estimate_bias_;
    Vector3d gravity_vector_;

    mutable std::mutex mutex_;
};

} // namespace cuda_slam
