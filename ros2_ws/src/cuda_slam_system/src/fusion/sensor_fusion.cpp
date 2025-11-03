#include "cuda_slam_system/fusion/sensor_fusion.hpp"

namespace cuda_slam {

SensorFusion::SensorFusion()
    : last_imu_time_(0.0),
      last_update_time_(0.0),
      imu_rate_(200.0),
      max_imu_queue_size_(1000),
      use_visual_(true),
      use_lidar_(true),
      estimate_bias_(true) {

    ekf_ = std::make_unique<ExtendedKalmanFilter>();

    // Default calibration (identity)
    T_cam_imu_ = SE3::Identity();
    T_lidar_imu_ = SE3::Identity();

    gravity_vector_ << 0, 0, -9.81;
}

void SensorFusion::initialize(const Vector3d& position,
                              const Quaterniond& orientation) {
    std::lock_guard<std::mutex> lock(mutex_);
    ekf_->initialize(position, orientation);
}

bool SensorFusion::isInitialized() const {
    return ekf_->isInitialized();
}

void SensorFusion::addImuMeasurement(const ImuMeasurement& imu) {
    std::lock_guard<std::mutex> lock(mutex_);

    imu_queue_.push_back(imu);

    // Limit queue size
    while (imu_queue_.size() > max_imu_queue_size_) {
        imu_queue_.pop_front();
    }

    last_imu_time_ = imu.timestamp;
}

void SensorFusion::processImuQueue() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!ekf_->isInitialized()) return;

    double current_time = last_imu_time_;

    for (const auto& imu : imu_queue_) {
        if (imu.timestamp <= last_update_time_) continue;

        double dt = (last_update_time_ > 0) ?
                    (imu.timestamp - last_update_time_) : (1.0 / imu_rate_);

        if (dt > 0 && dt < 1.0) {  // Sanity check
            ekf_->predict(imu, dt);
            last_update_time_ = imu.timestamp;
        }
    }

    // Clear old IMU measurements
    while (!imu_queue_.empty() &&
           imu_queue_.front().timestamp < last_update_time_ - 1.0) {
        imu_queue_.pop_front();
    }
}

void SensorFusion::updateWithVisualOdometry(const SE3& pose,
                                            double timestamp,
                                            const Matrix4d& covariance) {
    if (!use_visual_) return;

    std::lock_guard<std::mutex> lock(mutex_);

    // Transform pose from camera frame to IMU frame
    SE3 pose_imu = T_cam_imu_ * pose;

    // Update EKF
    ekf_->updateWithPose(pose_imu, covariance);
    last_update_time_ = timestamp;
}

void SensorFusion::updateWithLidarOdometry(const SE3& pose,
                                          double timestamp,
                                          const Matrix4d& covariance) {
    if (!use_lidar_) return;

    std::lock_guard<std::mutex> lock(mutex_);

    // Transform pose from LiDAR frame to IMU frame
    SE3 pose_imu = T_lidar_imu_ * pose;

    // Update EKF
    ekf_->updateWithPose(pose_imu, covariance);
    last_update_time_ = timestamp;
}

void SensorFusion::updateWithMapPoints(const std::vector<Vector3d>& measured_points,
                                      const std::vector<Vector3d>& predicted_points) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!ekf_->isInitialized()) return;

    ekf_->updateWithMapPoints(measured_points, predicted_points);
}

SE3 SensorFusion::getFusedPose() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return ekf_->getPose();
}

Vector3d SensorFusion::getVelocity() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return ekf_->getVelocity();
}

Vector3d SensorFusion::getAccelBias() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return ekf_->getAccelBias();
}

Vector3d SensorFusion::getGyroBias() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return ekf_->getGyroBias();
}

void SensorFusion::setImuToCamera(const SE3& T_cam_imu) {
    std::lock_guard<std::mutex> lock(mutex_);
    T_cam_imu_ = T_cam_imu;
}

void SensorFusion::setImuToLidar(const SE3& T_lidar_imu) {
    std::lock_guard<std::mutex> lock(mutex_);
    T_lidar_imu_ = T_lidar_imu;
}

} // namespace cuda_slam
