#pragma once

#include "cuda_slam_system/common.hpp"
#include <deque>

namespace cuda_slam {

// State vector: [position, velocity, orientation(quaternion), accel_bias, gyro_bias]
// 15-dimensional state: [px, py, pz, vx, vy, vz, qw, qx, qy, qz, bax, bay, baz, bgx, bgy, bgz]
constexpr int STATE_SIZE = 16;

class ExtendedKalmanFilter {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ExtendedKalmanFilter();
    ~ExtendedKalmanFilter() = default;

    // Initialization
    void initialize(const Vector3d& initial_position,
                   const Quaterniond& initial_orientation);

    void reset();

    // Prediction step (using IMU measurements)
    void predict(const ImuMeasurement& imu, double dt);

    // Update step (using visual odometry)
    void updateWithPose(const SE3& measured_pose, const Matrix4d& covariance);

    // Update step (using map points)
    void updateWithMapPoints(const std::vector<Vector3d>& measured_points,
                            const std::vector<Vector3d>& predicted_points);

    // Getters
    SE3 getPose() const;
    Vector3d getPosition() const;
    Vector3d getVelocity() const;
    Quaterniond getOrientation() const;
    Vector3d getAccelBias() const;
    Vector3d getGyroBias() const;
    Eigen::MatrixXd getCovariance() const { return P_; }

    // State vector access
    Eigen::VectorXd getState() const { return x_; }
    void setState(const Eigen::VectorXd& state) { x_ = state; }

    // Uncertainty
    double getPositionUncertainty() const;
    double getOrientationUncertainty() const;

    // Utility
    bool isInitialized() const { return initialized_; }
    void setProcessNoise(const Eigen::MatrixXd& Q) { Q_ = Q; }
    void setMeasurementNoise(const Eigen::MatrixXd& R) { R_ = R; }

private:
    // State and covariance
    Eigen::VectorXd x_;  // State vector (16x1)
    Eigen::MatrixXd P_;  // Covariance matrix (16x16)

    // Noise matrices
    Eigen::MatrixXd Q_;  // Process noise
    Eigen::MatrixXd R_;  // Measurement noise

    // Gravity vector
    Vector3d gravity_;

    // Flags
    bool initialized_;

    // Helper functions
    Eigen::MatrixXd computeStateTransitionMatrix(double dt) const;
    Eigen::MatrixXd computeProcessNoise(double dt) const;
    Eigen::MatrixXd computeMeasurementJacobian() const;

    void normalizeQuaternion();

    // Constants
    static constexpr double GRAVITY = 9.81;
};

} // namespace cuda_slam
