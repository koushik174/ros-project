#include "cuda_slam_system/fusion/ekf.hpp"
#include <cmath>

namespace cuda_slam {

ExtendedKalmanFilter::ExtendedKalmanFilter() : initialized_(false) {
    // Initialize state vector (16D)
    x_ = Eigen::VectorXd::Zero(STATE_SIZE);

    // Initialize covariance matrix
    P_ = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) * 0.1;

    // Process noise
    Q_ = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
    Q_.block<3, 3>(0, 0) *= 0.01;  // Position noise
    Q_.block<3, 3>(3, 3) *= 0.01;  // Velocity noise
    Q_.block<4, 4>(6, 6) *= 0.001; // Orientation noise
    Q_.block<3, 3>(10, 10) *= 0.0001; // Accel bias noise
    Q_.block<3, 3>(13, 13) *= 0.0001; // Gyro bias noise

    // Measurement noise
    R_ = Eigen::MatrixXd::Identity(7, 7) * 0.01; // Position + quaternion

    // Gravity vector
    gravity_ << 0, 0, -GRAVITY;
}

void ExtendedKalmanFilter::initialize(const Vector3d& initial_position,
                                      const Quaterniond& initial_orientation) {
    // Set position
    x_.segment<3>(0) = initial_position;

    // Set velocity to zero
    x_.segment<3>(3).setZero();

    // Set orientation (quaternion: w, x, y, z)
    x_(6) = initial_orientation.w();
    x_(7) = initial_orientation.x();
    x_(8) = initial_orientation.y();
    x_(9) = initial_orientation.z();

    // Set biases to zero
    x_.segment<3>(10).setZero(); // Accelerometer bias
    x_.segment<3>(13).setZero(); // Gyroscope bias

    initialized_ = true;
}

void ExtendedKalmanFilter::reset() {
    x_.setZero();
    P_ = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) * 0.1;
    initialized_ = false;
}

void ExtendedKalmanFilter::predict(const ImuMeasurement& imu, double dt) {
    if (!initialized_) return;

    // Get current state
    Vector3d position = x_.segment<3>(0);
    Vector3d velocity = x_.segment<3>(3);
    Quaterniond q(x_(6), x_(7), x_(8), x_(9));
    Vector3d accel_bias = x_.segment<3>(10);
    Vector3d gyro_bias = x_.segment<3>(13);

    // Compensate for biases
    Vector3d accel = imu.acceleration - accel_bias;
    Vector3d gyro = imu.angular_velocity - gyro_bias;

    // Predict orientation
    Quaterniond dq;
    Vector3d omega = gyro * dt;
    double theta = omega.norm();
    if (theta > 1e-6) {
        dq.w() = std::cos(theta / 2.0);
        dq.vec() = std::sin(theta / 2.0) * omega / theta;
    } else {
        dq.w() = 1.0;
        dq.vec() = omega / 2.0;
    }
    q = q * dq;
    q.normalize();

    // Rotate acceleration to world frame
    Vector3d accel_world = q * accel + gravity_;

    // Predict position and velocity
    position = position + velocity * dt + 0.5 * accel_world * dt * dt;
    velocity = velocity + accel_world * dt;

    // Update state
    x_.segment<3>(0) = position;
    x_.segment<3>(3) = velocity;
    x_(6) = q.w();
    x_(7) = q.x();
    x_(8) = q.y();
    x_(9) = q.z();

    // Compute state transition Jacobian
    Eigen::MatrixXd F = computeStateTransitionMatrix(dt);

    // Predict covariance
    Eigen::MatrixXd Q = computeProcessNoise(dt);
    P_ = F * P_ * F.transpose() + Q;

    normalizeQuaternion();
}

void ExtendedKalmanFilter::updateWithPose(const SE3& measured_pose,
                                         const Matrix4d& covariance) {
    if (!initialized_) return;

    // Measurement: [position, orientation (quaternion)]
    Eigen::VectorXd z(7);
    z.segment<3>(0) = measured_pose.translation();

    Quaterniond q_measured(measured_pose.rotation());
    z(3) = q_measured.w();
    z(4) = q_measured.x();
    z(5) = q_measured.y();
    z(6) = q_measured.z();

    // Predicted measurement
    Eigen::VectorXd z_pred(7);
    z_pred.segment<3>(0) = x_.segment<3>(0);
    z_pred.segment<4>(3) = x_.segment<4>(6);

    // Innovation
    Eigen::VectorXd y = z - z_pred;

    // Quaternion difference (handle ambiguity)
    if (y.segment<4>(3).dot(Eigen::Vector4d(1, 0, 0, 0)) < 0) {
        y.segment<4>(3) = -y.segment<4>(3);
    }

    // Measurement Jacobian (identity for direct state observation)
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(7, STATE_SIZE);
    H.block<7, 7>(0, 0) = Eigen::MatrixXd::Identity(7, 7);

    // Measurement noise from covariance
    Eigen::MatrixXd R = R_;
    if (covariance.norm() > 0) {
        R.block<3, 3>(0, 0) = covariance.block<3, 3>(0, 0);
    }

    // Kalman gain
    Eigen::MatrixXd S = H * P_ * H.transpose() + R;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    // Update state
    x_ = x_ + K * y;

    // Update covariance
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
    P_ = (I - K * H) * P_;

    normalizeQuaternion();
}

SE3 ExtendedKalmanFilter::getPose() const {
    SE3 pose = SE3::Identity();
    pose.translation() = getPosition();
    pose.linear() = getOrientation().toRotationMatrix();
    return pose;
}

Vector3d ExtendedKalmanFilter::getPosition() const {
    return x_.segment<3>(0);
}

Vector3d ExtendedKalmanFilter::getVelocity() const {
    return x_.segment<3>(3);
}

Quaterniond ExtendedKalmanFilter::getOrientation() const {
    return Quaterniond(x_(6), x_(7), x_(8), x_(9));
}

Vector3d ExtendedKalmanFilter::getAccelBias() const {
    return x_.segment<3>(10);
}

Vector3d ExtendedKalmanFilter::getGyroBias() const {
    return x_.segment<3>(13);
}

double ExtendedKalmanFilter::getPositionUncertainty() const {
    return P_.block<3, 3>(0, 0).trace();
}

double ExtendedKalmanFilter::getOrientationUncertainty() const {
    return P_.block<4, 4>(6, 6).trace();
}

Eigen::MatrixXd ExtendedKalmanFilter::computeStateTransitionMatrix(double dt) const {
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);

    // Position depends on velocity
    F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;

    // Velocity depends on orientation (through gravity rotation)
    // Simplified linear approximation

    return F;
}

Eigen::MatrixXd ExtendedKalmanFilter::computeProcessNoise(double dt) const {
    return Q_ * dt;
}

void ExtendedKalmanFilter::normalizeQuaternion() {
    Vector4d q = x_.segment<4>(6);
    double norm = q.norm();
    if (norm > 1e-6) {
        x_.segment<4>(6) = q / norm;
    } else {
        x_.segment<4>(6) << 1, 0, 0, 0;
    }
}

} // namespace cuda_slam
