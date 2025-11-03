#include <gtest/gtest.h>
#include "cuda_slam_system/fusion/ekf.hpp"

namespace cuda_slam {
namespace test {

class EKFTest : public ::testing::Test {
protected:
    void SetUp() override {
        ekf_ = std::make_unique<ExtendedKalmanFilter>();

        Vector3d initial_position(0, 0, 0);
        Quaterniond initial_orientation = Quaterniond::Identity();
        ekf_->initialize(initial_position, initial_orientation);
    }

    std::unique_ptr<ExtendedKalmanFilter> ekf_;
};

TEST_F(EKFTest, Initialization) {
    EXPECT_TRUE(ekf_->isInitialized());

    Vector3d pos = ekf_->getPosition();
    EXPECT_DOUBLE_EQ(pos.x(), 0.0);
    EXPECT_DOUBLE_EQ(pos.y(), 0.0);
    EXPECT_DOUBLE_EQ(pos.z(), 0.0);
}

TEST_F(EKFTest, PredictionWithIMU) {
    ImuMeasurement imu;
    imu.timestamp = 0.01;
    imu.acceleration << 0, 0, -9.81;
    imu.angular_velocity << 0, 0, 0;

    ekf_->predict(imu, 0.01);

    Vector3d pos = ekf_->getPosition();
    // Position should change slightly due to gravity compensation
    EXPECT_TRUE(ekf_->isInitialized());
}

TEST_F(EKFTest, UpdateWithPose) {
    SE3 measured_pose = SE3::Identity();
    measured_pose.translation() << 1.0, 0.5, 0.0;

    Matrix4d covariance = Matrix4d::Identity() * 0.01;

    ekf_->updateWithPose(measured_pose, covariance);

    Vector3d pos = ekf_->getPosition();
    // Position should be influenced by measurement
    EXPECT_GT(pos.norm(), 0.0);
}

TEST_F(EKFTest, QuaternionNormalization) {
    for (int i = 0; i < 100; i++) {
        ImuMeasurement imu;
        imu.timestamp = i * 0.01;
        imu.acceleration << 0, 0, -9.81;
        imu.angular_velocity << 0.1, 0.05, 0.02;

        ekf_->predict(imu, 0.01);

        Quaterniond q = ekf_->getOrientation();
        EXPECT_NEAR(q.norm(), 1.0, 1e-6);
    }
}

} // namespace test
} // namespace cuda_slam

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
