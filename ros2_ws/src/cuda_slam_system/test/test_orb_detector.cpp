#include <gtest/gtest.h>
#include "cuda_slam_system/core/orb_detector.hpp"
#include <opencv2/opencv.hpp>

namespace cuda_slam {
namespace test {

class OrbDetectorTest : public ::testing::Test {
protected:
    void SetUp() override {
        detector_ = std::make_unique<OrbDetector>(500, 1.2f, 8);

        // Create test image
        test_image_ = cv::Mat(480, 640, CV_8UC1);
        cv::randu(test_image_, cv::Scalar(0), cv::Scalar(255));

        // Add some corners
        for (int i = 0; i < 10; i++) {
            cv::circle(test_image_, cv::Point(100 + i * 50, 100 + i * 30), 5, cv::Scalar(255), -1);
        }
    }

    std::unique_ptr<OrbDetector> detector_;
    cv::Mat test_image_;
};

TEST_F(OrbDetectorTest, DetectFeatures) {
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    detector_->detectAndCompute(test_image_, keypoints, descriptors);

    EXPECT_GT(keypoints.size(), 0);
    EXPECT_EQ(descriptors.rows, keypoints.size());
    EXPECT_EQ(descriptors.cols, 32);  // ORB descriptor size
}

TEST_F(OrbDetectorTest, PyramidLevels) {
    EXPECT_EQ(detector_->getNumLevels(), 8);
    EXPECT_FLOAT_EQ(detector_->getScaleFactor(), 1.2f);
}

TEST_F(OrbDetectorTest, CudaDetection) {
    std::vector<cv::KeyPoint> keypoints_cpu, keypoints_cuda;
    cv::Mat descriptors_cpu, descriptors_cuda;

    detector_->detectAndCompute(test_image_, keypoints_cpu, descriptors_cpu);
    detector_->detectAndComputeCuda(test_image_, keypoints_cuda, descriptors_cuda);

    // CUDA and CPU should produce similar number of features
    EXPECT_NEAR(keypoints_cpu.size(), keypoints_cuda.size(), 100);
}

} // namespace test
} // namespace cuda_slam

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
