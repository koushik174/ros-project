#pragma once

#include "cuda_slam_system/common.hpp"
#include <opencv2/opencv.hpp>

namespace cuda_slam {

class OrbDetector {
public:
    OrbDetector(int num_features = MAX_FEATURES_PER_FRAME,
                float scale_factor = PYRAMID_SCALE,
                int num_levels = PYRAMID_LEVELS);

    ~OrbDetector();

    // Detect and compute features
    void detectAndCompute(const cv::Mat& image,
                         std::vector<cv::KeyPoint>& keypoints,
                         cv::Mat& descriptors);

    // CUDA-accelerated detection
    void detectAndComputeCuda(const cv::Mat& image,
                             std::vector<cv::KeyPoint>& keypoints,
                             cv::Mat& descriptors);

    // Multi-scale detection
    void detectMultiScale(const std::vector<cv::Mat>& image_pyramid,
                         std::vector<cv::KeyPoint>& keypoints,
                         cv::Mat& descriptors);

    // Getters
    int getNumLevels() const { return num_levels_; }
    float getScaleFactor() const { return scale_factor_; }
    std::vector<float> getScaleFactors() const { return scale_factors_; }
    std::vector<float> getInvScaleFactors() const { return inv_scale_factors_; }

private:
    void computePyramid(const cv::Mat& image);
    void computeOrientation(const cv::Mat& image,
                           std::vector<cv::KeyPoint>& keypoints);
    void computeDescriptors(const cv::Mat& image,
                           std::vector<cv::KeyPoint>& keypoints,
                           cv::Mat& descriptors);

    // CUDA device pointers
    void* d_image_;
    void* d_keypoints_;
    void* d_descriptors_;

    // Parameters
    int num_features_;
    float scale_factor_;
    int num_levels_;

    // Precomputed scale values
    std::vector<float> scale_factors_;
    std::vector<float> inv_scale_factors_;
    std::vector<float> level_sigma2_;
    std::vector<float> inv_level_sigma2_;

    // Image pyramid
    std::vector<cv::Mat> image_pyramid_;
    std::vector<int> features_per_level_;

    // FAST detector
    int fast_threshold_;
    int min_fast_threshold_;

    // Pattern for ORB descriptor
    static const int PATCH_SIZE = 31;
    static const int HALF_PATCH_SIZE = 15;
    std::vector<cv::Point> pattern_;

    void initializePattern();
};

} // namespace cuda_slam
