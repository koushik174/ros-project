#include "cuda_slam_system/core/orb_detector.hpp"

namespace cuda_slam {

extern "C" {
    void launchFastDetector(const unsigned char* d_image, int width, int height, int stride,
                           int threshold, int* d_corners_x, int* d_corners_y,
                           int* d_corner_count, int max_corners);
    void launchOrbDescriptor(const unsigned char* d_image, int width, int height, int stride,
                            const int* d_keypoints_x, const int* d_keypoints_y,
                            const float* d_angles, int num_keypoints,
                            unsigned char* d_descriptors, int descriptor_size);
}

OrbDetector::OrbDetector(int num_features, float scale_factor, int num_levels)
    : num_features_(num_features),
      scale_factor_(scale_factor),
      num_levels_(num_levels),
      fast_threshold_(20),
      min_fast_threshold_(7),
      d_image_(nullptr),
      d_keypoints_(nullptr),
      d_descriptors_(nullptr) {

    // Compute scale pyramid
    scale_factors_.resize(num_levels_);
    inv_scale_factors_.resize(num_levels_);
    level_sigma2_.resize(num_levels_);
    inv_level_sigma2_.resize(num_levels_);

    scale_factors_[0] = 1.0f;
    level_sigma2_[0] = 1.0f;

    for (int i = 1; i < num_levels_; i++) {
        scale_factors_[i] = scale_factors_[i-1] * scale_factor_;
        level_sigma2_[i] = scale_factors_[i] * scale_factors_[i];
    }

    for (int i = 0; i < num_levels_; i++) {
        inv_scale_factors_[i] = 1.0f / scale_factors_[i];
        inv_level_sigma2_[i] = 1.0f / level_sigma2_[i];
    }

    initializePattern();
}

OrbDetector::~OrbDetector() {
    // Free CUDA memory if allocated
}

void OrbDetector::detectAndCompute(const cv::Mat& image,
                                   std::vector<cv::KeyPoint>& keypoints,
                                   cv::Mat& descriptors) {
    // Use OpenCV ORB as fallback
    cv::Ptr<cv::ORB> orb = cv::ORB::create(num_features_, scale_factor_, num_levels_);
    orb->detectAndCompute(image, cv::noArray(), keypoints, descriptors);
}

void OrbDetector::detectAndComputeCuda(const cv::Mat& image,
                                      std::vector<cv::KeyPoint>& keypoints,
                                      cv::Mat& descriptors) {
    // CUDA implementation placeholder
    // In production, this would use CUDA kernels
    detectAndCompute(image, keypoints, descriptors);
}

void OrbDetector::initializePattern() {
    // Initialize ORB sampling pattern
    pattern_.resize(512);
    // Simplified pattern initialization
}

} // namespace cuda_slam
