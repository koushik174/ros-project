#pragma once

#include "cuda_slam_system/common.hpp"
#include <opencv2/opencv.hpp>

namespace cuda_slam {

class Frame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Frame(uint64_t id, double timestamp, const cv::Mat& image,
          const CameraIntrinsics& camera);

    ~Frame() = default;

    // Getters
    uint64_t getId() const { return id_; }
    double getTimestamp() const { return timestamp_; }
    const cv::Mat& getImage() const { return image_; }
    const std::vector<cv::Mat>& getImagePyramid() const { return image_pyramid_; }
    const std::vector<Feature>& getFeatures() const { return features_; }
    const SE3& getPose() const { return pose_; }
    const CameraIntrinsics& getCamera() const { return camera_; }

    // Setters
    void setPose(const SE3& pose) { pose_ = pose; }
    void setFeatures(const std::vector<Feature>& features) { features_ = features; }

    // Feature operations
    void extractFeatures();
    void computeBearingVectors();
    Vector3d pixel2Camera(const cv::Point2f& pixel) const;
    cv::Point2f camera2Pixel(const Vector3d& point) const;

    // Utility
    void buildImagePyramid();
    int getTrackedPoints() const;
    void undistortImage();

private:
    uint64_t id_;
    double timestamp_;
    cv::Mat image_;
    std::vector<cv::Mat> image_pyramid_;
    std::vector<Feature> features_;
    SE3 pose_;
    CameraIntrinsics camera_;

    bool features_extracted_;
    std::mutex mutex_;
};

} // namespace cuda_slam
