#include "cuda_slam_system/core/frame.hpp"

namespace cuda_slam {

Frame::Frame(uint64_t id, double timestamp, const cv::Mat& image, const CameraIntrinsics& camera)
    : id_(id), timestamp_(timestamp), image_(image.clone()), camera_(camera),
      features_extracted_(false) {
    pose_ = SE3::Identity();
}

void Frame::extractFeatures() {
    // Feature extraction implementation
    features_extracted_ = true;
}

void Frame::buildImagePyramid() {
    // Build image pyramid
}

Vector3d Frame::pixel2Camera(const cv::Point2f& pixel) const {
    Vector3d point;
    point.x() = (pixel.x - camera_.cx) / camera_.fx;
    point.y() = (pixel.y - camera_.cy) / camera_.fy;
    point.z() = 1.0;
    return point.normalized();
}

cv::Point2f Frame::camera2Pixel(const Vector3d& point) const {
    float x = camera_.fx * point.x() / point.z() + camera_.cx;
    float y = camera_.fy * point.y() / point.z() + camera_.cy;
    return cv::Point2f(x, y);
}

int Frame::getTrackedPoints() const {
    int count = 0;
    for (const auto& feat : features_) {
        if (feat.map_point) count++;
    }
    return count;
}

} // namespace cuda_slam
