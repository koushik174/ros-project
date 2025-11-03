#pragma once

#include "cuda_slam_system/common.hpp"
#include <map>
#include <mutex>

namespace cuda_slam {

class MapPoint {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MapPoint(uint64_t id, const Vector3d& position);
    ~MapPoint() = default;

    // Getters
    uint64_t getId() const { return id_; }
    Vector3d getPosition() const;
    Vector3d getNormal() const;
    cv::Mat getDescriptor() const;
    int getObservations() const;
    float getFoundRatio() const;

    // Setters
    void setPosition(const Vector3d& position);
    void setDescriptor(const cv::Mat& descriptor);
    void setNormal(const Vector3d& normal);

    // Observations
    void addObservation(KeyFramePtr keyframe, size_t idx);
    void eraseObservation(KeyFramePtr keyframe);
    std::map<KeyFramePtr, size_t> getObservations() const;
    bool isInKeyFrame(KeyFramePtr keyframe) const;

    // Distance and viewing
    void updateNormalAndDepth();
    void computeDistinctiveDescriptor();
    float getMinDistance() const { return min_distance_; }
    float getMaxDistance() const { return max_distance_; }
    int predictScale(float distance, const CameraIntrinsics& camera) const;

    // Quality
    void increaseVisible(int n = 1) { visible_count_ += n; }
    void increaseFound(int n = 1) { found_count_ += n; }
    bool isBad() const { return is_bad_; }
    void setBad();

    // Replace
    void replace(MapPointPtr new_map_point);
    MapPointPtr getReplacement();

private:
    uint64_t id_;
    Vector3d position_;
    Vector3d normal_;
    cv::Mat descriptor_;

    // Observations: keyframe -> feature index
    std::map<KeyFramePtr, size_t> observations_;
    int num_observations_;

    // Tracking counters
    int visible_count_;
    int found_count_;

    // Scale invariance distances
    float min_distance_;
    float max_distance_;

    // Flags
    bool is_bad_;
    MapPointPtr replacement_;

    mutable std::mutex mutex_position_;
    mutable std::mutex mutex_features_;
};

} // namespace cuda_slam
