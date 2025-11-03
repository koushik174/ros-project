#pragma once

#include "cuda_slam_system/common.hpp"
#include "cuda_slam_system/core/keyframe.hpp"
#include "cuda_slam_system/core/map_point.hpp"
#include <vector>
#include <set>

namespace cuda_slam {

class Map {
public:
    Map();
    ~Map() = default;

    // KeyFrame operations
    void addKeyFrame(KeyFramePtr keyframe);
    void eraseKeyFrame(KeyFramePtr keyframe);
    std::vector<KeyFramePtr> getAllKeyFrames() const;
    size_t numKeyFrames() const;

    // MapPoint operations
    void addMapPoint(MapPointPtr map_point);
    void eraseMapPoint(MapPointPtr map_point);
    std::vector<MapPointPtr> getAllMapPoints() const;
    size_t numMapPoints() const;

    // Reference KeyFrame
    void setReferenceKeyFrame(KeyFramePtr keyframe);
    KeyFramePtr getReferenceKeyFrame() const;

    // Local map
    std::vector<KeyFramePtr> getLocalKeyFrames() const;
    std::vector<MapPointPtr> getLocalMapPoints() const;

    // Statistics
    long unsigned int getMaxKeyFrameId() const;
    void clear();
    void reset();

    // Optimization
    void informNewBigChange();
    int getLastBigChangeIdx() const;

    // Save/Load
    void save(const std::string& filename) const;
    void load(const std::string& filename);

private:
    std::set<KeyFramePtr> keyframes_;
    std::set<MapPointPtr> map_points_;
    KeyFramePtr reference_keyframe_;

    std::vector<KeyFramePtr> keyframe_origins_;

    long unsigned int max_keyframe_id_;
    long unsigned int big_change_idx_;

    mutable std::mutex mutex_map_;
    mutable std::mutex mutex_point_creation_;
};

} // namespace cuda_slam
