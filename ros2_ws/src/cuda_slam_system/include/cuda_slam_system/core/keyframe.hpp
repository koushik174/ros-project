#pragma once

#include "cuda_slam_system/common.hpp"
#include "cuda_slam_system/core/frame.hpp"
#include <set>

namespace cuda_slam {

class KeyFrame : public Frame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    KeyFrame(const Frame& frame);
    KeyFrame(uint64_t id, double timestamp, const cv::Mat& image,
             const CameraIntrinsics& camera);

    ~KeyFrame() = default;

    // Graph connections
    void addConnection(KeyFramePtr keyframe, int weight);
    void eraseConnection(KeyFramePtr keyframe);
    void updateConnections();
    std::vector<KeyFramePtr> getConnectedKeyFrames() const;
    std::vector<KeyFramePtr> getBestCovisibleKeyFrames(int n) const;

    // Map points
    void addMapPoint(MapPointPtr map_point, size_t idx);
    void eraseMapPoint(size_t idx);
    void eraseMapPoint(MapPointPtr map_point);
    std::vector<MapPointPtr> getMapPoints() const;
    int trackedMapPoints(int min_obs = 1) const;

    // Loop closure
    void setLoopEdge(KeyFramePtr loop_keyframe);
    KeyFramePtr getLoopEdge() const { return loop_edge_; }
    bool hasLoopEdge() const { return loop_edge_ != nullptr; }

    // Spanning tree
    void setParent(KeyFramePtr parent);
    KeyFramePtr getParent() const { return parent_; }
    void addChild(KeyFramePtr child);
    void eraseChild(KeyFramePtr child);
    std::set<KeyFramePtr> getChildren() const;

    // Flags
    void setNotErase() { not_erase_ = true; }
    void setErase();
    bool isBad() const { return is_bad_; }

    // Pose optimization
    void setOptimizedPose(const SE3& pose);

private:
    // Graph connections (covisibility graph)
    std::map<KeyFramePtr, int> connected_keyframes_;
    std::vector<KeyFramePtr> ordered_connected_keyframes_;
    std::vector<int> ordered_weights_;

    // Map points observed by this keyframe
    std::vector<MapPointPtr> map_points_;

    // Spanning tree and graph
    KeyFramePtr parent_;
    std::set<KeyFramePtr> children_;
    KeyFramePtr loop_edge_;

    // Flags
    bool not_erase_;
    bool is_bad_;

    mutable std::mutex mutex_connections_;
    mutable std::mutex mutex_features_;
};

} // namespace cuda_slam
