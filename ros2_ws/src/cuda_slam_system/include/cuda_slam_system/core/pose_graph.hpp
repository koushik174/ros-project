#pragma once

#include "cuda_slam_system/common.hpp"
#include "cuda_slam_system/core/keyframe.hpp"
#include "cuda_slam_system/core/map_point.hpp"
#include <Eigen/Sparse>

namespace cuda_slam {

// Edge in pose graph
struct PoseGraphEdge {
    KeyFramePtr from;
    KeyFramePtr to;
    SE3 measurement;
    Eigen::Matrix<double, 6, 6> information;
    bool is_loop_closure;

    PoseGraphEdge() : is_loop_closure(false) {
        information.setIdentity();
    }
};

class PoseGraph {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PoseGraph();
    ~PoseGraph() = default;

    // Add nodes and edges
    void addKeyFrame(KeyFramePtr keyframe);
    void addEdge(const PoseGraphEdge& edge);
    void addLoopClosureEdge(KeyFramePtr from, KeyFramePtr to,
                           const SE3& measurement);

    // Optimization
    void optimize(int iterations = 10);
    void optimizeEssentialGraph(const std::vector<KeyFramePtr>& keyframes,
                               const std::vector<PoseGraphEdge>& edges);

    // Local bundle adjustment
    void localBundleAdjustment(KeyFramePtr keyframe,
                              const std::vector<KeyFramePtr>& local_keyframes,
                              const std::vector<MapPointPtr>& local_map_points);

    // Global bundle adjustment
    void globalBundleAdjustment(const std::vector<KeyFramePtr>& keyframes,
                               const std::vector<MapPointPtr>& map_points,
                               int iterations = 20);

    // Pose-only optimization (for tracking)
    static SE3 optimizePose(const Frame& frame,
                           const std::vector<MapPointPtr>& map_points,
                           std::vector<bool>& outliers);

    // Two-view optimization
    static void optimizeTwoViews(KeyFramePtr keyframe1, KeyFramePtr keyframe2,
                                std::vector<Match>& matches,
                                SE3& T12);

    // Sim3 optimization (for loop closure)
    static bool optimizeSim3(KeyFramePtr keyframe1, KeyFramePtr keyframe2,
                            std::vector<Match>& matches,
                            SE3& T12, float& scale);

    // Getters
    std::vector<PoseGraphEdge> getEdges() const { return edges_; }
    size_t numEdges() const { return edges_.size(); }

private:
    // Graph structure
    std::map<uint64_t, KeyFramePtr> keyframes_;
    std::vector<PoseGraphEdge> edges_;

    // Optimization utilities
    void buildOptimizationProblem(const std::vector<KeyFramePtr>& keyframes,
                                 const std::vector<MapPointPtr>& map_points,
                                 Eigen::SparseMatrix<double>& H,
                                 Eigen::VectorXd& b);

    void solveLeastSquares(const Eigen::SparseMatrix<double>& H,
                          const Eigen::VectorXd& b,
                          Eigen::VectorXd& delta);

    void updatePoses(const std::vector<KeyFramePtr>& keyframes,
                    const Eigen::VectorXd& delta);

    void updateMapPoints(const std::vector<MapPointPtr>& map_points,
                        const Eigen::VectorXd& delta,
                        size_t pose_dim);

    // Robust kernel
    double huberKernel(double error, double delta = 1.0) const;
    double tukeyKernel(double error, double c = 4.685) const;

    mutable std::mutex mutex_;
};

} // namespace cuda_slam
