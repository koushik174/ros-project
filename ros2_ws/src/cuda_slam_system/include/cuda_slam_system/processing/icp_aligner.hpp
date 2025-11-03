#pragma once

#include "cuda_slam_system/common.hpp"
#include "cuda_slam_system/processing/point_cloud_processor.hpp"

namespace cuda_slam {

class ICPAligner {
public:
    ICPAligner();
    ~ICPAligner();

    // Standard ICP
    bool align(const PointCloudPtr& source,
              const PointCloudPtr& target,
              SE3& transformation,
              PointCloudPtr& aligned);

    // CUDA-accelerated ICP
    bool alignCuda(const PointCloudPtr& source,
                  const PointCloudPtr& target,
                  SE3& transformation,
                  PointCloudPtr& aligned);

    // Point-to-point ICP
    bool alignPointToPoint(const PointCloudPtr& source,
                          const PointCloudPtr& target,
                          SE3& transformation);

    // Point-to-plane ICP
    bool alignPointToPlane(const PointCloudPtr& source,
                          const PointCloudPtr& target,
                          const pcl::PointCloud<pcl::Normal>::Ptr& target_normals,
                          SE3& transformation);

    // Generalized ICP
    bool alignGeneralizedICP(const PointCloudPtr& source,
                            const PointCloudPtr& target,
                            SE3& transformation);

    // Configuration
    void setMaxIterations(int max_iter) { max_iterations_ = max_iter; }
    void setMaxCorrespondenceDistance(float dist) { max_correspondence_distance_ = dist; }
    void setTransformationEpsilon(double eps) { transformation_epsilon_ = eps; }
    void setEuclideanFitnessEpsilon(double eps) { euclidean_fitness_epsilon_ = eps; }
    void setRANSACOutlierRejectionThreshold(double threshold) {
        ransac_outlier_threshold_ = threshold;
    }

    // Getters
    double getFitnessScore() const { return fitness_score_; }
    int getFinalIterations() const { return final_iterations_; }
    bool hasConverged() const { return has_converged_; }

private:
    // Find correspondences
    void findCorrespondences(const PointCloudPtr& source,
                            const PointCloudPtr& target,
                            std::vector<int>& correspondences,
                            std::vector<float>& distances);

    void findCorrespondencesCuda(const PointCloudPtr& source,
                                const PointCloudPtr& target,
                                std::vector<int>& correspondences,
                                std::vector<float>& distances);

    // Estimate transformation
    SE3 estimateRigidTransform(const PointCloudPtr& source,
                              const PointCloudPtr& target,
                              const std::vector<int>& correspondences);

    SE3 estimateRigidTransformSVD(const std::vector<Vector3d>& source_points,
                                 const std::vector<Vector3d>& target_points);

    // RANSAC for robust estimation
    SE3 estimateTransformRANSAC(const PointCloudPtr& source,
                               const PointCloudPtr& target,
                               const std::vector<int>& correspondences,
                               std::vector<bool>& inliers);

    // Convergence checking
    bool checkConvergence(const SE3& transform_delta);

    // CUDA device memory
    void* d_source_;
    void* d_target_;
    void* d_correspondences_;
    void* d_distances_;
    void* d_transformation_;

    // Parameters
    int max_iterations_;
    float max_correspondence_distance_;
    double transformation_epsilon_;
    double euclidean_fitness_epsilon_;
    double ransac_outlier_threshold_;
    bool use_reciprocal_correspondences_;

    // Results
    double fitness_score_;
    int final_iterations_;
    bool has_converged_;

    size_t allocated_size_;

    void allocateDeviceMemory(size_t source_size, size_t target_size);
    void freeDeviceMemory();
};

} // namespace cuda_slam
