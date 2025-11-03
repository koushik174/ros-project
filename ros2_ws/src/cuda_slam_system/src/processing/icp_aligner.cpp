#include "cuda_slam_system/processing/icp_aligner.hpp"
#include <pcl/registration/icp.h>

namespace cuda_slam {

ICPAligner::ICPAligner()
    : max_iterations_(30),
      max_correspondence_distance_(0.1),
      transformation_epsilon_(1e-6),
      euclidean_fitness_epsilon_(1e-6),
      fitness_score_(0.0),
      final_iterations_(0),
      has_converged_(false) {
}

ICPAligner::~ICPAligner() {
    freeDeviceMemory();
}

bool ICPAligner::align(const PointCloudPtr& source,
                      const PointCloudPtr& target,
                      SE3& transformation,
                      PointCloudPtr& aligned) {
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(source);
    icp.setInputTarget(target);
    icp.setMaximumIterations(max_iterations_);
    icp.setMaxCorrespondenceDistance(max_correspondence_distance_);
    icp.setTransformationEpsilon(transformation_epsilon_);

    icp.align(*aligned);

    has_converged_ = icp.hasConverged();
    fitness_score_ = icp.getFitnessScore();
    final_iterations_ = icp.nr_iterations_;

    if (has_converged_) {
        Eigen::Matrix4f T = icp.getFinalTransformation();
        transformation.matrix() = T.cast<double>();
    }

    return has_converged_;
}

bool ICPAligner::alignCuda(const PointCloudPtr& source,
                          const PointCloudPtr& target,
                          SE3& transformation,
                          PointCloudPtr& aligned) {
    return align(source, target, transformation, aligned);
}

void ICPAligner::allocateDeviceMemory(size_t source_size, size_t target_size) {
}

void ICPAligner::freeDeviceMemory() {
}

} // namespace cuda_slam
