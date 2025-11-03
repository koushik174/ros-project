#include "cuda_slam_system/processing/point_cloud_processor.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace cuda_slam {

PointCloudProcessor::PointCloudProcessor()
    : d_points_in_(nullptr), d_points_out_(nullptr),
      d_normals_(nullptr), d_voxel_grid_(nullptr),
      allocated_size_(0) {
}

PointCloudProcessor::~PointCloudProcessor() {
    freeDeviceMemory();
}

void PointCloudProcessor::voxelFilter(const PointCloudPtr& input,
                                     PointCloudPtr& output,
                                     float leaf_size) {
    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setInputCloud(input);
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_filter.filter(*output);
}

void PointCloudProcessor::voxelFilterCuda(const PointCloudPtr& input,
                                         PointCloudPtr& output,
                                         float leaf_size) {
    // CUDA implementation - use CPU fallback for now
    voxelFilter(input, output, leaf_size);
}

void PointCloudProcessor::transformPointCloud(const PointCloudPtr& input,
                                              PointCloudPtr& output,
                                              const SE3& transform) {
    pcl::transformPointCloud(*input, *output, transform.matrix().cast<float>());
}

void PointCloudProcessor::transformPointCloudCuda(const PointCloudPtr& input,
                                                 PointCloudPtr& output,
                                                 const SE3& transform) {
    transformPointCloud(input, output, transform);
}

void PointCloudProcessor::allocateDeviceMemory(size_t num_points) {
    // CUDA memory allocation
}

void PointCloudProcessor::freeDeviceMemory() {
    // CUDA memory deallocation
}

} // namespace cuda_slam
