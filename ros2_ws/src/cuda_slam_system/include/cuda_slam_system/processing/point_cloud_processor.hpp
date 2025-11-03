#pragma once

#include "cuda_slam_system/common.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace cuda_slam {

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;
using PointCloudPtr = PointCloudT::Ptr;

class PointCloudProcessor {
public:
    PointCloudProcessor();
    ~PointCloudProcessor();

    // Voxel filtering (downsampling)
    void voxelFilter(const PointCloudPtr& input,
                    PointCloudPtr& output,
                    float leaf_size);

    void voxelFilterCuda(const PointCloudPtr& input,
                        PointCloudPtr& output,
                        float leaf_size);

    // Statistical outlier removal
    void statisticalOutlierRemoval(const PointCloudPtr& input,
                                  PointCloudPtr& output,
                                  int mean_k = 50,
                                  float std_dev_mul = 1.0);

    // Radius outlier removal
    void radiusOutlierRemoval(const PointCloudPtr& input,
                             PointCloudPtr& output,
                             float radius = 0.8,
                             int min_neighbors = 2);

    // Normal estimation
    void estimateNormals(const PointCloudPtr& cloud,
                        pcl::PointCloud<pcl::Normal>::Ptr normals,
                        float radius = 0.03);

    void estimateNormalsCuda(const PointCloudPtr& cloud,
                            pcl::PointCloud<pcl::Normal>::Ptr normals,
                            float radius = 0.03);

    // Plane segmentation (RANSAC)
    void segmentPlane(const PointCloudPtr& cloud,
                     pcl::ModelCoefficients::Ptr coefficients,
                     pcl::PointIndices::Ptr inliers,
                     float distance_threshold = 0.01);

    // Transform point cloud
    void transformPointCloud(const PointCloudPtr& input,
                            PointCloudPtr& output,
                            const SE3& transform);

    void transformPointCloudCuda(const PointCloudPtr& input,
                                PointCloudPtr& output,
                                const SE3& transform);

    // Merge point clouds
    void mergePointClouds(const std::vector<PointCloudPtr>& clouds,
                         PointCloudPtr& output);

    // Extract features
    void extractKeypoints(const PointCloudPtr& cloud,
                         PointCloudPtr& keypoints,
                         float threshold = 0.01);

private:
    // CUDA device memory
    void* d_points_in_;
    void* d_points_out_;
    void* d_normals_;
    void* d_voxel_grid_;

    size_t allocated_size_;

    void allocateDeviceMemory(size_t num_points);
    void freeDeviceMemory();
};

} // namespace cuda_slam
