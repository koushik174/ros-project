#include <gtest/gtest.h>
#include "cuda_slam_system/processing/point_cloud_processor.hpp"
#include <pcl/io/pcd_io.h>

namespace cuda_slam {
namespace test {

class PointCloudProcessorTest : public ::testing::Test {
protected:
    void SetUp() override {
        processor_ = std::make_unique<PointCloudProcessor>();

        // Create test point cloud
        test_cloud_ = std::make_shared<PointCloudT>();
        test_cloud_->width = 1000;
        test_cloud_->height = 1;
        test_cloud_->points.resize(1000);

        for (size_t i = 0; i < test_cloud_->points.size(); ++i) {
            test_cloud_->points[i].x = static_cast<float>(rand()) / RAND_MAX;
            test_cloud_->points[i].y = static_cast<float>(rand()) / RAND_MAX;
            test_cloud_->points[i].z = static_cast<float>(rand()) / RAND_MAX;
        }
    }

    std::unique_ptr<PointCloudProcessor> processor_;
    PointCloudPtr test_cloud_;
};

TEST_F(PointCloudProcessorTest, VoxelFiltering) {
    PointCloudPtr filtered(new PointCloudT);

    processor_->voxelFilter(test_cloud_, filtered, 0.05f);

    EXPECT_GT(filtered->points.size(), 0);
    EXPECT_LT(filtered->points.size(), test_cloud_->points.size());
}

TEST_F(PointCloudProcessorTest, CudaVoxelFiltering) {
    PointCloudPtr filtered_cpu(new PointCloudT);
    PointCloudPtr filtered_cuda(new PointCloudT);

    processor_->voxelFilter(test_cloud_, filtered_cpu, 0.05f);
    processor_->voxelFilterCuda(test_cloud_, filtered_cuda, 0.05f);

    // Both should produce similar results
    EXPECT_NEAR(filtered_cpu->points.size(), filtered_cuda->points.size(), 50);
}

TEST_F(PointCloudProcessorTest, TransformPointCloud) {
    PointCloudPtr transformed(new PointCloudT);

    SE3 transform = SE3::Identity();
    transform.translation() << 1.0, 2.0, 3.0;

    processor_->transformPointCloud(test_cloud_, transformed, transform);

    EXPECT_EQ(transformed->points.size(), test_cloud_->points.size());

    // Check if transformation was applied
    for (size_t i = 0; i < 10; ++i) {
        EXPECT_NEAR(transformed->points[i].x, test_cloud_->points[i].x + 1.0, 1e-5);
        EXPECT_NEAR(transformed->points[i].y, test_cloud_->points[i].y + 2.0, 1e-5);
        EXPECT_NEAR(transformed->points[i].z, test_cloud_->points[i].z + 3.0, 1e-5);
    }
}

} // namespace test
} // namespace cuda_slam

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
