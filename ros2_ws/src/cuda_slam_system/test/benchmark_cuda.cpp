#include <iostream>
#include <chrono>
#include "cuda_slam_system/core/orb_detector.hpp"
#include "cuda_slam_system/processing/point_cloud_processor.hpp"
#include "cuda_slam_system/utils/timer.hpp"
#include <opencv2/opencv.hpp>

namespace cuda_slam {

class CudaBenchmark {
public:
    void runAll() {
        std::cout << "\n=== CUDA SLAM System Benchmarks ===" << std::endl;

        benchmarkOrbDetection();
        benchmarkOrbMatching();
        benchmarkVoxelFiltering();
        benchmarkICP();

        std::cout << "\n=== Benchmark Complete ===" << std::endl;
    }

private:
    void benchmarkOrbDetection() {
        std::cout << "\n--- ORB Feature Detection Benchmark ---" << std::endl;

        cv::Mat image = cv::imread("test_image.png", cv::IMREAD_GRAYSCALE);
        if (image.empty()) {
            image = cv::Mat(1024, 1280, CV_8UC1);
            cv::randu(image, 0, 255);
        }

        OrbDetector detector(2000);
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;

        // CPU benchmark
        Timer timer_cpu;
        timer_cpu.start();
        for (int i = 0; i < 10; i++) {
            detector.detectAndCompute(image, keypoints, descriptors);
        }
        timer_cpu.stop();
        double cpu_time = timer_cpu.getElapsedMs() / 10.0;

        // CUDA benchmark
        Timer timer_cuda;
        timer_cuda.start();
        for (int i = 0; i < 10; i++) {
            detector.detectAndComputeCuda(image, keypoints, descriptors);
        }
        timer_cuda.stop();
        double cuda_time = timer_cuda.getElapsedMs() / 10.0;

        std::cout << "CPU time: " << cpu_time << " ms" << std::endl;
        std::cout << "CUDA time: " << cuda_time << " ms" << std::endl;
        std::cout << "Speedup: " << (cpu_time / cuda_time) << "x" << std::endl;
    }

    void benchmarkOrbMatching() {
        std::cout << "\n--- ORB Feature Matching Benchmark ---" << std::endl;

        // Create random descriptors
        cv::Mat desc1(1000, 32, CV_8UC1);
        cv::Mat desc2(1000, 32, CV_8UC1);
        cv::randu(desc1, 0, 255);
        cv::randu(desc2, 0, 255);

        // Benchmark brute force matching
        Timer timer;
        timer.start();
        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<cv::DMatch> matches;
        for (int i = 0; i < 10; i++) {
            matcher.match(desc1, desc2, matches);
        }
        timer.stop();

        std::cout << "Average matching time: " << timer.getElapsedMs() / 10.0 << " ms" << std::endl;
        std::cout << "Matches found: " << matches.size() << std::endl;
    }

    void benchmarkVoxelFiltering() {
        std::cout << "\n--- Voxel Filtering Benchmark ---" << std::endl;

        // Create large point cloud
        PointCloudPtr cloud(new PointCloudT);
        cloud->width = 100000;
        cloud->height = 1;
        cloud->points.resize(100000);

        for (auto& point : cloud->points) {
            point.x = static_cast<float>(rand()) / RAND_MAX * 100;
            point.y = static_cast<float>(rand()) / RAND_MAX * 100;
            point.z = static_cast<float>(rand()) / RAND_MAX * 100;
        }

        PointCloudProcessor processor;
        PointCloudPtr filtered(new PointCloudT);

        // CPU benchmark
        Timer timer_cpu;
        timer_cpu.start();
        processor.voxelFilter(cloud, filtered, 0.1f);
        timer_cpu.stop();

        // CUDA benchmark
        Timer timer_cuda;
        timer_cuda.start();
        processor.voxelFilterCuda(cloud, filtered, 0.1f);
        timer_cuda.stop();

        std::cout << "Input points: " << cloud->points.size() << std::endl;
        std::cout << "Output points: " << filtered->points.size() << std::endl;
        std::cout << "CPU time: " << timer_cpu.getElapsedMs() << " ms" << std::endl;
        std::cout << "CUDA time: " << timer_cuda.getElapsedMs() << " ms" << std::endl;
        std::cout << "Speedup: " << (timer_cpu.getElapsedMs() / timer_cuda.getElapsedMs()) << "x" << std::endl;
    }

    void benchmarkICP() {
        std::cout << "\n--- ICP Alignment Benchmark ---" << std::endl;

        // Create two overlapping point clouds
        PointCloudPtr source(new PointCloudT);
        PointCloudPtr target(new PointCloudT);

        for (int i = 0; i < 10000; i++) {
            PointT p;
            p.x = static_cast<float>(rand()) / RAND_MAX * 10;
            p.y = static_cast<float>(rand()) / RAND_MAX * 10;
            p.z = static_cast<float>(rand()) / RAND_MAX * 10;
            source->points.push_back(p);

            p.x += 0.1f;
            p.y += 0.1f;
            target->points.push_back(p);
        }

        source->width = source->points.size();
        source->height = 1;
        target->width = target->points.size();
        target->height = 1;

        ICPAligner aligner;
        SE3 transformation = SE3::Identity();
        PointCloudPtr aligned(new PointCloudT);

        Timer timer;
        timer.start();
        aligner.align(source, target, transformation, aligned);
        timer.stop();

        std::cout << "ICP time: " << timer.getElapsedMs() << " ms" << std::endl;
        std::cout << "Final iterations: " << aligner.getFinalIterations() << std::endl;
        std::cout << "Fitness score: " << aligner.getFitnessScore() << std::endl;
    }
};

} // namespace cuda_slam

int main(int argc, char** argv) {
    cuda_slam::CudaBenchmark benchmark;
    benchmark.runAll();
    return 0;
}
