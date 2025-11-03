#pragma once

#include "cuda_slam_system/common.hpp"
#include "cuda_slam_system/core/frame.hpp"
#include "cuda_slam_system/core/keyframe.hpp"

namespace cuda_slam {

class OrbMatcher {
public:
    OrbMatcher(float nn_ratio = 0.6f, bool check_orientation = true);
    ~OrbMatcher();

    // Feature matching
    int searchByProjection(Frame& current_frame, const Frame& last_frame,
                          float th = 7.0f);

    int searchByBow(KeyFrame& keyframe1, KeyFrame& keyframe2,
                   std::vector<Match>& matches);

    int searchForTriangulation(KeyFrame& keyframe1, KeyFrame& keyframe2,
                              std::vector<Match>& matches);

    // CUDA-accelerated matching
    int searchByCudaBruteForce(const cv::Mat& descriptors1,
                              const cv::Mat& descriptors2,
                              std::vector<Match>& matches);

    int searchByCudaHamming(const cv::Mat& descriptors1,
                           const cv::Mat& descriptors2,
                           std::vector<Match>& matches);

    // Map point matching
    int searchByProjection(Frame& frame, const std::vector<MapPointPtr>& map_points,
                          float th = 3.0f);

    int searchForInitialization(Frame& frame1, Frame& frame2,
                               std::vector<Match>& matches,
                               int window_size = 100);

    // Loop closure matching
    int searchBySim3(KeyFrame& keyframe1, KeyFrame& keyframe2,
                    std::vector<Match>& matches,
                    const float th = 7.5f);

    // Utility
    static int descriptorDistance(const cv::Mat& a, const cv::Mat& b);
    static int hammingDistance(const cv::Mat& a, const cv::Mat& b);

private:
    void computeThreeMaxima(std::vector<int>* histo, int L,
                           int& ind1, int& ind2, int& ind3);

    bool checkDistEpipolarLine(const cv::KeyPoint& kp1, const cv::KeyPoint& kp2,
                              const Matrix3d& F12, const KeyFrame& keyframe2);

    float radiusByViewingCos(float viewing_cos);

    // CUDA device memory
    void* d_descriptors1_;
    void* d_descriptors2_;
    void* d_matches_;
    void* d_distances_;

    // Parameters
    float nn_ratio_;
    bool check_orientation_;
    static constexpr int HISTO_LENGTH = 30;
    static constexpr float MATCH_THRESHOLD = 50.0f;
};

} // namespace cuda_slam
