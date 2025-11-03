#pragma once

#include "cuda_slam_system/common.hpp"
#include "cuda_slam_system/core/keyframe.hpp"
#include "cuda_slam_system/core/map.hpp"
#include "cuda_slam_system/core/orb_matcher.hpp"
#include <thread>

namespace cuda_slam {

class LoopCloser {
public:
    LoopCloser(MapPtr map);
    ~LoopCloser();

    // Main loop
    void run();
    void requestStop();
    void stop();
    bool isRunning() const { return is_running_; }

    // Add new keyframe for loop detection
    void addKeyFrame(KeyFramePtr keyframe);

    // Loop closure detection
    bool detectLoop();
    bool computeSim3();
    void correctLoop();

    // Callbacks
    void setLoopClosureCallback(std::function<void(const LoopClosure&)> callback);

private:
    // Loop detection
    KeyFramePtr detectLoopCandidate(KeyFramePtr current_keyframe);
    bool checkLoopConsistency(KeyFramePtr candidate);

    // Similarity transformation (Sim3)
    bool computeSim3Transform(KeyFramePtr keyframe1, KeyFramePtr keyframe2,
                             SE3& T12, float& scale);

    // Graph optimization after loop closure
    void searchAndFuse(const std::map<KeyFramePtr, SE3>& corrected_poses);
    void optimizeEssentialGraph();

    // Database for place recognition
    struct BoWDatabase {
        // Simplified bag-of-words database
        std::map<KeyFramePtr, std::vector<float>> descriptors;

        std::vector<KeyFramePtr> query(const std::vector<float>& descriptor,
                                      int max_results = 5);
        void add(KeyFramePtr keyframe, const std::vector<float>& descriptor);
    };

    MapPtr map_;
    std::unique_ptr<OrbMatcher> matcher_;
    std::unique_ptr<BoWDatabase> bow_database_;

    // Loop detection queue
    std::deque<KeyFramePtr> keyframe_queue_;
    KeyFramePtr current_keyframe_;
    KeyFramePtr matched_keyframe_;

    // Loop closure result
    LoopClosure current_loop_;
    std::vector<LoopClosure> detected_loops_;

    // Thread management
    std::thread thread_;
    bool is_running_;
    bool stop_requested_;
    std::mutex mutex_queue_;
    std::mutex mutex_loop_;

    // Callback
    std::function<void(const LoopClosure&)> loop_closure_callback_;

    // Parameters
    float min_loop_score_;
    int min_loop_keyframes_;
    int consistency_threshold_;

    // Consistency checking
    struct ConsistentGroup {
        std::set<KeyFramePtr> keyframes;
        int consistency_counter;
    };
    std::vector<ConsistentGroup> consistent_groups_;
};

} // namespace cuda_slam
