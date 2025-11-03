#pragma once

#include "cuda_slam_system/common.hpp"
#include <yaml-cpp/yaml.h>
#include <string>

namespace cuda_slam {

class ConfigLoader {
public:
    ConfigLoader(const std::string& config_file);
    ~ConfigLoader() = default;

    // Load configuration
    bool load();
    bool loadFromFile(const std::string& filename);

    // Camera configuration
    CameraIntrinsics getCameraIntrinsics(const std::string& camera_name = "camera0") const;
    SE3 getStereoCameraBaseline() const;

    // ORB detector configuration
    int getOrbNumFeatures() const;
    float getOrbScaleFactor() const;
    int getOrbNumLevels() const;
    int getOrbFastThreshold() const;

    // IMU configuration
    SE3 getImuToCamera() const;
    Vector3d getAccelNoise() const;
    Vector3d getGyroNoise() const;
    Vector3d getAccelBias() const;
    Vector3d getGyroBias() const;

    // LiDAR configuration
    SE3 getLidarToCamera() const;
    float getLidarMaxRange() const;
    float getLidarMinRange() const;

    // SLAM parameters
    int getMaxKeyFrames() const;
    int getMaxMapPoints() const;
    float getKeyFrameInsertionThreshold() const;
    int getLocalMappingKeyframes() const;

    // Loop closure parameters
    float getLoopClosureScore() const;
    int getLoopClosureMinKeyframes() const;
    int getLoopClosureConsistency() const;

    // Point cloud processing
    float getVoxelLeafSize() const;
    float getICPMaxCorrespondenceDistance() const;
    int getICPMaxIterations() const;

    // Visualization
    bool enableVisualization() const;
    float getVisualizationRate() const;

    // Generic getters
    template<typename T>
    T get(const std::string& key, const T& default_value) const {
        try {
            return config_[key].as<T>();
        } catch (...) {
            return default_value;
        }
    }

private:
    std::string config_file_;
    YAML::Node config_;

    // Helper functions
    Matrix3d parseMatrix3d(const YAML::Node& node) const;
    Vector3d parseVector3d(const YAML::Node& node) const;
    SE3 parseSE3(const YAML::Node& node) const;
};

} // namespace cuda_slam
