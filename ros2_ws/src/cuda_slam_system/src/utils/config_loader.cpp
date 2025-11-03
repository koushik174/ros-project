#include "cuda_slam_system/utils/config_loader.hpp"
#include <fstream>

namespace cuda_slam {

ConfigLoader::ConfigLoader(const std::string& config_file)
    : config_file_(config_file) {
}

bool ConfigLoader::load() {
    return loadFromFile(config_file_);
}

bool ConfigLoader::loadFromFile(const std::string& filename) {
    try {
        config_ = YAML::LoadFile(filename);
        return true;
    } catch (const YAML::Exception& e) {
        return false;
    }
}

CameraIntrinsics ConfigLoader::getCameraIntrinsics(const std::string& camera_name) const {
    CameraIntrinsics cam;
    if (config_[camera_name]) {
        cam.fx = config_[camera_name]["fx"].as<float>();
        cam.fy = config_[camera_name]["fy"].as<float>();
        cam.cx = config_[camera_name]["cx"].as<float>();
        cam.cy = config_[camera_name]["cy"].as<float>();
        cam.width = config_[camera_name]["width"].as<int>();
        cam.height = config_[camera_name]["height"].as<int>();
    }
    return cam;
}

int ConfigLoader::getOrbNumFeatures() const {
    return get<int>("orb.num_features", 2000);
}

float ConfigLoader::getOrbScaleFactor() const {
    return get<float>("orb.scale_factor", 1.2f);
}

} // namespace cuda_slam
