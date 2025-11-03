#!/bin/bash

# CUDA SLAM System Build Script

set -e

echo "========================================="
echo "Building CUDA SLAM System"
echo "========================================="

# Check if CUDA is available
if ! command -v nvcc &> /dev/null; then
    echo "WARNING: CUDA compiler (nvcc) not found!"
    echo "Please install CUDA toolkit or disable CUDA in CMakeLists.txt"
fi

# Source ROS2
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "✓ ROS2 Humble sourced"
else
    echo "ERROR: ROS2 Humble not found!"
    exit 1
fi

# Navigate to workspace root
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

cd "$WS_DIR"
echo "Working directory: $WS_DIR"

# Install dependencies
echo ""
echo "Installing dependencies..."
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build
echo ""
echo "Building workspace..."
colcon build \
    --packages-select cuda_slam_system \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CUDA_ARCHITECTURES="61;75;86" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    --symlink-install

echo ""
echo "========================================="
echo "Build complete!"
echo "========================================="
echo ""
echo "Source the workspace with:"
echo "  source $WS_DIR/install/setup.bash"
echo ""
echo "Run tests with:"
echo "  colcon test --packages-select cuda_slam_system"
echo ""
echo "Launch SLAM with:"
echo "  ros2 launch cuda_slam_system stereo_slam.launch.py"
