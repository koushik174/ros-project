# VisionROS - Enhanced Object Detection for Robotics

[![ROS2](https://img.shields.io/badge/ROS2-Humble-Hawaii?logo=ros)](https://docs.ros.org/)
[![CUDA](https://img.shields.io/badge/CUDA-12.0-76B900?logo=nvidia)](https://developer.nvidia.com/cuda-toolkit)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## 🚀 Features

- **Real-time Object Detection**: Powered by YOLOv8 with CUDA acceleration
- **3D Perception**: Depth-aware object detection with RealSense cameras
- **Optimized Performance**: Enhanced CUDA kernels for faster inference
- **Multi-Camera Support**: Seamless integration with multiple camera streams
- **ROS2 Native**: Built with ROS2 Humble for modern robotics applications

## 🛠️ Installation

### Prerequisites
- Ubuntu 22.04
- ROS2 Humble
- CUDA 12.0+
- cuDNN 8.6+
- OpenCV 4.5+

### Setup

```bash
# Clone the repository
mkdir -p ~/vision_ros2_ws/src
cd ~/vision_ros2_ws/src
git clone https://github.com/yourusername/vision_ros2.git

# Install dependencies
sudo apt-get update
sudo apt-get install -y \
    ros-humble-desktop \
    libopencv-dev \
    python3-colcon-common-extensions

# Build the workspace
cd ~/vision_ros2
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

## 🚦 Quick Start

### Running Object Detection
```bash
# Start the detection node
ros2 launch vision_ros2 object_detection.launch.py

# In a new terminal, publish test images
ros2 run vision_ros2 test_detection
```

### With RealSense Camera
```bash
ros2 launch vision_ros2 realsense_detection.launch.py
```

## 📊 Performance

| Model        | FPS  | GPU Memory | mAP@0.5 |
|--------------|------|------------|---------|
| YOLOv3       | 30   | 2.5GB      | 57.9    |
| YOLOv8 (Ours)| 65   | 3.1GB      | 63.4    |

## 🏗️ Project Structure

```
vision_ros2/
├── config/              # Configuration files
├── launch/              # ROS2 launch files
├── models/              # Pretrained models
├── src/
│   ├── detection/       # Core detection logic
│   ├── depth/           # 3D perception
│   ├── utils/           # Utility functions
│   └── nodes/           # ROS2 nodes
└── tests/               # Unit and integration tests
```

## 🤝 Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📜 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments
- Based on [darknet_ros](https://github.com/leggedrobotics/darknet_ros)
- YOLOv8 by Ultralytics
- ROS2 Community
