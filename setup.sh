#!/bin/bash

# VisionROS Setup Script
# This script will install all necessary dependencies and set up the workspace

set -e

# Set colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== VisionROS Setup ===${NC}"

# Check if running as root
if [ "$EUID" -eq 0 ]; then
    echo -e "${YELLOW}Please do not run this script as root.${NC}"
    exit 1
fi

# Check Ubuntu version
UBUNTU_VERSION=$(lsb_release -rs)
if [ "$UBUNTU_VERSION" != "22.04" ]; then
    echo -e "${YELLOW}This script is tested on Ubuntu 22.04. You're running Ubuntu $UBUNTU_VERSION.${NC}"
    read -p "Continue anyway? [y/N] " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Install ROS2 Humble if not already installed
if ! command -v ros2 &> /dev/null; then
    echo -e "${GREEN}Installing ROS2 Humble...${NC}"
    sudo apt update && sudo apt install -y curl gnupg2 lsb-release
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    sudo apt install -y ros-humble-desktop
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source ~/.bashrc
else
    echo -e "${GREEN}ROS2 is already installed.${NC}"
fi

# Install CUDA if not already installed
if ! command -v nvcc &> /dev/null; then
    echo -e "${GREEN}Installing CUDA...${NC}"
    wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
    sudo dpkg -i cuda-keyring_1.1-1_all.deb
    sudo apt-get update
    sudo apt-get -y install cuda
    echo 'export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}' >> ~/.bashrc
    echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}' >> ~/.bashrc
    source ~/.bashrc
    rm cuda-keyring_1.1-1_all.deb
else
    echo -e "${GREEN}CUDA is already installed.${NC}"
fi

# Install system dependencies
echo -e "${GREEN}Installing system dependencies...${NC}"
sudo apt update
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    python3-rosinstall-generator \
    python3-rosinstall \
    python3-wstool \
    build-essential \
    cmake \
    git \
    libopencv-dev \
    libeigen3-dev \
    libboost-all-dev \
    libyaml-cpp-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libusb-1.0-0-dev \
    libcanberra-gtk-module \
    libcanberra-gtk3-module

# Initialize rosdep if not already done
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    echo -e "${GREEN}Initializing rosdep...${NC}"
    sudo rosdep init || true
    rosdep update
fi

# Install Python dependencies
echo -e "${GREEN}Installing Python dependencies...${NC}"
pip3 install --user -U \
    numpy \
    opencv-python \
    pyyaml \
    rospkg \
    setuptools \
    vcstool \
    catkin_pkg \
    empy \
    lark-parser \
    scipy \
    matplotlib \
    scikit-learn \
    tqdm

# Create workspace if it doesn't exist
WORKSPACE_DIR="$HOME/vision_ros2_ws"
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo -e "${GREEN}Creating workspace at $WORKSPACE_DIR...${NC}"
    mkdir -p "$WORKSPACE_DIR/src"
    cd "$WORKSPACE_DIR"
    colcon build --symlink-install
else
    echo -e "${GREEN}Workspace already exists at $WORKSPACE_DIR${NC}"
    cd "$WORKSPACE_DIR"
fi

# Clone the repository if not already cloned
if [ ! -d "$WORKSPACE_DIR/src/vision_ros2" ]; then
    echo -e "${GREEN}Cloning VisionROS repository...${NC}"
    cd "$WORKSPACE_DIR/src"
    git clone https://github.com/yourusername/vision_ros2.git
else
    echo -e "${GREEN}VisionROS repository already cloned.${NC}"
fi

# Install ROS dependencies
cd "$WORKSPACE_DIR"
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
echo -e "${GREEN}Building the workspace...${NC}"
cd "$WORKSPACE_DIR"
colcon build --symlink-install

# Source the workspace
echo -e "${GREEN}Sourcing the workspace...${NC}"
echo "source $WORKSPACE_DIR/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo -e "\n${GREEN}=== Setup Complete! ===${NC}"
echo -e "To get started, run the following commands:"
echo -e "1. source ~/.bashrc"
echo -e "2. ros2 launch vision_ros2 object_detection.launch.py"
echo -e "\nFor more information, please refer to the README.md file."
