# CUDA-enabled ROS2 SLAM System Docker Image
FROM nvidia/cuda:11.8.0-devel-ubuntu22.04

# Avoid prompts during build
ENV DEBIAN_FRONTEND=noninteractive

# Install ROS2 Humble
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-camera-info-manager \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-tf2-eigen \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Install development tools and dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    vim \
    libopencv-dev \
    libeigen3-dev \
    libpcl-dev \
    libyaml-cpp-dev \
    libgtest-dev \
    && rm -rf /var/lib/apt/lists/*

# Install CUDA libraries
RUN apt-get update && apt-get install -y \
    libcublas-dev-11-8 \
    cuda-nvcc-11-8 \
    && rm -rf /var/lib/apt/lists/*

# Setup ROS2 workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Copy package
COPY . /ros2_ws/src/cuda_slam_system/

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# Install package dependencies
RUN . /opt/ros/humble/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y

# Build workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CUDA_ARCHITECTURES="61;75;86" \
    --symlink-install

# Source workspace in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Setup entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
