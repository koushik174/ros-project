cmake_minimum_required(VERSION 3.18)
project(cuda_slam_system LANGUAGES CXX CUDA)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

# Set CUDA architectures (adjust based on your GPU)
set(CMAKE_CUDA_ARCHITECTURES 61 75 86)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# CUDA compiler options
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -O3 -use_fast_math")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# External libraries
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration)
find_package(yaml-cpp REQUIRED)

# CUDA
find_package(CUDAToolkit REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

# ROS2 message generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/KeyFrame.msg"
  "msg/MapPoint.msg"
  "msg/SlamState.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

# CUDA libraries
set(CUDA_SOURCES
  cuda/orb_detector.cu
  cuda/orb_matcher.cu
  cuda/voxel_filter.cu
  cuda/icp_alignment.cu
  cuda/feature_utils.cu
)

add_library(cuda_kernels SHARED ${CUDA_SOURCES})
target_include_directories(cuda_kernels PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CUDA_INCLUDE_DIRS}
)
target_link_libraries(cuda_kernels
  ${OpenCV_LIBS}
  CUDA::cudart
  CUDA::cublas
)

# Core SLAM library
set(SLAM_SOURCES
  src/core/orb_detector.cpp
  src/core/orb_matcher.cpp
  src/core/frame.cpp
  src/core/map_point.cpp
  src/core/keyframe.cpp
  src/core/map.cpp
  src/core/local_mapper.cpp
  src/core/loop_closer.cpp
  src/core/pose_graph.cpp
  src/fusion/ekf.cpp
  src/fusion/sensor_fusion.cpp
  src/processing/point_cloud_processor.cpp
  src/processing/voxel_filter.cpp
  src/processing/icp_aligner.cpp
  src/utils/config_loader.cpp
  src/utils/timer.cpp
)

add_library(slam_core SHARED ${SLAM_SOURCES})
ament_target_dependencies(slam_core
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
  tf2
  tf2_ros
  tf2_eigen
  pcl_conversions
)
target_link_libraries(slam_core
  cuda_kernels
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  yaml-cpp
  Eigen3::Eigen
)

# ROS2 Nodes
set(NODE_SOURCES
  src/nodes/visual_odometry_node.cpp
  src/nodes/mapping_node.cpp
  src/nodes/loop_closure_node.cpp
  src/nodes/fusion_node.cpp
  src/nodes/visualizer_node.cpp
)

# Visual Odometry Node
add_executable(visual_odometry_node src/nodes/visual_odometry_node.cpp)
ament_target_dependencies(visual_odometry_node rclcpp sensor_msgs geometry_msgs cv_bridge tf2_ros)
target_link_libraries(visual_odometry_node slam_core cuda_kernels)

# Mapping Node
add_executable(mapping_node src/nodes/mapping_node.cpp)
ament_target_dependencies(mapping_node rclcpp sensor_msgs nav_msgs pcl_conversions)
target_link_libraries(mapping_node slam_core cuda_kernels)

# Loop Closure Node
add_executable(loop_closure_node src/nodes/loop_closure_node.cpp)
ament_target_dependencies(loop_closure_node rclcpp sensor_msgs geometry_msgs)
target_link_libraries(loop_closure_node slam_core)

# Fusion Node
add_executable(fusion_node src/nodes/fusion_node.cpp)
ament_target_dependencies(fusion_node rclcpp sensor_msgs geometry_msgs nav_msgs tf2_ros)
target_link_libraries(fusion_node slam_core)

# Visualizer Node
add_executable(visualizer_node src/nodes/visualizer_node.cpp)
ament_target_dependencies(visualizer_node rclcpp visualization_msgs sensor_msgs)
target_link_libraries(visualizer_node slam_core)

# Install targets
install(TARGETS
  cuda_kernels
  slam_core
  visual_odometry_node
  mapping_node
  loop_closure_node
  fusion_node
  visualizer_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_lint_auto_find_test_dependencies()

  # Unit tests
  ament_add_gtest(test_orb_detector test/test_orb_detector.cpp)
  target_link_libraries(test_orb_detector slam_core cuda_kernels)

  ament_add_gtest(test_ekf test/test_ekf.cpp)
  target_link_libraries(test_ekf slam_core)

  ament_add_gtest(test_point_cloud test/test_point_cloud.cpp)
  target_link_libraries(test_point_cloud slam_core cuda_kernels)

  # Benchmark tests
  add_executable(benchmark_cuda test/benchmark_cuda.cpp)
  target_link_libraries(benchmark_cuda slam_core cuda_kernels)
  install(TARGETS benchmark_cuda DESTINATION lib/${PROJECT_NAME})
endif()

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(slam_core cuda_kernels)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
  pcl_conversions
  tf2_ros
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(slam_core "${cpp_typesupport_target}")

ament_package()
