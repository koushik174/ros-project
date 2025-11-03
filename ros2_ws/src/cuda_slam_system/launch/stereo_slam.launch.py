#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare arguments
    use_cuda_arg = DeclareLaunchArgument(
        'use_cuda', default_value='true',
        description='Enable CUDA acceleration'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('cuda_slam_system'),
            'config', 'stereo_config.yaml'
        ]),
        description='Path to configuration file'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2 for visualization'
    )

    # Nodes
    visual_odometry_node = Node(
        package='cuda_slam_system',
        executable='visual_odometry_node',
        name='visual_odometry',
        output='screen',
        parameters=[{
            'use_cuda': LaunchConfiguration('use_cuda'),
            'num_features': 2000,
            'publish_rate': 30.0
        }],
        remappings=[
            ('camera/image_raw', '/camera/left/image_raw'),
            ('camera/camera_info', '/camera/left/camera_info')
        ]
    )

    fusion_node = Node(
        package='cuda_slam_system',
        executable='fusion_node',
        name='sensor_fusion',
        output='screen',
        parameters=[{
            'imu_topic': '/imu/data',
            'visual_odom_topic': '/visual_odometry',
            'lidar_odom_topic': '/lidar_odometry',
            'publish_rate': 100.0,
            'use_visual': True,
            'use_lidar': True
        }]
    )

    mapping_node = Node(
        package='cuda_slam_system',
        executable='mapping_node',
        name='mapping',
        output='screen',
        parameters=[{
            'voxel_leaf_size': 0.05,
            'use_cuda': LaunchConfiguration('use_cuda')
        }],
        remappings=[
            ('points', '/lidar/points')
        ]
    )

    loop_closure_node = Node(
        package='cuda_slam_system',
        executable='loop_closure_node',
        name='loop_closure',
        output='screen'
    )

    visualizer_node = Node(
        package='cuda_slam_system',
        executable='visualizer_node',
        name='visualizer',
        output='screen'
    )

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('cuda_slam_system'),
            'config', 'slam_viz.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        use_cuda_arg,
        config_file_arg,
        use_rviz_arg,
        visual_odometry_node,
        fusion_node,
        mapping_node,
        loop_closure_node,
        visualizer_node,
        rviz_node
    ])
