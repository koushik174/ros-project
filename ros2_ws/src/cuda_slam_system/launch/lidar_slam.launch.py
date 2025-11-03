#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_cuda', default_value='true'),

        Node(
            package='cuda_slam_system',
            executable='mapping_node',
            name='lidar_mapping',
            output='screen',
            parameters=[{
                'voxel_leaf_size': 0.1,
                'use_cuda': LaunchConfiguration('use_cuda')
            }]
        ),

        Node(
            package='cuda_slam_system',
            executable='fusion_node',
            name='sensor_fusion',
            output='screen',
            parameters=[{
                'use_visual': False,
                'use_lidar': True
            }]
        ),

        Node(
            package='cuda_slam_system',
            executable='visualizer_node',
            name='visualizer',
            output='screen'
        )
    ])
