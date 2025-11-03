#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    use_cuda = LaunchConfiguration('use_cuda')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_cuda', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),

        Node(
            package='cuda_slam_system',
            executable='visual_odometry_node',
            name='visual_odometry',
            output='screen',
            parameters=[{
                'use_cuda': use_cuda,
                'num_features': 1500
            }]
        ),

        Node(
            package='cuda_slam_system',
            executable='fusion_node',
            name='sensor_fusion',
            output='screen',
            parameters=[{
                'use_visual': True,
                'use_lidar': False
            }]
        ),

        Node(
            package='cuda_slam_system',
            executable='visualizer_node',
            name='visualizer',
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(use_rviz)
        )
    ])
