#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Launch arguments
    data_path = LaunchConfiguration('data_path', default='/media/a/E/MyProjects/Water/ros2_ws/data/')
    camera_1_index = LaunchConfiguration('camera_1_index', default='0')
    camera_2_index = LaunchConfiguration('camera_2_index', default='1')
    camera_3_index = LaunchConfiguration('camera_3_index', default='2')
    
    # Create data directories
    os.makedirs(os.path.join(data_path.perform(None), 'camera_1'), exist_ok=True)
    os.makedirs(os.path.join(data_path.perform(None), 'camera_2'), exist_ok=True)
    os.makedirs(os.path.join(data_path.perform(None), 'camera_3'), exist_ok=True)
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'data_path',
            default_value='/media/a/E/MyProjects/Water/ros2_ws/data/',
            description='Base path for data storage'
        ),
        DeclareLaunchArgument(
            'camera_1_index',
            default_value='0',
            description='USB index for camera 1'
        ),
        DeclareLaunchArgument(
            'camera_2_index',
            default_value='1',
            description='USB index for camera 2'
        ),
        DeclareLaunchArgument(
            'camera_3_index',
            default_value='2',
            description='USB index for camera 3'
        ),
        
        # Camera 1 Node
        Node(
            package='seaqr_controller',
            executable='camera_gstreamer_with_ros2',
            name='camera_1_node',
            output='screen',
            parameters=[{
                'camera_index': camera_1_index,
                'data_path': os.path.join(data_path.perform(None), 'camera_1')
            }]
        ),
        
        # Camera 2 Node
        Node(
            package='seaqr_controller',
            executable='camera_gstreamer_with_ros2',
            name='camera_2_node',
            output='screen',
            parameters=[{
                'camera_index': camera_2_index,
                'data_path': os.path.join(data_path.perform(None), 'camera_2')
            }]
        ),
        
        # Camera 3 Node
        Node(
            package='seaqr_controller',
            executable='camera_gstreamer_with_ros2',
            name='camera_3_node',
            output='screen',
            parameters=[{
                'camera_index': camera_3_index,
                'data_path': os.path.join(data_path.perform(None), 'camera_3')
            }]
        ),
    ])



