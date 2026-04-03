#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Launch arguments
    data_path = LaunchConfiguration('data_path', default='/media/a/E/MyProjects/Water/ros2_ws/data/')
    
    # Create data directories
    os.makedirs(os.path.join(data_path.perform(None), 'adsb'), exist_ok=True)
    os.makedirs(os.path.join(data_path.perform(None), 'camera'), exist_ok=True)
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'data_path',
            default_value='/media/a/E/MyProjects/Water/ros2_ws/data/',
            description='Base path for data storage'
        ),
        
        # ADSB Node
        Node(
            package='seaqr_controller',
            executable='adsb_data',
            name='adsb_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'data_path': os.path.join(data_path.perform(None), 'adsb')
            }]
        ),
        
        # Camera Node
        Node(
            package='seaqr_controller',
            executable='camera_gstreamer_with_ros2',
            name='camera_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'data_path': os.path.join(data_path.perform(None), 'camera')
            }]
        ),
    ])
