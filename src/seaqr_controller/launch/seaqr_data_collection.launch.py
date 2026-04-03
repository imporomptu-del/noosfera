#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Set environment to use virtual environment
        SetEnvironmentVariable('PYTHONPATH', '/home/a/seaqr-horizon/lib/python3.10/site-packages:/home/a/Projects/ros2_ws/src:$PYTHONPATH'),
        
        # Launch arguments
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level'
        ),
        
        LogInfo(msg="🚀 Starting SEAQR Data Collection System..."),
        
        # ADSB Listener Node - Run directly from source
        ExecuteProcess(
            cmd=['/home/a/seaqr-horizon/bin/python3', '-m', 'seaqr_controller.adsb_reader.adsb_usb'],
            cwd='/home/a/Projects/ros2_ws/src/seaqr_controller',
            output='screen',
            name='adsb_listener',
            env={
                'PYTHONPATH': '/home/a/seaqr-horizon/lib/python3.10/site-packages:/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/home/a/Projects/ros2_ws/src',
                'LD_LIBRARY_PATH': '/opt/ros/humble/lib',
                'ROS_DOMAIN_ID': '0',
                'AMENT_PREFIX_PATH': '/opt/ros/humble',
                'ROS_DISTRO': 'humble'
            }
        ),
        
        # Camera Recorder Node - Run directly from source
        ExecuteProcess(
            cmd=['/home/a/seaqr-horizon/bin/python3', '-m', 'seaqr_controller.camera_reader.camera_recorder_node'],
            cwd='/home/a/Projects/ros2_ws/src/seaqr_controller',
            output='screen',
            name='camera_recorder',
            env={
                'PYTHONPATH': '/home/a/seaqr-horizon/lib/python3.10/site-packages:/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/home/a/Projects/ros2_ws/src',
                'LD_LIBRARY_PATH': '/opt/ros/humble/lib',
                'ROS_DOMAIN_ID': '0',
                'AMENT_PREFIX_PATH': '/opt/ros/humble',
                'ROS_DISTRO': 'humble'
            }
        ),
        
        LogInfo(msg="📡 ADSB Listener: /adsb_data topic"),
        LogInfo(msg="📹 Camera Recorder: /camera_status topic"),
        LogInfo(msg="💾 Data saved to: /home/a/Projects/ros2_ws/data/"),
    ])
