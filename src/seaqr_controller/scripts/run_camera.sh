#!/bin/bash

# Script to run a single camera
# Usage: ./run_camera.sh <camera_index> <camera_name>

CAMERA_INDEX=${1:-0}
CAMERA_NAME=${2:-"Camera_${CAMERA_INDEX}"}
DATA_PATH="/media/a/E/MyProjects/Water/ros2_ws/data/camera_${CAMERA_INDEX}"

echo "Starting camera ${CAMERA_INDEX} (${CAMERA_NAME})"
echo "Data will be saved to: ${DATA_PATH}"

# Source ROS2 workspace
source /media/a/E/MyProjects/Water/ros2_ws/install/setup.bash

# Run the camera node
ros2 run seaqr_controller camera_gstreamer_with_ros2 \
    --ros-args \
    -p camera_index:=${CAMERA_INDEX} \
    -p data_path:=${DATA_PATH}



