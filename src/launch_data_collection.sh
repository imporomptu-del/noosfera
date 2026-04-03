#!/bin/bash
# ================================================
# SEAQR Data Collection System Launch Script
# Launches ADS-B, Camera(s), IMU, and GPS recording simultaneously
# ================================================

set -euo pipefail

trap "echo '🛑 Stopping SEAQR Data Collection...'; kill 0; exit 0" SIGINT SIGTERM

echo "🚀 SEAQR Data Collection System"
echo "================================"

WORKDIR="/home/a/Projects/ros2_ws/src/seaqr_controller/seaqr_controller"
if [ ! -d "$WORKDIR" ]; then
    echo "❌ Directory not found: $WORKDIR"
    exit 1
fi
cd "$WORKDIR"

echo "📦 Activating Python/ROS2 environment..."
source /home/a/seaqr-horizon/bin/activate
# source /opt/ros/humble/setup.bash

echo ""
echo "🚀 Launching SEAQR Data Collection System..."
echo "---------------------------------------------"
echo "📡 ADS-B Reader:      adsb_reader/adsb_usb.py"
echo "📹 Camera 174 Reader: camera_reader/cam_174_reader.py"
echo "🌡️  Boson Reader:     camera_reader/cam_boson_reader.py"
echo "🧭 IMU Reader:        imu_reader/imu_store_to_mnt.py"
echo "📍 GPS Reader:        gps_reader/gps.py"
echo "💾 Data Storage:      /home/a/Projects/ros2_ws/data/"
echo ""
echo "Press Ctrl+C to stop all services"
echo "================================"
echo ""

sleep 8

# --- Launch all modules ---
python3 camera_reader/cam_174_reader.py &
CAM174_PID=$!

python3 camera_reader/cam_boson_reader.py &
BOSON_PID=$!

python3 adsb_reader/adsb_usb.py &
ADSB_PID=$!

python3 imu_reader/imu_store_to_mnt.py &
IMU_PID=$!

python3 gps_reader/gps.py &
GPS_PID=$!

# --- Wait for all processes to finish (blocking) ---
wait $CAM174_PID $BOSON_PID $ADSB_PID $IMU_PID $GPS_PID

