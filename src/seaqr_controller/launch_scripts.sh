#!/bin/bash
# ================================================
# SEAQR Data Collection System Launch Script
# Launches both ADS-B and Camera recording simultaneously
# ================================================

set -euo pipefail

# Trap ensures that if you press Ctrl+C or systemd stops the service,
# both background processes are terminated cleanly.
trap "echo '🛑 Stopping SEAQR Data Collection...'; kill 0; exit 0" SIGINT SIGTERM

echo "🚀 SEAQR Data Collection System"
echo "================================"

# Navigate to workspace
WORKDIR="/home/a/Projects/ros2_ws/src/seaqr_controller/seaqr_controller"
if [ ! -d "$WORKDIR" ]; then
    echo "❌ Directory not found: $WORKDIR"
    exit 1
fi
cd "$WORKDIR"

# Activate Python virtual environment
echo "📦 Activating ROS2 environment..."
source /home/a/seaqr-horizon/bin/activate

# Optional: also source ROS2 setup if your nodes depend on it
# source /opt/ros/humble/setup.bash

echo ""
echo "🚀 Launching SEAQR Data Collection System..."
echo "---------------------------------------------"
echo "📡 ADS-B Reader:    adsb_reader/adsb_usb.py"
echo "📹 Camera Reader:   camera_reader/cam_174_reader.py"
echo "💾 Data Storage:    /home/a/Projects/ros2_ws/data/"
echo ""
echo "Press Ctrl+C to stop all services"
echo "================================"
echo ""

# Wait a few seconds to allow USB devices (camera, RTL-SDR) to initialize
sleep 8

# Launch both scripts in background
python3 camera_reader/cam_174_reader.py &
CAM_PID=$!

python3 adsb_reader/adsb_usb.py &
ADSB_PID=$!

# Wait for both processes to finish
wait $CAM_PID $ADSB_PID

