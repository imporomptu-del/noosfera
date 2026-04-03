#!/usr/bin/env python3

import subprocess
import sys
import time
import signal
import os
import threading
import select

def signal_handler(sig, frame):
    print('\nShutting down...')
    sys.exit(0)

def stream_output(process, name):
    """Stream output from a process"""
    while True:
        output = process.stdout.readline()
        if output == '' and process.poll() is not None:
            break
        if output:
            print(f"[{name}] {output.strip()}")

def main():
    # Set up signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    
    # Change to workspace directory
    os.chdir('/media/a/E/MyProjects/Water/ros2_ws')
    
    print("Starting ADSB and Camera nodes...")
    print("=" * 50)
    
    # Start ADSB node
    adsb_process = subprocess.Popen([
        'bash', '-c', 'source install/setup.bash && ros2 run seaqr_controller adsb_data'
    ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True, bufsize=1)
    
    # Start Camera node
    camera_process = subprocess.Popen([
        'bash', '-c', 'source install/setup.bash && ros2 run seaqr_controller camera_gstreamer_with_ros2'
    ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True, bufsize=1)
    
    # Create threads to stream output
    adsb_thread = threading.Thread(target=stream_output, args=(adsb_process, "ADSB"))
    camera_thread = threading.Thread(target=stream_output, args=(camera_process, "CAMERA"))
    
    adsb_thread.daemon = True
    camera_thread.daemon = True
    
    adsb_thread.start()
    camera_thread.start()
    
    print("Both nodes started! Press Ctrl+C to stop...")
    print("=" * 50)
    
    try:
        # Wait for both processes
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping nodes...")
        adsb_process.terminate()
        camera_process.terminate()
        adsb_process.wait()
        camera_process.wait()
        print("Nodes stopped.")

if __name__ == '__main__':
    main()
