#!/usr/bin/env python3

import os
import sys
import time
import signal
import numpy as np
from seaqr_controller.camera_reader.camera import Camera
import threading

# Make sure the directory exists
out_dir = "/home/a/Projects/ros2_ws/data/camera"
os.makedirs(out_dir, exist_ok=True)

# --- 0) Path to SDK ---
ASI_LIB = '/usr/local/lib/libASICamera2.so'
os.environ['ZWO_ASI_LIB_PATH'] = ASI_LIB

# --- 1) Initialize ASI ---
import zwoasi as asi
asi.init(ASI_LIB)
cams = asi.list_cameras()
assert cams, "No ASI cameras found"

class CameraTestMinimal:
    def __init__(self):
        # Camera setup
        self.cam = Camera(0, 'ZWO ASI174MM')
        self.width = None
        self.height = None
        self.running = False
        
        # FPS monitoring
        self.fps_data = {
            'last_frame_time': time.time(),
            'frame_times': [],
            'current_fps': 0.0,
            'average_fps': 0.0,
            'fps_window_size': 30
        }
        
        self.frame_count = 0
        self.start_time = time.time()

        # Initialize camera
        self.init_camera_properties()
        self.start_camera()
        self.start_test()

    def init_camera_properties(self):
        info = self.cam.asi_camera.get_camera_property()
        self.width = info['MaxWidth']
        self.height = info['MaxHeight']
        
        # Use maximum resolution
        self.cam.asi_camera.set_roi(0, 0, self.width, self.height, 1, image_type=asi.ASI_IMG_RAW8)
        print(f"Camera: {self.width}x{self.height}")

    def start_camera(self):
        """Start camera streaming"""
        self.cam.start_stream()
        print("Camera streaming started")

    def start_test(self):
        """Run the test"""
        self.running = True
        print("Starting minimal camera test...")
        print("Testing different scenarios to find bottleneck...")
        
        # Test 1: Just capture frames (no processing)
        print("\n=== Test 1: Raw frame capture only ===")
        self.test_raw_capture()
        
        # Test 2: Capture + save to disk (no Bayer conversion)
        print("\n=== Test 2: Raw capture + save to disk ===")
        self.test_disk_write()
        
        # Test 3: Capture + simple processing
        print("\n=== Test 3: Raw capture + simple processing ===")
        self.test_simple_processing()

    def test_raw_capture(self):
        """Test 1: Just capture frames without any processing"""
        start_time = time.time()
        frame_count = 0
        
        for i in range(100):  # Capture 100 frames
            try:
                buf = self.cam.asi_camera.get_video_data()
                if buf is not None:
                    frame_count += 1
            except Exception as e:
                print(f"Error: {e}")
                break
        
        elapsed = time.time() - start_time
        fps = frame_count / elapsed if elapsed > 0 else 0
        print(f"Raw capture FPS: {fps:.2f} ({frame_count} frames in {elapsed:.2f}s)")

    def test_simple_processing(self):
        """Test 3: Capture + simple processing (no Bayer conversion)"""
        start_time = time.time()
        frame_count = 0
        
        for i in range(100):  # Capture 100 frames
            try:
                buf = self.cam.asi_camera.get_video_data()
                if buf is not None:
                    # Convert to numpy array
                    frame = np.frombuffer(buf, dtype=np.uint8)
                    frame = frame.reshape((self.height, self.width))
                    
                    # Simple processing - just basic operations
                    # Simulate some light processing without Bayer conversion
                    processed_frame = frame.astype(np.float32)
                    processed_frame = processed_frame * 1.1  # Simple brightness adjustment
                    processed_frame = np.clip(processed_frame, 0, 255).astype(np.uint8)
                    
                    frame_count += 1
            except Exception as e:
                print(f"Error: {e}")
                break
        
        elapsed = time.time() - start_time
        fps = frame_count / elapsed if elapsed > 0 else 0
        print(f"Simple processing FPS: {fps:.2f} ({frame_count} frames in {elapsed:.2f}s)")

    def test_disk_write(self):
        """Test 3: Capture + save to disk"""
        start_time = time.time()
        frame_count = 0
        
        for i in range(50):  # Capture 50 frames (fewer due to disk I/O)
            try:
                buf = self.cam.asi_camera.get_video_data()
                if buf is not None:
                    # Save raw frame to disk
                    filename = os.path.join(out_dir, f"test_frame_{i:04d}.raw")
                    with open(filename, 'wb') as f:
                        f.write(buf)
                    frame_count += 1
            except Exception as e:
                print(f"Error: {e}")
                break
        
        elapsed = time.time() - start_time
        fps = frame_count / elapsed if elapsed > 0 else 0
        print(f"Disk write FPS: {fps:.2f} ({frame_count} frames in {elapsed:.2f}s)")

    def stop_test(self):
        """Stop the test"""
        self.running = False
        try:
            self.cam.stop_stream()
        except Exception:
            pass
        print("Test completed")

def signal_handler(signum, frame):
    print(f"\nReceived signal {signum}, shutting down...")
    if 'test' in globals():
        test.stop_test()
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    global test
    test = CameraTestMinimal()
    
    try:
        # Keep running for a bit to see results
        time.sleep(5)
    except KeyboardInterrupt:
        test.stop_test()

if __name__ == '__main__':
    main()
