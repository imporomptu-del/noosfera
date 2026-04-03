#!/usr/bin/env python3

import os
import sys
import time
import signal
import numpy as np
import cv2
from datetime import datetime, timezone
from seaqr_controller.camera_reader.camera import Camera
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import subprocess

# Make sure the directory exists
out_dir = "/media/a/E/MyProjects/Water/ros2_ws"
os.makedirs(out_dir, exist_ok=True)

# --- 0) Path to SDK (change as needed) ---
ASI_LIB = '/usr/local/lib/libASICamera2.so'
if not os.path.exists(ASI_LIB):
    print(f"⚠️ ASI SDK library not found at {ASI_LIB}")
    print("Trying alternative paths...")
    # Try alternative paths
    alt_paths = [
        '/usr/lib/x86_64-linux-gnu/libASICamera2.so',
        '/usr/local/lib/libASICamera2.so',
        '/opt/asi/lib/libASICamera2.so'
    ]
    for alt_path in alt_paths:
        if os.path.exists(alt_path):
            ASI_LIB = alt_path
            print(f"Found ASI SDK at: {ASI_LIB}")
            break
    else:
        print("ASI SDK not found in any location. Please check installation.")
        sys.exit(1)

os.environ['ZWO_ASI_LIB_PATH'] = ASI_LIB

# --- 1) Initialize ASI ---
import zwoasi as asi
asi.init(ASI_LIB)
cams = asi.list_cameras()
assert cams, "No ASI cameras found"


class OpenCVRecorder(Node):
    def __init__(self):
        super().__init__('opencv_recorder')

        # 1) Camera wrapper (yours)
        self.cam = Camera(0, 'ZWO ASI676MC')
        self.width = None
        self.height = None
        self.is_color = None
        self.bayer_pattern = 'rggb'  # default

        # 2) FFmpeg recording state (for H.264)
        self.ffmpeg_process = None
        self.segment_duration = 60  # 1 minute in seconds
        self.segment_start_time = time.time()
        self.output_dir = "recordings"
        os.makedirs(self.output_dir, exist_ok=True)
        self.use_ffmpeg = True  # Use FFmpeg for H.264 instead of OpenCV
        
        # 3) Frame monitoring
        self.frame_counters = {
            'total_frames': 0,
            'dropped_frames': 0,
            'successful_frames': 0,
            'current_file_frames': 0,
            'current_file_dropped': 0
        }
        self.current_file_name = None
        
        # 4) FPS monitoring
        self.fps_data = {
            'last_frame_time': time.time(),
            'frame_times': [],
            'current_fps': 0.0,
            'average_fps': 0.0,
            'fps_window_size': 120  # Calculate FPS over last 30 frames
        }

        # 5) ROS2 publishers
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, 'camera/camera_info', 10)
        self.bridge = CvBridge()

        # 6) Initialize camera properties
        self.init_camera_properties()     # fills width/height/is_color/bayer

        # 7) Start camera stream
        self.cam.start_stream()

        # 8) Start recording timer
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.start_stats_timer()          # periodic frame statistics

    def init_camera_properties(self):
        info = self.cam.asi_camera.get_camera_property()
        self.width = info['MaxWidth']
        self.height = info['MaxHeight']
        self.is_color = (info['IsColorCam'] == 1)

        # Map SDK BayerPattern -> OpenCV pattern
        pattern_map = {0: 'rggb', 1: 'bggr', 2: 'grbg', 3: 'gbrg'}
        self.bayer_pattern = pattern_map.get(info.get('BayerPattern', 0), 'rggb')

        # Use maximum resolution - ASI676MC can do 31.2 FPS at 3552x3552
        self.cam.asi_camera.set_roi(0, 0, self.width, self.height, 1, image_type=asi.ASI_IMG_RAW8)
        
        # Debug: Check actual camera settings
        current_exposure = self.cam.asi_camera.get_control_value(asi.ASI_EXPOSURE)
        current_gain = self.cam.asi_camera.get_control_value(asi.ASI_GAIN)
        current_bandwidth = self.cam.asi_camera.get_control_value(asi.ASI_BANDWIDTHOVERLOAD)
        self.get_logger().info(f"Camera: {self.width}x{self.height}, Color={self.is_color}, Bayer={self.bayer_pattern}")
        self.get_logger().info(f"Actual Exposure: {current_exposure}μs, Gain: {current_gain}, Bandwidth: {current_bandwidth}%")
        self.get_logger().info(f"Config Exposure: {self.cam.EXPOSURE}μs, Gain: {self.cam.GAIN}, Bandwidth: {self.cam.BANDWIDTH}%")

    def start_new_segment(self, frame):
        """Start a new video segment using FFmpeg H.264"""
        # Stop current FFmpeg process
        if self.ffmpeg_process:
            self.ffmpeg_process.stdin.close()
            self.ffmpeg_process.wait()
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(self.output_dir, f"video_{timestamp}.mp4")
        height, width = frame.shape[:2]
        
        # Create FFmpeg command for H.264 encoding
        cmd = [
            'ffmpeg',
            '-y',  # Overwrite output file
            '-f', 'rawvideo',
            '-vcodec', 'rawvideo',
            '-s', f'{width}x{height}',
            '-pix_fmt', 'bgr24',
            '-r', '12',  # Target 30 FPS
            '-i', '-',  # Read from stdin
            '-c:v', 'h264_nvenc',  # Use NVIDIA hardware H.264 encoder
            '-preset', 'fast',  # Fast encoding
            '-crf', '23',  # Good quality (18-28 range)
            '-pix_fmt', 'yuv420p',  # Compatible pixel format
            '-movflags', '+faststart',  # Web optimization
            filename
        ]
        
        try:
            self.ffmpeg_process = subprocess.Popen(
                cmd, 
                stdin=subprocess.PIPE, 
                stdout=subprocess.DEVNULL, 
                stderr=subprocess.PIPE  # Capture stderr to see any errors
            )
            
            self.segment_start_time = time.time()
            self.current_file_name = filename
            
            # Reset counters for new file
            self.frame_counters['current_file_frames'] = 0
            self.frame_counters['current_file_dropped'] = 0
            
            self.get_logger().info(f"Started new H.264 segment: {filename}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to start FFmpeg: {e}")
            self.ffmpeg_process = None

    def _log_file_completion(self):
        """Log frame statistics when a file is completed"""
        if self.current_file_name:
            total_frames = self.frame_counters['current_file_frames']
            dropped_frames = self.frame_counters['current_file_dropped']
            successful_frames = total_frames - dropped_frames
            drop_rate = (dropped_frames / total_frames * 100) if total_frames > 0 else 0
            
            self.get_logger().info(f"File completed: {self.current_file_name}")
            self.get_logger().info(f"  Total frames: {total_frames}")
            self.get_logger().info(f"  Successful: {successful_frames}")
            self.get_logger().info(f"  Dropped: {dropped_frames} ({drop_rate:.1f}%)")

    def _demosaic_to_bgr(self, raw):
        """Convert raw Bayer pattern to BGR"""
        # For RAW8, no bit shifting needed
        raw8 = raw.astype(np.uint8)
        
        # Bayer pattern conversion
        code = {
            "RGGB": cv2.COLOR_BAYER_RG2BGR,
            "BGGR": cv2.COLOR_BAYER_BG2BGR,
            "GRBG": cv2.COLOR_BAYER_GR2BGR,
            "GBRG": cv2.COLOR_BAYER_GB2BGR,
        }.get(self.bayer_pattern.upper(), cv2.COLOR_BAYER_RG2BGR)
        
        return cv2.cvtColor(raw8, code)

    def _capture_frame(self):
        """Capture frame from ASI camera"""
        try:
            data = self.cam.grab_frame(timeout=5000)  # 5 seconds - should be enough for 30 FPS
            if data is None:
                self.get_logger().warn("Camera returned None data")
                return None
            return np.frombuffer(data, dtype=np.uint8).reshape(self.height, self.width)
        except Exception as e:
            self.get_logger().warn(f"Capture timeout: {e}")
            return None

    def timer_callback(self):
        """Main recording loop"""
        # Capture raw frame from ASI camera
        raw = self._capture_frame()
        if raw is None:
            self.frame_counters['dropped_frames'] += 1
            self.frame_counters['current_file_dropped'] += 1
            return

        # Process raw frame to BGR
        if self.is_color:
            frame = self._demosaic_to_bgr(raw)
        else:
            # Convert mono to BGR for recording
            frame = cv2.cvtColor(raw, cv2.COLOR_GRAY2BGR)

        # Start new segment if needed
        if self.ffmpeg_process is None:
            self.start_new_segment(frame)

        # Check if we need to start a new segment
        if time.time() - self.segment_start_time >= self.segment_duration:
            self._log_file_completion()  # Log completion of previous file
            self.start_new_segment(frame)

        # Write frame to FFmpeg H.264 encoder
        if self.ffmpeg_process and self.ffmpeg_process.stdin:
            try:
                # Check if FFmpeg process is still running
                if self.ffmpeg_process.poll() is not None:
                    # Process has terminated, check for errors
                    stderr_output = self.ffmpeg_process.stderr.read().decode('utf-8')
                    if stderr_output:
                        self.get_logger().error(f"FFmpeg process terminated with errors: {stderr_output}")
                    else:
                        self.get_logger().warn("FFmpeg process terminated unexpectedly")
                    # Restart the segment
                    self.start_new_segment(frame)
                    return
                
                # Convert BGR to bytes and write to FFmpeg
                frame_bytes = frame.tobytes()
                self.ffmpeg_process.stdin.write(frame_bytes)
                self.ffmpeg_process.stdin.flush()
                
                # Update frame counters only on success
                self.frame_counters['total_frames'] += 1
                self.frame_counters['successful_frames'] += 1
                self.frame_counters['current_file_frames'] += 1
                
            except Exception as e:
                self.get_logger().warn(f"Failed to write frame to FFmpeg: {e}")
                self.frame_counters['total_frames'] += 1
                self.frame_counters['dropped_frames'] += 1
                self.frame_counters['current_file_dropped'] += 1
                return

        # Publish to ROS2
        stamp = self.get_clock().now().to_msg()
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = 'camera_frame'
        self.image_pub.publish(img_msg)

        # Update FPS monitoring
        self._update_fps()

    def _update_fps(self):
        """Update FPS monitoring"""
        current_time = time.time()
        time_diff = current_time - self.fps_data['last_frame_time']
        
        if time_diff > 0:
            current_fps = 1.0 / time_diff
            self.fps_data['frame_times'].append(current_fps)
            
            # Keep only last N frames for average
            if len(self.fps_data['frame_times']) > self.fps_data['fps_window_size']:
                self.fps_data['frame_times'].pop(0)
            
            self.fps_data['current_fps'] = current_fps
            self.fps_data['average_fps'] = sum(self.fps_data['frame_times']) / len(self.fps_data['frame_times'])
        
        self.fps_data['last_frame_time'] = current_time

    def start_stats_timer(self):
        """Start periodic statistics logging"""
        self.stats_timer = self.create_timer(10.0, self._log_stats)

    def _log_stats(self):
        """Log periodic statistics"""
        total_frames = self.frame_counters['total_frames']
        dropped_frames = self.frame_counters['dropped_frames']
        successful_frames = self.frame_counters['successful_frames']
        drop_rate = (dropped_frames / total_frames * 100) if total_frames > 0 else 0
        
        self.get_logger().info(f"Stats - Total: {total_frames}, Successful: {successful_frames}, "
                              f"Dropped: {dropped_frames} ({drop_rate:.1f}%), "
                              f"FPS: {self.fps_data['average_fps']:.1f}")

    def destroy_node(self):
        """Clean up resources"""
        try:
            # Close FFmpeg process
            if self.ffmpeg_process:
                self.ffmpeg_process.stdin.close()
                self.ffmpeg_process.wait()
        except Exception:
            pass
        try:
            self.cam.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    
    # Graceful Ctrl+C
    def _sigint_handler(signum, frame):
        rclpy.shutdown()
    signal.signal(signal.SIGINT, _sigint_handler)

    node = None
    try:
        node = OpenCVRecorder()
        rclpy.spin(node)
    except Exception as e:
        if node:
            node.get_logger().error(f"Fatal error: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()