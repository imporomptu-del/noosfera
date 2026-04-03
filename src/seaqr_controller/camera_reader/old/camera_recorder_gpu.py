#!/usr/bin/env python3

import os
import sys
import time
import signal
import threading
import numpy as np
from datetime import datetime, timezone
from seaqr_controller.camera_reader.camera import Camera
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

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

class CameraRecorderGPU:
    def __init__(self):
        # Camera setup
        self.cam = Camera(0, 'ZWO ASI174MM')
        self.width = None
        self.height = None
        self.running = False
        
        # Recording settings
        self.chunk_duration = 60  # seconds per chunk
        self.current_chunk = 0
        self.start_time = None
        
        # FPS monitoring
        self.fps_data = {
            'last_frame_time': time.time(),
            'current_fps': 0.0,
            'average_fps': 0.0,
            'fps_window_start': time.time(),
            'fps_window_frames': 0
        }
        
        self.frame_count = 0
        self.total_frames = 0
        self.dropped_frames = 0
        
        # GStreamer pipeline
        self.pipeline = None
        self.appsrc = None
        self.loop = None
        self.pipeline_thread = None

        # Initialize camera and GStreamer
        self.init_camera_properties()
        self.init_gstreamer()
        self.start_camera()
        self.start_recording()

    def init_camera_properties(self):
        info = self.cam.asi_camera.get_camera_property()
        self.width = info['MaxWidth']
        self.height = info['MaxHeight']
        
        # Use maximum resolution
        self.cam.asi_camera.set_roi(0, 0, self.width, self.height, 1, image_type=asi.ASI_IMG_RAW8)
        print(f"Camera: {self.width}x{self.height}")

    def init_gstreamer(self):
        """Initialize GStreamer with GPU-accelerated H.264 encoding"""
        Gst.init(None)
        print("GStreamer initialized")
        
        # Create pipeline with NVIDIA hardware encoding
        pipeline_str = (
            f"appsrc name=source "
            f"! video/x-raw,format=GRAY8,width={self.width},height={self.height},framerate=60/1 "
            f"! videoconvert "
            f"! video/x-raw,format=I420 "
            f"! nvvidconv "
            f"! video/x-raw(memory:NVMM),format=NV12 "
            f"! nvv4l2h264enc "
            f"    bitrate=8000000 "
            f"    control-rate=1 "
            f"    preset-level=1 "
            f"    iframeinterval=30 "
            f"    maxperf-enable=true "
            f"    ratecontrol-enable=true "
            f"! h264parse "
            f"! splitmuxsink "
            f"    location={out_dir}/ZWO_ASI174MM_%Y_%m_%d_T%H_%M_%S_%05d.mp4 "
            f"    max-size-time={self.chunk_duration * 1000000000} "
            f"    muxer=mp4mux"
        )
        
        print(f"Pipeline: {pipeline_str}")
        self.pipeline = Gst.parse_launch(pipeline_str)
        
        if not self.pipeline:
            raise RuntimeError("Failed to create GStreamer pipeline")
        
        # Get appsrc element
        self.appsrc = self.pipeline.get_by_name("source")
        if not self.appsrc:
            raise RuntimeError("Failed to get appsrc element")
        
        # Configure appsrc
        self.appsrc.set_property("is-live", True)
        self.appsrc.set_property("do-timestamp", True)
        self.appsrc.set_property("format", Gst.Format.TIME)
        
        print("GStreamer pipeline created with GPU encoding")

    def start_camera(self):
        """Start camera streaming"""
        self.cam.start_stream()
        print("Camera streaming started")

    def start_recording(self):
        """Start the recording process with GPU encoding"""
        self.running = True
        self.start_time = time.time()
        print("Starting high-performance camera recording with GPU encoding...")
        print(f"Chunk duration: {self.chunk_duration} seconds")
        
        # Start GStreamer pipeline
        self.pipeline.set_state(Gst.State.PLAYING)
        print("GStreamer pipeline PLAYING")
        
        # Start recording thread
        self.recording_thread = threading.Thread(target=self._recording_loop)
        self.recording_thread.daemon = True
        self.recording_thread.start()
        
        # Start stats thread
        self.stats_thread = threading.Thread(target=self._stats_loop)
        self.stats_thread.daemon = True
        self.stats_thread.start()

    def _recording_loop(self):
        """Main recording loop - captures frames and feeds to GStreamer"""
        while self.running:
            try:
                # Capture frame
                buf = self.cam.asi_camera.get_video_data()
                if buf is not None:
                    # Convert to numpy array
                    frame = np.frombuffer(buf, dtype=np.uint8)
                    frame = frame.reshape((self.height, self.width))
                    
                    # Create GStreamer buffer
                    gst_buffer = Gst.Buffer.new_allocate(None, frame.nbytes, None)
                    gst_buffer.fill(0, frame.tobytes())
                    
                    # Set timestamp
                    gst_buffer.pts = Gst.CLOCK_TIME_NONE
                    gst_buffer.dts = Gst.CLOCK_TIME_NONE
                    gst_buffer.duration = Gst.CLOCK_TIME_NONE
                    
                    # Push to pipeline
                    ret = self.appsrc.emit("push-buffer", gst_buffer)
                    if ret != Gst.FlowReturn.OK:
                        print(f"Warning: Failed to push buffer: {ret}")
                        self.dropped_frames += 1
                    else:
                        self.frame_count += 1
                        self.total_frames += 1
                    
                    # Calculate FPS
                    self._calculate_fps()
                        
            except Exception as e:
                print(f"Recording error: {e}")
                self.dropped_frames += 1
                break

    def _calculate_fps(self):
        """Calculate current and average FPS"""
        current_time = time.time()
        time_diff = current_time - self.fps_data['last_frame_time']
        
        # Calculate current FPS (instantaneous)
        if time_diff > 0:
            self.fps_data['current_fps'] = 1.0 / time_diff
        else:
            self.fps_data['current_fps'] = 0
        
        # Calculate average FPS over last 10 seconds
        self.fps_data['fps_window_frames'] += 1
        window_time = current_time - self.fps_data['fps_window_start']
        
        if window_time > 10.0:  # Reset window every 10 seconds
            self.fps_data['fps_window_start'] = current_time
            self.fps_data['fps_window_frames'] = 1
            window_time = 0
        
        if window_time > 0:
            self.fps_data['average_fps'] = self.fps_data['fps_window_frames'] / window_time
        else:
            self.fps_data['average_fps'] = self.fps_data['current_fps']
        
        self.fps_data['last_frame_time'] = current_time

    def _stats_loop(self):
        """Periodic stats reporting"""
        while self.running:
            time.sleep(10)  # Report every 10 seconds
            if self.running:
                total = self.total_frames
                dropped = self.dropped_frames
                drop_rate = (dropped / total * 100) if total > 0 else 0
                
                print(f"Frame stats - Total: {total}, Dropped: {dropped}, Drop rate: {drop_rate:.2f}%")
                print(f"FPS - Current: {self.fps_data['current_fps']:.1f}, Average: {self.fps_data['average_fps']:.1f}")
                print(f"Current chunk: {self.current_chunk}")

    def stop_recording(self):
        """Stop the recording"""
        print("Stopping recording...")
        self.running = False
        
        # Stop GStreamer pipeline
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
            print("GStreamer pipeline stopped")
        
        if hasattr(self, 'recording_thread') and self.recording_thread.is_alive():
            self.recording_thread.join(timeout=2)
        
        if hasattr(self, 'stats_thread') and self.stats_thread.is_alive():
            self.stats_thread.join(timeout=2)
        
        try:
            self.cam.stop_stream()
        except Exception:
            pass
        
        # Final stats
        elapsed = time.time() - self.start_time if self.start_time else 0
        avg_fps = self.total_frames / elapsed if elapsed > 0 else 0
        
        print(f"\n=== FINAL RECORDING STATISTICS ===")
        print(f"Total frames recorded: {self.total_frames}")
        print(f"Dropped frames: {self.dropped_frames}")
        print(f"Drop rate: {(self.dropped_frames / self.total_frames * 100) if self.total_frames > 0 else 0:.2f}%")
        print(f"Average FPS: {avg_fps:.2f}")
        print(f"Total chunks created: {self.current_chunk}")
        print(f"Recording duration: {elapsed:.2f} seconds")
        print(f"================================")

def signal_handler(signum, frame):
    print(f"\nReceived signal {signum}, shutting down...")
    if 'recorder' in globals():
        recorder.stop_recording()
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    global recorder
    recorder = CameraRecorderGPU()
    
    try:
        # Keep running
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        recorder.stop_recording()

if __name__ == '__main__':
    main()
