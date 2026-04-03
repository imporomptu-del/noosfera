#!/usr/bin/env python3

import os
import sys
import time
import signal
import threading
import cv2
import numpy as np
from datetime import datetime, timezone
from seaqr_controller.camera_reader.camera import Camera
import queue

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

class CameraRecorderMinimal:
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
            'frame_times': [],
            'current_fps': 0.0,
            'average_fps': 0.0,
            'fps_window_size': 30
        }
        
        self.frame_count = 0
        self.total_frames = 0
        self.dropped_frames = 0
        
        # Asynchronous encoding
        self.frame_queue = queue.Queue(maxsize=1000)  # Buffer for frames
        self.encoding_thread = None
        self.stop_encoding = False

        # Initialize camera
        self.init_camera_properties()
        self.start_camera()
        self.start_recording()

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

    def start_recording(self):
        """Start the recording process with asynchronous encoding"""
        self.running = True
        self.start_time = time.time()
        print("Starting high-performance camera recording...")
        print(f"Chunk duration: {self.chunk_duration} seconds")
        
        # Start encoding thread
        self.stop_encoding = False
        self.encoding_thread = threading.Thread(target=self._encoding_worker, daemon=True)
        self.encoding_thread.start()
        
        # Start recording thread
        self.recording_thread = threading.Thread(target=self._recording_loop)
        self.recording_thread.daemon = True
        self.recording_thread.start()
        
        # Start stats thread
        self.stats_thread = threading.Thread(target=self._stats_loop)
        self.stats_thread.daemon = True
        self.stats_thread.start()

    def _recording_loop(self):
        """Main recording loop - captures frames and queues them for encoding"""
        chunk_start_time = time.time()
        
        while self.running:
            try:
                # Capture frame
                buf = self.cam.asi_camera.get_video_data()
                if buf is not None:
                    # Convert to numpy array
                    frame = np.frombuffer(buf, dtype=np.uint8)
                    frame = frame.reshape((self.height, self.width))
                    
                    # Add to queue for encoding (non-blocking)
                    try:
                        self.frame_queue.put_nowait((frame.copy(), time.time()))
                        self.frame_count += 1
                        self.total_frames += 1
                    except queue.Full:
                        # Queue is full, drop frame
                        self.dropped_frames += 1
                        print(f"Warning: Frame queue full, dropping frame")
                    
                    # Calculate FPS
                    self._calculate_fps()
                    
                    # Check if it's time to start new chunk
                    current_time = time.time()
                    if current_time - chunk_start_time >= self.chunk_duration:
                        self._start_new_chunk()
                        chunk_start_time = current_time
                        self.current_chunk += 1
                        
            except Exception as e:
                print(f"Recording error: {e}")
                self.dropped_frames += 1
                break

    def _encoding_worker(self):
        """Background thread for H.264 encoding"""
        frames_buffer = []
        current_chunk = 0
        chunk_start_time = time.time()
        
        while not self.stop_encoding:
            try:
                # Get frame from queue (with timeout)
                frame, timestamp = self.frame_queue.get(timeout=1.0)
                frames_buffer.append((frame, timestamp))
                
                # Check if we should save this chunk
                if time.time() - chunk_start_time >= self.chunk_duration:
                    self._save_chunk_async(frames_buffer, chunk_start_time, current_chunk)
                    frames_buffer = []
                    chunk_start_time = time.time()
                    current_chunk += 1
                    
            except queue.Empty:
                # No frames available, continue
                continue
            except Exception as e:
                print(f"Error in encoding worker: {e}")
                break

    def _save_chunk_async(self, frames_buffer, chunk_start_time, chunk_number):
        """Save a chunk of frames as H.264 video (runs in background)"""
        if not frames_buffer:
            return
            
        # Generate filename
        ts = datetime.fromtimestamp(chunk_start_time, tz=timezone.utc).strftime("%Y_%m_%d_T%H_%M_%S")
        filename = f"ZWO_ASI174MM_{ts}_{chunk_number:05d}.mp4"
        filepath = os.path.join(out_dir, filename)
        
        print(f"Encoding chunk {chunk_number}: {filename} ({len(frames_buffer)} frames)")
        
        # Use OpenCV VideoWriter for H.264 encoding
        fourcc = cv2.VideoWriter_fourcc(*'H264')
        out = cv2.VideoWriter(filepath, fourcc, 60.0, (self.width, self.height), isColor=False)
        
        if not out.isOpened():
            print(f"❌ Error: Could not open video writer for {filename}")
            return
        
        # Write all frames
        for frame, timestamp in frames_buffer:
            out.write(frame)
        
        out.release()
        print(f"✅ Chunk saved: {filename}")

    def _start_new_chunk(self):
        """Signal that a new chunk should start"""
        print(f"Starting chunk {self.current_chunk + 1}")

    def _calculate_fps(self):
        """Calculate current and average FPS"""
        current_time = time.time()
        time_diff = current_time - self.fps_data['last_frame_time']
        
        if time_diff > 0:
            current_fps = 1.0 / time_diff
            self.fps_data['current_fps'] = current_fps
            
            # Update frame times for average calculation
            self.fps_data['frame_times'].append(current_time)
            if len(self.fps_data['frame_times']) > self.fps_data['fps_window_size']:
                self.fps_data['frame_times'].pop(0)
            
            # Calculate average FPS
            if len(self.fps_data['frame_times']) > 1:
                time_span = self.fps_data['frame_times'][-1] - self.fps_data['frame_times'][0]
                frame_count = len(self.fps_data['frame_times']) - 1
                if time_span > 0:
                    self.fps_data['average_fps'] = frame_count / time_span
        
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
                print(f"Current chunk: {self.current_chunk}, Queue size: {self.frame_queue.qsize()}")

    def stop_recording(self):
        """Stop the recording"""
        print("Stopping recording...")
        self.running = False
        
        # Stop encoding thread
        self.stop_encoding = True
        if self.encoding_thread and self.encoding_thread.is_alive():
            self.encoding_thread.join(timeout=5)
        
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
    recorder = CameraRecorderMinimal()
    
    try:
        # Keep running
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        recorder.stop_recording()

if __name__ == '__main__':
    main()