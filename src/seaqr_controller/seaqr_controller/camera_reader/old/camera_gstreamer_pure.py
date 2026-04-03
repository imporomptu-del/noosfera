#!/usr/bin/env python3

import os
import sys
import time
import signal
import numpy as np
import gi
from datetime import datetime, timezone
import os
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
from seaqr_controller.camera_reader.camera import Camera
import threading

# Make sure the directory exists
out_dir = "/home/a/Projects/ros2_ws/data/camera"
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

class GstreamerPure:
    def __init__(self):
        # 1) Camera wrapper
        self.cam = Camera(0, 'ZWO ASI174MM')
        self.width = None
        self.height = None
        self.is_color = None
        self.bayer_pattern = 'rggb'

        # 2) GStreamer state
        self.pipeline = None
        self.appsrc = None
        self.encoder = None
        self.bus = None
        self.loop = None
        self.mainloop_thread = None
        self.producer_thread = None
        self.running = False
        
        # 3) Frame monitoring
        self.frame_counters = {
            'total_frames': 0,
            'current_file_frames': 0,
            'dropped_frames': 0
        }
        
        # 4) FPS monitoring
        self.fps_data = {
            'last_frame_time': time.time(),
            'frame_times': [],
            'current_fps': 0.0,
            'average_fps': 0.0,
            'fps_window_size': 30
        }

        # Initialize everything
        self.init_camera_properties()
        self.init_gstreamer()
        self.create_pipeline()
        self.start_pipeline()
        self.start_producer_thread()
        self.start_stats_timer()

    def init_camera_properties(self):
        info = self.cam.asi_camera.get_camera_property()
        self.width = info['MaxWidth']
        self.height = info['MaxHeight']
        self.is_color = (info['IsColorCam'] == 1)
        
        # Get camera name for filename
        self.camera_name = info.get('Name', 'ASI_Camera').replace(' ', '_').replace('-', '_')

        # Map SDK BayerPattern -> GStreamer caps string
        pattern_map = {0: 'rggb', 1: 'bggr', 2: 'grbg', 3: 'gbrg'}
        self.bayer_pattern = pattern_map.get(info.get('BayerPattern', 0), 'rggb')

        # Use maximum resolution
        self.cam.asi_camera.set_roi(0, 0, self.width, self.height, 1, image_type=asi.ASI_IMG_RAW8)
        print(f"Camera: {self.width}x{self.height}, Color={self.is_color}, Bayer={self.bayer_pattern}")
        print(f"Camera Name: {self.camera_name}")

    def init_gstreamer(self):
        """Initialize GStreamer"""
        Gst.init(None)
        print("GStreamer initialized")

    def create_pipeline(self):
        self.pipeline = Gst.Pipeline()

        # Elements
        self.appsrc = Gst.ElementFactory.make("appsrc", "source")
        bayer2rgb = Gst.ElementFactory.make("bayer2rgb", "bayer")
        convert = Gst.ElementFactory.make("videoconvert", "convert")
        to_i420 = Gst.ElementFactory.make("videoconvert", "to_i420")
        nvvidconv = Gst.ElementFactory.make("nvvidconv", "nvvidconv")
        self.encoder = Gst.ElementFactory.make("nvv4l2h264enc", "encoder")
        h264parse = Gst.ElementFactory.make("h264parse", "parse")
        splitmux = Gst.ElementFactory.make("splitmuxsink", "smux")

        # Validate
        for e in [self.appsrc, to_i420, nvvidconv, self.encoder, h264parse, splitmux]:
            if not e:
                raise RuntimeError("Failed to create a GStreamer element")

        # Configure appsrc
        self.appsrc.set_property("is-live", True)
        self.appsrc.set_property("do-timestamp", True)
        self.appsrc.set_property("format", Gst.Format.TIME)
        caps_bayer = Gst.Caps.from_string(f"video/x-bayer,format={self.bayer_pattern},width={self.width},height={self.height},framerate=60/1")
        self.appsrc.set_property("caps", caps_bayer)

        # Encoder settings
        self.encoder.set_property("bitrate", 2000000)
        self.encoder.set_property("control-rate", "constant_bitrate")
        self.encoder.set_property("preset-level", "UltraFastPreset")
        self.encoder.set_property("iframeinterval", 30)
        self.encoder.set_property("maxperf-enable", True)
        self.encoder.set_property("ratecontrol-enable", True)

        # Parser
        h264parse.set_property("config-interval", 1)

        # splitmuxsink
        splitmux.set_property("muxer-factory", "mpegtsmux")
        splitmux.set_property("max-size-time", 60 * 1000 * 1000 * 1000)  # 60s in ns
        splitmux.set_property("send-keyframe-requests", True)

        # Add to pipeline
        self.pipeline.add(self.appsrc)
        self.pipeline.add(bayer2rgb)
        self.pipeline.add(convert)
        self.pipeline.add(to_i420)
        self.pipeline.add(nvvidconv)
        self.pipeline.add(self.encoder)
        self.pipeline.add(h264parse)
        self.pipeline.add(splitmux)

        # Link elements
        if not self.appsrc.link(bayer2rgb): raise RuntimeError("link appsrc->bayer2rgb")
        if not bayer2rgb.link(convert): raise RuntimeError("link bayer2rgb->convert")
        if not convert.link(to_i420): raise RuntimeError("link convert->to_i420")
        if not to_i420.link(nvvidconv): raise RuntimeError("link to_i420->nvvidconv")
        if not nvvidconv.link(self.encoder): raise RuntimeError("link nvvidconv->encoder")
        if not self.encoder.link(h264parse): raise RuntimeError("link encoder->parse")
        if not h264parse.link(splitmux): raise RuntimeError("link parse->splitmux")

        # Set location
        splitmux.set_property("location", os.path.join(out_dir, "pure_%05d.ts"))

        print("Pipeline created (appsrc→bayer2rgb→convert→I420→nvvidconv→enc→parse→splitmuxsink)")

    def start_pipeline(self):
        """Start the GStreamer pipeline"""
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("Failed to start pipeline")
        print("GStreamer pipeline PLAYING")

    def start_producer_thread(self):
        """Start the frame producer thread"""
        self.cam.start_stream()
        self.running = True
        self.producer_thread = threading.Thread(target=self._producer_loop)
        self.producer_thread.daemon = True
        self.producer_thread.start()

    def _producer_loop(self):
        """Producer loop - feeds frames to GStreamer"""
        while self.running:
            try:
                # Grab RAW8 from ASI
                buf = self.cam.asi_camera.get_video_data()
                gst_buf = Gst.Buffer.new_wrapped(buf)
                ret = self.appsrc.emit("push-buffer", gst_buf)
                
                # Update counters
                self.frame_counters['total_frames'] += 1
                self.frame_counters['current_file_frames'] += 1
                
                # Calculate FPS
                if ret == Gst.FlowReturn.OK:
                    self._calculate_fps()
                else:
                    self.frame_counters['dropped_frames'] += 1
                    
            except Exception as e:
                print(f"Producer error: {e}")
                break

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

    def start_stats_timer(self):
        """Start periodic stats reporting"""
        def stats_timer():
            while self.running:
                time.sleep(10)  # Report every 10 seconds
                if self.running:
                    total = self.frame_counters['total_frames']
                    dropped = self.frame_counters['dropped_frames']
                    drop_rate = (dropped / total * 100) if total > 0 else 0
                    
                    print(f"Frame stats - Total: {total}, Dropped: {dropped}, Drop rate: {drop_rate:.2f}%")
                    print(f"FPS - Current: {self.fps_data['current_fps']:.1f}, Average: {self.fps_data['average_fps']:.1f}")
        
        stats_thread = threading.Thread(target=stats_timer)
        stats_thread.daemon = True
        stats_thread.start()

    def stop_pipeline(self):
        """Stop the pipeline and cleanup"""
        print("Shutting down...")
        self.running = False
        
        if self.producer_thread and self.producer_thread.is_alive():
            self.producer_thread.join(timeout=2)
        
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
        
        try:
            self.cam.stop_stream()
        except Exception:
            pass
        
        print("Pipeline stopped")

def signal_handler(signum, frame):
    print(f"\nReceived signal {signum}, shutting down...")
    if 'gstreamer' in globals():
        gstreamer.stop_pipeline()
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    global gstreamer
    gstreamer = GstreamerPure()
    
    try:
        # Keep running
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        gstreamer.stop_pipeline()

if __name__ == '__main__':
    main()
