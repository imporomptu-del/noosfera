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
import rclpy
from rclpy.node import Node
# Make sure the directory exists
out_dir = "/media/a/E/MyProjects/Water/ros2_ws"
os.makedirs(out_dir, exist_ok=True)
# --- 0) Path to SDK (change as needed) ---
ASI_LIB = '/home/a/ZWO/ASIStudio/lib/libASICamera2.so'
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


class GstreamerPureGstreamer(Node):
    def __init__(self):
        super().__init__('gstreamer_pure_gstreamer')

        # 1) Camera wrapper (yours)
        self.cam = Camera(0, 'ZWO ASI676MC')
        self.width = None
        self.height = None
        self.is_color = None
        self.bayer_pattern = 'rggb'  # default

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
            'fps_window_size': 30  # Calculate FPS over last 30 frames
        }

        # 3) Initialize camera props + GStreamer + pipeline
        self.init_camera_properties()     # fills width/height/is_color/bayer
        self.init_gstreamer()
        self.create_pipeline()            # builds appsrc→bayer2rgb→videoconvert→I420→enc→parse→splitmuxsink

        # 4) Start pipeline + GLib loop + producer
        self.start_pipeline()
        self.start_producer_thread()      # feeds appsrc from ASI SDK
        self.start_stats_timer()          # periodic frame statistics

    def init_camera_properties(self):
        info = self.cam.asi_camera.get_camera_property()
        self.width = info['MaxWidth']
        self.height = info['MaxHeight']
        self.is_color = (info['IsColorCam'] == 1)

        # Map SDK BayerPattern -> GStreamer caps string
        # Adjust if your SDK enum differs:
        pattern_map = {0: 'rggb', 1: 'bggr', 2: 'grbg', 3: 'gbrg'}
        self.bayer_pattern = pattern_map.get(info.get('BayerPattern', 0), 'rggb')

        # Ensure RAW8
        self.cam.asi_camera.set_roi(0, 0, self.width, self.height, 1, image_type=asi.ASI_IMG_RAW8)
        self.get_logger().info(f"Camera: {self.width}x{self.height}, Color={self.is_color}, Bayer={self.bayer_pattern}")

    def init_gstreamer(self):
        """Initialize GStreamer"""
        Gst.init(None)
        self.get_logger().info("GStreamer initialized")


    def _format_location(self, splitmux, fragment_id: int) -> str:
        # UTC wall-clock timestamp in the filename
        # Example: rec_20250904T21_15_30_00000.ts
        ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H_%M_%S")
        out_dir = "/media/a/E/MyProjects/Water/ros2_ws"
        filename = f"rec_{ts}_{fragment_id:05d}.ts"
        
        # Log completion of previous file if exists
        if self.current_file_name:
            self._log_file_completion()
        
        # Reset counters for new file
        self.current_file_name = filename
        self.frame_counters['current_file_frames'] = 0
        self.frame_counters['current_file_dropped'] = 0
        
        self.get_logger().info(f"Starting new file: {filename}")
        return os.path.join(out_dir, filename)
    
    def _log_file_completion(self):
        """Log frame statistics when a file is completed"""
        if self.current_file_name:
            total_frames = self.frame_counters['current_file_frames']
            dropped_frames = self.frame_counters['current_file_dropped']
            successful_frames = total_frames - dropped_frames
            drop_rate = (dropped_frames / total_frames * 100) if total_frames > 0 else 0
            
            self.get_logger().info(f"File completed: {self.current_file_name}")
            self.get_logger().info(f"  Total frames: {total_frames}")
            self.get_logger().info(f"  Successful frames: {successful_frames}")
            self.get_logger().info(f"  Dropped frames: {dropped_frames}")
            self.get_logger().info(f"  Drop rate: {drop_rate:.2f}%")
            self.get_logger().info(f"  Average FPS: {self.fps_data['average_fps']:.1f}")
            
            # Log overall statistics
            overall_total = self.frame_counters['total_frames']
            overall_dropped = self.frame_counters['dropped_frames']
            overall_drop_rate = (overall_dropped / overall_total * 100) if overall_total > 0 else 0
            self.get_logger().info(f"Overall stats - Total: {overall_total}, Dropped: {overall_dropped}, Drop rate: {overall_drop_rate:.2f}%")
    
    def _calculate_fps(self):
        """Calculate current and average FPS based on frame timing"""
        current_time = time.time()
        time_diff = current_time - self.fps_data['last_frame_time']
        
        if time_diff > 0:
            current_fps = 1.0 / time_diff
            self.fps_data['current_fps'] = current_fps
            
            # Add to rolling window
            self.fps_data['frame_times'].append(current_time)
            if len(self.fps_data['frame_times']) > self.fps_data['fps_window_size']:
                self.fps_data['frame_times'].pop(0)
            
            # Calculate average FPS over the window
            if len(self.fps_data['frame_times']) > 1:
                time_span = self.fps_data['frame_times'][-1] - self.fps_data['frame_times'][0]
                frame_count = len(self.fps_data['frame_times']) - 1
                if time_span > 0:
                    self.fps_data['average_fps'] = frame_count / time_span
        
        self.fps_data['last_frame_time'] = current_time
        return self.fps_data['current_fps'], self.fps_data['average_fps']
    
    def get_fps(self):
        """Get current and average FPS - can be called from outside"""
        return self.fps_data['current_fps'], self.fps_data['average_fps']
    
    def get_frame_stats(self):
        """Get comprehensive frame statistics - can be called from outside"""
        return {
            'total_frames': self.frame_counters['total_frames'],
            'dropped_frames': self.frame_counters['dropped_frames'],
            'successful_frames': self.frame_counters['successful_frames'],
            'current_fps': self.fps_data['current_fps'],
            'average_fps': self.fps_data['average_fps'],
            'drop_rate': (self.frame_counters['dropped_frames'] / self.frame_counters['total_frames'] * 100) if self.frame_counters['total_frames'] > 0 else 0
        }

    def create_pipeline(self):
        self.pipeline = Gst.Pipeline()

        # Elements
        self.appsrc = Gst.ElementFactory.make("appsrc", "source")
        bayer2rgb = Gst.ElementFactory.make("bayer2rgb", "bayer")
        convert = Gst.ElementFactory.make("videoconvert", "convert")
        to_i420 = Gst.ElementFactory.make("videoconvert", "to_i420")  # enforce I420 for encoder
        self.encoder = Gst.ElementFactory.make("nvh264enc", "encoder")  # RTX (adjust if Jetson)
        h264parse = Gst.ElementFactory.make("h264parse", "parse")
        splitmux = Gst.ElementFactory.make("splitmuxsink", "smux")

        # Validate
        # for e in [self.appsrc, bayer2rgb, convert, to_i420, self.encoder, h264parse, splitmux]:
        #     if not e:
        #         raise RuntimeError("Failed to create a GStreamer element")

        for e in [self.appsrc,  to_i420, self.encoder, h264parse, splitmux]:
            if not e:
                raise RuntimeError("Failed to create a GStreamer element")

        # Configure appsrc (live + timestamps + Bayer caps)
        self.appsrc.set_property("is-live", True)
        self.appsrc.set_property("do-timestamp", True)
        self.appsrc.set_property("format", Gst.Format.TIME)
        caps_bayer = Gst.Caps.from_string(f"video/x-bayer,format={self.bayer_pattern},width={self.width},height={self.height},framerate=10/1")
        #caps_bayer = Gst.Caps.from_string(f"video/x-bayer,format={self.bayer_pattern},width={self.width},height={self.height}")
        self.appsrc.set_property("caps", caps_bayer)

        # Encoder (low latency CBR + regular IDR)
        self.encoder.set_property("preset", "low-latency-hp")
        self.encoder.set_property("rc-mode", "cbr")
        self.encoder.set_property("bitrate", 12000)   # kbps
        self.encoder.set_property("gop-size", 30)     # ~1 sec @ 30fps

        # Parser: ensure config
        h264parse.set_property("config-interval", 1)

        # splitmuxsink: 5-minute rotation, TS container, keyframe-aligned
        splitmux.set_property("muxer-factory", "mpegtsmux")
        splitmux.set_property("max-size-time", 10 * 1000 * 1000 * 1000)  # 300s in ns
        splitmux.set_property("send-keyframe-requests", True)
        # Optional ring buffer: keep last N files
        # splitmux.set_property("max-files", 288)  # e.g., last 24h at 5-min segments

        # Add + link
        self.pipeline.add(self.appsrc)
        self.pipeline.add(bayer2rgb)
        self.pipeline.add(convert)
        self.pipeline.add(to_i420)
        self.pipeline.add(self.encoder)
        self.pipeline.add(h264parse)
        self.pipeline.add(splitmux)

        if not self.appsrc.link(bayer2rgb): raise RuntimeError("link appsrc->bayer2rgb")
        if not bayer2rgb.link(convert): raise RuntimeError("link bayer2rgb->convert")
        if not convert.link(to_i420): raise RuntimeError("link convert->to_i420")
        # enforce I420 before encoder
        caps_i420 = Gst.Caps.from_string("video/x-raw,format=I420")
        if not to_i420.link_filtered(self.encoder, caps_i420): raise RuntimeError("link to_i420->encoder (I420)")
        if not self.encoder.link(h264parse): raise RuntimeError("link encoder->parse")
        if not h264parse.link(splitmux): raise RuntimeError("link parse->splitmux")

        # Bus monitoring
        self.bus = self.pipeline.get_bus()
        self.bus.add_signal_watch()
        self.bus.connect("message", self.on_bus_message)

        # Set location for splitmuxsink after pipeline is created
        #splitmux.set_property("location", "/media/a/E/MyProjects/Water/ros2_ws/rec_%05d.ts")

        splitmux.set_property("location", os.path.join(out_dir, "rec_%05d.ts"))  # dummy; will be overridden
        splitmux.connect("format-location", self._format_location)

        self.get_logger().info("Pipeline created (appsrc→bayer2rgb→convert→I420→enc→parse→splitmuxsink)")    

    def start_pipeline(self):
        # GLib mainloop in a background thread
        self.loop = GLib.MainLoop()
        self.mainloop_thread = threading.Thread(target=self.loop.run, daemon=True)
        self.mainloop_thread.start()

        self.pipeline.set_state(Gst.State.PLAYING)
        self.running = True
        self.get_logger().info("GStreamer pipeline PLAYING")

    def stop_pipeline(self):
        self.running = False
        
        # Log final statistics for current file
        if self.current_file_name:
            self._log_file_completion()
        
        # Log final overall statistics
        total = self.frame_counters['total_frames']
        dropped = self.frame_counters['dropped_frames']
        successful = self.frame_counters['successful_frames']
        drop_rate = (dropped / total * 100) if total > 0 else 0
        
        self.get_logger().info("=== FINAL FRAME STATISTICS ===")
        self.get_logger().info(f"Total frames processed: {total}")
        self.get_logger().info(f"Successful frames: {successful}")
        self.get_logger().info(f"Dropped frames: {dropped}")
        self.get_logger().info(f"Overall drop rate: {drop_rate:.2f}%")
        self.get_logger().info(f"Average FPS: {self.fps_data['average_fps']:.1f}")
        self.get_logger().info("=============================")
        
        if self.pipeline:
            self.pipeline.send_event(Gst.Event.new_eos())
            self.pipeline.set_state(Gst.State.NULL)
        if self.loop:
            self.loop.quit()
        if self.mainloop_thread and self.mainloop_thread != threading.current_thread():
            self.mainloop_thread.join(timeout=2.0)
        self.get_logger().info("Pipeline stopped")

    def destroy_node(self):
        self.get_logger().info("Shutting down...")
        self.stop_pipeline()
        try:
            self.cam.stop_stream()
        except Exception:
            pass
        super().destroy_node()


    def start_producer_thread(self):
        # Make sure camera streaming is active
        self.cam.start_stream()

        def run():
            while self.running:
                try:
                    # 1) Grab RAW8 (H x W) from ASI
                    buf = self.cam.asi_camera.get_video_data()  # bytes
                    # 2) Wrap into GstBuffer
                    gst_buf = Gst.Buffer.new_wrapped(buf)
                    # 3) Push to appsrc
                    ret = self.appsrc.emit("push-buffer", gst_buf)
                    
                    # Update frame counters
                    self.frame_counters['total_frames'] += 1
                    self.frame_counters['current_file_frames'] += 1
                    
                    # Calculate FPS for successful frames only
                    if ret == Gst.FlowReturn.OK:
                        self.frame_counters['successful_frames'] += 1
                        self._calculate_fps()
                    else:
                        self.frame_counters['dropped_frames'] += 1
                        self.frame_counters['current_file_dropped'] += 1
                        self.get_logger().warn(f"appsrc push returned {ret} - Frame dropped")
                        time.sleep(0.001)
                        continue
                        
                except Exception as e:
                    self.frame_counters['dropped_frames'] += 1
                    self.frame_counters['current_file_dropped'] += 1
                    self.get_logger().error(f"Producer error: {e} - Frame dropped")
                    time.sleep(0.01)
            # On exit, send EOS to appsrc
            try:
                self.appsrc.end_of_stream()
            except Exception:
                pass

        self.producer_thread = threading.Thread(target=run, daemon=True)
        self.producer_thread.start()

    def start_stats_timer(self):
        """Start periodic frame statistics logging"""
        def log_stats():
            while self.running:
                time.sleep(10)  # Log every 10 seconds
                if self.running:
                    total = self.frame_counters['total_frames']
                    dropped = self.frame_counters['dropped_frames']
                    current_file = self.frame_counters['current_file_frames']
                    current_dropped = self.frame_counters['current_file_dropped']
                    drop_rate = (dropped / total * 100) if total > 0 else 0
                    
                    # Get current FPS
                    current_fps, avg_fps = self._calculate_fps()
                    
                    self.get_logger().info(f"Frame stats - Total: {total}, Dropped: {dropped}, Drop rate: {drop_rate:.2f}%")
                    self.get_logger().info(f"FPS - Current: {current_fps:.1f}, Average: {avg_fps:.1f}")
                    if self.current_file_name:
                        current_drop_rate = (current_dropped / current_file * 100) if current_file > 0 else 0
                        self.get_logger().info(f"Current file ({self.current_file_name}): {current_file} frames, {current_dropped} dropped ({current_drop_rate:.2f}%)")
        
        self.stats_thread = threading.Thread(target=log_stats, daemon=True)
        self.stats_thread.start()

    def on_bus_message(self, bus, message):
        t = message.type
        if t == Gst.MessageType.ERROR:
            try:
                err, dbg = message.parse_error()
                self.get_logger().error(f"Gst ERROR: {err} | {dbg}")
            except UnicodeDecodeError:
                self.get_logger().error("Gst ERROR: Failed to parse error message (Unicode error)")
            self.stop_pipeline()
        elif t == Gst.MessageType.EOS:
            self.get_logger().info("Gst EOS")
            self.stop_pipeline()
        return True

def main():
    rclpy.init()
    node = GstreamerPureGstreamer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
