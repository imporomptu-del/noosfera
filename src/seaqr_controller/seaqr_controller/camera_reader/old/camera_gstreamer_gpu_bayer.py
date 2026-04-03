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
import cv2

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


class GstreamerGPUBayer(Node):
    def __init__(self):
        super().__init__('gstreamer_gpu_bayer')

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

        # 5) GPU bayer conversion setup
        self.use_gpu_bayer = False
        self.gpu_bayer_available = False
        self.setup_gpu_bayer()

        # 6) Initialize camera props + GStreamer + pipeline
        self.init_camera_properties()     # fills width/height/is_color/bayer
        self.init_gstreamer()
        self.create_pipeline()            # builds GPU-accelerated pipeline

        # 7) Start pipeline + GLib loop + producer
        self.start_pipeline()
        self.start_producer_thread()      # feeds appsrc from ASI SDK
        self.start_stats_timer()          # periodic frame statistics

    def setup_gpu_bayer(self):
        """Setup GPU-accelerated bayer conversion using OpenCV"""
        try:
            # Check if OpenCV has CUDA support
            if cv2.cuda.getCudaEnabledDeviceCount() > 0:
                self.gpu_bayer_available = True
                self.use_gpu_bayer = True
                self.get_logger().info("✅ GPU-accelerated bayer conversion available (OpenCV CUDA)")
            else:
                self.get_logger().warn("⚠️ OpenCV CUDA not available, using CPU bayer conversion")
                self.gpu_bayer_available = False
                self.use_gpu_bayer = False
        except Exception as e:
            self.get_logger().warn(f"⚠️ GPU bayer setup failed: {e}, using CPU")
            self.gpu_bayer_available = False
            self.use_gpu_bayer = False

    def convert_bayer_gpu(self, bayer_data, width, height, bayer_pattern='rggb'):
        """Convert bayer data to RGB using GPU acceleration"""
        if not self.use_gpu_bayer:
            # Fallback to CPU conversion
            return self.convert_bayer_cpu(bayer_data, width, height, bayer_pattern)
        
        try:
            # Reshape bayer data
            bayer_image = bayer_data.reshape((height, width))
            
            # Upload to GPU
            gpu_bayer = cv2.cuda_GpuMat()
            gpu_bayer.upload(bayer_image)
            
            # Convert bayer pattern to OpenCV constant
            bayer_codes = {
                'rggb': cv2.COLOR_BayerRG2BGR,
                'bggr': cv2.COLOR_BayerBG2BGR,
                'grbg': cv2.COLOR_BayerGB2BGR,
                'gbrg': cv2.COLOR_BayerGR2BGR
            }
            
            bayer_code = bayer_codes.get(bayer_pattern, cv2.COLOR_BayerRG2BGR)
            
            # Convert on GPU
            gpu_rgb = cv2.cuda.cvtColor(gpu_bayer, bayer_code)
            
            # Download from GPU
            rgb_image = gpu_rgb.download()
            
            return rgb_image
            
        except Exception as e:
            self.get_logger().error(f"GPU bayer conversion failed: {e}, falling back to CPU")
            return self.convert_bayer_cpu(bayer_data, width, height, bayer_pattern)

    def convert_bayer_cpu(self, bayer_data, width, height, bayer_pattern='rggb'):
        """Convert bayer data to RGB using CPU"""
        try:
            # Reshape bayer data
            bayer_image = bayer_data.reshape((height, width))
            
            # Convert bayer pattern to OpenCV constant
            bayer_codes = {
                'rggb': cv2.COLOR_BayerRG2BGR,
                'bggr': cv2.COLOR_BayerBG2BGR,
                'grbg': cv2.COLOR_BayerGB2BGR,
                'gbrg': cv2.COLOR_BayerGR2BGR
            }
            
            bayer_code = bayer_codes.get(bayer_pattern, cv2.COLOR_BayerRG2BGR)
            
            # Convert on CPU
            rgb_image = cv2.cvtColor(bayer_image, bayer_code)
            
            return rgb_image
            
        except Exception as e:
            self.get_logger().error(f"CPU bayer conversion failed: {e}")
            return None

    def init_camera_properties(self):
        info = self.cam.asi_camera.get_camera_property()
        self.width = info['MaxWidth']
        self.height = info['MaxHeight']
        self.is_color = (info['IsColorCam'] == 1)

        # Map SDK BayerPattern -> OpenCV bayer pattern
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
            'drop_rate': (self.frame_counters['dropped_frames'] / self.frame_counters['total_frames'] * 100) if self.frame_counters['total_frames'] > 0 else 0,
            'gpu_bayer_enabled': self.use_gpu_bayer
        }

    def create_pipeline(self):
        self.pipeline = Gst.Pipeline()

        # Elements - Simplified pipeline with pre-processed RGB data
        self.appsrc = Gst.ElementFactory.make("appsrc", "source")
        
        # GPU memory upload
        cudaupload = Gst.ElementFactory.make("cudaupload", "cuda_upload")
        
        # GPU color conversion
        cudaconvert = Gst.ElementFactory.make("cudaconvert", "cuda_convert")
        
        # Download from GPU memory for encoder
        cudadownload = Gst.ElementFactory.make("cudadownload", "cuda_download")
        
        # Standard conversion for I420
        to_i420 = Gst.ElementFactory.make("videoconvert", "to_i420")
        
        self.encoder = Gst.ElementFactory.make("nvh264enc", "encoder")  # RTX (adjust if Jetson)
        h264parse = Gst.ElementFactory.make("h264parse", "parse")
        splitmux = Gst.ElementFactory.make("splitmuxsink", "smux")

        # Validate elements
        elements_to_check = [self.appsrc, cudaupload, cudaconvert, cudadownload, to_i420, self.encoder, h264parse, splitmux]
        for e in elements_to_check:
            if not e:
                raise RuntimeError(f"Failed to create a GStreamer element: {e}")

        # Configure appsrc (live + timestamps + RGB caps)
        self.appsrc.set_property("is-live", True)
        self.appsrc.set_property("do-timestamp", True)
        self.appsrc.set_property("format", Gst.Format.TIME)
        
        # RGB caps instead of bayer
        caps_rgb = Gst.Caps.from_string(f"video/x-raw,format=BGR,width={self.width},height={self.height}")
        self.appsrc.set_property("caps", caps_rgb)

        # Encoder (low latency CBR + regular IDR)
        self.encoder.set_property("preset", "low-latency-hp")
        self.encoder.set_property("rc-mode", "cbr")
        self.encoder.set_property("bitrate", 12000)   # kbps
        self.encoder.set_property("gop-size", 30)     # ~1 sec @ 30fps

        # Parser: ensure config
        h264parse.set_property("config-interval", 1)

        # splitmuxsink: 10-minute rotation, TS container, keyframe-aligned
        splitmux.set_property("muxer-factory", "mpegtsmux")
        splitmux.set_property("max-size-time", 10 * 1000 * 1000 * 1000)  # 10s in ns
        splitmux.set_property("send-keyframe-requests", True)

        # Add elements to pipeline
        self.pipeline.add(self.appsrc)
        self.pipeline.add(cudaupload)
        self.pipeline.add(cudaconvert)
        self.pipeline.add(cudadownload)
        self.pipeline.add(to_i420)
        self.pipeline.add(self.encoder)
        self.pipeline.add(h264parse)
        self.pipeline.add(splitmux)

        # Link elements: appsrc -> cudaupload -> cudaconvert -> cudadownload -> to_i420 -> encoder -> parse -> splitmux
        if not self.appsrc.link(cudaupload): raise RuntimeError("link appsrc->cudaupload")
        if not cudaupload.link(cudaconvert): raise RuntimeError("link cudaupload->cudaconvert")
        if not cudaconvert.link(cudadownload): raise RuntimeError("link cudaconvert->cudadownload")
        if not cudadownload.link(to_i420): raise RuntimeError("link cudadownload->to_i420")
        
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
        splitmux.set_property("location", os.path.join(out_dir, "rec_%05d.ts"))  # dummy; will be overridden
        splitmux.connect("format-location", self._format_location)

        pipeline_type = "GPU-accelerated" if self.use_gpu_bayer else "CPU"
        self.get_logger().info(f"Pipeline created ({pipeline_type} bayer conversion): appsrc→cudaupload→cudaconvert→cudadownload→I420→enc→parse→splitmuxsink")

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
        self.get_logger().info(f"GPU bayer conversion: {'Enabled' if self.use_gpu_bayer else 'Disabled'}")
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
                    
                    # 2) Convert bayer to RGB (GPU or CPU)
                    bayer_data = np.frombuffer(buf, dtype=np.uint8)
                    rgb_image = self.convert_bayer_gpu(bayer_data, self.width, self.height, self.bayer_pattern)
                    
                    if rgb_image is None:
                        self.frame_counters['dropped_frames'] += 1
                        self.frame_counters['current_file_dropped'] += 1
                        self.get_logger().warn("Bayer conversion failed - Frame dropped")
                        time.sleep(0.001)
                        continue
                    
                    # 3) Wrap RGB data into GstBuffer
                    rgb_bytes = rgb_image.tobytes()
                    gst_buf = Gst.Buffer.new_wrapped(rgb_bytes)
                    
                    # 4) Push to appsrc
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
                    
                    gpu_status = "GPU" if self.use_gpu_bayer else "CPU"
                    self.get_logger().info(f"Frame stats ({gpu_status} bayer) - Total: {total}, Dropped: {dropped}, Drop rate: {drop_rate:.2f}%")
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
    node = GstreamerGPUBayer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

