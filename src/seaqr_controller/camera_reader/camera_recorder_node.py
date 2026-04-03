#!/usr/bin/env python3

import os
import sys
import time
import numpy as np
import cv2
import zwoasi as asi
import gi
from datetime import datetime
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# Make sure the directory exists
out_dir = "/home/a/Projects/ros2_ws/data/camera/cam_174"
os.makedirs(out_dir, exist_ok=True)

ASI_LIB = "/usr/local/lib/libASICamera2.so"


class CameraRecorderNode(Node):
    def __init__(self):
        super().__init__('camera_recorder_node')
        
        # Make sure the directory exists
        self.out_dir = "/home/a/Projects/ros2_ws/data/camera/cam_174"
        os.makedirs(self.out_dir, exist_ok=True)
        
        self.ASI_LIB = "/usr/local/lib/libASICamera2.so"
        
        # Publisher for camera status
        self.status_publisher = self.create_publisher(String, '/camera_status', 10)
        
        self.get_logger().info("Camera recorder node starting...")
        
        # Initialize camera in a separate thread to avoid blocking ROS2
        self.camera_thread = threading.Thread(target=self.initialize_and_run_camera, daemon=True)
        self.camera_thread.start()
        
        # Timer for status updates
        self.create_timer(5.0, self.publish_status)
        
        self.camera_running = False
        self.frame_count = 0
        self.chunk_count = 0
        self.current_fps = 0.0
        
    def initialize_and_run_camera(self):
        try:
            # --- ASI SDK ---
            asi.init(self.ASI_LIB)
            
            cameras = asi.list_cameras()
            if not cameras:
                self.get_logger().error("No ASI cameras detected")
                return
            
            self.get_logger().info(f"Found cameras: {cameras}")
            
            camera = asi.Camera(0)
            # Camera settings
            camera.set_control_value(asi.ASI_EXPOSURE, 0, auto=True)  # Auto exposure
            camera.set_control_value(asi.ASI_GAIN, 0, auto=True)         # Auto gain for brightness
            camera.set_control_value(asi.ASI_HIGH_SPEED_MODE, 1)           # Enable high-speed mode
            camera.set_control_value(asi.ASI_BANDWIDTHOVERLOAD, 100)       # Maximum bandwidth
            camera.set_roi(0, 0,
                           camera.get_camera_property()['MaxWidth'],
                           camera.get_camera_property()['MaxHeight'],
                           1, image_type=asi.ASI_IMG_RAW8)
            
            props = camera.get_camera_property()
            self.get_logger().info(f"Camera properties: {props}")
            camera.start_video_capture()
            
            # --- Measure actual FPS for 3 seconds ---
            self.get_logger().info("📊 Measuring actual camera FPS...")
            measure_start = time.time()
            measure_frames = 0
            while time.time() - measure_start < 3.0:
                frame = camera.capture_video_frame()
                measure_frames += 1
            
            measured_fps = measure_frames / 3.0
            self.get_logger().info(f"✅ Measured FPS: {measured_fps:.1f}")
            
            # Round to nearest integer for cleaner framerate
            target_fps = round(measured_fps)
            
            # --- Init GStreamer ---
            Gst.init(None)
            
            pipeline = Gst.Pipeline.new("pipeline")
            
            appsrc     = Gst.ElementFactory.make("appsrc", "source")
            convert    = Gst.ElementFactory.make("videoconvert", "convert")
            to_i420    = Gst.ElementFactory.make("videoconvert", "to_i420")
            nvvidconv  = Gst.ElementFactory.make("nvvidconv", "nvvidconv")
            encoder    = Gst.ElementFactory.make("nvv4l2h264enc", "encoder")
            parse      = Gst.ElementFactory.make("h264parse", "parse")
            splitter   = Gst.ElementFactory.make("splitmuxsink", "splitter")
            
            if not all([appsrc, convert, to_i420, nvvidconv, encoder, parse, splitter]):
                self.get_logger().error("Failed to create one or more GStreamer elements")
                return
            
            # --- Caps & appsrc config with measured FPS ---
            width, height, _, _ = camera.get_roi_format()
            caps = Gst.Caps.from_string(
                f"video/x-raw,format=GRAY8,width={width},height={height},framerate={target_fps}/1"
            )
            self.get_logger().info(f"🎬 Configuring pipeline for {target_fps} FPS")
            appsrc.set_property("is-live", True)
            appsrc.set_property("caps", caps)
            appsrc.set_property("do-timestamp", True)
            appsrc.set_property("format", Gst.Format.TIME)
            appsrc.set_property("max-bytes", 100485760)  # 10MB buffer limit
            appsrc.set_property("block", False)  # Don't block if buffer is full - drop frames instead
            
            # --- Encoder settings - OPTIMIZED FOR SPEED ---
            encoder.set_property("bitrate", 8000000)  # 8 Mbps bitrate
            encoder.set_property("control-rate", 0)   # CBR for more consistent performance
            encoder.set_property("preset-level", 4)   # UltraFast preset for maximum speed
            encoder.set_property("iframeinterval", 100)  # Force I-frame every 100 frames for better performance
            encoder.set_property("insert-sps-pps", True)  # Insert SPS/PPS at every IDR
            encoder.set_property("maxperf-enable", True)
            encoder.set_property("EnableTwopassCBR", False)  # Disable two-pass for speed
            
            # --- h264parse - CRITICAL for chunking ---
            parse.set_property("config-interval", 1)  # Send SPS/PPS with every keyframe
            
            # Monitor splitmuxsink signals for chunk verification
            self.chunk_count = 0
            self.last_chunk_time = time.time()
            
            # --- SplitMuxSink configuration - 1 MINUTE CHUNKS ---
            chunk_duration = 60  # seconds per chunk (1 minute)
            
            def on_format_location(splitmux, fragment_id):
                """Generate filename with current timestamp: ZWO_ASI174MM_YYYY_MM_DD_THH_MM_SS_mmm.mp4"""
                current_time = time.time()
                elapsed = current_time - self.last_chunk_time if self.chunk_count > 0 else 0
                self.chunk_count += 1
                
                # Generate timestamp-based filename
                now = datetime.now()
                timestamp = now.strftime("%Y_%m_%d_T%H_%M_%S")
                milliseconds = f"{now.microsecond // 1000:03d}"
                filename = f"ZWO_ASI174MM_{timestamp}_{milliseconds}.mp4"
                filepath = os.path.join(self.out_dir, filename)
                
                self.get_logger().info(f"🎬 NEW CHUNK #{self.chunk_count}: {filename} (after {elapsed:.1f}s)")
                self.last_chunk_time = current_time
                return filepath
            
            splitter.set_property("location", f"{self.out_dir}/ZWO_ASI174MM_%05d.mp4")  # Fallback template
            splitter.connect("format-location", on_format_location)  # Custom filename callback
            splitter.set_property("max-size-time", chunk_duration * Gst.SECOND)  # 1 minute per file
            splitter.set_property("max-size-bytes", 0)  # Disable size-based splitting
            splitter.set_property("send-keyframe-requests", True)    # clean splits
            splitter.set_property("async-finalize", True)  # Allow async file finalization
            splitter.set_property("alignment-threshold", 0)  # Split immediately
            
            # --- Build pipeline ---
            for el in (appsrc, convert, to_i420, nvvidconv, encoder, parse, splitter):
                pipeline.add(el)
            
            if not appsrc.link(convert):
                raise RuntimeError("Link fail: appsrc→convert")
            if not convert.link(to_i420):
                raise RuntimeError("Link fail: convert→to_i420")
            
            caps_i420 = Gst.Caps.from_string("video/x-raw,format=I420")
            if not to_i420.link_filtered(nvvidconv, caps_i420):
                raise RuntimeError("Link fail: to_i420→nvvidconv with I420 caps")
            
            caps_nvmm = Gst.Caps.from_string("video/x-raw(memory:NVMM),format=NV12")
            if not nvvidconv.link_filtered(encoder, caps_nvmm):
                raise RuntimeError("Link fail: nvvidconv→encoder with NVMM caps")
            
            if not encoder.link(parse):
                raise RuntimeError("Link fail: encoder→h264parse")
            
            if not parse.link(splitter):
                raise RuntimeError("Link fail: h264parse→splitmuxsink")
            
            # Add bus watch for errors
            def on_bus_message(bus, message):
                t = message.type
                if t == Gst.MessageType.ERROR:
                    err, debug = message.parse_error()
                    self.get_logger().error(f"GStreamer Error: {err}, {debug}")
                    return False
                elif t == Gst.MessageType.WARNING:
                    warn, debug = message.parse_warning()
                    self.get_logger().warning(f"GStreamer Warning: {warn}")
                return True
            
            bus = pipeline.get_bus()
            bus.add_signal_watch()
            bus.connect("message", on_bus_message)
            
            # --- Start pipeline ---
            pipeline.set_state(Gst.State.PLAYING)
            self.get_logger().info(f"🔴 Recording at {target_fps} FPS (AUTO exposure/gain) to: {self.out_dir}")
            self.get_logger().info("📦 Creating 1-MINUTE MP4 chunks with timestamp filenames.")
            self.get_logger().info("    Format: ZWO_ASI174MM_YYYY_MM_DD_THH_MM_SS_mmm.mp4")
            
            self.frame_count = 0
            dropped_frames = 0
            start_time = time.time()
            last_fps_time = start_time
            fps_frame_count = 0
            
            self.camera_running = True
            
            # Main recording loop
            while rclpy.ok() and self.camera_running:
                try:
                    frame = camera.capture_video_frame()
                    buf = Gst.Buffer.new_wrapped(frame.tobytes())
                    
                    ret = appsrc.emit("push-buffer", buf)
                    if ret != Gst.FlowReturn.OK:
                        if ret == Gst.FlowReturn.FLUSHING:
                            dropped_frames += 1
                        else:
                            self.get_logger().error(f"appsrc push-buffer returned: {ret}")
                            break
                    
                    self.frame_count += 1
                    fps_frame_count += 1
                    
                    current_time = time.time()
                    
                    # FPS reporting every 5 seconds
                    if current_time - last_fps_time >= 5.0:
                        elapsed_total = current_time - start_time
                        elapsed_fps = current_time - last_fps_time
                        
                        self.current_fps = fps_frame_count / elapsed_fps
                        average_fps = self.frame_count / elapsed_total
                        
                        self.get_logger().info(f"📊 {self.current_fps:.1f} FPS | Avg: {average_fps:.1f} | "
                                             f"Target: {target_fps} | Frames: {self.frame_count} | "
                                             f"Dropped: {dropped_frames} | Chunks: {self.chunk_count}")
                        
                        last_fps_time = current_time
                        fps_frame_count = 0
                        
                except Exception as e:
                    self.get_logger().error(f"Error in camera loop: {e}")
                    break
            
            # --- Cleanup ---
            self.get_logger().info("Finalizing last chunk...")
            appsrc.emit("end-of-stream")
            pipeline.send_event(Gst.Event.new_eos())
            bus.timed_pop_filtered(5 * Gst.SECOND, Gst.MessageType.EOS | Gst.MessageType.ERROR)
            pipeline.set_state(Gst.State.NULL)
            camera.stop_video_capture()
            camera.close()
            
            self.camera_running = False
            self.get_logger().info("✅ Camera recording stopped")
            
        except Exception as e:
            self.get_logger().error(f"Camera initialization failed: {e}")
            self.camera_running = False
    
    def publish_status(self):
        """Publish camera status to ROS2 topic"""
        status_msg = String()
        status_msg.data = f"camera_running:{self.camera_running},frames:{self.frame_count},chunks:{self.chunk_count},fps:{self.current_fps:.1f}"
        self.status_publisher.publish(status_msg)
    
    def shutdown(self):
        """Graceful shutdown"""
        self.get_logger().info("Shutting down camera recorder...")
        self.camera_running = False


def main(args=None):
    rclpy.init(args=args)
    
    camera_node = CameraRecorderNode()
    
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        camera_node.get_logger().info("Keyboard interrupt received")
    finally:
        camera_node.shutdown()
        camera_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
