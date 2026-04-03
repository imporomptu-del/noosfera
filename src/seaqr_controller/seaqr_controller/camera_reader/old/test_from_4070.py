#!/usr/bin/env python3
import os
import sys
import time
import numpy as np
import cv2
import zwoasi as asi
import gi
from datetime import datetime
gi.require_version('Gst', '1.0')
from gi.repository import Gst


# Make sure the directory exists
out_dir = "/home/a/Projects/ros2_ws/data/camera"
os.makedirs(out_dir, exist_ok=True)

ASI_LIB = "/usr/local/lib/libASICamera2.so"  # Fixed path

# # --- 0) ASI SDK ---
# ASI_LIB = '/home/a/ZWO/ASIStudio/lib/libASICamera2.so'
# if not os.path.exists(ASI_LIB):
#     print(f"⚠️ ASI SDK library not found at {ASI_LIB}")
#     sys.exit(1)
# os.environ['ZWO_ASI_LIB_PATH'] = ASI_LIB
asi.init(ASI_LIB)

cameras = asi.list_cameras()
if not cameras:
    raise RuntimeError("No ASI cameras detected")
print("Found cameras:", cameras)

camera = asi.Camera(0)
# OPTIMAL SETTINGS FOR MAXIMUM FPS (from diagnostic):
camera.set_control_value(asi.ASI_EXPOSURE, 0,True)   # 1ms - FASTEST exposure for 152+ FPS
camera.set_control_value(asi.ASI_GAIN, 0,True)        # Higher gain to compensate for fast exposure
camera.set_control_value(asi.ASI_HIGH_SPEED_MODE, 1)  # Enable high-speed mode
camera.set_control_value(asi.ASI_BANDWIDTHOVERLOAD, 100)  # Maximum bandwidth
camera.set_roi(0, 0,
               camera.get_camera_property()['MaxWidth'],
               camera.get_camera_property()['MaxHeight'],
               1, image_type=asi.ASI_IMG_RAW8)

props = camera.get_camera_property()
print(props)
camera.start_video_capture()

# --- Init GStreamer ---
Gst.init(None)

pipeline = Gst.Pipeline.new("pipeline")

appsrc     = Gst.ElementFactory.make("appsrc", "source")
convert    = Gst.ElementFactory.make("videoconvert", "convert")
to_i420    = Gst.ElementFactory.make("videoconvert", "to_i420")
nvvidconv  = Gst.ElementFactory.make("nvvidconv", "nvvidconv")  # Jetson Orin GPU conversion
encoder    = Gst.ElementFactory.make("nvv4l2h264enc", "encoder")   # Jetson Orin hardware encoder
parse      = Gst.ElementFactory.make("h264parse", "parse")
splitter   = Gst.ElementFactory.make("splitmuxsink", "splitter")  # <<< chunking

if not all([appsrc, convert, to_i420, nvvidconv, encoder, parse, splitter]):
    raise RuntimeError("Failed to create one or more GStreamer elements")

# --- Caps & appsrc config (natural fps) ---
width, height, _, _ = camera.get_roi_format()
caps = Gst.Caps.from_string(
    f"video/x-raw,format=GRAY8,width={width},height={height}"
)
appsrc.set_property("is-live", True)
appsrc.set_property("caps", caps)
appsrc.set_property("do-timestamp", True)  # Let GStreamer handle timing

# --- Encoder (Jetson Orin nvv4l2h264enc settings) ---
encoder.set_property("bitrate", 5000000)  # 8 Mbps
encoder.set_property("control-rate", 1)   # VBR
encoder.set_property("preset-level", 1)   # Low latency
encoder.set_property("iframeinterval", 60)  # I-frame every 60 frames
encoder.set_property("maxperf-enable", True)
encoder.set_property("ratecontrol-enable", True)

# --- SplitMuxSink (30-second MP4 chunks with timestamp filenames) ---
def on_format_location(splitmux, fragment_id):
    """Generate filename with current timestamp: ZWO_ASI174MM_YYYY_MM_DD_THH_MM_SS_mmm.mp4"""
    now = datetime.now()
    timestamp = now.strftime("%Y_%m_%d_T%H_%M_%S")
    milliseconds = f"{now.microsecond // 1000:03d}"
    filename = f"ZWO_ASI174MM_{timestamp}_{milliseconds}.mp4"
    filepath = os.path.join(out_dir, filename)
    print(f"📁 Creating chunk: {filename}")
    return filepath

splitter.set_property("location", f"{out_dir}/ZWO_ASI174MM_%05d.mp4")  # Fallback template
splitter.connect("format-location", on_format_location)  # Custom filename callback
splitter.set_property("max-size-time", 30 * Gst.SECOND)  # 30s per file

splitter.set_property("max-size-bytes", 0)  # Disable size-based splitting
splitter.set_property("send-keyframe-requests", True)    # clean splits
splitter.set_property("async-finalize", True)  # Allow async file finalization
splitter.set_property("alignment-threshold", 0)  # Split immediately
# REMOVED: splitter.set_property("muxer", Gst.ElementFactory.make("mp4mux", "mux"))  # This breaks chunking!

# --- Build pipeline: appsrc → convert → I420 → nvvidconv → encoder → parse → splitmuxsink ---
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

# --- Start pipeline ---
pipeline.set_state(Gst.State.PLAYING)
print("Recording at natural FPS… 1-minute MP4 chunks. Press Ctrl+C to stop.")

frame_count = 0
start_time = time.time()
last_fps_time = start_time
fps_frame_count = 0

try:
    while True:
        frame = camera.capture_video_frame()           # np.ndarray (H, W) RAW8
        buf = Gst.Buffer.new_wrapped(frame.tobytes())

        ret = appsrc.emit("push-buffer", buf)
        if ret != Gst.FlowReturn.OK:
            print("appsrc push-buffer returned:", ret)
            break

        frame_count += 1
        fps_frame_count += 1
        
        # More frequent FPS reporting to catch issues
        current_time = time.time()
        if current_time - last_fps_time >= 2.0:  # Every 2 seconds
            elapsed_total = current_time - start_time
            elapsed_fps = current_time - last_fps_time
            
            current_fps = fps_frame_count / elapsed_fps
            average_fps = frame_count / elapsed_total
            
            print(f"📊 Current: {current_fps:.1f} FPS | Average: {average_fps:.1f} FPS | Total frames: {frame_count}")
            
            # Reset FPS counter
            last_fps_time = current_time
            fps_frame_count = 0

except KeyboardInterrupt:
    print("Stopping…")

# --- Cleanup (EOS so mp4 files finalize) ---
appsrc.emit("end-of-stream")
pipeline.send_event(Gst.Event.new_eos())
bus = pipeline.get_bus()
bus.timed_pop_filtered(Gst.CLOCK_TIME_NONE, Gst.MessageType.EOS | Gst.MessageType.ERROR)
pipeline.set_state(Gst.State.NULL)
camera.stop_video_capture()
camera.close()
print("Done.")
