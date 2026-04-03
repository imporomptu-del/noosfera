#!/usr/bin/env python3
import os
import sys
import time
import numpy as np
import cv2
import zwoasi as asi
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst


# Make sure the directory exists
out_dir = "/home/a/Projects/ros2_ws/data/camera/cam_646"
os.makedirs(out_dir, exist_ok=True)

ASI_LIB = "/usr/local/lib/libASICamera2.so"

cameras = asi.list_cameras()
if not cameras:
    raise RuntimeError("No ASI cameras detected")
print("Found cameras:", cameras)

camera = asi.Camera(0)

# Apply your camera settings
camera.set_control_value(asi.ASI_OFFSET, 1)
camera.set_control_value(asi.ASI_BANDWIDTHOVERLOAD, 90)
camera.set_control_value(asi.ASI_FLIP, 0)
camera.set_control_value(asi.ASI_AUTO_MAX_GAIN, 180)
camera.set_control_value(asi.ASI_AUTO_MAX_EXP, 35000)  # 35000 microseconds = 35ms
# Note: AutoExpTargetBrightness might be ASI_BRIGHTNESS or a different control
# camera.set_control_value(asi.ASI_BRIGHTNESS, 110)  # Uncomment if available
camera.set_control_value(asi.ASI_HIGH_SPEED_MODE, 1)
camera.set_control_value(asi.ASI_WB_R, 50)
camera.set_control_value(asi.ASI_WB_B, 75)

# Set auto exposure and gain
camera.set_control_value(asi.ASI_EXPOSURE, 0, True)  # Auto exposure
camera.set_control_value(asi.ASI_GAIN, 0, True)      # Auto gain

camera.set_roi(0, 0,
               camera.get_camera_property()['MaxWidth'],
               camera.get_camera_property()['MaxHeight'],
               1, image_type=asi.ASI_IMG_RAW16)

props = camera.get_camera_property()
print(props)
camera.start_video_capture()

# --- Init GStreamer ---
Gst.init(None)

pipeline = Gst.Pipeline.new("pipeline")

appsrc     = Gst.ElementFactory.make("appsrc", "source")
bayer2rgb  = Gst.ElementFactory.make("bayer2rgb", "bayer")
convert    = Gst.ElementFactory.make("videoconvert", "convert")
to_i420    = Gst.ElementFactory.make("videoconvert", "to_i420")
nvvidconv  = Gst.ElementFactory.make("nvvidconv", "nvvidconv")
encoder    = Gst.ElementFactory.make("nvv4l2h264enc", "encoder")
parse      = Gst.ElementFactory.make("h264parse", "parse")
splitter   = Gst.ElementFactory.make("splitmuxsink", "splitter")  # <<< chunking

if not all([appsrc, bayer2rgb, convert, to_i420, nvvidconv, encoder, parse, splitter]):
    raise RuntimeError("Failed to create one or more GStreamer elements")

# --- Caps & appsrc config (10 fps) ---
width, height, _, _ = camera.get_roi_format()
caps = Gst.Caps.from_string(
    f"video/x-bayer,format=rggb,width={width},height={height},framerate=100/1"
)
appsrc.set_property("is-live", True)
appsrc.set_property("caps", caps)
appsrc.set_property("do-timestamp", False)  # we'll set PTS manually
appsrc.set_property("max-bytes", 10485760)  # 10MB buffer limit
appsrc.set_property("block", False)  # Don't block if buffer is full - drop frames instead

# --- Encoder (tune for low-latency, stable CBR) ---
encoder.set_property("bitrate", 1200000)   # kbps
encoder.set_property("control-rate", 0)  # CBR mode
encoder.set_property("preset-level", 1)  # UltraFast for low latency
encoder.set_property("iframeinterval", 30)  # I-frame every 30 frames
encoder.set_property("maxperf-enable", True)  # Maximum performance
# Note: nvh264enc doesn't support iframeinterval property
# GOP size is controlled by the encoder's internal settings

# --- SplitMuxSink (roll 1-minute MP4 chunks) ---
# location uses a numeric sequence; change path/pattern as you like
splitter.set_property("location", f"{out_dir}/out_%05d.mp4")
splitter.set_property("max-size-time", 10 * Gst.SECOND)  # 60s per file
splitter.set_property("max-files", 0)                    # unlimited
splitter.set_property("send-keyframe-requests", True)    # clean splits
splitter.set_property("muxer", Gst.ElementFactory.make("mp4mux", "mux"))  # MP4

# --- Build pipeline: appsrc → bayer2rgb → convert → I420 → enc → parse → splitmuxsink ---
for el in (appsrc, bayer2rgb, convert, to_i420, nvvidconv, encoder, parse, splitter):
    pipeline.add(el)

if not appsrc.link(bayer2rgb):
    raise RuntimeError("Link fail: appsrc→bayer2rgb")
if not bayer2rgb.link(convert):
    raise RuntimeError("Link fail: bayer2rgb→videoconvert")
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

# splitmuxsink is a sink; link parse → splitmuxsink
if not parse.link(splitter):
    raise RuntimeError("Link fail: h264parse→splitmuxsink")

# --- Start pipeline ---
pipeline.set_state(Gst.State.PLAYING)
print("Recording @100 fps… 1-minute MP4 chunks (out_00000.mp4, out_00001.mp4, …). Press Ctrl+C to stop.")

frame_count = 0
start_time = time.time()
frame_duration = Gst.util_uint64_scale_int(1, Gst.SECOND, 10)  # 100 fps

try:
    while True:
        frame = camera.capture_video_frame()           # np.ndarray (H, W) RAW8
        buf = Gst.Buffer.new_wrapped(frame.tobytes())

        pts = frame_count * frame_duration
        buf.pts = pts
        buf.dts = pts
        buf.duration = frame_duration

        ret = appsrc.emit("push-buffer", buf)
        if ret != Gst.FlowReturn.OK:
            if ret == Gst.FlowReturn.FLUSHING:
                # Buffer is full, frame dropped - this is normal
                continue
            else:
                print("appsrc push-buffer returned:", ret)
                break

        frame_count += 1
        if frame_count % 30 == 0:
            elapsed = time.time() - start_time
            print(f"≈ {frame_count/elapsed:.2f} fps encoded")

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
