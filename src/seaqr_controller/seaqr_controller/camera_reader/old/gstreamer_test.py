#!/usr/bin/env python3
import os
import time
from datetime import datetime
import zwoasi as asi
import gi

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# Make sure the directory exists
out_dir = "/home/a/Projects/ros2_ws/data/camera"
os.makedirs(out_dir, exist_ok=True)

ASI_LIB = "/usr/local/lib/libASICamera2.so"  # Fixed path
asi.init(ASI_LIB)

cameras = asi.list_cameras()
if not cameras:
    raise RuntimeError("No ASI cameras detected")
print("Found cameras:", cameras)

camera = asi.Camera(0)  # use first camera
camera.set_control_value(asi.ASI_EXPOSURE, 10000)  # 10ms
camera.set_control_value(asi.ASI_GAIN, 100)
camera.set_roi(0, 0, camera.get_camera_property()['MaxWidth'], camera.get_camera_property()['MaxHeight'], 1, image_type=asi.ASI_IMG_RAW8)

props = camera.get_camera_property()
print(props)

camera.start_video_capture()

# --- Init GStreamer ---
Gst.init(None)

pipeline = Gst.Pipeline.new("pipeline")

appsrc = Gst.ElementFactory.make("appsrc", "source")
convert = Gst.ElementFactory.make("videoconvert", "convert")
to_i420 = Gst.ElementFactory.make("videoconvert", "to_i420")
nvvidconv = Gst.ElementFactory.make("nvvidconv", "nvvidconv")  # Added for Jetson
encoder = Gst.ElementFactory.make("nvv4l2h264enc", "encoder")  # Fixed for Jetson
parse = Gst.ElementFactory.make("h264parse", "parse")
sink = Gst.ElementFactory.make("splitmuxsink", "sink")  # Use splitmuxsink for chunking

if not all([appsrc, convert, to_i420, nvvidconv, encoder, parse, sink]):
    raise RuntimeError("Failed to create one or more GStreamer elements")

# Configure caps for GRAY8 from ZWO (no Bayer conversion)
width = camera.get_roi_format()[0]
height = camera.get_roi_format()[1]
caps = Gst.Caps.from_string(f"video/x-raw,format=GRAY8,width={width},height={height},framerate=100/1")
appsrc.set_property("is-live", True)
appsrc.set_property("caps", caps)
appsrc.set_property("format", Gst.Format.TIME)
appsrc.set_property("do-timestamp", True)

# Configure encoder for Jetson (same as camera_recorder_gpu.py)
encoder.set_property("bitrate", 8000000)  # 8 Mbps
encoder.set_property("control-rate", 1)   # VBR
encoder.set_property("preset-level", 1)  # Low latency
encoder.set_property("iframeinterval", 30)  # I-frame every 30 frames for better splitting
encoder.set_property("maxperf-enable", True)
encoder.set_property("ratecontrol-enable", True)

# Use splitmuxsink for chunking with proper timestamp
chunk_duration = 60  # seconds per chunk
timestamp = datetime.now().strftime("%Y_%m_%d_T%H_%M_%S")
sink.set_property("location", f"{out_dir}/ZWO_ASI174MM_{timestamp}_%05d.mp4")
sink.set_property("max-size-time", chunk_duration * 1000000000)  # nanoseconds
sink.set_property("send-keyframe-requests", True)  # CRITICAL: Request keyframes for splitting
sink.set_property("max-size-bytes", 0)  # Disable size-based splitting to use time-based only

# Build pipeline: appsrc → convert → I420 → nvvidconv → encoder → parse → splitmuxsink
pipeline.add(appsrc)
pipeline.add(convert)
pipeline.add(to_i420)
pipeline.add(nvvidconv)
pipeline.add(encoder)
pipeline.add(parse)
pipeline.add(sink)  # splitmuxsink replaces mux + filesink

appsrc.link(convert)
convert.link(to_i420)
caps_i420 = Gst.Caps.from_string("video/x-raw,format=I420")
to_i420.link_filtered(nvvidconv, caps_i420)
caps_nvmm = Gst.Caps.from_string("video/x-raw(memory:NVMM),format=NV12")
nvvidconv.link_filtered(encoder, caps_nvmm)
encoder.link(parse)
parse.link(sink)  # Direct link to splitmuxsink

# --- Start pipeline ---
pipeline.set_state(Gst.State.PLAYING)

print(f"Recording to: {out_dir}/ZWO_ASI174MM_{timestamp}_*.mp4 (chunked every {chunk_duration}s)")
print("Recording continuous 1-minute chunks. Press Ctrl+C to stop")

frame_count = 0
start_time = time.time()
last_chunk_time = start_time

try:
    while True:
        current_time = time.time()
        elapsed = current_time - start_time
        chunk_elapsed = current_time - last_chunk_time
        
        # Check if we've completed a chunk
        if chunk_elapsed >= chunk_duration:
            chunk_number = int(elapsed // chunk_duration)
            print(f"✅ Chunk {chunk_number} completed ({chunk_duration}s) - Starting chunk {chunk_number + 1}")
            last_chunk_time = current_time
            
        frame = camera.capture_video_frame()
        buf = Gst.Buffer.new_wrapped(frame.tobytes())
        
        # Set proper timing for splitmuxsink chunking
        frame_duration = Gst.util_uint64_scale_int(1, Gst.SECOND, 100)  # 100 fps
        pts = frame_count * frame_duration
        buf.pts = pts
        buf.duration = frame_duration
        buf.dts = pts  # Set decode timestamp same as presentation timestamp

        appsrc.emit("push-buffer", buf)

        frame_count += 1
        if frame_count % 100 == 0:
            chunk_number = int(elapsed // chunk_duration)
            chunk_progress = chunk_elapsed % chunk_duration
            print(f"≈ {frame_count/elapsed:.2f} fps | Chunk {chunk_number}: {chunk_progress:.1f}s/{chunk_duration}s")
except KeyboardInterrupt:
    print("Stopping...")

# --- Cleanup ---
# Tell appsrc no more data will come
appsrc.emit("end-of-stream")

# Push EOS downstream so mp4mux finalizes moov atom
pipeline.send_event(Gst.Event.new_eos())

# Wait until EOS travels through the pipeline
bus = pipeline.get_bus()
msg = bus.timed_pop_filtered(
    Gst.CLOCK_TIME_NONE,
    Gst.MessageType.EOS | Gst.MessageType.ERROR
)
print("EOS message received:", msg.type)

# Now safe to stop pipeline and close camera
pipeline.set_state(Gst.State.NULL)
camera.stop_video_capture()
camera.close()

print(f"Recording saved to: {out_dir}/ZWO_ASI174MM_{timestamp}_*.mp4")