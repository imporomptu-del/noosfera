#!/usr/bin/env python3
import os
import sys
import time
import numpy as np
import cv2
import zwoasi as asi
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib


# Make sure the directory exists
out_dir = "/home/a/Projects/ros2_ws/data/camera"
os.makedirs(out_dir, exist_ok=True)

ASI_LIB = "/usr/local/lib/libASICamera2.so"

# --- 0) ASI SDK ---
asi.init(ASI_LIB)

cameras = asi.list_cameras()
if not cameras:
    raise RuntimeError("No ASI cameras detected")
print("Found cameras:", cameras)

camera = asi.Camera(0)
# Limit camera FPS to what encoder can handle in real-time (~40-50 FPS with current settings)
camera.set_control_value(asi.ASI_EXPOSURE, 0, auto=True)  # 20ms exposure = max 50 FPS
camera.set_control_value(asi.ASI_GAIN, 0, auto=True)         # Auto gain for brightness
camera.set_control_value(asi.ASI_HIGH_SPEED_MODE, 1)           # Enable high-speed mode
camera.set_control_value(asi.ASI_BANDWIDTHOVERLOAD, 100)       # Maximum bandwidth
camera.set_roi(0, 0,
               camera.get_camera_property()['MaxWidth'],
               camera.get_camera_property()['MaxHeight'],
               1, image_type=asi.ASI_IMG_RAW8)

props = camera.get_camera_property()
print(props)
camera.start_video_capture()

# --- Measure actual FPS for 3 seconds ---
print("📊 Measuring actual camera FPS...")
measure_start = time.time()
measure_frames = 0
while time.time() - measure_start < 3.0:
    frame = camera.capture_video_frame()
    measure_frames += 1

measured_fps = measure_frames / 3.0
print(f"✅ Measured FPS: {measured_fps:.1f}")

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
muxer      = Gst.ElementFactory.make("mp4mux", "muxer")
splitter   = Gst.ElementFactory.make("splitmuxsink", "splitter")

if not all([appsrc, convert, to_i420, nvvidconv, encoder, parse, muxer, splitter]):
    raise RuntimeError("Failed to create one or more GStreamer elements")

# --- Caps & appsrc config with measured FPS ---
width, height, _, _ = camera.get_roi_format()
# Use measured FPS for accurate playback timing
caps = Gst.Caps.from_string(
    f"video/x-raw,format=GRAY8,width={width},height={height},framerate={target_fps}/1"
)
print(f"🎬 Configuring pipeline for {target_fps} FPS")
appsrc.set_property("is-live", True)
appsrc.set_property("caps", caps)
appsrc.set_property("do-timestamp", True)
appsrc.set_property("format", Gst.Format.TIME)
appsrc.set_property("max-bytes", 100485760)  # 10MB buffer limit
appsrc.set_property("block", False)  # Don't block if buffer is full - drop frames instead



# --- Encoder settings - OPTIMIZED FOR SPEED ---
encoder.set_property("bitrate", 8000000)  # Lower bitrate = faster encoding (3 Mbps)
encoder.set_property("control-rate", 0)   # CBR for more consistent performance
encoder.set_property("preset-level", 4)   # UltraFast preset (was 1=low latency)
encoder.set_property("iframeinterval", 100)  # Force I-frame every 100 frames (~1 sec at 100fps)
encoder.set_property("insert-sps-pps", True)  # Insert SPS/PPS at every IDR
encoder.set_property("maxperf-enable", True)
encoder.set_property("EnableTwopassCBR", False)  # Disable two-pass for speed

# --- h264parse - CRITICAL for chunking ---
parse.set_property("config-interval", 1)  # Send SPS/PPS with every keyframe

# --- mp4mux settings ---
muxer.set_property("faststart", False)  # Don't use faststart with splitmuxsink
muxer.set_property("fragment-duration", 1000)  # 1 second fragments for better splitting

# --- SplitMuxSink configuration - 10 SECOND CHUNKS ---
splitter.set_property("location", f"{out_dir}/ZWO_ASI174MM_%05d.mp4")
splitter.set_property("max-size-time", 10 * Gst.SECOND)  # 10 seconds per file
splitter.set_property("muxer", muxer)  # Use explicit muxer
splitter.set_property("send-keyframe-requests", True)
splitter.set_property("async-finalize", True)

# Monitor splitmuxsink signals for chunk verification
chunk_count = 0
last_chunk_time = time.time()

def on_format_location(splitmux, fragment_id):
    global chunk_count, last_chunk_time
    current_time = time.time()
    elapsed = current_time - last_chunk_time if chunk_count > 0 else 0
    chunk_count += 1
    filename = f"{out_dir}/ZWO_ASI174MM_{fragment_id:05d}.mp4"
    print(f"🎬 NEW CHUNK #{chunk_count}: {os.path.basename(filename)} (after {elapsed:.1f}s)")
    last_chunk_time = current_time
    return filename

splitter.connect("format-location", on_format_location)

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
        print(f"❌ Error: {err}, {debug}")
        return False
    elif t == Gst.MessageType.WARNING:
        warn, debug = message.parse_warning()
        print(f"⚠️ Warning: {warn}")
    return True

bus = pipeline.get_bus()
bus.add_signal_watch()
bus.connect("message", on_bus_message)

# --- Start pipeline ---
pipeline.set_state(Gst.State.PLAYING)
print(f"🔴 Recording at {target_fps} FPS (AUTO exposure/gain) to: {out_dir}")
print("📦 Creating 10-SECOND MP4 chunks. Press Ctrl+C to stop.")
print("-" * 60)

frame_count = 0
dropped_frames = 0
start_time = time.time()
last_fps_time = start_time
last_file_check = start_time
fps_frame_count = 0
last_file_count = 0

print(f"Note: Encoder may run slower than capture rate.")
print(f"      Chunks will contain 10 seconds of video, but may take longer to create.")
print(f"      This is normal when capture FPS exceeds encoding capability.")

try:
    while True:
        frame = camera.capture_video_frame()
        buf = Gst.Buffer.new_wrapped(frame.tobytes())
        
        # GStreamer will timestamp automatically with do-timestamp=True

        ret = appsrc.emit("push-buffer", buf)
        if ret != Gst.FlowReturn.OK:
            if ret == Gst.FlowReturn.FLUSHING:
                dropped_frames += 1
            else:
                print(f"❌ appsrc push-buffer returned: {ret}")
                break

        frame_count += 1
        fps_frame_count += 1
        
        current_time = time.time()
        
        # FPS reporting every 2 seconds
        if current_time - last_fps_time >= 2.0:
            elapsed_total = current_time - start_time
            elapsed_fps = current_time - last_fps_time
            
            current_fps = fps_frame_count / elapsed_fps
            average_fps = frame_count / elapsed_total
            
            # Check if FPS has drifted significantly from target
            drift_indicator = ""
            fps_drift = abs(current_fps - target_fps)
            drift_pct = (fps_drift / target_fps) * 100
            if drift_pct > 10:
                drift_indicator = f" ⚠️ DRIFT: {drift_pct:.1f}%"
            
            # Count actual files on disk
            files = [f for f in os.listdir(out_dir) if f.startswith("ZWO_ASI174MM_") and f.endswith(".mp4")]
            file_count = len(files)
            
            print(f"📊 {current_fps:.1f} FPS | Avg: {average_fps:.1f} | Target: {target_fps} | "
                  f"Frames: {frame_count} | Dropped: {dropped_frames} | "
                  f"Chunks: {chunk_count} | Files: {file_count}{drift_indicator}")
            
            last_fps_time = current_time
            fps_frame_count = 0
        
        # Check for new files every 3 seconds
        if current_time - last_file_check >= 3.0:
            files = sorted([f for f in os.listdir(out_dir) if f.startswith("ZWO_ASI174MM_") and f.endswith(".mp4")])
            file_count = len(files)
            
            if file_count > last_file_count:
                print(f"✅ New file detected! Total files: {file_count}")
                last_file_count = file_count
            
            if files:
                latest = files[-1]
                size_mb = os.path.getsize(os.path.join(out_dir, latest)) / (1024*1024)
                print(f"📁 Latest: {latest} ({size_mb:.1f} MB)")
            
            last_file_check = current_time

except KeyboardInterrupt:
    print("\n⏹️  Stopping recording...")

# --- Cleanup ---
print("Finalizing last chunk...")
appsrc.emit("end-of-stream")
pipeline.send_event(Gst.Event.new_eos())
bus.timed_pop_filtered(5 * Gst.SECOND, Gst.MessageType.EOS | Gst.MessageType.ERROR)
pipeline.set_state(Gst.State.NULL)
camera.stop_video_capture()
camera.close()

# Final summary
files = sorted([f for f in os.listdir(out_dir) if f.startswith("ZWO_ASI174MM_") and f.endswith(".mp4")])
print(f"\n✅ Done! Created {len(files)} chunk(s):")
total_size = 0
for f in files:
    filepath = os.path.join(out_dir, f)
    size_mb = os.path.getsize(filepath) / (1024*1024)
    total_size += size_mb
    # Try to get duration
    try:
        import subprocess
        result = subprocess.run(
            ['ffprobe', '-v', 'error', '-show_entries', 'format=duration', 
             '-of', 'default=noprint_wrappers=1:nokey=1', filepath],
            capture_output=True, text=True, timeout=5
        )
        duration = float(result.stdout.strip())
        print(f"   • {f} ({size_mb:.1f} MB, {duration:.1f}s)")
    except:
        print(f"   • {f} ({size_mb:.1f} MB)")

print(f"\n📊 Total: {total_size:.1f} MB across {len(files)} files")