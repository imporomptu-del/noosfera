#!/usr/bin/env python3
import os
import time
import zwoasi as asi
import gi

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib


ASI_LIB = "/home/rahim/dev/noosfera/ASI_linux_mac_SDK_V1.39/lib/x64/libASICamera2.so"   # adjust if needed
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
bayer2rgb = Gst.ElementFactory.make("bayer2rgb", "bayer")
convert = Gst.ElementFactory.make("videoconvert", "convert")
to_i420 = Gst.ElementFactory.make("videoconvert", "to_i420")
encoder = Gst.ElementFactory.make("nvh264enc", "encoder")  # RTX 3060; "nvv4l2h264enc" - Jetson
parse = Gst.ElementFactory.make("h264parse", "parse")
mux = Gst.ElementFactory.make("mp4mux", "mux")
sink = Gst.ElementFactory.make("filesink", "sink")

if not all([appsrc, bayer2rgb, convert, to_i420, encoder, parse, mux, sink]):
    raise RuntimeError("Failed to create one or more GStreamer elements")

# Configure caps for RAW8 Bayer from ZWO
width = camera.get_roi_format()[0]
height = camera.get_roi_format()[1]
caps = Gst.Caps.from_string(f"video/x-bayer,format=rggb,width={width},height={height},framerate=30/1")
appsrc.set_property("is-live", True)
appsrc.set_property("caps", caps)

# Configure encoder
encoder.set_property("preset", "low-latency-hq")
encoder.set_property("bitrate", 12000)   # kbps
encoder.set_property("rc-mode", "cbr")

sink.set_property("location", "output.mp4")

# Build pipeline: appsrc → bayer2rgb → convert → I420 → encoder → parse → mux → sink
pipeline.add(appsrc)
pipeline.add(bayer2rgb)
pipeline.add(convert)
pipeline.add(to_i420)
pipeline.add(encoder)
pipeline.add(parse)
pipeline.add(mux)
pipeline.add(sink)

appsrc.link(bayer2rgb)
bayer2rgb.link(convert)
convert.link(to_i420)
caps_i420 = Gst.Caps.from_string("video/x-raw,format=I420")
to_i420.link_filtered(encoder, caps_i420)
encoder.link(parse)
parse.link(mux)
mux.link(sink)

# --- Start pipeline ---
pipeline.set_state(Gst.State.PLAYING)

print("Recording... press Ctrl+C to stop")
import time

frame_count = 0
start_time = time.time()

try:
    while True:
        frame = camera.capture_video_frame()
        buf = Gst.Buffer.new_wrapped(frame.tobytes())
        frame_duration = Gst.util_uint64_scale_int(1, Gst.SECOND, 30)  # 30 fps

        pts = frame_count * frame_duration
        buf.pts = pts
        buf.duration = frame_duration

        appsrc.emit("push-buffer", buf)

        frame_count += 1
        if frame_count % 100 == 0:
            elapsed = time.time() - start_time
            print(f"≈ {frame_count/elapsed:.2f} fps encoded")
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



