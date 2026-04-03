#!/usr/bin/env python3

import os
import sys
import time
import signal
import numpy as np
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib


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

cam = asi.Camera(0)  # choose different index if needed
info = cam.get_camera_property()

cam.set_control_value(asi.ASI_GAIN, 0, auto=True)
cam.set_control_value(asi.ASI_EXPOSURE,0, auto=True)
cam.set_control_value(asi.ASI_OFFSET, 1)
cam.set_control_value(asi.ASI_BANDWIDTHOVERLOAD, 90)
cam.set_control_value(asi.ASI_FLIP, 0)
cam.set_control_value(asi.ASI_AUTO_MAX_GAIN, 1000)
cam.set_control_value(asi.ASI_AUTO_MAX_EXP, 20000)
#cam.set_control_value(asi.ASI_AUTO_TARGET_BRIGHTNESS, 110)
cam.set_control_value(asi.ASI_HIGH_SPEED_MODE, 1)

is_color = (info['IsColorCam'] == 1)
# White balance for color camera
if is_color:
    cam.set_control_value(asi.ASI_WB_R, 50)
    cam.set_control_value(asi.ASI_WB_B, 75)

bayer_pattern = info.get('BayerPattern', 0)
# ROI/format (RAW8 at full resolution)
W, H = info['MaxWidth'], info['MaxHeight']
cam.set_roi(0, 0, W, H, 1, image_type=asi.ASI_IMG_RAW8)

# Start video capture
cam.start_video_capture()

# --- 2) Pure GStreamer pipeline ---
Gst.init(None)

# Create pipeline
pipeline = Gst.Pipeline()

# Create elements
appsrc = Gst.ElementFactory.make("appsrc", "source")
bayer2rgb = Gst.ElementFactory.make("bayer2rgb", "bayer")
videoconvert = Gst.ElementFactory.make("videoconvert", "convert")

# Choose encoder based on system - prioritize NVIDIA hardware
if os.path.exists('/usr/lib/x86_64-linux-gnu/gstreamer-1.0/libgstnvcodec.so') or \
   os.path.exists('/usr/lib/x86_64-linux-gnu/gstreamer-1.0/libgstnvcodec.so.0'):
    # Desktop NVIDIA GPU (RTX 4070)
    encoder = Gst.ElementFactory.make("nvh264enc", "encoder")
    if encoder:
        encoder.set_property("preset", "low-latency-hp")
        encoder.set_property("rc-mode", "cbr")  # constant bitrate
        encoder.set_property("bitrate", 12000)  # 12 Mbps
        encoder.set_property("gop-size", 60)
        print("Using NVIDIA hardware encoder (nvh264enc)")
    else:
        print("nvh264enc not available, falling back to CPU encoder")
        encoder = Gst.ElementFactory.make("x264enc", "encoder")
        encoder.set_property("speed-preset", 0)  # ultrafast
        encoder.set_property("tune", 0x00000004)  # zerolatency
elif os.path.exists('/etc/nv_tegra_release'):
    # Jetson
    encoder = Gst.ElementFactory.make("nvv4l2h264enc", "encoder")
    encoder.set_property("preset-level", 1)
    encoder.set_property("insert-sps-pps", True)
    encoder.set_property("iframeinterval", 30)
    print("Using Jetson hardware encoder (nvv4l2h264enc)")
else:
    # CPU fallback
    encoder = Gst.ElementFactory.make("x264enc", "encoder")
    encoder.set_property("speed-preset", 0)  # ultrafast
    encoder.set_property("tune", 0x00000004)  # zerolatency
    print("Using CPU encoder (x264enc)")

h264parse = Gst.ElementFactory.make("h264parse", "parse")
mpegtsmux = Gst.ElementFactory.make("mpegtsmux", "mux")
filesink = Gst.ElementFactory.make("filesink", "sink")
filesink.set_property("location", "out.ts")

# Add elements to pipeline
elements = [appsrc, bayer2rgb, videoconvert, encoder, h264parse, mpegtsmux, filesink]
for element in elements:
    if not element:
        raise RuntimeError(f"Failed to create element")
    pipeline.add(element)

# Link elements
appsrc.link(bayer2rgb)
bayer2rgb.link(videoconvert)
videoconvert.link(encoder)
encoder.link(h264parse)
h264parse.link(mpegtsmux)
mpegtsmux.link(filesink)

# Configure appsrc
appsrc.set_property("is-live", True)
appsrc.set_property("do-timestamp", True)
appsrc.set_property("format", Gst.Format.TIME)

# Set caps for appsrc - RAW8 Bayer data from camera
caps = Gst.Caps.from_string(
    f"video/x-bayer,format=rggb,width={W},height={H},framerate=30/1"
)
appsrc.set_property("caps", caps)


# Set state and check bus for errors
pipeline.set_state(Gst.State.PLAYING)

# Check bus for immediate errors
bus = pipeline.get_bus()
message = bus.timed_pop_filtered(2 * Gst.SECOND, Gst.MessageType.ERROR)
if message:
    err, debug = message.parse_error()
    raise RuntimeError(f"Pipeline error: {err}")

# Print pipeline info
print(f"Pipeline created successfully: {pipeline.get_name()}")


# Check if we're using hardware acceleration
if encoder and "nvh264enc" in encoder.get_name():
    print("✅ Using NVIDIA RTX 4070 hardware encoder")
    print("   - Lower CPU usage")
    print("   - Better quality at same bitrate")
    print("   - Real-time encoding capability")
else:
    print("⚠️  Using CPU encoder (slower, higher CPU usage)")

print(f"Recording started: {W}x{H} at 30fps to out.ts")
print("Press Ctrl+C to stop")

try:
    # Main loop
    loop = GLib.MainLoop()
    
    def on_bus_message(bus, message):
        msg_type = message.type
        if msg_type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"GStreamer error: {err}")
            loop.quit()
        elif msg_type == Gst.MessageType.EOS:
            print("End of stream")
            loop.quit()
        return True
    
    # Connect bus message handler
    bus = pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect("message", on_bus_message)
    
    # Start recording loop
    while True:
        # Get RAW8 buffer
        buf = cam.get_video_data()
        raw = np.frombuffer(buf, dtype=np.uint8).reshape(H, W)
        
        # Send RAW8 data directly to GStreamer
        # GStreamer will handle the Bayer conversion internally
        raw_bytes = raw.tobytes()
        
        # Create GStreamer buffer
        gst_buffer = Gst.Buffer.new_wrapped(raw_bytes)
        
        # Push to appsrc
        ret = appsrc.emit("push-buffer", gst_buffer)
        if ret != Gst.FlowReturn.OK:
            print(f"Failed to push buffer: {ret}")
            break
        
        time.sleep(1.0/30.0)  # 30fps
    
    # Run the main loop
    loop.run()

except KeyboardInterrupt:
    print("\nStopping...")
finally:
    # Cleanup
    pipeline.set_state(Gst.State.NULL)
    cam.stop_video_capture()
    cam.close()
    print("Recording stopped")
