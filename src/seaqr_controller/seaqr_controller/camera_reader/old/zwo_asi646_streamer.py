#!/usr/bin/env python3
"""
ZWO ASI646MC USB Camera Streamer
Streams camera feed via GStreamer for live viewing
"""

import os
import sys
import time
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

# Add ZWO ASI SDK path
sys.path.append('/opt/zwoasi/lib')
import zwoasi as asi

# --- Configuration ---
ASI_LIB = '/opt/zwoasi/lib/libASICamera2.so.1.33.0'
STREAM_PORT = 5000
STREAM_HOST = "127.0.0.1"  # Change to 0.0.0.0 for network access
BITRATE = 4000000  # 4 Mbps for streaming
TARGET_FPS = 30

def main():
    print("🎥 ZWO ASI646MC USB Camera Streamer")
    
    # --- Initialize ZWO ASI SDK ---
    asi.init(ASI_LIB)
    
    cameras = asi.list_cameras()
    if not cameras:
        raise RuntimeError("No ASI cameras detected")
    print("Found cameras:", cameras)
    
    camera = asi.Camera(0)
    
    # Apply camera settings (from your configuration)
    camera.set_control_value(asi.ASI_OFFSET, 1)
    camera.set_control_value(asi.ASI_BANDWIDTHOVERLOAD, 90)
    camera.set_control_value(asi.ASI_FLIP, 0)
    camera.set_control_value(asi.ASI_AUTO_MAX_GAIN, 500)
    camera.set_control_value(asi.ASI_AUTO_MAX_EXP, 35000)  # 35ms
    camera.set_control_value(asi.ASI_HIGH_SPEED_MODE, 1)
    
    # Set auto exposure and gain
    camera.set_control_value(asi.ASI_EXPOSURE, 0, auto=True)
    camera.set_control_value(asi.ASI_GAIN, 0, auto=True)
    
    # Set camera format - using GRAY8 for streaming efficiency
    camera.set_roi(0, 0,
                   camera.get_camera_property()['MaxWidth'],
                   camera.get_camera_property()['MaxHeight'],
                   1, image_type=asi.ASI_IMG_RAW8)
    
    props = camera.get_camera_property()
    print(f"Camera: {props['Name']}")
    print(f"Resolution: {props['MaxWidth']}x{props['MaxHeight']}")
    camera.start_video_capture()
    
    # --- Initialize GStreamer ---
    Gst.init(None)
    
    pipeline = Gst.Pipeline.new("streaming-pipeline")
    
    # Create elements
    appsrc = Gst.ElementFactory.make("appsrc", "source")
    convert = Gst.ElementFactory.make("videoconvert", "convert")
    nvvidconv = Gst.ElementFactory.make("nvvidconv", "nvvidconv")
    encoder = Gst.ElementFactory.make("nvv4l2h264enc", "encoder")
    parse = Gst.ElementFactory.make("h264parse", "parse")
    rtppay = Gst.ElementFactory.make("rtph264pay", "rtppay")
    udpsink = Gst.ElementFactory.make("udpsink", "udpsink")
    
    if not all([appsrc, convert, nvvidconv, encoder, parse, rtppay, udpsink]):
        raise RuntimeError("Failed to create one or more GStreamer elements")
    
    # Add elements to pipeline
    for element in [appsrc, convert, nvvidconv, encoder, parse, rtppay, udpsink]:
        pipeline.add(element)
    
    # Configure appsrc
    width, height, _, _ = camera.get_roi_format()
    caps = Gst.Caps.from_string(
        f"video/x-raw,format=GRAY8,width={width},height={height},framerate={TARGET_FPS}/1"
    )
    appsrc.set_property("is-live", True)
    appsrc.set_property("caps", caps)
    appsrc.set_property("do-timestamp", True)
    appsrc.set_property("format", Gst.Format.TIME)
    appsrc.set_property("max-bytes", 10485760)  # 10MB buffer
    appsrc.set_property("block", False)  # Don't block on full buffer
    
    # Configure encoder for streaming
    encoder.set_property("bitrate", BITRATE)
    encoder.set_property("control-rate", 0)  # CBR
    encoder.set_property("preset-level", 1)  # UltraFast for low latency
    encoder.set_property("maxperf-enable", True)
    encoder.set_property("insert-sps-pps", True)
    
    # Configure RTP payload
    rtppay.set_property("config-interval", 1)
    rtppay.set_property("pt", 96)
    
    # Configure UDP sink
    udpsink.set_property("host", STREAM_HOST)
    udpsink.set_property("port", STREAM_PORT)
    udpsink.set_property("sync", False)
    
    # Link elements
    if not appsrc.link(convert):
        raise RuntimeError("Link fail: appsrc→convert")
    if not convert.link(nvvidconv):
        raise RuntimeError("Link fail: convert→nvvidconv")
    
    # Set NVMM caps between nvvidconv and encoder
    nvmm_caps = Gst.Caps.from_string("video/x-raw(memory:NVMM)")
    if not nvvidconv.link_filtered(encoder, nvmm_caps):
        raise RuntimeError("Link fail: nvvidconv→encoder")
    
    if not encoder.link(parse):
        raise RuntimeError("Link fail: encoder→parse")
    if not parse.link(rtppay):
        raise RuntimeError("Link fail: parse→rtppay")
    if not rtppay.link(udpsink):
        raise RuntimeError("Link fail: rtppay→udpsink")
    
    # Start pipeline
    pipeline.set_state(Gst.State.PLAYING)
    
    print(f"🚀 Streaming started!")
    print(f"📡 Stream URL: udp://{STREAM_HOST}:{STREAM_PORT}")
    print(f"🎬 Resolution: {width}x{height} @ {TARGET_FPS} FPS")
    print(f"💾 Bitrate: {BITRATE/1000000:.1f} Mbps")
    print("\n📺 To view the stream, run this command:")
    print(f"gst-launch-1.0 -v udpsrc port={STREAM_PORT} ! application/x-rtp,encoding-name=H264,payload=96 ! rtph264depay ! h264parse ! avdec_h264 ! autovideosink")
    print("\nPress Ctrl+C to stop...")
    
    frame_count = 0
    start_time = time.time()
    frame_duration = Gst.util_uint64_scale_int(1, Gst.SECOND, TARGET_FPS)
    
    try:
        while True:
            frame = camera.capture_video_frame()  # GRAY8 format
            buf = Gst.Buffer.new_wrapped(frame.tobytes())
            
            # Set timestamp
            pts = frame_count * frame_duration
            buf.pts = pts
            buf.dts = pts
            buf.duration = frame_duration
            
            # Push buffer
            ret = appsrc.emit("push-buffer", buf)
            if ret != Gst.FlowReturn.OK:
                if ret == Gst.FlowReturn.FLUSHING:
                    continue  # Buffer full, frame dropped
                else:
                    print(f"Push buffer error: {ret}")
                    break
            
            frame_count += 1
            
            # Print stats every 5 seconds
            if frame_count % (TARGET_FPS * 5) == 0:
                elapsed = time.time() - start_time
                actual_fps = frame_count / elapsed
                print(f"📊 Frames: {frame_count}, FPS: {actual_fps:.1f}, Streaming for {elapsed:.0f}s")
    
    except KeyboardInterrupt:
        print("\n🛑 Stopping stream...")
    
    finally:
        pipeline.set_state(Gst.State.NULL)
        camera.stop_video_capture()
        camera.close()
        print("✅ Stream stopped")

if __name__ == "__main__":
    main()
