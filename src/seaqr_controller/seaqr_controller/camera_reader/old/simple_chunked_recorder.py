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

ASI_LIB = "/usr/local/lib/libASICamera2.so"
asi.init(ASI_LIB)

cameras = asi.list_cameras()
if not cameras:
    raise RuntimeError("No ASI cameras detected")
print("Found cameras:", cameras)

camera = asi.Camera(0)
camera.set_control_value(asi.ASI_EXPOSURE, 10000)  # 10ms
camera.set_control_value(asi.ASI_GAIN, 100)
camera.set_roi(0, 0, camera.get_camera_property()['MaxWidth'], camera.get_camera_property()['MaxHeight'], 1, image_type=asi.ASI_IMG_RAW8)

props = camera.get_camera_property()
print(props)

camera.start_video_capture()

# --- Init GStreamer ---
Gst.init(None)

def create_new_pipeline(chunk_num):
    """Create a completely new pipeline for each chunk"""
    timestamp = datetime.now().strftime("%Y_%m_%d_T%H_%M_%S")
    filename = f"{out_dir}/ZWO_ASI174MM_{timestamp}_chunk{chunk_num:03d}.mp4"
    
    width = camera.get_roi_format()[0]
    height = camera.get_roi_format()[1]
    
    # Use simple filesink approach (not splitmuxsink)
    pipeline_str = f"""
    appsrc name=source is-live=true format=time do-timestamp=true 
    caps=video/x-raw,format=GRAY8,width={width},height={height},framerate=100/1 !
    videoconvert !
    video/x-raw,format=I420 !
    nvvidconv !
    video/x-raw(memory:NVMM),format=NV12 !
    nvv4l2h264enc bitrate=8000000 control-rate=1 preset-level=1 iframeinterval=30 maxperf-enable=true ratecontrol-enable=true !
    h264parse !
    mp4mux !
    filesink location={filename}
    """
    
    pipeline = Gst.parse_launch(pipeline_str)
    appsrc = pipeline.get_by_name("source")
    
    print(f"📁 Created chunk {chunk_num}: {filename}")
    return pipeline, appsrc, filename

chunk_duration = 60  # seconds
current_pipeline = None
current_appsrc = None
chunk_number = 0
frame_count = 0
start_time = time.time()
chunk_start_time = time.time()

print("🎬 Starting simple chunked recording")
print("Each chunk = 60 seconds")
print("Press Ctrl+C to stop")

try:
    while True:
        current_time = time.time()
        elapsed = current_time - start_time
        chunk_elapsed = current_time - chunk_start_time
        
        # Start new chunk if needed
        if current_pipeline is None or chunk_elapsed >= chunk_duration:
            # Clean up previous pipeline
            if current_pipeline is not None:
                print(f"⏹️  Stopping chunk {chunk_number}")
                current_appsrc.emit("end-of-stream")
                
                # Wait for EOS with longer timeout
                bus = current_pipeline.get_bus()
                msg = bus.timed_pop_filtered(10 * Gst.SECOND, Gst.MessageType.EOS | Gst.MessageType.ERROR)
                if msg:
                    if msg.type == Gst.MessageType.EOS:
                        print(f"✅ Chunk {chunk_number} EOS received")
                    elif msg.type == Gst.MessageType.ERROR:
                        err, debug = msg.parse_error()
                        print(f"❌ Pipeline error: {err}")
                else:
                    print(f"⚠️  Timeout waiting for EOS on chunk {chunk_number}")
                
                # Force pipeline to stop and wait for state change
                current_pipeline.set_state(Gst.State.NULL)
                ret = current_pipeline.get_state(10 * Gst.SECOND)  # Wait for NULL state
                if ret[0] == Gst.StateChangeReturn.SUCCESS:
                    print(f"✅ Chunk {chunk_number} pipeline stopped cleanly")
                else:
                    print(f"⚠️  Pipeline state change issues for chunk {chunk_number}")
                
                # Give extra time for file to be properly written
                time.sleep(1)
                
            # Start new chunk
            chunk_number += 1
            current_pipeline, current_appsrc, filename = create_new_pipeline(chunk_number)
            current_pipeline.set_state(Gst.State.PLAYING)
            chunk_start_time = current_time
            frame_count = 0  # Reset frame count for new chunk
            
            # Give pipeline time to start
            time.sleep(0.1)
            
        # Capture and push frame
        frame = camera.capture_video_frame()
        buf = Gst.Buffer.new_wrapped(frame.tobytes())
        
        # Set timing
        frame_duration = Gst.util_uint64_scale_int(1, Gst.SECOND, 100)  # 100 fps
        pts = frame_count * frame_duration
        buf.pts = pts
        buf.duration = frame_duration
        buf.dts = pts
        
        ret = current_appsrc.emit("push-buffer", buf)
        
        frame_count += 1
        if frame_count % 100 == 0:
            print(f"📊 Chunk {chunk_number}: {chunk_elapsed:.1f}s/{chunk_duration}s | {frame_count/chunk_elapsed:.1f} fps")
            
except KeyboardInterrupt:
    print("🛑 Stopping...")

# Final cleanup
if current_pipeline:
    print(f"⏹️  Finalizing chunk {chunk_number}")
    current_appsrc.emit("end-of-stream")
    
    bus = current_pipeline.get_bus()
    msg = bus.timed_pop_filtered(10 * Gst.SECOND, Gst.MessageType.EOS | Gst.MessageType.ERROR)
    if msg and msg.type == Gst.MessageType.EOS:
        print(f"✅ Final chunk EOS received")
    else:
        print(f"⚠️  Final chunk may not have completed properly")
    
    current_pipeline.set_state(Gst.State.NULL)
    ret = current_pipeline.get_state(10 * Gst.SECOND)
    if ret[0] == Gst.StateChangeReturn.SUCCESS:
        print(f"✅ Final chunk completed cleanly")
    
    # Extra time for final file write
    time.sleep(2)

camera.stop_video_capture()
camera.close()

print(f"🎬 Recording saved to: {out_dir}/ZWO_ASI174MM_*_chunk*.mp4")
