#!/usr/bin/env python3
import os, sys, logging, time
from datetime import datetime, timezone
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib, GObject

DEVICE      = "/dev/video0"
WIDTH, HEIGHT = 640, 512
FPS         = 30
BITRATE     = 4_000_000
IFRAMES     = 60
CHUNK_SEC   = 30
OUT_DIR     = "/home/a/Projects/ros2_ws/data/boson"
LOG_DIR     = "/home/a/Projects/ros2_ws/data/boson_logs"

os.makedirs(OUT_DIR, exist_ok=True); os.makedirs(LOG_DIR, exist_ok=True)

logf = os.path.join(LOG_DIR, f"boson_log_{datetime.now(timezone.utc).strftime('%Y_%m_%dT%H_%M_%S')}.log")
logging.basicConfig(level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[logging.FileHandler(logf), logging.StreamHandler(sys.stdout)])
log = logging.getLogger("boson")

log.info("="*70)
log.info("BOSON RECORDING SESSION STARTED")
log.info(f"Video output directory: {OUT_DIR}")
log.info(f"Log file: {logf}")
log.info("="*70)

Gst.init(None)

# 1) Побудувати ПОВНІСТЮ той самий пайплайн, що працює у вас в gst-launch
pipeline_str = f"""
v4l2src device={DEVICE} io-mode=2 !
video/x-raw,width={WIDTH},height={HEIGHT},framerate={FPS}/1 !
queue leaky=2 max-size-buffers=60 !
nvvidconv !
video/x-raw(memory:NVMM),format=NV12 !
nvv4l2h264enc bitrate={BITRATE} iframeinterval={IFRAMES} insert-sps-pps=true control-rate=1 maxperf-enable=1 !
h264parse !
splitmuxsink name=mux muxer-factory=mp4mux
              location="{OUT_DIR}/record_%05d.mp4"
              max-size-time={CHUNK_SEC * Gst.SECOND}
              send-keyframe-requests=true async-finalize=true alignment-threshold=0
"""

pipeline = Gst.parse_launch(pipeline_str)

# 2) Іменування чанків по UTC-таймстампу
chunk_count = 0
last_chunk_time = time.time()
mux = pipeline.get_by_name("mux")

def on_format_location(splitmux, fragment_id):
    global chunk_count, last_chunk_time
    now = datetime.now(timezone.utc)
    ts = now.strftime("%Y_%m_%dT%H_%M_%S")
    ms = f"{now.microsecond//1000:03d}"
    name = f"BOSON_{WIDTH}x{HEIGHT}_{ts}_{ms}.mp4"
    path = os.path.join(OUT_DIR, name)
    elapsed = time.time() - last_chunk_time if chunk_count else 0
    chunk_count += 1
    last_chunk_time = time.time()
    log.info(f"🎬 NEW CHUNK #{chunk_count}: {name} (aft_
