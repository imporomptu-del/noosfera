#!/usr/bin/env python3
import os
import sys
import time
import logging
from datetime import datetime, timezone

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# ===========================
# Параметри під себе
# ===========================
DEVICE      = "/dev/video0"            # FLIR Boson V4L2 пристрій
WIDTH       = 640
HEIGHT      = 512
FR_NUM      = 30                       # fps
BITRATE     = 4_000_000                # 4 Mbps
IFRAME_INT  = 60                       # ключовий кадр кожні 60 кадрів (~2 c)
CHUNK_SEC   = 30                       # тривалість чанку в секундах
OUT_DIR     = "/home/a/Projects/ros2_ws/data/boson"
LOG_DIR     = "/home/a/Projects/ros2_ws/data/boson_logs"

os.makedirs(OUT_DIR, exist_ok=True)
os.makedirs(LOG_DIR, exist_ok=True)

log_filename = os.path.join(
    LOG_DIR, f"boson_log_{datetime.now(timezone.utc).strftime('%Y_%m_%dT%H_%M_%S')}.log"
)
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    handlers=[logging.FileHandler(log_filename), logging.StreamHandler(sys.stdout)]
)
logger = logging.getLogger("boson")

logger.info("="*70)
logger.info("BOSON RECORDING SESSION STARTED")
logger.info(f"Video output directory: {OUT_DIR}")
logger.info(f"Log file: {log_filename}")
logger.info("="*70)

# ===========================
# GStreamer ініціалізація
# ===========================
Gst.init(None)

pipeline = Gst.Pipeline.new("boson-pipeline")

src        = Gst.ElementFactory.make("v4l2src", "src")
queue0     = Gst.ElementFactory.make("queue", "q0")
capsfilter = Gst.ElementFactory.make("capsfilter", "capsfilter")
nvvidconv  = Gst.ElementFactory.make("nvvidconv", "nvvidconv")
enc        = Gst.ElementFactory.make("nvv4l2h264enc", "enc")
parse      = Gst.ElementFactory.make("h264parse", "parse")
splitter   = Gst.ElementFactory.make("splitmuxsink", "splitter")

if not all([src, queue0, capsfilter, nvvidconv, enc, parse, splitter]):
    raise RuntimeError("Не вдалося створити один або більше елементів GStreamer")

# --- v4l2src (FLIR Boson) ---
src.set_property("device", DEVICE)
# io-mode=2 (DMABUF) у gst-launch часто задають через властивість, але тут достатньо дефолту.
# За потреби можна увімкнути: src.set_property("io-mode", 2)

# --- Queue: дропаємо найстаріші кадри при заторах (як у вашому прикладі) ---
queue0.set_property("leaky", 2)                 # downstream
queue0.set_property("max-size-buffers", 60)     # ~2 сек буфера при 30 fps
queue0.set_property("max-size-bytes", 0)
queue0.set_property("max-size-time", 0)

# --- Caps перед конвертацією: жорстко задаємо розмір і fps (формат не фіксуємо, як у вашій команді) ---
caps = Gst.Caps.from_string(
    f"video/x-raw,width={WIDTH},height={HEIGHT},framerate={FR_NUM}/1"
)
capsfilter.set_property("caps", caps)

# --- Конвертація в NV12 у пам'ять NVMM (для NVENC) ---
# nvvidconv → caps NVMM/NV12
caps_nvmm = Gst.Caps.from_string("video/x-raw(memory:NVMM),format=NV12")

# --- Апаратний H.264 енкодер (Jetson) ---
enc.set_property("bitrate", BITRATE)
enc.set_property("iframeinterval", IFRAME_INT)
enc.set_property("insert-sps-pps", True)
enc.set_property("control-rate", 1)     # VBR (як у вас). Для CBR: 0
enc.set_property("maxperf-enable", True)

# --- h264parse: вставляти SPS/PPS регулярно (важливо для splitmuxsink) ---
parse.set_property("config-interval", 1)

# --- splitmuxsink: чанкування MP4 по часу ---
# Використаємо колбек для іменування файлів за UTC таймстампом
chunk_count = 0
last_chunk_time = time.time()

def on_format_location(splitmux, fragment_id):
    global chunk_count, last_chunk_time
    now = datetime.now(timezone.utc)
    ts = now.strftime("%Y_%m_%dT%H_%M_%S")
    ms = f"{now.microsecond // 1000:03d}"
    filename = f"BOSON_{WIDTH}x{HEIGHT}_{ts}_{ms}.mp4"
    filepath = os.path.join(OUT_DIR, filename)

    elapsed = time.time() - last_chunk_time if chunk_count > 0 else 0
    chunk_count += 1
    logger.info(f"🎬 NEW CHUNK #{chunk_count}: {filename} (after {elapsed:.1f}s)")
    last_chunk_time = time.time()
    return filepath

splitter.set_property("location", os.path.join(OUT_DIR, "record_%05d.mp4"))  # fallback
splitter.set_property("max-size-bytes", 0)
splitter.set_property("max-size-time", CHUNK_SEC * Gst.SECOND)
splitter.set_property("send-keyframe-requests", True)
splitter.set_property("async-finalize", True)
splitter.set_property("alignment-threshold", 0)
splitter.connect("format-location", on_format_location)

# --- Побудова графа ---
for el in (src, queue0, capsfilter, nvvidconv, enc, parse, splitter):
    pipeline.add(el)

if not src.link(queue0):         raise RuntimeError("Link fail: v4l2src→queue")
if not queue0.link(capsfilter):  raise RuntimeError("Link fail: queue→capsfilter")
if not capsfilter.link(nvvidconv): raise RuntimeError("Link fail: capsfilter→nvvidconv")
if not nvvidconv.link_filtered(enc, caps_nvmm):
    raise RuntimeError("Link fail: nvvidconv→enc (NVMM/NV12 caps)")
if not enc.link(parse):          raise RuntimeError("Link fail: enc→h264parse")
if not parse.link(splitter):     raise RuntimeError("Link fail: h264parse→splitmuxsink")

# --- Обробка повідомлень шини ---
def on_bus_message(bus, message):
    t = message.type
    if t == Gst.MessageType.ERROR:
        err, debug = message.parse_error()
        logger.error(f"❌ GStreamer Error: {err}. Debug: {debug}")
        loop.quit()
    elif t == Gst.MessageType.WARNING:
        warn, debug = message.parse_warning()
        logger.warning(f"⚠️ GStreamer Warning: {warn}. Debug: {debug}")
    elif t == Gst.MessageType.EOS:
        logger.info("✅ EOS received")
        loop.quit()
    return True

bus = pipeline.get_bus()
bus.add_signal_watch()
bus.connect("message", on_bus_message)

# --- Періодичний моніторинг файлів (кожні 3 секунди) ---
last_file_count = 0
def poll_files():
    nonlocal_vars = {"last_file_count": 0}
    def _inner():
        try:
            files = sorted(
                f for f in os.listdir(OUT_DIR)
                if (f.startswith("BOSON_") or f.startswith("record_")) and f.endswith(".mp4")
            )
            file_count = len(files)
            if file_count > nonlocal_vars["last_file_count"]:
                logger.info(f"✅ New file detected! Total files: {file_count}")
                nonlocal_vars["last_file_count"] = file_count
            if files:
                latest = files[-1]
                size_mb = os.path.getsize(os.path.join(OUT_DIR, latest)) / (1024*1024)
                logger.info(f"📁 Latest: {latest} ({size_mb:.1f} MB)")
        except Exception as e:
            logger.warning(f"File poll error: {e}")
        return True  # continue timer
    return _inner

GLib.timeout_add_seconds(3, poll_files())

# --- Старт ---
logger.info(f"🔴 Recording BOSON {WIDTH}x{HEIGHT}@{FR_NUM} → H.264 {BITRATE/1e6:.1f} Mbps")
logger.info(f"🧩 Chunk duration: {CHUNK_SEC}s | I-frame interval: {IFRAME_INT}")
logger.info("Натисніть Ctrl+C для зупинки…")

pipeline.set_state(Gst.State.PLAYING)
loop = GLib.MainLoop()

try:
    loop.run()
except KeyboardInterrupt:
    logger.info("⏹️  Stopping… (sending EOS)")
    pipeline.send_event(Gst.Event.new_eos())
    # зачекаємо трохи, щоб splitmuxsink дописав файл
    bus.timed_pop_filtered(5 * Gst.SECOND, Gst.MessageType.EOS | Gst.MessageType.ERROR)
finally:
    pipeline.set_state(Gst.State.NULL)

# --- Фінальне резюме ---
try:
    files = sorted(
        f for f in os.listdir(OUT_DIR)
        if (f.startswith("BOSON_") or f.startswith("record_")) and f.endswith(".mp4")
    )
    total_size = 0.0
    logger.info("\n✅ Done! Created %d chunk(s):", len(files))
    for f in files:
        fp = os.path.join(OUT_DIR, f)
        sz = os.path.getsize(fp) / (1024*1024)
        total_size += sz
        # опціонально: спробувати дістати тривалість через ffprobe (як у вашому коді)
        try:
            import subprocess
            r = subprocess.run(
                ["ffprobe", "-v", "error", "-show_entries", "format=duration",
                 "-of", "default=noprint_wrappers=1:nokey=1", fp],
                capture_output=True, text=True, timeout=5
            )
            dur = float(r.stdout.strip())
            logger.info(f"   • {f} ({sz:.1f} MB, {dur:.1f}s)")
        except Exception:
            logger.info(f"   • {f} ({sz:.1f} MB)")
    logger.info(f"\n📊 Total: {total_size:.1f} MB across {len(files)} files")
finally:
    logger.info("="*70)
    logger.info("BOSON RECORDING SESSION ENDED")
    logger.info("="*70)
