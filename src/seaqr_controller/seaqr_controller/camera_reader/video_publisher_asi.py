#!/usr/bin/env python3
# asi_gst_recorder_node.py
import os
import time
import signal
import subprocess
from typing import Optional

import rclpy
from rclpy.node import Node

import numpy as np
import cv2
import zwoasi as asi
import sys

import os
os.environ.setdefault("GST_PLUGIN_PATH", "/usr/lib/x86_64-linux-gnu/gstreamer-1.0")
os.environ.setdefault("GST_PLUGIN_SCANNER", "/usr/lib/x86_64-linux-gnu/gstreamer-1.0/gst-plugin-scanner")
os.environ.setdefault("GST_DEBUG", "2")  # на час діагностики

os.environ['ZWO_ASI_LIB_PATH'] = '/home/a/ZWO/ASIStudio/lib/libASICamera2.so'
lib_path = '/home/a/ZWO/ASIStudio/lib/libASICamera2.so'
if not os.path.exists(lib_path):
    print(f"⚠️ ASI SDK library not found at {lib_path}")
    sys.exit(1)
os.environ['ZWO_ASI_LIB_PATH'] = lib_path


def gst_has(element: str) -> bool:
    try:
        subprocess.run(
            ["gst-inspect-1.0", element],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=True,
        )
        return True
    except Exception:
        return False


class ASIRecorder(Node):
    def __init__(self):
        super().__init__("asi_gst_recorder")

        # ===== Parameters =====
        self.declare_parameter("zwo_lib", "/home/a/ZWO/ASIStudio/lib/libASICamera2.so")
        self.declare_parameter("camera_index", 0)          # 0,1,2... з asi.list_cameras()
        self.declare_parameter("image_type", "RAW16")       # RAW8 | RAW16
        self.declare_parameter("bayer_pattern", "RGGB")     # RGGB | BGGR | GRBG | GBRG
        self.declare_parameter("exposure_us", 10000)        # 10 ms
        self.declare_parameter("gain", 200)
        self.declare_parameter("wb_r", 50)                  # тільки для кольорових
        self.declare_parameter("wb_b", 50)

        self.declare_parameter("fps", 30)
        self.declare_parameter("bitrate_kbps", 12000)
        self.declare_parameter("gop", 60)                   # keyframe interval (frames)
        self.declare_parameter("preset", "llhp")            # для nvh264enc; ігнорується для x264
        self.declare_parameter("segment_minutes", 5)
        self.declare_parameter("output_dir", "recordings")

        # ===== Fetch params =====
        self.zwo_lib = self._p("zwo_lib")
        self.cam_idx = int(self._p("camera_index"))
        self.img_type_str = self._p("image_type").upper()
        self.bayer_pattern = self._p("bayer_pattern").upper()
        self.exposure_us = int(self._p("exposure_us"))
        self.gain = int(self._p("gain"))
        self.wb_r = int(self._p("wb_r"))
        self.wb_b = int(self._p("wb_b"))

        self.fps = int(self._p("fps"))
        self.bitrate_kbps = int(self._p("bitrate_kbps"))
        self.gop = int(self._p("gop"))
        self.preset = self._p("preset")
        self.segment_minutes = int(self._p("segment_minutes"))
        self.output_dir = self._p("output_dir")
        os.makedirs(self.output_dir, exist_ok=True)

        # ===== Init ZWO SDK =====
        asi.init(self.zwo_lib)
        cams = asi.list_cameras()
        if not cams:
            raise RuntimeError("No ZWO ASI cameras found.")
        if self.cam_idx < 0 or self.cam_idx >= len(cams):
            raise RuntimeError(f"camera_index {self.cam_idx} out of range (found {len(cams)} cams)")

        self.cam = asi.Camera(self.cam_idx)
        self.props = self.cam.get_camera_property()
        self.is_color = bool(self.props.get("IsColorCam", 0) == 1)
        self.width, self.height = self.props["MaxWidth"], self.props["MaxHeight"]

        # Controls
        self.cam.set_control_value(asi.ASI_EXPOSURE, self.exposure_us, auto=False)
        self.cam.set_control_value(asi.ASI_GAIN, self.gain, auto=False)
        if self.is_color:
            self.cam.set_control_value(asi.ASI_WB_R, self.wb_r, auto=False)
            self.cam.set_control_value(asi.ASI_WB_B, self.wb_b, auto=False)

        # ROI & type
        self.cam.set_roi(start_x=0, start_y=0, width=self.width, height=self.height)
        if self.img_type_str == "RAW8":
            self.cam.set_image_type(asi.ASI_IMG_RAW8)
            self.raw_dtype = np.uint8
        else:
            self.cam.set_image_type(asi.ASI_IMG_RAW16)
            self.raw_dtype = np.uint16

        self.cam.start_video_capture()

        # ===== GStreamer writer =====
        self.writer: Optional[cv2.VideoWriter] = None
        self._start_writer()

        # ===== Timer (capture → write) =====
        period = max(1.0 / float(self.fps if self.fps > 0 else 30), 0.001)
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"Recording from {self.props['Name']} {self.width}x{self.height} @ {self.fps} FPS → {self.output_dir}"
        )

    # ---------------- Helpers ----------------
    def _p(self, name):
        p = self.get_parameter(name)
        return p.get_parameter_value().string_value if p.get_parameter_value().type == 4 else p.value

    def _select_encoder(self) -> str:
        if gst_has("nvh264enc"):
            # Desktop NVIDIA
            return (
                f"nvh264enc preset={self.preset} rc-mode=cbr bitrate={self.bitrate_kbps} "
                f"key-int-max={max(self.gop, self.fps*2)} "
            )
        if gst_has("nvv4l2h264enc"):
            # Jetson
            return (
                f"nvv4l2h264enc insert-sps-pps=true iframeinterval={max(self.gop, self.fps*2)} "
                f"bitrate={self.bitrate_kbps*1000} control-rate=1 preset-level=1 ! "
            )
        # CPU fallback
        return (
            "x264enc tune=zerolatency speed-preset=superfast "
            f"bitrate={self.bitrate_kbps} key-int-max={max(self.gop, 60)} byte-stream=false "
        )

    def _gst_pipeline(self) -> str:
        max_size_time_ns = int(self.segment_minutes * 60 * 1e9)
        encoder = self._select_encoder()
        location = os.path.join(self.output_dir, "rec_%Y%m%d-%H%M%S_%05d.mp4")

        pipe = (
            "appsrc is-live=true do-timestamp=true format=time ! "
            f"video/x-raw,format=BGR,width={self.width},height={self.height},framerate={self.fps}/1 ! "
            "videoconvert ! video/x-raw,format=NV12 ! "
            f"{encoder}"
            "h264parse config-interval=1 ! "
            f"splitmuxsink muxer=mp4mux max-size-time={max_size_time_ns} "
            "muxer-properties=fragment-duration=2000,faststart=true "
            f"location=\"{location}\""
        )
        return pipe

    def _start_writer(self):
        pipeline = self._gst_pipeline()
        self.writer = cv2.VideoWriter(
            pipeline, cv2.CAP_GSTREAMER, 0, float(self.fps if self.fps > 0 else 30),
            (self.width, self.height), True
        )
        if not self.writer or not self.writer.isOpened():
            raise RuntimeError(
                "Failed to open GStreamer writer. "
                "Check GStreamer install and plugins (nvh264enc/nvv4l2h264enc/x264enc, mp4mux, splitmuxsink)."
            )

    def _capture_raw(self) -> Optional[np.ndarray]:
        try:
            data = self.cam.capture_video_frame(timeout=2000)  # bytes
        except Exception as e:
            self.get_logger().warn(f"Capture timeout: {e}")
            return None
        return np.frombuffer(data, dtype=self.raw_dtype).reshape(self.height, self.width)

    def _demosaic_to_bgr(self, raw: np.ndarray) -> np.ndarray:
        # RAW16 → 8-bit для демозаїку
        raw8 = (raw >> 8).astype(np.uint8) if raw.dtype == np.uint16 else raw
        code = {
            "RGGB": cv2.COLOR_BAYER_RG2BGR,
            "BGGR": cv2.COLOR_BAYER_BG2BGR,
            "GRBG": cv2.COLOR_BAYER_GR2BGR,
            "GBRG": cv2.COLOR_BAYER_GB2BGR,
        }.get(self.bayer_pattern, cv2.COLOR_BAYER_RG2BGR)
        return cv2.cvtColor(raw8, code)

    # ---------------- Main loop ----------------
    def _on_timer(self):
        raw = self._capture_raw()
        if raw is None:
            return

        if self.is_color:
            frame_bgr = self._demosaic_to_bgr(raw)
        else:
            # моно → BGR для запису (дублюємо канали)
            mono8 = (raw >> 8).astype(np.uint8) if raw.dtype == np.uint16 else raw
            frame_bgr = cv2.cvtColor(mono8, cv2.COLOR_GRAY2BGR)

        self.writer.write(frame_bgr)

    # ---------------- Cleanup ----------------
    def destroy_node(self):
        try:
            if self.writer is not None:
                self.writer.release()
        except Exception:
            pass
        try:
            self.cam.stop_video_capture()
        except Exception:
            pass
        try:
            self.cam.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()

    def _sigint_handler(signum, frame):
        rclpy.shutdown()
    signal.signal(signal.SIGINT, _sigint_handler)

    node = None
    try:
        node = ASIRecorder()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
