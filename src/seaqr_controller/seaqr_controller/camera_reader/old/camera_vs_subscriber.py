#!/usr/bin/env python3
import os
import glob
import time
import yaml
import cv2
import signal
import subprocess
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


def gst_has(element: str) -> bool:
    """Return True if a GStreamer element exists on this system."""
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


class VideoPublisher(Node):
    def __init__(self):
        super().__init__("video_publisher")

        # ===== Parameters =====
        self.declare_parameter("video_source", "/dev/video10")  # device path or file
        self.declare_parameter("camera_info_file", "")
        self.declare_parameter("output_dir", "recordings")
        self.declare_parameter("fps", 30)
        self.declare_parameter("bitrate_kbps", 12000)
        self.declare_parameter("gop", 60)  # keyframe interval (frames)
        self.declare_parameter("preset", "llhp")  # NVENC preset; ignored for x264
        self.declare_parameter("segment_minutes", 5)

        # Fetch parameters
        self.video_source = self._get_param("video_source")
        self.output_dir = self._get_param("output_dir")
        self.fps = int(self._get_param("fps"))
        self.bitrate_kbps = int(self._get_param("bitrate_kbps"))
        self.gop = int(self._get_param("gop"))
        self.preset = self._get_param("preset")
        self.segment_minutes = int(self._get_param("segment_minutes"))
        os.makedirs(self.output_dir, exist_ok=True)

        # ===== Publishers =====
        qos = 10
        self.image_pub = self.create_publisher(Image, "camera/image_raw", qos)
        self.info_pub = self.create_publisher(CameraInfo, "camera/camera_info", qos)

        # ===== Capture =====
        self.cap = cv2.VideoCapture(self.video_source)
        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open video source: {self.video_source}")
            raise RuntimeError("Video open failed")

        # Probe one frame to get size; reset for live devices if needed
        ok, probe = self.cap.read()
        if not ok:
            self.get_logger().error("Could not grab initial frame for size probe.")
            raise RuntimeError("Initial read failed")

        self.height, self.width = probe.shape[:2]

        # If FPS not set, try to infer
        if self.fps <= 0:
            src_fps = self.cap.get(cv2.CAP_PROP_FPS) or 0
            self.fps = int(src_fps) if src_fps > 0 else 30

        self.bridge = CvBridge()
        self.camera_info_msg = self._load_camera_info(self._get_param("camera_info_file"))

        # ===== GStreamer writer (created lazily) =====
        self.writer: Optional[cv2.VideoWriter] = None

        # For “file result” preview on logs
        self.last_listing_ts = 0.0

        # ===== Timer loop =====
        self.timer = self.create_timer(max(1.0 / float(self.fps), 0.001), self._on_timer)

        self.get_logger().info(
            f"Started: src={self.video_source} size={self.width}x{self.height} "
            f"fps={self.fps} outdir={self.output_dir}"
        )

    # -------- Helpers --------
    def _get_param(self, name):
        p = self.get_parameter(name)
        # Handle string/int typed params gracefully
        val = p.get_parameter_value().string_value if p.get_parameter_value().type == 4 else p.value
        return val

    def _load_camera_info(self, filename: str) -> CameraInfo:
        msg = CameraInfo()
        if not filename:
            self.get_logger().warn("No camera_info_file specified; using empty calibration.")
            msg.width = self.width
            msg.height = self.height
            return msg
        try:
            with open(filename, "r") as f:
                calib = yaml.safe_load(f)
            msg.width = calib.get("image_width", self.width)
            msg.height = calib.get("image_height", self.height)
            msg.k = calib.get("camera_matrix", {}).get("data", [0.0] * 9)
            msg.d = calib.get("distortion_coefficients", {}).get("data", [])
            msg.r = calib.get("rectification_matrix", {}).get("data", [0.0] * 9)
            msg.p = calib.get("projection_matrix", {}).get("data", [0.0] * 12)
            msg.distortion_model = calib.get("distortion_model", "plumb_bob")
        except Exception as e:
            self.get_logger().warn(f"Failed to load camera_info_file '{filename}': {e}")
            msg.width = self.width
            msg.height = self.height
        return msg

    def _select_encoder(self) -> str:
        """
        Choose the best available H.264 encoder in this order:
        - nvh264enc (desktop NVIDIA)
        - nvv4l2h264enc (Jetson)
        - x264enc (CPU fallback)
        """
        if gst_has("nvh264enc"):
            return (
                f"nvh264enc preset={self.preset} rc-mode=cbr bitrate={self.bitrate_kbps} "
                f"key-int-max={max(self.gop, self.fps*2)} "  # keyframe ≤ ~2s
            )
        if gst_has("nvv4l2h264enc"):
            # Good default for Jetson
            return (
                f"nvv4l2h264enc insert-sps-pps=true iframeinterval={max(self.gop, self.fps*2)} "
                f"bitrate={self.bitrate_kbps*1000} control-rate=1 preset-level=1 ! "
            )
        # CPU fallback
        return (
            "x264enc tune=zerolatency speed-preset=superfast key-int-max=60 "
            f"bitrate={self.bitrate_kbps} byte-stream=false "
        )

    def _gst_pipeline(self) -> str:
        """Build GStreamer pipeline string for OpenCV VideoWriter with timed MP4 splits."""
        max_size_time_ns = int(self.segment_minutes * 60 * 1e9)
        encoder = self._select_encoder()
        location = os.path.join(self.output_dir, "rec_%Y%m%d-%H%M%S_%05d.mp4")

        # OpenCV provides appsrc under CAP_GSTREAMER; caps are critical for timing & colorspace
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

    def _start_writer(self) -> bool:
        pipeline = self._gst_pipeline()
        self.writer = cv2.VideoWriter(
            pipeline, cv2.CAP_GSTREAMER, 0, float(self.fps), (self.width, self.height), True
        )
        if not self.writer or not self.writer.isOpened():
            self.get_logger().error(
                "Failed to open GStreamer writer. "
                "Ensure OpenCV has GStreamer support and required plugins are installed "
                "(check: `gst-inspect-1.0 nvh264enc nvv4l2h264enc x264enc mp4mux splitmuxsink`)."
            )
            self.writer = None
            return False
        self.get_logger().info("Recording started (segmented MP4).")
        return True

    def _log_latest_files(self):
        """Periodically log the newest few files so you can see 'file result'."""
        now = time.time()
        if now - self.last_listing_ts < 30:  # list at most every 30s
            return
        self.last_listing_ts = now
        files = sorted(glob.glob(os.path.join(self.output_dir, "rec_*.mp4")))
        if not files:
            self.get_logger().info("No MP4 segments yet...")
            return
        tail = files[-3:]
        self.get_logger().info("Latest files:\n  " + "\n  ".join(tail))

    # -------- Main loop --------
    def _on_timer(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("End of stream or read error; stopping.")
            self._cleanup()
            rclpy.shutdown()
            return

        if self.writer is None:
            if not self._start_writer():
                # continue publishing even if recording failed
                pass

        if self.writer is not None:
            self.writer.write(frame)

        # Publish to ROS
        stamp = self.get_clock().now().to_msg()
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = stamp
        msg.header.frame_id = "camera_frame"
        self.camera_info_msg.header.stamp = stamp
        self.camera_info_msg.header.frame_id = "camera_frame"
        self.image_pub.publish(msg)
        self.info_pub.publish(self.camera_info_msg)

        # Occasionally show “file result”
        self._log_latest_files()

    def _cleanup(self):
        try:
            if self.writer is not None:
                self.writer.release()
                self.writer = None
            if self.cap is not None and self.cap.isOpened():
                self.cap.release()
        except Exception as e:
            self.get_logger().warn(f"Cleanup error: {e}")


def main():
    rclpy.init()

    # Graceful Ctrl+C
    def _sigint_handler(signum, frame):
        rclpy.shutdown()
    signal.signal(signal.SIGINT, _sigint_handler)

    node = None
    try:
        node = VideoPublisher()
        rclpy.spin(node)
    except Exception as e:
        if node:
            node.get_logger().error(f"Fatal error: {e}")
    finally:
        if node:
            node._cleanup()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
