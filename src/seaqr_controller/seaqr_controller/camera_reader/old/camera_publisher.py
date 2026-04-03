## Script does wokr only with OPENCV no GSTRAEMR
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import yaml
import time
from seaqr_controller.camera_reader.camera import Camera
import os
from datetime import datetime

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')

        # Params: video source and calibration file
        self.declare_parameter('video_source', 0)  # 0 = webcam, or path to file
        self.declare_parameter('camera_info_file', '')
        self.segment_duration = 5  # 5 minutes in seconds  ### NEW ###
        self.segment_start_time = time.time()               ### NEW ###
        self.video_writer = None                            ### NEW ###
        self.output_dir = "recordings"                      ### NEW ###
        os.makedirs(self.output_dir, exist_ok=True)         ### NEW ###

        video_source = self.get_parameter('video_source').get_parameter_value().string_value or \
                       self.get_parameter('video_source').value

        # Publishers
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, 'camera/camera_info', 10)

        # Video capture
        self.cap = cv2.VideoCapture(video_source)
        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open video source: {video_source}")
            rclpy.shutdown()
            return

        self.bridge = CvBridge()
        self.camera_info_msg = self.load_camera_info(self.get_parameter('camera_info_file').value)

        # Timer (publish at ~30 FPS)
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)


    def start_new_segment(self, frame):  ### NEW ###
        if self.video_writer:
            self.video_writer.release()
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(self.output_dir, f"video_{timestamp}.mp4")
        height, width = frame.shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.video_writer = cv2.VideoWriter(filename, fourcc, 30, (width, height))
        self.segment_start_time = time.time()
        self.get_logger().info(f"Started new segment: {filename}")    

    def load_camera_info(self, filename):
        if not filename:
            self.get_logger().warn("No camera_info_file specified. Using empty calibration.")
            return CameraInfo()
        with open(filename, 'r') as f:
            calib_data = yaml.safe_load(f)
        msg = CameraInfo()
        msg.width = calib_data['image_width']
        msg.height = calib_data['image_height']
        msg.k = calib_data['camera_matrix']['data']
        msg.d = calib_data['distortion_coefficients']['data']
        msg.r = calib_data['rectification_matrix']['data']
        msg.p = calib_data['projection_matrix']['data']
        msg.distortion_model = calib_data['distortion_model']
        return msg

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info("End of video or cannot grab frame.")
            rclpy.shutdown()
            return

        if self.video_writer is None:  ### NEW ###
            self.start_new_segment(frame)

        if time.time() - self.segment_start_time >= self.segment_duration:  ### NEW ###
            self.start_new_segment(frame)

        self.video_writer.write(frame)  ### NEW ###

        stamp = self.get_clock().now().to_msg()
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = 'camera_frame'

        self.camera_info_msg.header.stamp = stamp
        self.camera_info_msg.header.frame_id = 'camera_frame'

        self.image_pub.publish(img_msg)
        self.info_pub.publish(self.camera_info_msg)

def main():
    rclpy.init()
    node = VideoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()