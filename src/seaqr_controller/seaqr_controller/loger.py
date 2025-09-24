#!/home/a/ros2/bin python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import datetime
import logging
import os
import json
#LOG_PATH = f"/media/a/E/MyProjects/Water/ros2_ws/logs/cam/logs/cam_log_{datetime.now().date()}.csv"
logger = logging.getLogger(__name__)

class Publish(Node):
    def __init__(self):
        super().__init__('log_topic')
        self.loger = self.create_publisher(String, 'log_topic', 10)
        self.log_dir = "/media/a/E/MyProjects/Water/ros2_ws/logs/cam/"
        os.makedirs(self.log_dir, exist_ok=True)

    def send_logs(self, message):
        msg = String()
        msg.data = message
        self.loger.publish(msg)


    def log_and_publish(self, message: str, level: str = "info"):
        message = str(message)

        if level == "info":
            logger.info(message)
        elif level == "debug":
            logger.debug(message)
        elif level == "warning":
            logger.warning(message)
        elif level == "error":
            logger.error(message)
        else:
            logger.info(message)

        self.send_logs(message) 

        self.save_logs_(message, '/media/a/E/MyProjects/Water/ros2_ws/logs/cam/')






    def save_logs_(self, message, destination_path):
        """Save a log message to cam_<YYYY-MM-DD>.log in destination_path (UTC date) with milliseconds."""
        os.makedirs(destination_path, exist_ok=True)

        # File name based on current UTC date
        date_str = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%d")
        filename = f"cam_{date_str}.log"
        path = os.path.join(destination_path, filename)

        # Timestamp with milliseconds in UTC
        utc_ts = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        with open(path, "a", encoding="utf-8") as f:
            f.write(f"[{utc_ts} UTC] {message}\n")




class LogSubscriber(Node):

    def __init__(self):
        super().__init__('pose_suscriber')
        self.subscription = self.create_subscription(String, 'log_topic',
            self.listener_callback,
            10)
        self.log_file = None #open(LOG_PATH, mode='a', newline='')


    def listener_callback(self, msg:String):
        try:
            self.get_logger().info(msg.data)
        
        except:
            pass

        try:
            import json
            cfg = json.loads(msg.data)

            # If it's a list of camera configs, pretty-print each one
            if isinstance(cfg, list) and all(isinstance(c, dict) for c in cfg):
                for cam in cfg:
                    self.get_logger().info(f"{cam['name']} (id={cam['id']})")
                    for k, v in cam.get('settings', {}).items():
                        self.get_logger().info(f"  {k}: {v}")
                return  # Done, skip normal log printing

        except json.JSONDecodeError:
            pass  # Not JSON, just log it normally



    def destroy_node(self):
        self.log_file.close()
        super().destroy_node()  


    def cb(self, msg: String):
        
        cfg = json.loads(msg.data)
        for cam in cfg:
            self.get_logger().info(f"{cam['name']} (id={cam['id']})")
            for k, v in cam.get('settings', {}).items():
                self.get_logger().info(f"  {k}: {v}")    
          

def main(args=None):
    rclpy.init(args=args)
    node = LogSubscriber()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()