#!/home/a/ros2/bin python
from abc import ABC, abstractmethod
from collections.abc import  Sequence
import time
import logging
import zwoasi as asi
from my_robot_controller.loger import Publish
logger = logging.getLogger(__name__)


class CameraUnitTest(ABC):
    def __init__(self, camera):#, test_list: list) -> None:

        self.camera = camera


    @abstractmethod
    def run(self):
        pass


class FPS_testing(CameraUnitTest):
    def run(self):

        node = Publish()
        start = time.time()
        frame_count = 0
        
        for _ in range(30):
            frame = self.camera.capture_video_frame(timeout=5000)
            if frame is not None:
                frame_count += 1
        elapsed = time.time() - start
        fps = frame_count / elapsed

        print(f"Measured FPS: {fps:.2f}")
        logger.info(f"Measured FPS: {fps:.2f}")
        node.log_and_publish(f"✅ Measured FPS: {fps:.2f}") 

        return fps
    
class color_format_verification(CameraUnitTest):
    def run(self):

        node = Publish()
        #print("Supported image formats:")
        node.log_and_publish(f"Supported image formats:") 
        for img_type in [asi.ASI_IMG_RAW8, asi.ASI_IMG_RGB24, asi.ASI_IMG_Y8]:
            try:
                self.camera.set_image_type(img_type)
                print(f"  ✓ {img_type}")
                node.log_and_publish(f"  ✓ {img_type}")
            except:
                print(f"  ✗ {img_type} not supported")
                node.log_and_publish(f"  ✗ {img_type} not supported")



class dump_all_control_values(CameraUnitTest):
        
        def run(self):

            node = Publish()
            controls = self.camera.get_controls()
            print("\nCamera Controls:")
            node.log_and_publish("\nCamera Controls:") 
            for control_name, control in controls.items():
                current_value = self.camera.get_control_value(control['ControlType'])[0]
                print(f"  {control_name}: {current_value} (Min: {control['MinValue']}, Max: {control['MaxValue']}, Default: {control['DefaultValue']})")
                logger.info(f"  {control_name}: {current_value} (Min: {control['MinValue']}, Max: {control['MaxValue']}, Default: {control['DefaultValue']})")
                node.log_and_publish(f"  {control_name}: {current_value} (Min: {control['MinValue']}, Max: {control['MaxValue']}, Default: {control['DefaultValue']})") 




