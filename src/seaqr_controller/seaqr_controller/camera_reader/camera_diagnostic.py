#!/home/a/ros2/bin python3
from abc import ABC, abstractmethod
import cv2
import os
from datetime import datetime
import sys
from datetime import datetime
import json
from seaqr_controller.camera_reader.unit_test import FPS_testing,  color_format_verification, dump_all_control_values
from seaqr_controller.camera_reader.camera import Camera



os.environ['ZWO_ASI_LIB_PATH'] = '/home/a/ZWO/ASIStudio/lib/libASICamera2.so'
lib_path = '/home/a/ZWO/ASIStudio/lib/libASICamera2.so'
if not os.path.exists(lib_path):
    print(f"⚠️ ASI SDK library not found at {lib_path}")
    sys.exit(1)
os.environ['ZWO_ASI_LIB_PATH'] = lib_path
import zwoasi as asi
import rclpy
from rclpy.node import Node
from my_robot_controller.loger import Publish
from my_robot_controller.config import ConfigLoader

asi.init(lib_path)


CNFGPATH = f'/media/a/E/MyProjects/Water/ros2_ws/src/my_robot_controller/config/cameras.yaml'
    

class CameraDiagnostic(Camera):
    def __init__(self, camera_id, camera_name):
        super().__init__(camera_id, camera_name )


    def run_unit_test(self):
        test_set = [FPS_testing(self.asi_camera), 
                    color_format_verification(self.asi_camera), 
                    dump_all_control_values(self.asi_camera)]
        for test in test_set:
            test.run()


    def bayer_to_rgb_map(self, frame, node):
        bayer_pattern = self.asi_camera.get_camera_property().get('BayerPattern', 'RG')
        bayer_to_rgb_map = {
            'RG': cv2.COLOR_BayerRG2RGB,
            'BG': cv2.COLOR_BayerBG2RGB,
            'GR': cv2.COLOR_BayerGR2RGB,
            'GB': cv2.COLOR_BayerGB2RGB
        }
        convert_code = bayer_to_rgb_map.get(bayer_pattern, cv2.COLOR_BayerRG2RGB)

        if self.asi_camera.get_camera_property()['IsColorCam']:
            rgb_frame = cv2.cvtColor(frame, convert_code)
            node.log_and_publish(f"✅ Demosaiced color image shape: {rgb_frame.shape}")
        else:
            rgb_frame = frame
        return rgb_frame
    



    def perform_diagnostics(self, node):
        node.log_and_publish(f"Initializing diagnostics for {self.camera} (ID: {self.camera_id})")

        # Keep stream open for both first frame and unit tests
        self.start_stream()
        try:
            frame = self.grab_frame(timeout=5000)
            if frame is None:
                node.log_and_publish("❌ Failed to capture frame (None).")
                return

            node.log_and_publish(f"Frame shape: {frame.shape}")

            rgb_frame = self.bayer_to_rgb_map(frame, node)
            filename = f"captured_frame_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
            if cv2.imwrite(filename, rgb_frame):
                node.log_and_publish(f"✅ Frame saved as: {filename}")
            else:
                node.log_and_publish(f"❌ Failed to save frame to: {filename}")

            # Run tests while stream is still active (prevents timeouts)
            self.run_unit_test()

        finally:
            # Stop stream before closing device
            self.stop_stream()
            self.close()

 



def main(args = None):
    rclpy.init(args = args)
    node = Publish()
    node.send_logs("\n=== Diagnostic Starts ===")
    num_cameras = asi.get_num_cameras()
    node.send_logs(f"Number of cameras detected: {num_cameras}")
    print(f"Number of cameras detected: {num_cameras}")
    

    # List camera names and properties
    if num_cameras > 0:
        camera_names = asi.list_cameras()
        
        print("Detected Cameras:")
        node.send_logs(f'Detected Cameras:')

        for i, name in enumerate(camera_names):
            print(f'[{i}] {name}')
            camera = asi.Camera(i)
            props = camera.get_camera_property()

            msg = (
                f"Camera name: {name}, Camera ID: {i}, Resolution: {props['MaxWidth']}x{props['MaxHeight']}\n"
                 )
            
            print(f"     Resolution: {props['MaxWidth']}x{props['MaxHeight']}")
            print(f"     ID: {i}")
            
            node.send_logs(msg)
            camera_diagnostic = CameraDiagnostic(i, name)#, 178, 12500, 'asi.ASI_IMG_RAW8', 1)
            #camera_diagnostic
            node.send_logs(json.dumps(camera_diagnostic.load_config))
            camera_diagnostic.perform_diagnostics(node)
  
    else:
        node.send_logs("⚠️  No cameras found. Check USB connection and power.")


    node.send_logs("\n=== Diagnostic Complete ===")
    rclpy.shutdown()


if __name__ == "__main__":
    main()