#!/home/a/ros2/bin python3
from abc import ABC, abstractmethod
import cv2
import os
from datetime import datetime
import sys
from datetime import datetime
import json
from seaqr_controller.camera_reader.unit_test import FPS_testing,  color_format_verification, dump_all_control_values



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

class Camera:
    def __init__(self, camera_id, camera_name):#:, camera_name, gain, exposure, type, ASI_HIGH_SPEED_MODE):
        self.camera = camera_name
        self.camera_id = camera_id
        self.load_config = self.get_config()

        self.config = self.get_cam_by_id(self.camera)

        # Assign common settings
        self.GAIN                     = self.config.get('Gain')
        self.EXPOSURE                 = self.config.get('Exposure')
        self.OFFSET                   = self.config.get('Offset')
        self.BANDWIDTH                = self.config.get('BandWidth')
        self.FLIP                     = self.config.get('Flip')
        self.AUTO_EXP_MAX_GAIN        = self.config.get('AutoExpMaxGain')
        self.AUTO_EXP_MAX_EXPMS       = self.config.get('AutoExpMaxExpMS')
        self.AUTO_EXP_TARGET_BRIGHTNESS = self.config.get('AutoExpTargetBrightness')
        self.ASI_HIGH_SPEED_MODE      = self.config.get('HighSpeedMode')

        
        # Optional color camera params  
        self.WB_R = self.config.get('WB_R', None)
        self.WB_B = self.config.get('WB_B', None) 

                # Open hardware and apply configuration
        self.asi_camera = asi.Camera(camera_id)
        self._apply_settings() 





    def _apply_settings(self):
        camera_info = self.asi_camera.get_camera_property()
        max_width = camera_info['MaxWidth']
        max_height = camera_info['MaxHeight']

        self.asi_camera.set_roi(width=max_width, height=max_height)
        self.asi_camera.set_control_value(asi.ASI_HIGH_SPEED_MODE, self.ASI_HIGH_SPEED_MODE)
        #self.asi_camera.set_control_value(asi.ASI_EXPOSURE, self.EXPOSURE)
        #self.asi_camera.set_control_value(asi.ASI_GAIN, self.GAIN)
        self.asi_camera.set_control_value(asi.ASI_EXPOSURE, 0, True)# self.EXPOSURE)
        self.asi_camera.set_control_value(asi.ASI_GAIN, 0, True)
        self.asi_camera.set_control_value(asi.ASI_BANDWIDTHOVERLOAD, self.BANDWIDTH)
        self.asi_camera.set_control_value(asi.ASI_GAMMA, 50)

        if 'WhiteBalance_R' in self.asi_camera.get_controls() and self.WB_R is not None:
            self.asi_camera.set_control_value(asi.ASI_WB_R, self.WB_R)
        if 'WhiteBalance_B' in self.asi_camera.get_controls() and self.WB_B is not None:
            self.asi_camera.set_control_value(asi.ASI_WB_B, self.WB_B)




    # Explicit stream control so callers decide when to keep it open
    def start_stream(self):
        self.asi_camera.start_video_capture()

    def grab_frame(self, timeout=5000):
        return self.asi_camera.capture_video_frame(timeout=timeout)

    def stop_stream(self):
        self.asi_camera.stop_video_capture()

    def close(self):
        # IMPORTANT: never assign anything to self.asi_camera.close to avoid shadowing
        self.stop_stream()  # safe even if not started
        self.asi_camera.close() 


    def get_config(self):
        general_settings = ConfigLoader(CNFGPATH)
        ALSH = general_settings.get('cameras')

        return(ALSH)

    def get_cam_by_id(self, cam_name):

        cam_config = next((cam for cam in self.load_config if cam.get("name") == self.camera), None)
        return cam_config.get('settings')
