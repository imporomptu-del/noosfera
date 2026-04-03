from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'seaqr_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='a',
    maintainer_email='a@todo.todo',
    description='SEAQR Data Collection System - ADSB and Camera Recording',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'adsb_usb = seaqr_controller.adsb_reader.adsb_usb:main',
            'camera_recorder_node = seaqr_controller.camera_reader.camera_recorder_node:main',
            'camera_diagnostic = seaqr_controller.camera_reader.camera_diagnostic:main',
        ],
    },
)
