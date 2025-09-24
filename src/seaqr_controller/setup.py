from setuptools import find_packages, setup

package_name = 'seaqr_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='a',
    maintainer_email='a@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_node = seaqr_controller.adsb:main",
            "adsb_data = seaqr_controller.adsb_usb:main",
            "camera_diagnostic = seaqr_controller.camera_reader.camera_diagnostic:main",
            "camera_listener = seaqr_controller.loger:main",
            "camera_vs_publisher = seaqr_controller.camera_reader.camera_vs_publisher:main",
        ],
    },
)

