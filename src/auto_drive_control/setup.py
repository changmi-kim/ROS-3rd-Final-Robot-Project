from setuptools import find_packages, setup

package_name = 'auto_drive_control'

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
    maintainer='ckdal',
    maintainer_email='rlackdal5320@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'arduino_protocol_sender = auto_drive_control.arduino_protocol_sender:main',
        'EVCS_navigator_BasicNavigator = auto_drive_control.EVCS_navigator_BasicNavigator:main',
        'vision_object_tracker = vision_object_detector.vision_object_tracker:main',
        'vision_obstacle_avoidance = vision_object_detector.vision_obstacle_avoidance:main',
        'lidar = auto_drive_control.lidar:main'        
        ],
    },
)
