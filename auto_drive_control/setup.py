from setuptools import find_packages, setup

import os
import glob

package_name = 'auto_drive_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', '*.launch.py'))),
        ('share/' + package_name + '/param', glob.glob(os.path.join('param', '*.yaml'))),
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
        'ultrasonic_dist_publisher = auto_drive_control.ultrasonic_dist_publisher:main',
        'ultrasonic_dist_subscliber = auto_drive_control.ultrasonic_dist_subscliber:main',
        'arduino_protocol_sender = auto_drive_control.arduino_protocol_sender:main',
        'connect_arduino = auto_drive_control.connect_arduino:main',
        ],
    },
)
