import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'communication'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jo',
    maintainer_email='sungho9807@gmail.com',
    description='서로 다른 도메인을 가진 로봇과 통신',
    license='jo',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_talker = communication.controller_talker:main',
            'controller_listener = communication.controller_listener:main',
            'minibot_talker = communication.minibot_talker:main',
            'minibot_listener = communication.minibot_listener:main',
            'connect_ros = communication.connect:main',
        ],
    },
)
