from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_minibot_aruco_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kang',
    maintainer_email='qydcjf93@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_minibot_aruco_node = my_minibot_aruco_package.my_minibot_aruco_node:main',
            'test_node = my_minibot_aruco_package.test_node:main',
            'the_sub = my_minibot_aruco_package.the_sub:main',
            'the_pub = my_minibot_aruco_package.the_pub:main',
            'pi_cam_node = my_minibot_aruco_package.pi_cam_node:main',
            'visual = my_minibot_aruco_package.visual:main',
            'calibrate= my_minibot_aruco_package.calibrate:main', 
            'pose_estimation = my_minibot_aruco_package.pose_estimation:main',
            'pose_estimation_test2_pinkbot = my_minibot_aruco_package.pose_estimation_test2_pinkbot:main',
            'pose_estimation_test1 = my_minibot_aruco_package.pose_estimation_test1:main',  
            'aruco_processing = my_minibot_aruco_package.aruco_processing:main',
            'pose_estimation_test2 = my_minibot_aruco_package.pose_estimation_test2:main',
            'visionTrackor = my_minibot_aruco_package.visionTrackor:main',
            'aruco_minibit_cmd_and_pose = my_minibot_aruco_package.aruco_minibit_cmd_and_pose:main',
            'minibit_cmd_and_pose = my_minibot_aruco_package.minibit_cmd_and_pose:main'


        ],
    },
)
