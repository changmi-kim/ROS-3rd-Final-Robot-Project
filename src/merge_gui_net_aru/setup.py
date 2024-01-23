from setuptools import find_packages, setup
from glob import glob
import os


package_name = 'merge_gui_net_aru'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jo',
    maintainer_email='sungho9807@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_estimation_test2_pinkbot = my_minibot_aruco_package.pose_estimation_test2_pinkbot:main',
            'the_sub = my_minibot_aruco_package.the_sub:main',
        ],
    },
)
