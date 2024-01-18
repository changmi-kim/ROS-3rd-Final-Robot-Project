from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'minibot_network'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch' '*.launch.py'))),
        ('share/' + package_name + '/param', glob(os.path.join('param' '*.yaml'))),
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
            'minibot_cam_publish = minibot_network.minibot_cam_publish:main',
            'minibot_cam_subscriber = minibot_network.minibot_cam_subscriber:main',
            'main_client = minibot_network.main_client:main',
            'main_server = minibot_network.main_server:main',
        ],
    },
)
