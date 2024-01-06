from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gui_package'

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
    maintainer='wintercamo',
    maintainer_email='wintercamo3482@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_node = gui_package.test_node:main',
            'the_sub = gui_package.the_sub:main',
            'the_pub = gui_package.the_pub:main',
            'camera_viewer = your_package_name.camera_viewer:main',
            'visual = gui_package.visual:main',
        ],
    },
)
