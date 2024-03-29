from setuptools import find_packages, setup

package_name = 'vision_object_detector'

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
            'img_publisher = vision_object_detector.img_publisher:main',
            'parking_obstacle_detector_yolo_v8 = vision_object_detector.parking_obstacle_detector_yolo_v8:main'
        ],
    },
)
