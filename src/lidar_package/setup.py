from setuptools import find_packages, setup

package_name = 'lidar_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'sensor_msgs', 'geometry_msgs'],
    zip_safe=True,
    maintainer='sj',
    maintainer_email='psj961031@gmail.com',
    description='This package provides a LIDAR sensor interface for ROS2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_node = lidar_package.sensor_node:main',
            'auto_navigation = lidar_package.auto_navigation:main',
            'sec_auto_navigation = lidar_package.sec_auto_navigation:main'
        ],
    },
)
