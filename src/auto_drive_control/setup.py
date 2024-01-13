from setuptools import find_packages, setup

package_name = 'gui_package'

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
    maintainer='wintercamo',
    maintainer_email='wintercamo3482@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
<<<<<<<< HEAD:src/gui_package/setup.py
            'gui_node = gui_package.gui_node:main',
            'camera_test = gui_package.camera_test:main',
            'cam_listener = gui_package.cam_listener:main',
========
        'arduino_protocol_sender = auto_drive_control.arduino_protocol_sender:main'
>>>>>>>> auto_control:src/auto_drive_control/setup.py
        ],
    },
)
