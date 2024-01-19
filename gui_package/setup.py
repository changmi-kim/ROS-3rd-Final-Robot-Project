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
            'gui_4_controlPC = gui_package.gui_4_controlPC:main',
            'camera_test = gui_package.camera_test:main',
            'gui_4_kiosk = gui_package.gui_4_kiosk:main'
        ],
    },
)
