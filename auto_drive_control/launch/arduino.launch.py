import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('auto_drive_control'),
            'param',
            'arduino_config.yaml'
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'param_dir',
                default_value=param_dir
            ),

            Node(
                package='auto_drive_control',
                executable='arduino_protocol_sender',
                name='arduino_protocol_sender',
                parameters=[param_dir],
                output='screen'
            ),


            Node(
                package='auto_drive_control',
                executable='ultrasonic_dist_publisher',
                name='ultrasonic_dist_publisher',
                parameters=[param_dir],
                output='screen'
            ),

            Node(
                package='auto_drive_control',
                executable='ultrasonic_dist_subscliber',
                name='ultrasonic_dist_subscliber',
                parameters=[param_dir],
                output='screen'
            ),
        ]
    )
