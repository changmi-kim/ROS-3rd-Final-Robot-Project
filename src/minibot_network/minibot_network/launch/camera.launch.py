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
            get_package_share_directory('minibot_network'),
            'param',
            'camera_config.yaml'
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'param_dir',
                default_value=param_dir
            ),

            Node(
                package='minibot_network',
                executable='minibot_cam_publish',
                name='minibot_cam_publish',
                parameters=[param_dir],
                output='screen'
            ),
        ]
    )