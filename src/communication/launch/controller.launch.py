from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    return LaunchDescription([
        Node(
            package='communication',
            executable='controller_talker',
            output='screen'
        ),

        Node(
            package='communication',
            executable='controller_listener',
            output='screen'
        ),
    ])