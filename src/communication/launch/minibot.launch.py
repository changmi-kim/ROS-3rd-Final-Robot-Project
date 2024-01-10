from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    return LaunchDescription([
        Node(
            package='communication',
            executable='minibot_talker',
            output='screen'
        ),

        Node(
            package='communication',
            executable='minibot_listener',
            output='screen'
        ),
    ])