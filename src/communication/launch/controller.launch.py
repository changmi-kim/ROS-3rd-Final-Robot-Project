from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'talker_name',
            default_value='Main_PC_talker',
            description='Name for the talker node'
        ),
        DeclareLaunchArgument(
            'listener_name',
            default_value='Main_PC_listener',
            description='Name for the listener node'
        ),
        Node(
            package='communication',
            executable='controller_talker',
            output='screen',
            name=LaunchConfiguration('talker_name')
        ),

        Node(
            package='communication',
            executable='controller_listener',
            output='screen',
            name=LaunchConfiguration('listener_name')
        ),
    ])