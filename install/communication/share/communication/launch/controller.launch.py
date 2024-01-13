from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def generate_launch_description():
    talker_name = LaunchConfiguration('talker_name')
    listener_name = LaunchConfiguration('listener_name')

    declare_talker_name = DeclareLaunchArgument(
        'talker_name',
        default_value='Controller_talker',
        description='Name for the talker node'
    )

    declare_listener_name = DeclareLaunchArgument(
        'listener_name',
        default_value='Controller_listener',
        description='Name for the listener node'
    )

    talker_node = Node(
        package='communication',
        name=talker_name,
        executable='controller_talker',
        output='screen',
    )

    listener_node = Node(
        package='communication',
        name=listener_name,
        executable='controller_listener',
        output='screen',
    )

    return LaunchDescription([
        declare_talker_name,
        declare_listener_name,
        talker_node,
        listener_node,
    ])