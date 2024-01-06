from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gui_package',
            namespace='gui_node',
            executable='the_pub',
            name='gui_pub'
        ),
        Node(
            package='gui_package',
            namespace='gui_node',
            executable='the_sub',
            name='gui_sub'
        )
    ])