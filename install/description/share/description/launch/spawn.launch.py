import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():

    # Check if we're told to use sim time
    robot_name = LaunchConfiguration('robot_name')
    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('description'))
    xacro_file = os.path.join(pkg_path,'urdf','robot.urdf.xacro')
    # robot_description_config = xacro.process_file(xacro_file).toxml()
    robot_description_config = Command(['xacro ', xacro_file, ' robot_name:=', robot_name, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-entity', robot_name,
            '-topic', 'robot_description',
            '-timeout', '20.0',
            '-x', robot_x,
            '-y', robot_y,
            '-package_to_model'
        ],
        prefix="bash -c 'sleep 2.0; $0 $@' ",
        parameters=[{
            "use_sim_time": True
        }],
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'joint_state_broadcaster'],
        output='screen'
    )

    load_base_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'base_controller'],
        output='screen'
    )


    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'),

        DeclareLaunchArgument(
            'robot_name',
            default_value='Minibot',
            description='write robot name'),

        DeclareLaunchArgument(
            'robot_x',
            default_value='0.4',
            description='로봇 x좌표'),

        DeclareLaunchArgument(
            'robot_y',
            default_value='0.4',
            description='로봇 y좌표'),

        node_robot_state_publisher,
        spawn_robot
    ])