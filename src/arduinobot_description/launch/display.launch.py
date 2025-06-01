import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare the model argument
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(get_package_share_directory('arduino_bot_description'), 'urdf', 'arduino-bot.urdf.xacro'),
        description='Absolute path to robot URDF file'
    )

    # Define the robot_description parameter using xacro
    robot_description = {'robot_description': Command(['xacro ', LaunchConfiguration('model')])}

    # Define the nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[robot_description]
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('arduino_bot_description'), 'rviz', 'arduino_bot.rviz')]
    )

    # Return the LaunchDescription
    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz
    ])
