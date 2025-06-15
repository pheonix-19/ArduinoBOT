from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction



def generate_launch_description():

    robot_description = ParameterValue(Command([
            "xacro ",
            os.path.join(get_package_share_directory("arduinobot_description"), "urdf", "arduino-bot.urdf.xacro"),
            
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description},
                    os.path.join(get_package_share_directory("arduinobot_controller"), "config", "arduinobot_controllers.yaml")],
        output="screen"
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher_node,
        # controller_manager,
        joint_state_broadcaster_spawner,
        TimerAction(period=5.0, actions=[arm_controller_spawner]),  # 3 second delay
        TimerAction(period=6.0, actions=[gripper_controller_spawner]),  # 3.5 second delay
    ])
