from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    urdf = PathJoinSubstitution([
        FindPackageShare("qr_robot_description"),
        "urdf",
        "qr_robot_description.urdf"
    ])

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "robot_description": ParameterValue(
                    Command(["xacro ", urdf]),
                    value_type=str
                )
            }],
            output="screen"
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen"
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen"
        )
    ])