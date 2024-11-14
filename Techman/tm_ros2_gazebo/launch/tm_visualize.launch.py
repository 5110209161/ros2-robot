#!/usr/bin/python3

# Import libraries:
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "tm_type",
            default_value="tm5-900",
            description="Type/series of used Techman robot",
            choices=["tm5-700", "tm5-900", "tm5x-700", "tm5x-900", "tm12", "tm12x", "tm14", "tm14x"]
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="tm_ros2_gazebo",
            description="Description package with robot URDF/XACRO files. Usually the argument "
            "is not set, it enables use of a custom description.",
        )
    )

    # Initialize Arguments
    tm_type = LaunchConfiguration("tm_type")
    description_package = LaunchConfiguration("description_package")
    
    def create_robot_description(context):
        tm_type_value = context.launch_configurations['tm_type']
        description_file = f"{tm_type_value}.urdf.xacro"
        robot_description_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution([FindPackageShare(description_package), "xacro", description_file]),
                " ",
                "tm_type:=",
                tm_type,
            ]
        )
        robot_description = {"robot_description": robot_description_content}
        return [Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description],
        )]

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_robot.rviz"]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
    robot_state_publisher_node = OpaqueFunction(function=create_robot_description)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    nodes_to_start = [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ]
    
    return LaunchDescription(declared_arguments + nodes_to_start)