#!/usr/bin/python3

# Import libraries:
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

# TODO still can not run Gazebo
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
    declared_arguments.append(
      DeclareLaunchArgument(
        "controllers_file",
        default_value="tm_controllers.yaml",
        description="YAML file with the controllers configuration"
      )
    )
    declared_arguments.append(
      DeclareLaunchArgument(
        "gazebo_gui", default_value="true", description="Start Gazebo with GUI"
      )
  )

    declared_arguments.append(
      DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )

    # Initialize Arguments
    tm_type = LaunchConfiguration("tm_type")
    description_package = LaunchConfiguration("description_package")
    controllers_file = LaunchConfiguration("controllers_file")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    launch_rviz = LaunchConfiguration("launch_rviz")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")
    
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
          tm_type
        ]
      )
      robot_description = {"robot_description": robot_description_content}
      return [Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": True},],
      )]

    robot_state_publisher_node = OpaqueFunction(function=create_robot_description)

    joint_state_broadcaster_spawner = Node(
      package="controller_manager",
      executable="spawner",
      arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    rviz_config_file = PathJoinSubstitution(
      [FindPackageShare(description_package), "rviz", "view_robot.rviz"]
    )

    rviz_node = Node(
      package="rviz2",
      executable="rviz2",
      name="rviz2",
      output="log",
      arguments=["-d", rviz_config_file],
    )

    gazebo = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
      ),
      launch_arguments={"gui": gazebo_gui}.items()
    )
    gazebo_spawn_robot = Node(
      package="gazebo_ros",
      executable="spawn_entity.py",
      name="spawn_tm",
      arguments=["-entity", "tm5-900", "-topic", "robot_description"],
      output="screen"
    )

    # Joint STATE BROADCASTER:
    joint_state_broadcaster_spawner = Node(
      package="controller_manager",
      executable="spawner",
      arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Joint TRAJECTORY Controller:
    tm_arm_controller_spawner = Node(
      package="controller_manager",
      executable="spawner",
      arguments=["tmr_arm_controller", "-c", "/controller_manager"]
    )

    # ros2_controllers_path = os.path.join(
    #     get_package_share_directory("tm_ros2_gazebo"),
    #     "config/tm_controllers.yaml"
    # )

    # ros2_control_node = Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     parameters=[ros2_controllers_path],
    #     remappings=[
    #         ('/controller_manager/robot_description', '/robot_description'),
    #     ],
    #     output='both',
    # )

    nodes_to_start = [
      robot_state_publisher_node,
      rviz_node,
      gazebo,
      gazebo_spawn_robot,
      joint_state_broadcaster_spawner,
      tm_arm_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)

