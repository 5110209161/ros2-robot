#!/usr/bin/python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
  description_package = LaunchConfiguration("description_package")
  robot_type = LaunchConfiguration("robot_type")

  robot_type_value = context.launch_configurations["robot_type"]
  if not robot_type_value:
     raise ValueError("Robot type is not set.")
  
  urdf_file_name = f"{robot_type_value}_robot.xacro"

  robot_description_content = Command(
    [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare(description_package), "urdf", urdf_file_name]),
        " ",
        "name:=",
        "dobot",
        " ",
        "robot_type:=",
        robot_type
    ]
  )
  robot_description = {"robot_description": robot_description_content}

  robot_state_publisher_node = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    output="screen",
    parameters=[robot_description, {"use_sim_time": True}]
  )

  rviz_config_file = PathJoinSubstitution(
     [FindPackageShare(description_package), "rviz", "view_robot.rviz"]
  )

  rviz_node = Node(
    package="rviz2",
    executable="rviz2",
    name="rviz2",
    output="log",
    arguments=["-d", rviz_config_file]
  )

  gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(get_package_share_directory("gazebo_ros"), "launch"), "/gazebo.launch.py"
      ]),
      launch_arguments={}.items(),
  )

  spawn_entity_node = Node(
    package='gazebo_ros', 
    executable='spawn_entity.py',
    arguments=['-topic', 'robot_description',
                '-entity', f"{robot_type_value}_robot"],
    output='screen')
  
  joint_state_brodcaster_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
  )

  joint_trajectory_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["dobot_arm_controller", "-c", "/controller_manager"],
  )
  
  return [
    robot_state_publisher_node, 
    rviz_node,
    gazebo,
    spawn_entity_node,
    joint_state_brodcaster_spawner,
    joint_trajectory_controller_spawner
  ]


def generate_launch_description():
  declared_arguments = []
  declared_arguments.append(
     DeclareLaunchArgument(
      "description_package",
      default_value="dobot_ros2_gazebo",
      description="Description package with robot URDF/XACRO files."
    )
  )

  declared_arguments.append(
     DeclareLaunchArgument(
      name="robot_type",
      description="Type/series of used Dobot robot",
      choices=["cr3", "cr5", "cr7", "cr10", "cr12", "cr16", "nova2", "nova5"]
    )
  )
  
  return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])