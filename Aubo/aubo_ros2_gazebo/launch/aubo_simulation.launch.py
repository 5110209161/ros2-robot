#!/usr/bin/python3
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
  robot_type = LaunchConfiguration("robot_type")
  robot_type_value = context.launch_configurations["robot_type"]
  if not robot_type_value:
    raise ValueError("Robot type is not set.")
  
  description_package = LaunchConfiguration("description_package")

  rviz_config_file = PathJoinSubstitution(
    [FindPackageShare(description_package), "rviz", "view_robot.rviz"]
  )

  robot_description_content = Command(
    [
      PathJoinSubstitution(FindExecutable(name="xacro")),
      " ",
      PathJoinSubstitution([
        FindPackageShare(description_package),
        "urdf/xacro/inc",
        "aubo_ros2.xacro",
      ]),
      " ",
      "name:=",
      "aubo",
      " "
      "aubo_type:=",
      robot_type
    ]
  )
  robot_description = {"robot_description": robot_description_content}

  robot_state_publisher_node = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    output="screen",
    parameters=[robot_description, {"use_sim_time": True}],
  )

  rviz_node = Node(
    package="rviz2",
    executable="rviz2",
    output="screen",
    arguments=["-d", rviz_config_file],
    condition=IfCondition(LaunchConfiguration("launch_rviz")),
  )

  gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"
    ])
  )

  gazebo_spawn_robot = Node(
    package="gazebo_ros",
    executable="spawn_entity.py",
    name="spawn_aubo",
    arguments=["-entity", robot_type_value, "-topic", "robot_description"],
    output="screen",
  )

  joint_state_broadcaster_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
  )

  joint_trajectory_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
  )

  return [
    robot_state_publisher_node,
    rviz_node,
    gazebo,
    gazebo_spawn_robot,
    joint_state_broadcaster_spawner,
    joint_trajectory_controller_spawner,
  ]


def generate_launch_description():
  declared_arguments = []
  declared_arguments.append(
    DeclareLaunchArgument(
      "robot_type",
      description="Type/series of used aubo robot.",
      choices=["aubo_C3", "aubo_E10", "aubo_i3", "aubo_i5", "aubo_i7", "aubo_i10", "aubo_i12", "aubo_i16", "aubo_i20", 
               "aubo_S3", "aubo_S5", "aubo_T6", "aubo_T12"],
    )
  )
  declared_arguments.append(
    DeclareLaunchArgument(
      "safety_limits",
      default_value="true",
      description="Enables the safety limits controller if true.",
    )
  )
  declared_arguments.append(
    DeclareLaunchArgument(
      "safety_pos_margin",
      default_value="0.15",
      description="The margin to lower and upper limits in the safety controller.",
    )
  )
  declared_arguments.append(
    DeclareLaunchArgument(
      "safety_k_position",
      default_value="20",
      description="k-position factor in the safety controller.",
    )
  )
  declared_arguments.append(
    DeclareLaunchArgument(
      "controllers_file",
      default_value="aubo_controllers.yaml",
      description="YAML file with the controllers configuration.",
    )
  )
  declared_arguments.append(
    DeclareLaunchArgument(
      "description_package",
      default_value="aubo_ros2_gazebo",
      description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
    )
  )
  declared_arguments.append(
    DeclareLaunchArgument(
      "description_file",
      default_value="aubo_ros2.xacro",
      description="URDF/XACRO description file with the robot.",
    )
  )
  
  declared_arguments.append(
    DeclareLaunchArgument(
      "start_joint_controller",
      default_value="true",
      description="Enable headless mode for robot control",
    )
  )
  declared_arguments.append(
    DeclareLaunchArgument(
      "initial_joint_controller",
      default_value="joint_trajectory_controller",
      description="Robot controller to start.",
    )
  )
  declared_arguments.append(
    DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
  )

  return LaunchDescription(
    declared_arguments +
    [OpaqueFunction(function=launch_setup)]
  )
