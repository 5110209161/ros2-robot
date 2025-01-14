#!/usr/bin/python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration , PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
  description_package = LaunchConfiguration("description_package")
  package_name = context.launch_configurations["description_package"]
  
  robot_type_value = context.launch_configurations["aubo_type"]
  if not robot_type_value:
    raise ValueError("Robot type is not set.")
  
  urdf_model_name = f"{robot_type_value}.urdf"

  rviz_config_file = PathJoinSubstitution(
    [FindPackageShare(description_package), "rviz", "view_robot.rviz"]
  )

  robot_state_publisher_node = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    output="screen",
    arguments=[
      os.path.join(
        FindPackageShare(description_package).find(package_name), 
        "urdf", urdf_model_name
      )
    ],
  )

  joint_state_publisher_node = Node(
    package="joint_state_publisher_gui",
    executable="joint_state_publisher_gui",
    name="joint_state_publisher_gui",
  )

  rviz_node = Node(
    package="rviz2",
    executable="rviz2",
    name="rviz2",
    output="screen",
    arguments=["-d", rviz_config_file],
  )

  return [robot_state_publisher_node, joint_state_publisher_node, rviz_node]


def generate_launch_description():
  declared_arguments = []
  declared_arguments.append(
    DeclareLaunchArgument(
      "aubo_type", 
      description="Type/series of used aubo robot.",
      choices=["aubo_C3", "aubo_E10", "aubo_i3", "aubo_i5", "aubo_i7", "aubo_i10", "aubo_i12", "aubo_i16", "aubo_i20", 
               "aubo_S3", "aubo_S5", "aubo_T6", "aubo_T12"],
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

  return LaunchDescription(
    declared_arguments + 
    [OpaqueFunction(function=launch_setup)])
  
  