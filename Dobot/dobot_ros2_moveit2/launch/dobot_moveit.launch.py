#!/usr/bin/python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
  robot_type_value = context.launch_configurations["robot_type"]
  if not robot_type_value:
    raise ValueError("Robot type is not set.")
  
  moveit_package_name = f"{robot_type_value}_moveit_config"
  moveit_launch_file = f"{robot_type_value}_moveit.launch.py"

  moveit_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      launch_file_path=PathJoinSubstitution(
        [FindPackageShare(moveit_package_name), "launch", moveit_launch_file]
      )
    )
  )

  return [moveit_launch]


def generate_launch_description():
  declared_arguments = []
  declared_arguments.append(
     DeclareLaunchArgument(
      name="robot_type",
      description="Type/series of used Dobot robot",
      choices=["cr3", "cr5", "cr7", "cr10", "cr12", "cr16", "nova2", "nova5"]
    )
  )

  return LaunchDescription(
    declared_arguments + [OpaqueFunction(function=launch_setup)])