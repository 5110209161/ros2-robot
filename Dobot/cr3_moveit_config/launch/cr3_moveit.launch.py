#!/usr/bin/python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
  # Initialize Arguments
  robot_type = "cr3"
  xacro_path = "config/cr3_robot.urdf.xacro"
  srdf_path = "config/cr3_robot.srdf"
  moveit_controller_path = "config/moveit_controllers.yaml"
  joint_limits_path = "config/joint_limits.yaml"

  moveit_config_pkg_path = "cr3_moveit_config"
  rviz_path = "/rviz/moveit.rviz"
  ros2_control_controller_path = "config/ros2_controllers.yaml"
  
  moveit_config = (
    MoveItConfigsBuilder(robot_type)
    .robot_description(file_path=xacro_path)
    .robot_description_semantic(file_path=srdf_path)
    .trajectory_execution(file_path=moveit_controller_path)
    .joint_limits(file_path=joint_limits_path)
    .to_moveit_configs()
  )

  move_group_node = Node(
    package="moveit_ros_move_group",
    executable="move_group",
    output="screen",
    parameters=[
      moveit_config.to_dict(),
      {"use_sim_time": True},
    ]
  )

  rviz_config_file = (
    get_package_share_directory(moveit_config_pkg_path) + rviz_path
  )
  rviz_node = Node(
    package="rviz2",
    executable="rviz2",
    name="rviz2",
    output="log",
    arguments=["-d", rviz_config_file],
    parameters=[
      moveit_config.robot_description,
      moveit_config.robot_description_semantic,
      moveit_config.planning_pipelines,
      moveit_config.robot_description_kinematics,
      moveit_config.joint_limits,
      {"use_sim_time": True},
    ]
  )

  robot_state_publisher_node = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    name="robot_state_publisher",
    output="both",
    parameters=[
      moveit_config.robot_description,
      {"use_sim_time": True},
    ],
  )

  ros2_controller_path = os.path.join(
    get_package_share_directory(moveit_config_pkg_path),
    ros2_control_controller_path
  )

  ros2_control_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[ros2_controller_path],
    remappings=[
      ("/controller_manager/robot_description", "/robot_description"),
    ],
    output="both",
  )

  joint_state_broadcaster_spawner_node = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
  )

  joint_trajectory_controller_spawner_node = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
      "cr3_group_controller",
      "--controller-manager",
      "/controller_manager"
    ]
  )

  return LaunchDescription([
    rviz_node,
    robot_state_publisher_node,
    move_group_node,
    ros2_control_node,
    joint_state_broadcaster_spawner_node,
    joint_trajectory_controller_spawner_node,
  ])

