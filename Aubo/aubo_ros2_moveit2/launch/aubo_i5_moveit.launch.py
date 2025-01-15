from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import (
  Command,
  FindExecutable,
  LaunchConfiguration,
  PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import yaml


def load_yaml(package_name, file_path):
  package_path = get_package_share_directory(package_name)
  absolute_file_path = os.path.join(package_path, file_path)

  try:
    with open(absolute_file_path, "r") as file:
      return yaml.safe_load(file)
  except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
    return None


def load_file(package_name, file_path):
  package_path = get_package_share_directory(package_name)
  absolute_file_path = os.path.join(package_path, file_path)

  try:
    with open(absolute_file_path, 'r') as file:
      return file.read()
  except OSError:  # parent of IOError, OSError *and* WindowsError where available
    return None


def generate_launch_description():
  description_package = "aubo_ros2_gazebo"
  robot_xacro_file = "aubo_ros2.xacro"
  moveit_config_package = "aubo_ros2_moveit2"
  moveit_config_file = "aubo_i5.srdf"
  aubo_type = "aubo_i5"

  robot_description_content = Command([
    PathJoinSubstitution([FindExecutable(name="xacro")]),
    " ",
    PathJoinSubstitution([FindPackageShare(description_package), "urdf/xacro/inc/", robot_xacro_file]),
    " ",
    "aubo_type:=",
    aubo_type,
  ])
  robot_description = {"robot_description": robot_description_content}

  robot_description_semantic_config = load_file(moveit_config_package, f"config/{moveit_config_file}")
  robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config}

  kinematics_yaml = load_yaml(moveit_config_package, "config/kinematics.yaml")
  robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

  joint_limits_yaml = {
    "robot_description_planning": load_yaml(moveit_config_package, "config/joint_limits.yaml")
  }

  ompl_planning_pipeline_config = {
    "move_group": {
      "planning_plugin": "ompl_interface/OMPLPlanner",
      "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
      "start_state_max_bounds_error": 0.1,
      "sample_duration": 0.005,
    }
  }
  ompl_planning_yaml = load_yaml(moveit_config_package, "config/ompl_planning.yaml")
  ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

  moveit_simple_controllers_yaml = load_yaml(moveit_config_package, "config/moveit_controllers.yaml")
  moveit_controllers = {
    "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
    "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
  }

  trajectory_execution = {
    # MoveIt does not handle controller switching automatically
    "moveit_manage_controllers": False,
    "trajectory_execution.allowed_execution_duration_scaling": 1.2,
    "trajectory_execution.allowed_goal_duration_margin": 0.5,
    "trajectory_execution.allowed_start_tolerance": 0.01,
  }

  planning_scene_monitor_parameters = {
    "publish_planning_scene": True,
    "publish_geometry_updates": True,
    "publish_state_updates": True,
    "publish_transforms_updates": True,
    # "planning_scene_monitor_options": {
    #   "name": "planning_scene_monitor",
    #   "robot_description": "robot_description",
    #   "joint_state_topic": "/joint_states",
    #   "attached_collision_object_topic": "/move_group/planning_scene_monitor",
    #   "publish_planning_scene_topic": "/move_group/publish_planning_scene",
    #   "monitored_planning_scene_topic": "/move_group/monitored_planning_scene",
    #   "wait_for_initial_state_timeout": 10.0,
    # }
  }

  move_group_node = Node(
    package="moveit_ros_move_group",
    executable="move_group",
    output="screen",
    parameters=[
      robot_description,
      robot_description_semantic,
      robot_description_kinematics,
      ompl_planning_pipeline_config,
      trajectory_execution,
      moveit_controllers,
      planning_scene_monitor_parameters,
      joint_limits_yaml,
      {"use_sim_time": True},
    ],
  )

  rviz_config_file = PathJoinSubstitution([
    FindPackageShare(moveit_config_package), "config", "moveit.rviz"
  ])
  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='log',
    arguments=['-d', rviz_config_file],
    parameters=[
      robot_description,
      robot_description_semantic,
      ompl_planning_pipeline_config,
      robot_description_kinematics,
      joint_limits_yaml,
    ],
  )

  static_tf = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="static_transform_publisher",
    output="log",
    arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base_link']
  )

  robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='both',
    parameters=[robot_description]
  )
  

  return LaunchDescription([
    rviz_node,
    static_tf,
    robot_state_publisher,
    move_group_node,
  ])
