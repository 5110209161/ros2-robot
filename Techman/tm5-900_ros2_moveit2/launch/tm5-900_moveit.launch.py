#!/usr/bin/python3
import os
import sys
import xacro
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None
    

def generate_launch_description():
    # Initialize Arguments
    xacro_path = f"config/tm5-900.urdf.xacro"
    moveit_config_path = "tm5-900_ros2_moveit2"    
    srdf_path = "config/tm5-900.srdf"
    kinematics_yaml_path = "config/kinematics.yaml"
    planning_pipeline_path = "config/ompl_planning.yaml"
    rviz_path = "/rviz/moveit.rviz"
    ros2_control_controller_path = "config/ros2_controllers.yaml"
    joint_limits_path = "config/joint_limits.yaml"
    
    # ****** the way to get robot_description of original repository, but always error here ******
    # robot_description_config = xacro.process_file(
    #     os.path.join(
    #         get_package_share_directory(description_path), xacro_path
    #     )
    # )
    # robot_description = {"robot_description", robot_description_config.toxml()}

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(moveit_config_path), xacro_path]),
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    
    # SRDF Configuration
    robot_description_semantic_config = load_file(moveit_config_path, srdf_path)
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config}
    
    # Kinematics
    kinematics_yaml = load_yaml(moveit_config_path, kinematics_yaml_path)
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}
    
    # Joint limits
    joint_limits_yaml = {
        "robot_description_planning": load_yaml(
            moveit_config_path, joint_limits_path
        )
    }

    # Move group: OMPL Planning.
    ompl_planning_pipeline_config = {
        "planning_pipelines": ["ompl"],
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(moveit_config_path, planning_pipeline_path)
    ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml)
    
    # Trajectory execution configuration (controllers)
    controllers_yaml = load_yaml(moveit_config_path, "config/controllers.yaml")
    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager"
    }

    # Trajectory execution
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.1
    }

    # Planning scene
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # *********************** MoveIt!2 *********************** #
    # Move group node/action server
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
        ]
    )

    # RViz config
    rviz_config_file = (
        get_package_share_directory(moveit_config_path) + rviz_path
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            joint_limits_yaml,
            {"use_sim_time": True}, 
        ]
    )

     # ***** STATIC TRANSFORM ***** #
    # Static TF node: publish world -> base transform
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base"]
    )

    # Robot State Publisher Node: Publishes tf"s for the robot
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
            # {"use_sim_time": True}
        ],
    )

    # ***** ROS2_CONTROL -> LOAD CONTROLLERS ***** #
    ros2_controllers_path = os.path.join(
        get_package_share_directory(moveit_config_path),
        ros2_control_controller_path
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )

    # Joint STATE BROADCASTER:
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Joint TRAJECTORY Controller:
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "tmr_arm_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    return LaunchDescription(
        [
            rviz_node,
            static_tf,
            robot_state_publisher,
            move_group_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            joint_trajectory_controller_spawner
        ]
    )

