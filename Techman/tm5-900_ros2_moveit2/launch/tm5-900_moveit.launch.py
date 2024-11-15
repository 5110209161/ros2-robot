#!/usr/bin/python3
import os
import xacro
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


# LOAD FILE:
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None


# LOAD YAML:
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None
    

def generate_launch_description():
    # Initialize Arguments
    tm_robot_type = "tm5-900"
    xacro_path = "config/tm5-900.urdf.xacro"
    moveit_config_path = "tm5-900_ros2_moveit2"    
    srdf_path = "config/tm5-900.srdf"
    kinematics_yaml_path = "config/kinematics.yaml"
    planning_pipeline_path = "config/ompl_planning.yaml"
    rviz_path = "/rviz/moveit.rviz"
    moveit_controller_path = "config/moveit2_controllers.yaml"
    ros2_control_controller_path = "config/ros2_controllers.yaml"
    joint_limits_path = "config/joint_limits.yaml"

    robot_description_path = os.path.join(
        get_package_share_directory(moveit_config_path), xacro_path
    )
    robot_description_config = xacro.parse(open(robot_description_path)).toxml()
    robot_description = {"robot_description", robot_description_config}

    robot_description_semantic_config = load_file(moveit_config_path, srdf_path)
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config}

    robot_description_kinematics = load_yaml(moveit_config_path, kinematics_yaml_path)
    trajectory_execution = load_yaml(moveit_config_path, moveit_controller_path)
    joint_limits = load_yaml(moveit_config_path, joint_limits_path)
    # Move group: OMPL Planning.
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(moveit_config_path, planning_pipeline_path)
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # ***** STATIC TRANSFORM ***** #
    # Static TF node: publish world -> base transform
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base']
    )

    # Robot State Publisher Node: Publishes tf's for the robot
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
            {"use_sim_time": True}
        ],
    )

    # ***** ROS2_CONTROL -> LOAD CONTROLLERS ***** #
    ros2_controllers_path = os.path.join(
        get_package_share_directory(moveit_config_path),
        ros2_control_controller_path
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[ros2_controllers_path],
        remappings=[
            ('/controller_manager/robot_description', '/robot_description'),
        ],
        output='both',
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
            "tmr_arm_control",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    # *********************** MoveIt!2 *********************** #
    # Move group node/action server
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            trajectory_execution,
            joint_limits,
            {"use_sim_time": True}, 
        ]
    )

    # RViz config
    # rviz_config_file = (
    #     get_package_share_directory(moveit_config_path) + rviz_path
    # )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        # arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            joint_limits,
            {"use_sim_time": True}, 
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

