#!/usr/bin/python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction


def generate_launch_description():
    # Initialize Arguments
    tm_robot_type = "tm5-900"
    xacro_path = "config/tm5-900.urdf.xacro"
    moveit_config_path = "tm5-900_moveit_config"    
    srdf_path = 'config/tm5-900.srdf'
    rviz_path = "/rviz/moveit.rviz"
    moveit_controller_path = "config/moveit2_controllers.yaml"
    ros2_control_controller_path = "config/ros2_controllers.yaml"
    joint_limits_path = "config/joint_limits.yaml"

    # *********************** MoveIt!2 *********************** #
    # MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder(tm_robot_type)
        .robot_description(file_path=xacro_path)
        .robot_description_semantic(file_path=srdf_path)
        .trajectory_execution(file_path=moveit_controller_path)
        .joint_limits(file_path=joint_limits_path)
        .to_moveit_configs()
    )

    # Move group node/action server
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
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
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
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
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base']
    )

    # Robot State Publisher Node: Publishes tf's for the robot
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": True}, 
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
    tm_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "tmr_arm_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    # *********************** ROS2.0 Robot/End-Effector Actions/Triggers *********************** #
    # MoveJ Action
    moveJ_interface = Node(
        name="moveJ_action",
        package="ros2_actions",
        executable="moveJ_action",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True}, 
            {"ROB_PARAM": 'tmr_arm'}
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
            tm_arm_controller_spawner,
            # ROS2.0 Actions:
            TimerAction(
                period=2.0,
                actions=[
                    moveJ_interface
                ]
            )
        ]
    )

