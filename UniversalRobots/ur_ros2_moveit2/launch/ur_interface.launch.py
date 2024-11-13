#!/usr/bin/python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="xxx.xxx.xxx.xxx",
            description="IP address by which the robot can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="false",
            description="Launch RViz?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="scaled_joint_trajectory_controller",
            description="Initially loaded robot controller.",
            choices=[
                "scaled_joint_trajectory_controller",
                "joint_trajectory_controller",
                "forward_velocity_controller",
                "forward_position_controller",
            ],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Activate loaded joint controller.",
        )
    )

    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    launch_rviz = LaunchConfiguration("launch_rviz")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ur_robot_driver'), 'launch'), '/ur_control.launch.py'
        ]),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "launch_rviz": launch_rviz,
            "use_fake_hardware": use_fake_hardware,
            "initial_joint_controller": initial_joint_controller,
            "activate_joint_controller": activate_joint_controller,
        }.items(),
    )

    # moveit_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(
    #         get_package_share_directory('ur_moveit_config'), 'launch', 'ur_moveit.launch.py')),
    #     launch_arguments={
    #         "ur_type": ur_type,
    #         "launch_rviz": "true",
    #     }.items(),
    # )

    moveit_launch = IncludeLaunchDescription(
        os.path.join(get_package_share_directory('ur_moveit_config'), 'launch', 'ur_moveit.launch.py'),
        launch_arguments={
            "ur_type": ur_type,
            "launch_rviz": "true",
        }.items()
    )

    # *********************** ROS2.0 Robot/End-Effector Actions/Triggers *********************** #
    # MoveJ Action
    moveJ_interface = Node(
        name="moveJ_action",
        package="ros2_actions",
        executable="moveJ_action",
        output="screen",
        parameters=[]
    )
    # MoveG Action
    moveG_interface = Node(
        name="moveG_action",
        package="ros2_actions",
        executable="moveG_action",
        output="screen",
        parameters=[]
    )
    # MoveXYZW ACTION:
    moveXYZW_interface = Node(
        name="moveXYZW_action",
        package="ros2_actions",
        executable="moveXYZW_action",
        output="screen",
        parameters=[],
    )
    # MoveL ACTION:
    moveL_interface = Node(
        name="moveL_action",
        package="ros2_actions",
        executable="moveL_action",
        output="screen",
        parameters=[]
    )
    # MoveR ACTION:
    moveR_interface = Node(
        name="moveR_action",
        package="ros2_actions",
        executable="moveR_action",
        output="screen",
        parameters=[]
    )
    # MoveXYZ ACTION:
    moveXYZ_interface = Node(
        name="moveXYZ_action",
        package="ros2_actions",
        executable="moveXYZ_action",
        output="screen",
        parameters=[]
    )
    # MoveYPR ACTION:
    moveYPR_interface = Node(
        name="moveYPR_action",
        package="ros2_actions",
        executable="moveYPR_action",
        output="screen",
        parameters=[]
    )
    # MoveROT ACTION:
    moveROT_interface = Node(
        name="moveROT_action",
        package="ros2_actions",
        executable="moveROT_action",
        output="screen",
        parameters=[]
    )
    # MoveRP ACTION:
    moveRP_interface = Node(
        name="moveRP_action",
        package="ros2_actions",
        executable="moveRP_action",
        output="screen",
        parameters=[]
    )

    delayed_actions_launch = TimerAction(
        period=20.0,
        actions=[
            moveJ_interface,
            moveG_interface,
            moveL_interface,
            moveR_interface,
            moveXYZ_interface,
            moveXYZW_interface,
            moveYPR_interface,
            moveROT_interface,
            moveRP_interface,
        ]
    )

    return LaunchDescription(declared_arguments + [
        base_launch,
        delayed_actions_launch

        # RegisterEventHandler(
        #     OnProcessExit(
        #         target_action = base_launch,
        #         on_exit = [moveit_launch]
        #     )
        # ),

        # RegisterEventHandler(
        #     OnProcessExit(
        #         target_action = moveit_launch,
        #         on_exit = [
        #             TimerAction(
        #                 period = 25.0,
        #                 actions = [
        #                     moveJ_interface,
        #                     moveG_interface,
        #                     moveL_interface,
        #                     moveR_interface,
        #                     moveXYZ_interface,
        #                     moveXYZW_interface,
        #                     moveYPR_interface,
        #                     moveROT_interface,
        #                     moveRP_interface,
        #                 ]
        #             )
        #         ]
        #     )
        # )
    ])