#!/usr/bin/python3
import os

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ur_moveit_config.launch_common import load_yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    OrSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction

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
            description="IP address by which the robot can be reached."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("runtime_config_package", default_value="ur_robot_driver")
    )

    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare(LaunchConfiguration("runtime_config_package")), "/launch", "/ur_control.launch.py"
        ]),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "robot_ip": LaunchConfiguration("robot_ip"),
            "use_fake_hardware": "false",
            "launch_rviz": "false",
            "headless_mode": "true"
        }.items()
    )

    return LaunchDescription(declared_arguments + [driver_launch])
