#!/usr/bin/python3
import copy
import threading
from enum import Enum
from typing import Any, List, Optional, Tuple, Union

import numpy as np
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
)
from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetCartesianPath,
    GetMotionPlan,
    GetPlanningScene,
    GetPositionFK,
    GetPositionIK,
)
from rclpy.action import ActionClient
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.task import Future
from sensor_msgs.msg import JointState
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from std_msgs.msg import Header, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MoveIt2State(Enum):
    """
    An enum the represents the current execution state of the MoveIt2 interface.
    - IDLE: No motion is being requested or executed
    - REQUESTING: Execution has been requested, but the request has not yet been
      accepted.
    - EXECUTING: Execution has been requested and accepted, and has not yet been
      completed.
    """
    IDLE = 0
    REQUESTING = 1
    EXECUTING = 2


class MoveIt2:
    """
    Python interface for MoveIt 2 that enables planning and execution of trajectories.
    For execution, this interface requires that robot utilises JointTrajectoryController.
    """
    def __init__(
            self,
            node: Node,
            joint_names: List[str],
            base_link_name: str,
            end_effector_name: str,
            group_name: str = "arm",
            execute_via_moveit: bool = False,
            ignore_new_calls_while_executing: bool = False,
            callback_group: Optional[CallbackGroup] = None,
            follow_joint_trajectory_action_name: str = "DEPRECATED",
            use_move_group_action: bool = False,
        ):
        """
        Construct an instance of `MoveIt2` interface.
          - `node` - ROS 2 node that this interface is attached to
          - `joint_names` - List of joint names of the robot (can be extracted from URDF)
          - `base_link_name` - Name of the robot base link
          - `end_effector_name` - Name of the robot end effector
          - `group_name` - Name of the planning group for robot arm
          - [DEPRECATED] `execute_via_moveit` - Flag that enables execution via MoveGroup action (MoveIt 2)
                                   FollowJointTrajectory action (controller) is employed otherwise
                                   together with a separate planning service client
          - `ignore_new_calls_while_executing` - Flag to ignore requests to execute new trajectories
                                                 while previous is still being executed
          - `callback_group` - Optional callback group to use for ROS 2 communication (topics/services/actions)
          - [DEPRECATED] `follow_joint_trajectory_action_name` - Name of the action server for the controller
          - `use_move_group_action` - Flag that enables execution via MoveGroup action (MoveIt 2)
                               ExecuteTrajectory action is employed otherwise
                               together with a separate planning service client
        """
        self._node = node
        self._callback_group = callback_group

        # region Check for deprecated parameters
        if execute_via_moveit:
            self._node.get_logger().warn(
                "Parameter `execute_via_moveit` is deprecated. Please use `use_move_group_action` instead."
            )
            use_move_group_action = True
        
        if follow_joint_trajectory_action_name != "DEPRECATED":
            self._node.get_logger().warn(
                "Parameter `follow_joint_trajectory_action_name` is deprecated. `MoveIt2` uses the `execute_trajectory` action instead."
            )
        
        # endregion

        self.__collision_object_publisher = self._node.create_publisher(
            CollisionObject, "/collision_object", 10
        )
        self.__attached_collision_object_publisher = self._node.create_publisher(
            AttachedCollisionObject, "/attached_collision_object", 10
        )

        self.__cancellation_pub = self._node.create_publisher(
            String, "/trajectory_execution_event", 1
        )

        self.__move_action_goal = self.__init_move_action_goal(
            frame_id=base_link_name,
            group_name=group_name,
            end_effector=end_effector_name
        )

        # Flag to determine whether to execute trajectories via Move Group Action, or rather by calling
        # the separate ExecuteTrajectory action
        # Applies to `move_to_pose()` and `move_to_configuration()`
        self.__use_move_group_action = use_move_group_action

        # Flag that determines whether a new goal can be sent while the previous one is being executed
        self.__ignore_new_calls_while_executing = ignore_new_calls_while_executing

        # region Store additional variables for later use
        self.__base_link_name = base_link_name
        self.__end_effector_name = end_effector_name
        # endregion

        # region Internal states that monitor the current motion requests and execution
        self.__is_motion_requested = False
        self.__is_executing = False
        self.__execution_goal_handle = None
        self.__last_error_code = None
        self.__execution_mutex = threading.Lock()
        # endregion

    # region Polling Functions

    def query_state(self) -> MoveIt2State:
        with self.__execution_mutex:
            if self.__is_motion_requested:
                return MoveIt2State.REQUESTING
            elif self.__is_executing:
                return MoveIt2State.EXECUTING
            else:
                return MoveIt2State.IDLE
    
    def cancel_execution(self):
        if self.query_state() != MoveIt2State.EXECUTING:
            self._node.get_logger().warn("Attempted to cancel without active goal.")
            return None

        cancel_string = String()
        cancel_string.data = "stop"
        self.__cancellation_pub.publish(cancel_string)

    def get_execution_future(self) -> Optional[Future]:
        if self.query_state() != MoveIt2State.EXECUTING:
            self._node.get_logger().warn("Need active goal for future.")
            return None
        
        return self.__execution_goal_handle.get_result_async()
    
    def get_last_execution_error_code(self) -> Optional[MoveItErrorCodes]:
        return self.__last_error_code
    
    # endregion 

    def move_to_pose(
        self,
        pose: Optional[Union[PoseStamped, Pose]] = None,
        position: Optional[Union[Point, Tuple[float, float, float]]] = None,
        quat_xyzw: Optional[Union[Quaternion, Tuple[float, float, float, float]]] = None,
        target_link: Optional[str] = None,
        frame_id: Optional[str] = None,
        tolerance_position: float = 0.001,
        tolerance_orientation: float = 0.001,
        weight_position: float = 1.0,
        cartesian: bool = False,
        weight_orientation: float = 1.0,
        cartesian_max_step: float = 0.0025,
        cartesian_fraction_threshold: float = 0.0,
    ):
        """
        Plan and execute motion based on previously set goals. Optional arguments can be
        passed in to internally use `set_pose_goal()` to define a goal during the call.
        """
        if isinstance(pose, PoseStamped):
            pose_stamped = pose
        elif isinstance(pose, Pose):
            pose_stamped = PoseStamped(
                header=Header(
                    stamp=self._node.get_clock().now().to_msg(),
                    frame_id=(frame_id if frame_id is not None else self.__base_link_name)
                ),
                pose=pose,
            )
        else:
            if not isinstance(position, Point):
                position = Point(
                    x = float(position[0]),
                    y=float(position[1]), 
                    z=float(position[2])
                )
            if not isinstance(quat_xyzw, Quaternion):
                quat_xyzw = Quaternion(
                    x=float(quat_xyzw[0]),
                    y=float(quat_xyzw[1]),
                    z=float(quat_xyzw[2]),
                    w=float(quat_xyzw[3]),
                )
            pose_stamped = PoseStamped(
                header=Header(
                    stamp=self._node.get_clock().now().to_msg(),
                    frame_id=(frame_id if frame_id is not None else self.__base_link_name)
                ),
                pose=Pose(position=position, orientation=quat_xyzw)
            )
        
        if self.__use_move_group_action and not cartesian:
            if self.__ignore_new_calls_while_executing and (
                self.__is_motion_requested or self.__is_executing
            ):
                self._node.get_logger().warn("Controller is already following a trajectory. Skipping motion.")
                return
            # Set goal

    def set_pose_goal(
        self,
        pose: Optional[Union[PoseStamped, Pose]] = None,
        position: Optional[Union[Point, Tuple[float, float, float]]] = None,
        quat_xyzw: Optional[
            Union[Quaternion, Tuple[float, float, float, float]]
        ] = None,
        frame_id: Optional[str] = None,
        target_link: Optional[str] = None,
        tolerance_position: float = 0.001,
        tolerance_orientation: Union[float, Tuple[float, float, float]] = 0.001,
        weight_position: float = 1.0,
        weight_orientation: float = 1.0,
    ):
        """
        This is direct combination of `set_position_goal()` and `set_orientation_goal()`.
        """
        if (pose is None) and (position is None or quat_xyzw is None):
            raise ValueError("Either 'pose' or 'position' and 'quat_xyzw' must be specified.")
        
        if isinstance(pose, PoseStamped):
            pose_stamped = pose
        elif isinstance(pose, Pose):
            pose_stamped = PoseStamped(
                header=Header(
                    stamp=self._node.get_clock().now().to_msg(),
                    frame_id=(frame_id if frame_id is not None else self.__base_link_name),
                ),
                pose=pose,
            )
        else:
            if not isinstance(position, Point):
                position = Point(
                    x=float(position[0]), y=float(position[1]), z=float(position[2])
                )
            if not isinstance(quat_xyzw, Quaternion):
                quat_xyzw = Quaternion(
                    x=float(quat_xyzw[0]),
                    y=float(quat_xyzw[1]),
                    z=float(quat_xyzw[2]),
                    w=float(quat_xyzw[3]),
                )
            pose_stamped = PoseStamped(
                header=Header(
                    stamp=self._node.get_clock().now().to_msg(),
                    frame_id=(frame_id if frame_id is not None else self.__base_link_name),
                ),
                pose=Pose(position=position, orientation=quat_xyzw),
            )
        
        # set position goal
        self.set_position_goal(
            position=pose_stamped.pose.position,
            frame_id=pose_stamped.header.frame_id,
            target_link=target_link,
            tolerance=tolerance_position,
            weight=weight_position
        )

        # set orientation goal
    
    def set_position_goal(
        self,
        position: Union[Point, Tuple[float, float, float]],
        frame_id: Optional[str] = None,
        target_link: Optional[str] = None,
        tolerance: float = 0.001,
        weight: float = 1.0,
    ):
        """
        Set Cartesian position goal of `target_link` with respect to `frame_id`.
          - `frame_id` defaults to the base link
          - `target_link` defaults to end effector
        """
        constraint = self.create_position_constraint(
            position=position,
            frame_id=frame_id,
            target_link=target_link,
            tolerance=tolerance,
            weight=weight
        )
        # Append to other constraints
        self.__move_action_goal.request.goal_constraints[-1].position_constraints.append(constraint)

    def create_position_constraint(
        self,
        position: Union[Point, Tuple[float, float, float]],
        frame_id: Optional[str] = None,
        target_link: Optional[str] = None,
        tolerance: float = 0.001,
        weight: float = 1.0,
    ) -> PositionConstraint:
        """
        Refer to: 
            http://docs.ros.org/en/jade/api/moveit_msgs/html/msg/Constraints.html
            http://docs.ros.org/en/ros2_packages/humble/api/moveit_msgs/interfaces/msg/Constraints.html
        Create Cartesian position constraint of `target_link` with respect to `frame_id`.
          - `frame_id` defaults to the base link
          - `target_link` defaults to end effector
        """
        # Create new position constraint
        constraint = PositionConstraint()
        # Define reference frame and target link
        constraint.header.frame_id = (
            frame_id if frame_id is not None else self.__base_link_name
        )
        constraint.link_name = (
            target_link if target_link is not None else self.__end_effector_name
        )
        # Define target position
        constraint.constraint_region.primitive_poses.append(Pose())
        if isinstance(position, Point):
            constraint.constraint_region.primitive_poses[0].position = position
        else:
            constraint.constraint_region.primitive_poses[0].position.x = float(position[0])
            constraint.constraint_region.primitive_poses[0].position.y = float(position[1])
            constraint.constraint_region.primitive_poses[0].position.z = float(position[2])
        # Define goal region as a sphere with radius equal to the tolerance
        constraint.constraint_region.primitives.append(SolidPrimitive())
        constraint.constraint_region.primitives[0].type = 2  # Sphere
        constraint.constraint_region.primitives[0].dimensions = [tolerance]
        # Set weight of the constraint
        constraint.weight = weight

        return constraint

    def set_orientation_goal(
        self,
        quat_xyzw: Union[Quaternion, Tuple[float, float, float, float]],
        frame_id: Optional[str] = None,
        target_link: Optional[str] = None,
        tolerance: Union[float, Tuple[float, float, float]] = 0.001,
        weight: float = 1.0,
        parameterization: int = 0,  # 0: Euler, 1: Rotation Vector
    ):
        """
        Set Cartesian orientation goal of `target_link` with respect to `frame_id`.
          - `frame_id` defaults to the base link
          - `target_link` defaults to end effector
        """
        pass
        
    def create_orientation_constraint(
        self,
        quat_xyzw: Union[Quaternion, Tuple[float, float, float, float]],
        frame_id: Optional[str] = None,
        target_link: Optional[str] = None,
        tolerance: Union[float, Tuple[float, float, float]] = 0.001,
        weight: float = 1.0,
        parameterization: int = 0,  # 0: Euler, 1: Rotation Vector
    ) -> OrientationConstraint:
        """
        Create a Cartesian orientation constraint of `target_link` with respect to `frame_id`.
          - `frame_id` defaults to the base link
          - `target_link` defaults to end effector
        """
        # TODO
        pass

    @classmethod
    def __init_move_action_goal(
        cls, frame_id: str, group_name: str, end_effector: str
    ) -> MoveGroup.Goal:
        move_action_goal = MoveGroup.Goal()  # refer to: http://docs.ros.org/en/noetic/api/moveit_msgs/html/action/MoveGroup.html
        move_action_goal.request.workspace_parameters.header.frame_id = frame_id
        # move_action_goal.request.workspace_parameters.header.stamp = "Set during request"
        move_action_goal.request.workspace_parameters.min_corner.x = -1.0
        move_action_goal.request.workspace_parameters.min_corner.y = -1.0
        move_action_goal.request.workspace_parameters.min_corner.z = -1.0
        move_action_goal.request.workspace_parameters.max_corner.x = 1.0
        move_action_goal.request.workspace_parameters.max_corner.y = 1.0
        move_action_goal.request.workspace_parameters.max_corner.z = 1.0
        # move_action_goal.request.start_state = "Set during request"
        move_action_goal.request.goal_constraints = [Constraints()]
        move_action_goal.request.path_constraints = Constraints()
        # move_action_goal.request.trajectory_constraints = "Ignored"
        # move_action_goal.request.reference_trajectories = "Ignored"
        move_action_goal.request.pipeline_id = ""
        move_action_goal.request.planner_id = ""
        move_action_goal.request.group_name = group_name
        move_action_goal.request.num_planning_attempts = 5
        move_action_goal.request.allowed_planning_time = 0.5
        move_action_goal.request.max_velocity_scaling_factor = 0.0
        move_action_goal.request.max_acceleration_scaling_factor = 0.0
        # Note: Attribute was renamed in Iron (https://github.com/ros-planning/moveit_msgs/pull/130)
        if hasattr(move_action_goal.request, "cartesian_speed_limited_link"):
            move_action_goal.request.cartesian_speed_limited_link = end_effector
        else:
            move_action_goal.request.cartesian_speed_end_effector_link = end_effector

        move_action_goal.request.max_cartesian_speed = 0.0
        # move_action_goal.planning_options.planning_scene_diff = "Ignored"
        move_action_goal.planning_options.plan_only = False
        # move_action_goal.planning_options.look_around = "Ignored"
        # move_action_goal.planning_options.look_around_attempts = "Ignored"
        # move_action_goal.planning_options.max_safe_execution_cost = "Ignored"
        # move_action_goal.planning_options.replan = "Ignored"
        # move_action_goal.planning_options.replan_attempts = "Ignored"
        # move_action_goal.planning_options.replan_delay = "Ignored"

        return move_action_goal


