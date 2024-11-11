#!/usr/bin/python3

from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float64, String
from rclpy.action.client import ClientGoalHandle, GoalStatus
from rclpy.task import Future

from ros2_data.action import MoveXYZ

# API Doc: https://docs.ros2.org/foxy/api/rclpy/api/actions.html#module-rclpy.action.client

# Class: robot moves to specific waypoint -> Position(x, y, z) maintaining the end-effector orientation
class MoveXYZclient(Node):
    def __init__(self):
        super().__init__('MoveXYZ_Client')
        self._action_client = ActionClient(self, MoveXYZ, 'MoveXYZ')
        self.get_logger().info('Waiting for MoveXYZ action server to be available...')
        self._action_client.wait_for_server()
        self.get_logger().info('MoveXYZ action server detected.')
        self.RES: String = 'null'

    def send_goal(self, GoalXYZx: Float64, GoalXYZy: Float64, GoalXYZz: Float64, JointSPEED: Float64):
        # assign variables
        goal_msg = MoveXYZ.Goal()
        goal_msg.positionx = GoalXYZx
        goal_msg.positiony = GoalXYZy
        goal_msg.positionz = GoalXYZz
        goal_msg.speed = JointSPEED
        # action call
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future: Future):
        goal_handle: ClientGoalHandle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future: Future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future: Future):
        result: MoveXYZ.Result = future.result().result
        status: GoalStatus = future.result().status
        self.RES = result.result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'MoveXYZ action call successed, result: {str(result.result)}')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error(f'MoveXYZ action call aborted, result: {str(result.result)}')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(f'MoveXYZ action call canceled, result: {str(result.result)}')
        else:
            self.get_logger().info('MoveXYZ action call finished.')
    
    def feedback_callback(self, feedback_msg):
        feedback: MoveXYZ.Feedback = feedback_msg.feedback
        self.get_logger(f'MoveXYZ got feedback: {str(feedback.feedback)}')