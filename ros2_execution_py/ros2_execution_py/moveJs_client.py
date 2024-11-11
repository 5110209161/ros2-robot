#!/usr/bin/python3

from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float64, String
from rclpy.action.client import ClientGoalHandle, GoalStatus
from rclpy.task import Future

from ros2_data.action import MoveJs
from ros2_data.msg import JointPoseS

# API Doc: https://docs.ros2.org/foxy/api/rclpy/api/actions.html#module-rclpy.action.client

# Class: robot moves to specific waypoint, which is specified by Joint Pose values with one extra joint
class MoveJsclient(Node):
    def __init__(self):
        super().__init__('MoveJs_Client')
        self._action_client = ActionClient(self, MoveJs, 'MoveJs')
        self.get_logger().info('Waiting for MoveJs action server to be available...')
        self._action_client.wait_for_server()
        self.get_logger().info('MoveJs action server detected.')
        self.RES: String = 'null'

    def send_goal(self, GoalJPS: JointPoseS, JointSPEED: Float64):
        # assign variables
        goal_msg = MoveJs.Goal()
        goal_msg.goal = GoalJPS
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
        result: MoveJs.Result = future.result().result
        status: GoalStatus = future.result().status
        self.RES = result.result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'MoveJs action call successed, result: {str(result.result)}')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error(f'MoveJs action call aborted, result: {str(result.result)}')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(f'MoveJs action call canceled, result: {str(result.result)}')
        else:
            self.get_logger().info('MoveJs action call finished.')
    
    def feedback_callback(self, feedback_msg):
        feedback: MoveJs.Feedback = feedback_msg.feedback
        self.get_logger(f'MoveJs got feedback: {str(feedback.feedback)}')