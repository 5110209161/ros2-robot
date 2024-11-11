#!/usr/bin/python3

from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float64, String
from rclpy.action.client import ClientGoalHandle, GoalStatus
from rclpy.task import Future

from ros2_data.action import MoveL

# API Doc: https://docs.ros2.org/foxy/api/rclpy/api/actions.html#module-rclpy.action.client

# Class: robot executes a Linear / Cartesian path, the end-effector orientation is kept constant, and position changes by x, y, z
class MoveLclient(Node):
    def __init__(self):
        super().__init__('MoveL_Client')
        self._action_client = ActionClient(self, MoveL, 'MoveL')
        self.get_logger().info('Waiting for MoveL action server to be available...')
        self._action_client.wait_for_server()
        self.get_logger().info('MoveL action server detected.')
        self.RES: String = 'null'

    def send_goal(self, GoalLx: Float64, GoalLy: Float64, GoalLz: Float64, JointSPEED: Float64):
        # assign variables
        goal_msg = MoveL.Goal()
        goal_msg.movex = GoalLx
        goal_msg.movey = GoalLy
        goal_msg.movez = GoalLz
        goal_msg.speed = JointSPEED
        # action call
        self._send_goal_future: Future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
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
        result: MoveL.Result = future.result().result
        status: GoalStatus = future.result().status
        self.RES = result.result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'MoveL action call successed, result: {str(result.result)}')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error(f'MoveL action call aborted, result: {str(result.result)}')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(f'MoveL action call canceled, result: {str(result.result)}')
        else:
            self.get_logger().info('MoveL action call finished.')
        self._get_result_future 

    def feedback_callback(self, feedback_msg):
        feedback: MoveL.Feedback = feedback_msg.feedback
        self.get_logger(f'MoveL got feedback: {str(feedback.feedback)}')

