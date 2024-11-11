#!/usr/bin/python3

from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float64, String
from rclpy.action.client import ClientGoalHandle, GoalStatus
from rclpy.task import Future

from ros2_data.action import MoveRP

# API Doc: https://docs.ros2.org/foxy/api/rclpy/api/actions.html#module-rclpy.action.client

# Class: end-effector rotation AROUND A POINT
# the robot rotates / orientates + moves the end-effector frame according to EulerAngle(yaw, pitch, roll) + Point(x, y, z)
# the ROT determines the ADDED ROTATION of the end-effector which is applied to the END-EFFECTOR coordinate frame AROUND the POINT
class MoveRPclient(Node):
    def __init__(self):
        super().__init__('MoveRP_Client')
        self._action_client = ActionClient(self, MoveRP, 'MoveRP')
        self.get_logger().info('Waiting for MoveRP action server to be available...')
        self._action_client.wait_for_server()
        self.get_logger().info('MoveRP action server detected.')
        self.RES: String = 'null'

    def send_goal(self, GoalRPyaw: Float64, GoalRPpitch: Float64, GoalRProll: Float64, 
                  GoalRPx: Float64, GoalRPy: Float64, GoalRPz: Float64, JointSPEED: Float64):
        # assign variables
        goal_msg = MoveRP.Goal()
        goal_msg.yaw = GoalRPyaw
        goal_msg.pitch = GoalRPpitch
        goal_msg.roll = GoalRProll
        goal_msg.x = GoalRPx
        goal_msg.y = GoalRPy
        goal_msg.z = GoalRPz
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
        result: MoveRP.Result = future.result().result
        status: GoalStatus = future.result().status
        self.RES = result.result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'MoveRP action call successed, result: {str(result.result)}')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error(f'MoveRP action call aborted, result: {str(result.result)}')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(f'MoveRP action call canceled, result: {str(result.result)}')
        else:
            self.get_logger().info('MoveRP action call finished.')
    
    def feedback_callback(self, feedback_msg):
        feedback: MoveRP.Feedback = feedback_msg.feedback
        self.get_logger(f'MoveRP got feedback: {str(feedback.feedback)}')