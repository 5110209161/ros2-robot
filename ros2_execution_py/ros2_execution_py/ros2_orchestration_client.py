#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, GoalResponse, CancelResponse
from rclpy.action.client import ClientGoalHandle, GoalStatus
from ros2_data.action import Orchestration

import json


class OrchestrationClient(Node):
    def __init__(self):
        super().__init__('orchestration_action_client')
        self._action_client = ActionClient(self, Orchestration, 'Orchestration')
        self.get_logger().info('Orchestration Action Client has been started')

    def send_goal(self, goal_data: dict):
        self._action_client.wait_for_server()
        goal_msg = Orchestration.Goal()
        goal_msg.goal = json.dumps(goal_data)
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle: ClientGoalHandle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Successed with result: {str(result.result)}')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error(f'Aborted with result: {str(result.result)}')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(f'Canceled with result: {str(result.result)}')
        self.get_logger().info(f'Result: {result.result}')
    
    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Got Feedback: {feedback_msg.feedback}')


def main(args=None):
    rclpy.init(args=args)
    node = OrchestrationClient()
    goal_data = {'actionType': 'MoveJ', 'payload': {'x': 123}}  # Example dynamic data
    node.send_goal(goal_data)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()