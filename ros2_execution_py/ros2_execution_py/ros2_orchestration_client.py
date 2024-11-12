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
        self.goal_handle: ClientGoalHandle = None
        self.get_logger().info('Orchestration Action Client has been started')

    def send_goal(self, goal_data: dict):
        self._action_client.wait_for_server()
        goal_msg = Orchestration.Goal()
        goal_msg.goal = json.dumps(goal_data)
        self._action_client.\
            send_goal_async(goal_msg, feedback_callback=self.feedback_callback).\
            add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle: ClientGoalHandle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        self.goal_handle.get_result_async().add_done_callback(self.get_result_callback)
    
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
    
    def cancel_goal(self):
        if self.goal_handle is not None:
            self.get_logger().info('Send a cancel request')
            self.goal_handle.cancel_goal_async()


def main(args=None):
    rclpy.init(args=args)

    node = OrchestrationClient()

    while True:
        # read goal data from command line input
        goal_data_input = input('Enter goal data in JSON format (or type "exit" to quit): ')

        if goal_data_input.lower() == 'exit':
            print('Terminate process')
            rclpy.shutdown()
            break

        try:
            goal_data = json.loads(goal_data_input)
        except json.JSONDecodeError as e:
            print(f'Invalid JSON input: {e}')
            continue
        
        node.send_goal(goal_data)
        while rclpy.ok():
            rclpy.spin_once(node)

    exit(0)


if __name__ == '__main__':
    main()