#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from ros2_data.action import Orchestration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import json


class OrchestrationServer(Node):
    def __init__(self):
        super().__init__('orchestration_action_server')
        self._action_server = ActionServer(
            self, Orchestration, 'Orchestration',
            goal_callback = self.goal_callback,
            cancel_callback = self.cancel_callback,
            handle_accepted_callback = self.handle_accepted_callback,
            execute_callback = self.execute_callback,
            callback_group=ReentrantCallbackGroup()
        )
        self.get_logger().info('Orchestration Action Server has been started')
    
    def goal_callback(self, goal_request: Orchestration.Goal):
        goal_data = json.loads(goal_request.goal)
        self.get_logger().info(f'Received goal request: {goal_data}')
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info('Received cancel request.')
        return CancelResponse.ACCEPT
    
    def handle_accepted_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info('Executing goal...')
        goal_handle.execute()
    
    def execute_callback(self, goal_handle: ServerGoalHandle):
        goal_data = json.loads(goal_handle.request.goal)
        action_type = goal_data['actionType']
        payload = goal_data['payload']
        self.get_logger().info(f'Start action {action_type} with specified payload {payload}')
        result = Orchestration.Result()
        result.result = 'SUCCESS'
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = OrchestrationServer()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
