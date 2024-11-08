#!/usr/bin/python3

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
import rclpy.executors
from rclpy.node import Node

from pymoveit2 import MoveIt2

class Ros2RobotTrigger(Node):
    def __init__(self):
        super().__init__('ros2_robot_trigger_param')
        self.declare_parameter('ROB_PARAM', 'null')
        self.my_param = self.get_parameter('ROB_PARAM').get_parameter_value().string_value
        self.get_logger().info(f'ROB_PARAM received -> {self.my_param}')


def main(args=None):
    rclpy.init(args=args)
    node = Ros2RobotTrigger()
    rclpy.spin_once(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()