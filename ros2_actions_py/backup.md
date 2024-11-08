```python
#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from ros2_data.action import MoveJ
from ros2_data.msg import JointPose
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander



class Ros2RobotTrigger(Node):
    def __init__(self):
        super().__init__('ros2_robot_trigger_param')
        self.declare_parameter('ROB_PARAM', 'null')
        self.my_param = self.get_parameter('ROB_PARAM').get_parameter_value().string_value
        self.get_logger().info(f'ROB_PARAM received -> {self.my_param}')

class MoveJActionServer(Node):
    def __init__(self):
        super().__init__('movej_action_server')
        self._action_server = ActionServer(
            self, MoveJ, 'MoveJ',
            self.handle_goal,
            self.handle_cancel,
            self.handle_accepted 
        )
        self.move_group = MoveGroupCommander(self.my_param)
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
    
    def handle_goal(self, goal_handle):
        joint_goal: JointPose = goal_handle.request.goal
        self.get_logger().info(f'Received a goal request, with JointPose -> (
            {joint_goal.joint1:.2f},{joint_goal.joint2:.2f},{joint_goal.joint3:.2f},
            {joint_goal.joint4:.2f},{joint_goal.joint5:.2f},{joint_goal.joint6:.2f})'
        )
        return GoalResponse.ACCEPT
    
    def handle_cancel(self, goal_handle):
        self.get_logger().info('Received a cancel request.')
        self.move_group.stop()
        return CancelResponse.ACCEPT
    
    def handle_accepted(self, goal_handle):
        self.get_logger().info('Starting MoveJ motion to desired waypoint...')
        self.execute(goal_handle)

    def execute(self, goal_handle):
        goal = goal_handle.request.goal
        result = MoveJ.Result()

        # Check joint limits
        joint_limits = {
            "irb120_arm": [(165, -165), (110, -110), (70, -110), (160, -160), (120, -120), (400, -400)],
            "irb1200_arm": [(170, -170), (130, -100), (70, -200), (270, -270), (130, -130), (360, -360)],
            "irb6640_arm": [(170, -170), (85, -65), (70, -180), (300, -300), (120, -120), (360, -360)],
            "cr35ia_arm": [(170, -170), (120, 45), (135, -122), (200, -200), (110, -110), (450, -450)],
            "ur3_arm": [(360, -360), (360, -360), (180, -180), (360, -360), (360, -360), (360, -360)],
            "ur5_arm": [(360, -360), (360, -360), (180, -180), (360, -360), (360, -360), (360, -360)],
            "ur10_arm": [(360, -360), (360, -360), (180, -180), (360, -360), (360, -360), (360, -360)]
        }

        limits = joint_limits.get(self.my_param, [(float('inf'), float('-inf'))] * 6)
        joints = [goal.joint1, goal.joint2, goal.joint3, goal.joint4, goal.joint5, goal.joint6]
        limit_check = all(lower <= joint <= upper for joint, (upper, lower) in zip(joints, limits))

        if not limit_check:
            self.get_logger().info(f'{self.my_param} - MoveJ: Planning failed, JOINT LIMITS exceeded!')
            result.result = 'MoveJ:FAILED'
            goal_handle.succeed(result)
            return
        
        # Set joint target and plan
        self.move_group.set_joint_value_target(joints)
        plan = self.move_group.plan()

        if plan:
            self.get_logger().info(f'{self.my_param} - MoveJ: Planning successful!')
            self.move_group.go(wait=True)

            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled.')
                result.result = 'MoveJ:CANCELED'
                goal_handle.canceled(result)
            else:
                self.get_logger().info(f'{self.my_param} - MoveJ: Movement executed!')
                result.result = 'MoveJ:SUCCESS'
                goal_handle.succeed(result)
        else:
            self.get_logger().info(f'{self.my_param} - MoveJ: Planning failed!')
            result.result = 'MoveJ:FAILED'
            goal_handle.succeed(result)


def main(args=None):
    rclpy.init(args=args)

    # Obtain ros2_RobotTrigger parameter
    node_param = Ros2RobotTrigger()
    rclpy.spin_once(node_param)

    # Create and spin the action server
    action_server = MoveJActionServer()
    rclpy.spin(action_server)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
    

```