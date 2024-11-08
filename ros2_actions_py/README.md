## moveit2 Python interface
Refer to: https://github.com/moveit/moveit2/tree/main/moveit_py

The ROS2 package is still under construction and not ready to use, other optional project also provides Moveit2 Python interface, more details refer to: https://github.com/moveit/moveit2/issues/1279

This project based on [pymoveit2](https://github.com/AndrejOrsula/pymoveit2) that execute different robot / end-effector.

## ROS2 Actions (implementation in Python)
The Robot Actions/Triggers are independent ROS2 Actions that execute different robot/end-effector motions in Gazebo simulation. The list below explains what every single ROS2 Action does, and how the actions are executed independently using the Ubuntu Terminal:

* MoveJ: The Robot moves to the specific waypoint, which is specified by Joint Pose values.
  ```sh
  ros2 action send_goal -f /MoveJ ros2_data/action/MoveJ "{goal: {joint1: 0.00, joint2: 0.00, joint3: 0.00, joint4: 0.00, joint5: 0.00, joint6: 0.00}, speed: 1.0}" # (6-DOF)
  ros2 action send_goal -f /MoveJs ros2_data/action/MoveJs "{goal: {joint1: 0.00, joint2: 0.00, joint3: 0.00, joint4: 0.00, joint5: 0.00, joint6: 0.00, joint7: 0.00}, speed: 1.0}" # (7-DOF)
  ```