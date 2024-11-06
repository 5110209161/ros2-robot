<div align="center">
    <h2 align="center">ROS2.0 ROBOT SIMULATION - ROS2.0 Humble</h2>
</div>

<!-- ABOUT THE PROJECT -->
## About

### ros2_RobotSimulation Repository

<p align="left">(<a href="https://github.com/IFRA-Cranfield/ros2_RobotSimulation.git">link to repo</a>)</p>

ROS (Robotics Operating System) is a great tool that, combined with Gazebo and MoveIt! frameworks, provides a great amount of resources and capabilities to develop different Robotics Applications with a huge range of Industrial and Collaborative Robots.

Nonetheless, developing new applications in ROS requires a huge previous work, which consists in developing ROS Packages that contain all the Robot data required for the Simulation and Control, including:
  - Kinematics.
  - Control parametres.
  - CAD files.
  - Physical parametres.
  - Joint limits.
  - Extra features.

As a common rule, the software stack needed to execute and test an Industrial Robot Simulation in ROS is:
  - A "robot_gazebo" package, which simulates the behaviour of the Robot.
  - A "robot_moveit" package, which controls the performance of the Robot.
With both combined, different applications and implementations can be developed using the Robotics Operating System (ROS).

ROS is now undertaking a transformation process to ROS2, a newer and improved version of ROS1. Thus, this involves that all the developments, implementations and packages released for ROS1 have to be forked/translated to ROS2 or are directly unavailable in ROS2.

The IFRA Group in Cranfield University (Bedfordshire, UK) has identified a huge gap in the availability of "ready-to-use" ROS2 Industrial Robot Simulation packages, and that is why the ros2_RobotSimulation ROS2 repository has been developed and released. The repository consists of Gazebo (simulation) + MoveIt!2 (control) package combinations for each supported Industrial Robot (or Robot + Gripper combinations), and follows a common standard for a better understanding and further development.

### Why make changes to ros2_RobotSimulation Repository

Part of the robots are still not working:

- UR5 Robot
- UR10 robot
- Fanuc CR35iA (poor model)

So in my repository, reuse the working robots and follow its structure to support more robots.