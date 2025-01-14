## From Aubo Robot Official Repository
<p align="left">(<a href="https://github.com/AuboRobot/aubo_ros2_driver">link to repo</a>)</p>

### supported Aubo robot type:
- cr3, cr5, cr7, cr10, cr12, cr16
- nova2, nova5

### Launch Simulation (RViz / Gazebo) Environment

show robot in RViz
```sh
ros2 launch dobot_ros2_gazebo dobot_visualize.launch.py robot_type:=<robot-type>
```

show robot in Gazebo
```sh
ros2 launch dobot_ros2_gazebo dobot_simulation.launch.py robot_type:=<robot-type>
```
control robot to move via ros2_control
```sh
ros2 topic pub --once /dobot_arm_controller/joint_trajectory trajectory_msgs/JointTrajectory "{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: ''
  },
  joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
  points: [
    {
      positions: [0.0, -1.57, 1.57, 0.0, 0.0, 0.0],
      velocities: [],
      accelerations: [],
      effort: [],
      time_from_start: {sec: 2, nanosec: 0}
    }
  ]
}"
```

### Launch MoveIt!2 Environment

unified launch entry
```sh
ros2 launch dobot_ros2_moveit2 dobot_moveit.launch.py robot_type:=<robot_type>
```

cr3
```sh
ros2 launch cr3_moveit_config demo.launch.py
ros2 launch cr3_moveit_config cr3_moveit.launch.py
```

other robot (crx)
```sh
ros2 launch crx_moveit_config crx_moveit.launch.py
```

### Launch Gazebo + MoveIt!2 Environment + ROS2 Robot Triggers/Actions

