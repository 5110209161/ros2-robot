## From Techman Official Repository
<p align="left">(<a href="https://github.com/TechmanRobotInc/tmr_ros2">link to repo</a>)</p>

### Supported TM Robot Type

- tm5-900

### Launch Simulation (RViz / Gazebo) Environment

```sh
ros2 launch tm_ros2_gazebo tm_visualize.launch.py tm_type:=<robot-type>
```

supported TM robot type:
- tm7-700
- tm5-900
- tm5x-700
- tm5x-900
- tm12
- tm12x
- tm14
- tm14x

### Launch MoveIt!2 Environment

tm5-900:
```sh
ros2 launch tm5-900_moveit_config tm5-900_moveit.launch.py
```

### Launch Gazebo + MoveIt!2 Environment + ROS2 Robot Triggers/Actions

tm5-900:
```sh
ros2 launch tm5-900_moveit_config tm5-900_interface.launch.py
```