## Current Repository
<p align="left">(<a href="https://github.com/UniversalRobots">link to repo</a>)</p>

### Supported UR Robot Type

- ur3
- ur3e
- ur5
- ur5e
- ur10
- ur10e
- ur16e
- ur20
- ur30

### Launch Simulation (RViz) Environment

```sh
ros2 launch ur_ros2_gazebo ur_visualize.launch.py ur_type:=<ur_type>
```

### Launch Simulation (Gazebo) Environment

```sh
ros2 launch ur_ros2_gazebo ur_simulation.launch.py ur_type:=<ur_type>
```

### Launch MoveIt2 + Simulation (Gazebo) Environment

```sh
ros2 launch ur_ros2_moveit2 ur_moveit.launch.py ur_type:=<ur_type>
```



## From UR Official Repository
<p align="left">(<a href="https://github.com/UniversalRobots">link to repo</a>)</p>

### Getting Started

1. **Install the driver package**
   <p align="left">(<a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver">link to repo</a>)</p>
   ```bash
   sudo apt-get install ros-humble-ur
   ```

2. **Start with mock hardware**
   <p align="left">(<a href="https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/usage/toc.html">link to usage</a>)</p>
   ```bash
   ros2 launch ur_robot_driver ur_control.launch.py ur_type:=<ur_type> robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true launch_rviz:=false
   ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=<ur_type> launch_rviz:=true
   ```

3. **Start with UR simulator**
   <p align="left">(<a href="https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/usage/simulation.html#usage-with-official-ur-simulator">link to usage</a>)</p>
   1. pull the docker image
   ```bash
   docker pull universalrobots/ursim_e-series
   ```
   
   2. run the image
   ```bash
   # VNC port: 5900
   # Web browser VNC port: 6080
   docker run --rm -it -p 5900:5900 -p 6080:6080 universalrobots/ursim_e-series
   ```
   You can view the polyscope GUI by opening http://192.168.56.101:6080/vnc.html.

   3. control simulate robot
   ```bash
   ros2 launch ur_robot_driver ur_control.launch.py ur_type:=<ur_type> robot_ip:=192.168.56.101 launch_rviz:=true
   ```


## From ros2_RobotSimulation Repository

Package ur3e_ros2_gazebo can launch Gazebo, but load with limited controllers, cannot be driveren by ru3e_ros2_moveit2.

Package ur3e_ros2_moveit2 can launch Moveit, and can also drive robot if launch ur_robot_driver package.

Recommand to launch UR Gazebo and Moveit from official repository / packages.

### Launch Simulation Environment

UR3:
```sh
ros2 launch ur3_ros2_gazebo ur3_simulation.launch.py
```

UR5:
```sh
ros2 launch ur5_ros2_gazebo ur5_simulation.launch.py
```

UR10:
```sh
ros2 launch ur10_ros2_gazebo ur10_simulation.launch.py
```

### Launch Gazebo + MoveIt!2 Environment

UR3:
```sh
ros2 launch ur3_ros2_moveit2 ur3.launch.py
```

UR5:
```sh
ros2 launch ur5_ros2_moveit2 ur5.launch.py
```

UR10:
```sh
ros2 launch ur10_ros2_moveit2 ur10.launch.py
```

### Launch Gazebo + MoveIt!2 Environment + ROS2 Robot Triggers/Actions

UR3:
```sh
ros2 launch ur3_ros2_moveit2 ur3_interface.launch.py
```

UR5:
```sh
ros2 launch ur5_ros2_moveit2 ur5_interface.launch.py
```

UR10:
```sh
ros2 launch ur10_ros2_moveit2 ur10_interface.launch.py
```