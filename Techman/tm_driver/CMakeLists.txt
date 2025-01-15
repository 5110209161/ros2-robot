cmake_minimum_required(VERSION 3.8)
project(tm_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tm_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
set(moveit2_lib_auto_judge true)
if (moveit2_lib_auto_judge)
  find_package(moveit_ros_planning_interface QUIET)
endif()
if (moveit2_lib_auto_judge)
  if ( NOT moveit_ros_planning_interface_FOUND)
    if (tm_driver_message_print)
    message("${PROJECT_NAME}:--- TMDriver build error : MoveIt2_LIB not found")
    #do you forget to source moveit2 relative setup.bash? example: & ~/COLCON_WS/install/setup.bash
    endif()  
  endif (NOT moveit_ros_planning_interface_FOUND)
endif()

if (moveit2_lib_auto_judge) 
  if (moveit_ros_planning_interface_FOUND)
    find_package(trajectory_msgs REQUIRED)
    find_package(control_msgs REQUIRED)
    find_package(rclcpp_action REQUIRED)
  endif (moveit_ros_planning_interface_FOUND)
endif()

include_directories(include
  ${CMAKE_CURRENT_SOURCE_DIR}
)

if (moveit2_lib_auto_judge) 
  if (moveit_ros_planning_interface_FOUND)
    add_executable(tm_driver
      #src/tm_ros2_node.cpp
      src/tm_ros2_composition_moveit.cpp
      src/tm_ros2_sct.cpp
      src/tm_ros2_svr.cpp
      src/tm_driver.cpp
      src/tm_command.cpp
      src/tm_sct_communication.cpp
      src/tm_svr_communication.cpp
      src/tm_communication.cpp
      src/tm_robot_state.cpp
      src/tm_packet.cpp
      src/tm_print.cpp
      src/tm_driver_utilities.cpp
      src/tm_listen_node_connect.cpp
      src/tm_ethernet_slave_connect.cpp
      src/tm_ros2_movit_sct.cpp
    )
    ament_target_dependencies(tm_driver
      rclcpp
      std_msgs
      sensor_msgs
      tm_msgs
      tf2_geometry_msgs
      trajectory_msgs
      control_msgs
      rclcpp_action
    )
  else()
    add_executable(tm_driver
      src/tm_ros2_composition.cpp
      src/tm_ros2_sct.cpp
      src/tm_ros2_svr.cpp
      src/tm_driver.cpp
      src/tm_command.cpp
      src/tm_sct_communication.cpp
      src/tm_svr_communication.cpp
      src/tm_communication.cpp
      src/tm_robot_state.cpp
      src/tm_packet.cpp
      src/tm_print.cpp
      src/tm_driver_utilities.cpp
      src/tm_listen_node_connect.cpp
      src/tm_ethernet_slave_connect.cpp
    )
    ament_target_dependencies(tm_driver
      rclcpp
      std_msgs
      sensor_msgs
      tm_msgs
      tf2_geometry_msgs
    )
  endif (moveit_ros_planning_interface_FOUND)
else()
  add_executable(tm_driver
    src/tm_ros2_composition.cpp
    src/tm_ros2_sct.cpp
    src/tm_ros2_svr.cpp
    src/tm_driver.cpp
    src/tm_command.cpp
    src/tm_sct_communication.cpp
    src/tm_svr_communication.cpp
    src/tm_communication.cpp
    src/tm_robot_state.cpp
    src/tm_packet.cpp
    src/tm_print.cpp
    src/tm_driver_utilities.cpp
    src/tm_listen_node_connect.cpp
    src/tm_ethernet_slave_connect.cpp
  )
  ament_target_dependencies(tm_driver
    rclcpp
    std_msgs
    sensor_msgs
    tm_msgs
    tf2_geometry_msgs
  )
endif()
  
install(TARGETS
  tm_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
