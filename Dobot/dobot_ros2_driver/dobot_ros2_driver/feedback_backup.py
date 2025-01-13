#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy                          
from rclpy.node import Node                      
from dobot_ros2_msgs.msg import ToolVectorActual                  
from sensor_msgs.msg import JointState  
import socket
import numpy as np
import os


MyType = np.dtype([
  ('len',np.int64,), 
  ('digital_input_bits',np.uint64,), 
  ('digital_output_bits',np.uint64,), 
  ('robot_mode',np.uint64,), 
  ('time_stamp',np.uint64,), 
  ('time_stamp_reserve_bit', np.uint64,),
  ('test_value',np.uint64,), 
  ('test_value_keep_bit', np.float64,), 
  ('speed_scaling',np.float64,), 
  ('linear_momentum_norm',np.float64,),
  ('v_main',np.float64,), 
  ('v_robot',np.float64,), 
  ('i_robot',np.float64,), 
  ('i_robot_keep_bit1',np.float64,), 
  ('i_robot_keep_bit2',np.float64,),
  ('tool_accelerometer_values', np.float64, (3, )),
  ('elbow_position', np.float64, (3, )),
  ('elbow_velocity', np.float64, (3, )),
  ('q_target', np.float64, (6, )),
  ('qd_target', np.float64, (6, )),
  ('qdd_target', np.float64, (6, )),
  ('i_target', np.float64, (6, )),
  ('m_target', np.float64, (6, )),
  ('q_actual', np.float64, (6, )),
  ('qd_actual', np.float64, (6, )),
  ('i_actual', np.float64, (6, )),
  ('actual_TCP_force', np.float64, (6, )),
  ('tool_vector_actual', np.float64, (6, )),
  ('TCP_speed_actual', np.float64, (6, )),
  ('TCP_force', np.float64, (6, )),
  ('Tool_vector_target', np.float64, (6, )),
  ('TCP_speed_target', np.float64, (6, )),
  ('motor_temperatures', np.float64, (6, )),
  ('joint_modes', np.float64, (6, )),
  ('v_actual', np.float64, (6, )),
  ('hand_type', np.byte, (4,)),
  ('user', np.byte,),
  ('tool', np.byte,),
  ('run_queued_cmd', np.byte,),
  ('pause_cmd_flag', np.byte,),
  ('velocity_ratio', np.int8,),
  ('acceleration_ratio', np.int8,),
  ('jerk_ratio', np.int8,),
  ('xyz_velocity_ratio', np.int8,),
  ('r_velocity_ratio', np.int8,),
  ('xyz_acceleration_ratio', np.int8,),
  ('r_acceleration_ratio', np.int8,),
  ('xyz_jerk_ratio', np.int8,),
  ('r_jerk_ratio', np.int8,),
  ('brake_status', np.int8,),
  ('enable_status', np.int8,),
  ('drag_status', np.int8,),
  ('running_status', np.int8,),
  ('error_status',np.int8,),
  ('jog_status', np.int8,),
  ('robot_type', np.int8,),
  ('drag_button_signal', np.int8,),
  ('enable_button_signal', np.int8,),
  ('record_button_signal', np.int8,),
  ('reappear_button_signal', np.int8,),
  ('jaw_button_signal', np.int8,),
  ('six_force_online', np.int8,),
  ('reserve2', np.int8, (82,)),
  ('m_actual', np.float64, (6,)),
  ('load', np.float64,),
  ('center_x', np.float64,),
  ('center_y', np.float64,),
  ('center_z', np.float64,),
  ('user1', np.float64, (6,)),
  ('Tool1', np.float64, (6,)),
  ('trace_index', np.float64,),
  ('six_force_value', np.float64, (6,)),
  ('target_quaternion', np.float64, (4,)),
  ('actual_quaternion', np.float64, (4,)),
  ('reserve3',np.int8, (24,))
])


class Feedbacks():
  def __init__(self, ip, port):
    self.ip = ip
    self.port = port
    self.socket_feedback = None

    if self.port == 30005 or self.port == 30004:
      try:
        self.socket_feedback = socket.socket()
        self.socket_feedback.settimeout(1)
        self.socket_feedback.connect((self.ip, self.port))
      except socket.error as e:
        print(f"Failed to connect to {self.ip}:{self.port}, error: {e}")
        self.socket_feedback = None
    else:
      print("Invalid port number")
      os._exit(1)
  
  def validate_feedback(self):
    if self.socket_feedback is None:
      print("Socket connection not established")
      return ["NG"]

    try:
      self.socket_feedback.setblocking(True)
      self.all = self.socket_feedback.recv(10240)
      data = self.all[0:1440]

      a = np.frombuffer(data, dtype=MyType)
      if hex((a['test_value'][0])) == '0x123456789abcdef':
          tool_v = a['tool_vector_actual'][0]
          tool_j = a['q_target'][0]
      return [tool_v, tool_j]
    except Exception as e:
      print(f"Parsing feedbacks failed, error: {e}")
      return ["NG"]
    
class PublisherNode(Node):
  def __init__(self, name):
    super().__init__(name)
    self.IP = str(os.getenv("IP_address"))
    self.connect()

    self.pub = self.create_publisher(ToolVectorActual, "dobot_ros2_msgs/ToolVectorActual", 10)

  def connect(self):
    try:
      self.get_logger().info(f"Connecting to {self.IP}:30004")
      self.feed_v = Feedbacks(self.IP, 30004)
      self.get_logger().info("Connected")
    except: 
      self.get_logger().info("Failed to connect")
      os._exit(1)
  

    