#!/usr/bin/python

import os
import time
import ast
import rclpy
from rclpy.node import Node
from ros2_data.msg import JointPose, JointPoseS

from ros2_execution_py.moveJ_client import MoveJclient
from ros2_execution_py.moveJs_client import MoveJsclient
from ros2_execution_py.moveG_client import MoveGclient
from ros2_execution_py.moveL_client import MoveLclient
from ros2_execution_py.moveR_client import MoveRclient
from ros2_execution_py.moveXYZW_client import MoveXYZWclient
from ros2_execution_py.moveXYZ_client import MoveXYZclient
from ros2_execution_py.moveYPR_client import MoveYPRclient
from ros2_execution_py.moveROT_client import MoveROTclient
from ros2_execution_py.moveRP_client import MoveRPclient


# Define GLOBAL VARIABLE -> RES:
RES = "null"


# CLASS: Warning + Close
class CloseProgram():
    def CLOSE():
        print('')
        print('Please execute the program and input all ROS2 parameters in the Ubuntu Terminal as stated below:')
        print('COMMAND -> ros2 run ros2_execution ros2_execution.py --ros-args -p PROGRAM_FILENAME:="---" -p ROBOT_MODEL:="---" -p EE_MODEL:="---"')
        print("Closing... BYE!")
        time.sleep(5)
        exit()


# CLASS: Input program (.txt) as ROS2 PARAMETER:
PARAM_PROGRAM = 'default'
P_CHECK_PROGRAM = False
class ProgramPARAM(Node):
    def __init__(self):
        global PARAM_PROGRAM
        global P_CHECK_PROGRAM

        super().__init__('ros2_program_param')
        self.declare_parameter('PROGRAM_FILENAME', 'default')
        PARAM_PROGRAM = self.get_parameter('PROGRAM_FILENAME').get_parameter_value().string_value
        if (PARAM_PROGRAM == 'default'):
            self.get_logger().error('PROGRAM_FILENAME ROS2 Parameter was not defined.')
            CloseProgram.CLOSE()
        else:
            self.get_logger().info('PROGRAM_FILENAME ROS2 Parameter received: ' + PARAM_PROGRAM)
        P_CHECK_PROGRAM = True


# CLASS: Input ROBOT MODEL as ROS2 PARAMETER:
PARAM_ROBOT = 'default'
P_CHECK_ROBOT = False
ROBOT_LIST = ['irb120', 'irb1200', 'irb6640', 'cr35ia', 'ur3', 'ur5', 'ur10', 'panda', 'iiwa']
class RobotPARAM(Node):
    def __init__(self):
        global PARAM_ROBOT
        global P_CHECK_ROBOT

        super().__init__('ros2_robot_param')
        self.declare_parameter('ROBOT_MODEL', 'default')
        PARAM_ROBOT = self.get_parameter('ROBOT_MODEL').get_parameter_value().string_value
        if (PARAM_ROBOT == 'default'):
            self.get_logger().error('ROBOT_MODEL ROS2 Parameter was not defined.')
            CloseProgram.CLOSE()
        else:
            self.get_logger().info('ROBOT_MODEL ROS2 Parameter received: ' + PARAM_ROBOT)
            # check value
            if (PARAM_ROBOT in ROBOT_LIST):
                None  # do nothing
            else:
                self.get_logger().error('ERROR: The Robot model defined is not in the system.')
                CloseProgram.CLOSE()
        P_CHECK_ROBOT = True


# CLASS: Input ROBOT End-Effector MODEL as ROS2 PARAMETER:
PARAM_EE = 'default'
P_CHECK_EE = False
EE_LIST = ['schunk', 'panda_hand', 'none']
class eePARAM(Node):
    def __init__(self):
        global PARAM_EE
        global P_CHECK_EE

        super().__init__('ros2_ee_param')
        self.declare_parameter('EE_MODEL', 'default')
        PARAM_EE = self.get_parameter('EE_MODEL').get_parameter_value().string_value
        if (PARAM_EE == 'default'):
            self.get_logger().error('EE_MODEL ROS2 Parameter was not defined.')
            CloseProgram.CLOSE()
        else:
            self.get_logger().info('EE_MODEL ROS2 Parameter received: ' + PARAM_EE)
            # check value
            if (PARAM_EE in EE_LIST):
                None  # do nothing
            else:
                self.get_logger().error('ERROR: The End-Effector model defined is not in the system.')
                CloseProgram.CLOSE()
        P_CHECK_EE = True

PARAM_WS = 'default'  # workspace name
P_CHECK_PATH = False
class PathPARAM(Node):
    def __init__(self):
        global PARAM_WS
        global P_CHECK_PATH

        super().__init__('ros2_path_param')

        self.declare_parameter('WORKSPACE_NAME', 'default')
        PARAM_WS = self.get_parameter('WORKSPACE_NAME').get_parameter_value().string_value
        if (PARAM_WS == 'default'):
            self.get_logger().error('WORKSPACE_NAME ROS2 Parameter was not defined.')
            CloseProgram.CLOSE()
        else:
            self.get_logger().info('WORKSPACE_NAME ROS2 Parameter received: ' + PARAM_WS)
        
        P_CHECK_PATH = True


def main(args=None):
    # import global variables RES
    global RES

    rclpy.init(args=args)
    # Create NODE for LOGGING:
    nodeLOG = rclpy.create_node('node_LOG')

    print('')
    print('Python script -> ros2_execution.py')
    print('')

    # 1. initialize received ROS2 parameters
    global PARAM_PROGRAM
    global P_CHECK_PROGRAM
    global PARAM_ROBOT
    global P_CHECK_ROBOT
    global PARAM_EE
    global P_CHECK_EE
    global PARAM_WS
    global P_CHECK_PATH

    paramNODE = ProgramPARAM()
    while (P_CHECK_PROGRAM == False):
        rclpy.spin_once(paramNODE)
    paramNODE.destroy_node()

    robotNODE = RobotPARAM()
    while (P_CHECK_ROBOT == False):
        rclpy.spin_once(robotNODE)
    robotNODE.destroy_node()

    eeNODE = eePARAM()
    while (P_CHECK_EE == False):
        rclpy.spin_once(eeNODE)
    eeNODE.destroy_node()

    pathNODE = PathPARAM()
    while (P_CHECK_PATH == False):
        rclpy.spin_once(pathNODE)
    pathNODE.destroy_node()

    # 2. Load components --> According to input ROS2 Parameters:
    print('')

    # MoveJ or MoveJs, depending on the DOF of the robot:
    if (PARAM_ROBOT in ['irb120', 'irb1200', 'irb6640', 'cr35ia', 'ur3', 'ur5', 'ur10']):
        MoveJ_CLIENT = MoveJclient()
    elif (PARAM_ROBOT in ['panda', 'iiwa']):
        MoveJs_CLIENT = MoveJsclient()
    
    # if parallel gripper: MoveG and attacher plugin activated
    if (PARAM_EE == 'schunk' or PARAM_EE == 'panda_hand'):
        MoveG_CLIENT = MoveGclient()
        # FIXME
        # Attach_Client = ATTACHERclient()
        # Detach_Client = DetacherPUB()
    
    MoveL_CLIENT = MoveLclient()
    MoveR_CLIENT = MoveRclient()
    MoveXYZW_CLIENT = MoveXYZWclient()
    MoveXYZ_CLIENT = MoveXYZclient()
    MoveYPR_CLIENT = MoveYPRclient()
    MoveROT_CLIENT = MoveROTclient()
    MoveRP_CLIENT = MoveRPclient()

    print('')
    print('All checks complete!')
    print('')

    # 3. Get program filename
    EXISTS = False
    PR_NAME = PARAM_PROGRAM
    filepath = os.path.join(os.path.expanduser('~'), PARAM_WS, 'src', 'ros2-robot', 'ros2_execution_py', 'programs', PR_NAME + '.txt')
    EXISTS = os.path.exists(filepath)
    if (EXISTS == True):
        print(PR_NAME + ' file found! Executing program...')
        nodeLOG.get_logger().info('SUCCESS: ' + PR_NAME + ' file (program) found.')
        time.sleep(1)
    elif (EXISTS == False):
        print(PR_NAME + ' file not found. Please input the PROGRAM FILENAME correctly as a ROS2 parameter in the Ubuntu Terminal:')
        nodeLOG.get_logger().info('ERROR: " + PR_NAME + " file (program) not found. Please try again.')
        print('COMMAND -> ROS2 run ros2_execution ros2_execution_file.py --ros-args '
              + '-p WORKSPACE_NAME:="---" -p PACKAGE_NAME:="---" -p PROGRAM_FILENAME:="---" -p ROBOT_MODEL:="---" -p EE_MODEL:="---"')
        print('Closing... BYE!')
        time.sleep(5)
        exit()
    # Open PR_NAME.txt file
    with open(filepath) as file:
        f = file.readlines()
        i = 1
        seq = dict()
        for line in f:
            seq[str(i)] = ast.literal_eval(line)
            i = i + 1
        file.close()
    # Log number of steps
    nodeLOG.get_logger().info(PR_NAME + ': Number of steps -> ' + str(len(seq)))
    time.sleep(1)

    # 4. Move sequence --> According to read program file
    for i in range(1, len(seq) + 1):
        trigger = seq[str(i)]

        if (trigger['action'] == 'MoveJ'):
            print('')
            print('STEP NUMBER ' + str(i) + ' -> MoveJ:')
            print(trigger['value'])

            # joint speed
            JointSPEED = trigger['speed']
            if (JointSPEED <= 0 or JointSPEED > 1):
                print('Joint speed -> ' + str(JointSPEED) + ' not valid. Must be (0, 1]. Assigned: 0.01')
                JointSPEED = 0.01
            else:
                print('Joint speed -> ' + str(JointSPEED))
            
            JP = JointPose()
            JP.joint1 = trigger['value']['joint1']
            JP.joint2 = trigger['value']['joint2']
            JP.joint3 = trigger['value']['joint3']
            JP.joint4 = trigger['value']['joint4']
            JP.joint5 = trigger['value']['joint5']
            JP.joint6 = trigger['value']['joint6']
            MoveJ_CLIENT.send_goal(JP, JointSPEED)

            while rclpy.ok():
                rclpy.spin_once(MoveJ_CLIENT)
                RES = MoveJ_CLIENT.RES
                if (RES != 'null'):
                    break

            print('Result of MoveJ action call is -> { ' + RES + ' }')
            if (RES == 'MoveJ:SUCCESS'):
                print('MoveJ ACTION in step number -> ' + str(i) + ' successfully executed.')
                RES = 'null'
            else:
                print('MoveJ ACTION in step number -> ' + str(i) + ' failed.')
                print('The program will be closed. Bye!')
                nodeLOG.get_logger().error('ERROR: Program finished since MoveJ ACTION in step number -> ' + str(i) + ' failed.')
                break
        
        elif (trigger['action'] == 'MoveJs'):
            print('')
            print('STEP NUMBER ' + str(i) + ' -> MoveJs:')
            print(trigger['value'])

            # Joint speed
            JointSPEED = trigger['speed']
            if (JointSPEED <= 0 or JointSPEED > 1):
                print('Joint speed -> ' + str(JointSPEED) + ' not valid. Must be (0, 1]. Assigned: 0.01')
                JointSPEED = 0.01
            else:
                print('Joint speed -> ' + str(JointSPEED))
            
            JPS = JointPoseS()
            JPS.joint1 = trigger['value']['joint1']
            JPS.joint2 = trigger['value']['joint2']
            JPS.joint3 = trigger['value']['joint3']
            JPS.joint4 = trigger['value']['joint4']
            JPS.joint5 = trigger['value']['joint5']
            JPS.joint6 = trigger['value']['joint6']
            JPS.joint7 = trigger['value']['joint7']
            MoveJs_CLIENT.send_goal(JPS, JointSPEED)

            while rclpy.ok():
                rclpy.spin_once(MoveJs_CLIENT)
                RES = MoveJs_CLIENT.RES
                if (RES != 'null'):
                    break
            
            print('Result of MoveJs action call is -> { ' + RES + ' }')

            if (RES == 'MoveJs:SUCCESS'):
                print('MoveJs action in step number -> ' + str(i) + ' successfully executed.')
                RES = 'null'
            else:
                print('MoveJs action in step number -> ' + str(i) + ' failed.')
                print('The program will be closed. Bye!')
                nodeLOG.get_logger().error('ERROR: Program finished since MoveJs action in step number -> ' + str(i) + ' failed.')
                break
        
        elif (trigger['action'] == 'MoveL'):
            print('')
            print('STEP NUMBER ' + str(i) + ' -> MoveL:')
            print(trigger['value'])

            # joint speed
            JointSPEED = trigger['speed']
            if (JointSPEED <= 0 or JointSPEED > 1):
                print('Joint speed -> ' + str(JointSPEED) + ' not valid. Must be (0, 1]. Assigned: 0.01')
                JointSPEED = 0.01
            else:
                print('Joint speed -> ' + str(JointSPEED))
            
            MoveX = trigger['value']['movex']
            MoveY = trigger['value']['movey']
            MoveZ = trigger['value']['movez']
            MoveL_CLIENT.send_goal(MoveX, MoveY, MoveZ, JointSPEED)

            while rclpy.ok():
                rclpy.spin_once(MoveL_CLIENT)
                RES = MoveL_CLIENT.RES
                if (RES != 'null'):
                    break
            
            print('Result of MoveL action call is -> { ' + RES + " }")

            if (RES == 'MoveL:SUCCESS'):
                print('MoveL action in step number -> ' + str(i) + ' successfully executed.')
                RES = 'null'
            else:
                print('MoveL action in step number -> ' + str(i) + ' failed.')
                print('The program will be closed. Bye!')
                nodeLOG.get_logger().error('ERROR: Program finished since MoveL action in step number -> ' + str(i) + ' failed')
                break

        elif (trigger['action'] == 'MoveR'):
            print('')
            print('STEP NUMBER ' + str(i) + ' -> MoveR:')
            print(trigger['value'])

            # joint speed
            JointSPEED = trigger['speed']
            if (JointSPEED <= 0 or JointSPEED > 1):
                print('Joint speed -> ' + str(JointSPEED) + ' not valid. Must be (0, 1]. Assigned: 0.01')
                JointSPEED = 0.01
            else:
                print('Joint speed -> ' + str(JointSPEED))
            
            joint = trigger['value']['joint']
            value = trigger['value']['value']
            MoveR_CLIENT.send_goal(joint,value, JointSPEED)

            while rclpy.ok():
                rclpy.spin_once(MoveR_CLIENT)
                RES = MoveR_CLIENT.RES
                if (RES != 'null'):
                    break
            
            print('Result of MoveR action call is -> { ' + RES + ' }')

            if (RES == 'MoveR:SUCCESS'):
                print('MoveR action in step number -> ' + str(i) + ' successfully executed.')
                RES = 'null'
            else:
                print('MoveR action in step number -> ' + str(i) + ' failed.')
                print('The program will be clised. Bye!')
                nodeLOG.get_logger().error('ERROR: Program finished since MoveR action in step number -> ' + str(i) + ' failed.')
                break
        
        elif (trigger['action'] == 'MoveXYZW'):
            print('')
            print('STEP NUMBER ' + str(i) + ' -> MoveXYZW:')
            print(trigger['value'])

            # joint speed
            JointSPEED = trigger['speed']
            if (JointSPEED <= 0 or JointSPEED > 1):
                print('Joint speed -> ' + str(JointSPEED) + ' not valid. Must be (0,1]. Assigned: 0.01')
                JointSPEED = 0.01
            else:
                print('Joint speed -> ' + str(JointSPEED))

            positionx = trigger['value']['positionx']
            positiony = trigger['value']['positiony']
            positionz = trigger['value']['positionz']
            yaw = trigger['value']['yaw']
            pitch = trigger['value']['pitch']
            roll = trigger['value']['roll']

            MoveXYZW_CLIENT.send_goal(positionx,positiony,positionz,yaw,pitch,roll, JointSPEED)
            
            while rclpy.ok():
                rclpy.spin_once(MoveXYZW_CLIENT)
                RES = MoveXYZW_CLIENT.RES
                if (RES != "null"):
                    break
            
            print('Result of MoveXYZW action is -> { ' + RES + ' }')

            if (RES == 'MoveXYZW:SUCCESS'):
                print('MoveXYZW action in step number -> ' + str(i) + ' successfully executed.')
                RES = 'null'
            else:
                print('MoveXYZW action in step number -> ' + str(i) + ' failed')
                print('The program will be closed. Bye!')
                nodeLOG.get_logger().error('ERROR: Program finished since MoveXYZW action in step number -> ' + str(i) + ' failed.')
                break

        elif (trigger['action'] == 'MoveXYZ'):
            print('')
            print('STEP NUMBER ' + str(i) + ' -> MoveXYZ:')
            print(trigger['value'])

            # joint speed
            JointSPEED = trigger['speed']
            if (JointSPEED <= 0 or JointSPEED > 1):
                print('Joint speed -> ' + str(JointSPEED) + ' not valid. Must be (0,1]. Assigned: 0.01')
                JointSPEED = 0.01
            else:
                print('Joint speed -> ' + str(JointSPEED))

            positionx = trigger['value']['positionx']
            positiony = trigger['value']['positiony']
            positionz = trigger['value']['positionz']
            MoveXYZ_CLIENT.send_goal(positionx,positiony,positionz, JointSPEED)
            
            while rclpy.ok():
                rclpy.spin_once(MoveXYZ_CLIENT)
                RES = MoveXYZ_CLIENT.RES
                if (RES != "null"):
                    break
            
            print('Result of MoveXYZ action is -> { ' + RES + ' }')

            if (RES == 'MoveXYZ:SUCCESS'):
                print('MoveXYZ action in step number -> ' + str(i) + ' successfully executed.')
                RES = 'null'
            else:
                print('MoveXYZ action in step number -> ' + str(i) + ' failed')
                print('The program will be closed. Bye!')
                nodeLOG.get_logger().error('ERROR: Program finished since MoveXYZ action in step number -> ' + str(i) + ' failed.')
                break

        elif (trigger['action'] == 'MoveYPR'):
            print('')
            print('STEP NUMBER ' + str(i) + ' -> MoveYPR:')
            print(trigger['value'])

            # joint speed
            JointSPEED = trigger['speed']
            if (JointSPEED <= 0 or JointSPEED > 1):
                print('Joint speed -> ' + str(JointSPEED) + ' not valid. Must be (0,1]. Assigned: 0.01')
                JointSPEED = 0.01
            else:
                print('Joint speed -> ' + str(JointSPEED))

            yaw = trigger['value']['yaw']
            pitch = trigger['value']['pitch']
            roll = trigger['value']['roll']
            MoveYPR_CLIENT.send_goal(yaw,pitch,roll, JointSPEED)
            
            while rclpy.ok():
                rclpy.spin_once(MoveYPR_CLIENT)
                RES = MoveYPR_CLIENT.RES
                if (RES != "null"):
                    break
            
            print('Result of MoveYPR action is -> { ' + RES + ' }')

            if (RES == 'MoveYPR:SUCCESS'):
                print('MoveYPR action in step number -> ' + str(i) + ' successfully executed.')
                RES = 'null'
            else:
                print('MoveYPR action in step number -> ' + str(i) + ' failed')
                print('The program will be closed. Bye!')
                nodeLOG.get_logger().error('ERROR: Program finished since MoveYPR action in step number -> ' + str(i) + ' failed.')
                break

        elif (trigger['action'] == 'MoveROT'):
            print('')
            print('STEP NUMBER ' + str(i) + ' -> MoveROT:')
            print(trigger['value'])

            # joint speed
            JointSPEED = trigger['speed']
            if (JointSPEED <= 0 or JointSPEED > 1):
                print('Joint speed -> ' + str(JointSPEED) + ' not valid. Must be (0,1]. Assigned: 0.01')
                JointSPEED = 0.01
            else:
                print('Joint speed -> ' + str(JointSPEED))

            yaw = trigger['value']['yaw']
            pitch = trigger['value']['pitch']
            roll = trigger['value']['roll']
            MoveROT_CLIENT.send_goal(yaw,pitch,roll, JointSPEED)

            while rclpy.ok():
                rclpy.spin_once(MoveROT_CLIENT)
                RES = MoveROT_CLIENT.RES
                if (RES != 'null'):
                    break
            
            print('Result of MoveROT action call is -> { ' + RES + " }")

            if (RES == 'MoveROT:SUCCESS'):
                print('MoveROT action in step number -> ' + str(i) + ' successfully executed.')
                RES = 'null'
            else:
                print('MoveROT action in step number -> ' + str(i) + ' failed')
                print('The program will be closed. Bye!')
                nodeLOG.get_logger().error('ERROR: Program finished since MoveROT action in step number -> ' + str(i) + ' failed.')
                break
        
        elif (trigger['action'] == 'MoveRP'):
            print('')
            print('STEP NUMBER ' + str(i) + ' -> MoveRP:')
            print(trigger['value'])

            # joint speed
            JointSPEED = trigger['speed']
            if (JointSPEED <= 0 or JointSPEED > 1):
                print('Joint speed -> ' + str(JointSPEED) + ' not valid. Must be (0,1]. Assigned: 0.01')
                JointSPEED = 0.01
            else:
                print('Joint speed -> ' + str(JointSPEED))

            yaw = trigger['value']['yaw']
            pitch = trigger['value']['pitch']
            roll = trigger['value']['roll']
            x = trigger['value']['x']
            y = trigger['value']['y']
            z = trigger['value']['z']
            MoveRP_CLIENT.send_goal(yaw,pitch,roll,x,y,z, JointSPEED)

            while rclpy.ok():
                rclpy.spin_once(MoveRP_CLIENT)
                RES = MoveRP_CLIENT.RES
                if (RES != 'null'):
                    break
            
            print('Result of MoveRP action call is -> { ' + RES + " }")

            if (RES == 'MoveRP:SUCCESS'):
                print('MoveRP action in step number -> ' + str(i) + ' successfully executed.')
                RES = 'null'
            else:
                print('MoveRP action in step number -> ' + str(i) + ' failed')
                print('The program will be closed. Bye!')
                nodeLOG.get_logger().error('ERROR: Program finished since MoveRP action in step number -> ' + str(i) + ' failed.')
                break
        
        elif (trigger['action'] == 'GripperOpen'):
            GP = 0.0
            if (PARAM_EE == 'schunk'):
                GP = 0.0
            elif (PARAM_EE == 'panda_hand'):
                GP = 0.035

            print('')
            print('STEP NUMBER ' + str(i) + ' -> GripperOpen (MoveG).')
            MoveG_CLIENT.send_goal(GP)

            while rclpy.ok():
                rclpy.spin_once(MoveG_CLIENT)
                RES = MoveG_CLIENT.RES
                if (RES != 'null'):
                    break
            
            print('Result of MoveG action call is -> { ' + RES + ' }')

            if (RES == 'MoveG:SUCCESS'):
                print('MoveG action in step number -> ' + str(i) + ' successfully executed.')
                RES = 'null'
            else:
                print('MoveG action in step number -> ' + str(i) + ' failed')
                print('The program will be closed. Bye!')
                break

        elif (trigger['action'] == 'GripperClose'):
            GP = 0.0
            if (PARAM_EE == 'schunk'):
                GP = 0.005
            elif (PARAM_EE == 'panda_hand'):
                GP = 0.0
            
            print('')
            print('STEP NUMBER ' + str(i) + ' -> GripperClose (MoveG).')
            MoveG_CLIENT.send_goal(GP)

            while rclpy.ok():
                rclpy.spin_once(MoveG_CLIENT)
                RES = MoveG_CLIENT.RES
                if (RES != 'null'):
                    break
            
            print('Result of MoveG action is -> { ' + RES + ' }')

            if (RES == 'MoveG:SUCCESS'):
                print('MoveG action in step number -> ' + str(i) + ' successfully executed.')
                RES = 'null'
            else:
                print('MoveG action in step number -> ' + str(i) + ' failed')
                print('The program will be closed. Bye!')
                break

        else:
            print('Step number ' + str(i) + ' -> Action type not identified. Please check.')
            print('The program will be closed. Bye!')
            nodeLOG.get_logger().info('ERROR: Program finished since ACTION NAME in step number -> ' + str(i) + ' was not identified.')
            break

    print('')
    print('SEQUENCE EXECUTION FINISHED!')
    print('Program will be close. Bye!')
    nodeLOG.get_logger().info('SUCCESS: Program execution successfully finished.')
    nodeLOG.destroy_node()
    print('Closing... Bye!')
    time.sleep(5)


if __name__ == '__main__':
    main()

