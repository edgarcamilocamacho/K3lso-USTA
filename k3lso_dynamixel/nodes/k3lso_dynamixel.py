#!/usr/bin/env python

import time 
from enum import Enum

import rospy, rosparam
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
import std_msgs.msg

from dxl_driver import DxlDriver

# TODO: Modo fake
# TODO: Archivo de poses
# TODO: FSM y rampas
# TODO: Inicial entre zero o stand
# TODO: Timeout de entrada

# FSM
class FsmStates(Enum):
    INIT            = {'text':'Init State'}
    WAIT_READ       = {'text':'Waiting for First Dxl Read'}
    MOVING_1        = {'text':'Moving Hips', 'substr':'_hip_'}
    MOVING_2        = {'text':'Moving Legs', 'substr':'_leg_'}
    LISTENING       = {'text':'Listening JointTrajectory'}
fsmState = FsmStates.INIT
fsmState_pre = fsmState

# Global Variables
motors_in_initial_pos = []

# ROS node init
rospy.init_node('k3lso_dynamixel', anonymous=False)
rate = rospy.Rate(rospy.get_param('~loop_rate')) # 10hz

dxl_poses = rosparam.load_file(rospy.get_param('k3lso_poses_yaml'))[0][0]
initial_pose = dxl_poses[rospy.get_param('~initial_pose')]
init_pose_step = rospy.get_param('~init_pose_step')

# Dynamixel Driver
motors = DxlDriver( port = rospy.get_param('~dxl_usb_port'), 
                    baudrate = rospy.get_param('~dxl_baud_rate'), 
                    dxl_info = rosparam.load_file(rospy.get_param('dynamixel_info'))[0][0],
                    initial_fake_pose = dxl_poses[rospy.get_param('~initial_fake_pose')], 
                    only_monitor = rospy.get_param('~only_monitor'), 
                    fake_mode = rospy.get_param('~fake_mode'))
def dxl_error():
    rospy.logerr('DXL Error: {}'.format(motors.error_str))
    exit()
if not motors.correct: dxl_error()
rospy.loginfo('All motors connected')

# Subscribers
def trajectory_callback(data):
    global joints_gpos, fmsState
    if fsmState==FsmStates.LISTENING:
        if len(data.points)>1:
            rospy.logwarn('Received trajectory with more that 1 point')
        for name, pos in zip(data.joint_names, data.points[0].positions):
            motors.set_goal_position(name, pos)
rospy.Subscriber('/joint_group_position_controller/command', JointTrajectory, trajectory_callback)

# Publishers
pub_joint_states = rospy.Publisher('joint_states', JointState, queue_size=10)
def publish_joint_states():
    jointState = JointState()
    jointState.header = std_msgs.msg.Header()
    jointState.header.stamp = rospy.Time.now()
    jointState.name, jointState.position = motors.get_names_positions()
    pub_joint_states.publish(jointState)

# Main loop
while not rospy.is_shutdown():

    # FSM
    if fsmState == FsmStates.INIT:
        fsmState = FsmStates.WAIT_READ
    elif fsmState == FsmStates.WAIT_READ:
        if motors.first_read:
            fsmState = FsmStates.MOVING_1
    elif fsmState == FsmStates.MOVING_1:
        if len(motors_in_initial_pos)==4:
            fsmState = FsmStates.MOVING_2
    elif fsmState == FsmStates.MOVING_2:
        if len(motors_in_initial_pos)==12:
            fsmState = FsmStates.LISTENING

    # FSM Actions
    if fsmState in [FsmStates.MOVING_1, FsmStates.MOVING_2]:
        for motor_name in initial_pose:
            if (fsmState.value['substr'] in motor_name) and (motor_name not in motors_in_initial_pos):
                motor_pos = motors.get_motor_position(motor_name)
                diff = motor_pos - initial_pose[motor_name]
                if diff > init_pose_step:
                    motors.set_goal_position(motor_name, motor_pos-init_pose_step)
                elif diff > 0.0:
                    motors.set_goal_position(motor_name, motor_pos-diff)
                    motors_in_initial_pos.append(motor_name)
                elif diff < -init_pose_step:
                    motors.set_goal_position(motor_name, motor_pos+init_pose_step)
                elif diff < 0.0:
                    motors.set_goal_position(motor_name, motor_pos-diff)
                    motors_in_initial_pos.append(motor_name)
                else:
                    pass
    # FSM Monitor
    if fsmState!=fsmState_pre:
        rospy.loginfo('FSM STATE: {}'.format(fsmState.value['text']))
        fsmState_pre = fsmState
    # Read
    if not motors.read(): dxl_error()
    # Write
    if not motors.write(): dxl_error()
    # Publish
    publish_joint_states()
    rate.sleep()