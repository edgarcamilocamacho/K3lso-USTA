#!/usr/bin/env python

from dynamixel_sdk import *

import time 
import math
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
import std_msgs.msg

DEVICENAME = '/dev/ttyUSB0'
PROTOCOL_VERSION = 2.0
# BAUDRATE = 1000000
BAUDRATE = 4000000
DXL_ID = 11

ADDR_PRESENT_POSITION = 132 
LEN_PRESENT_POSITION = 4
ADDR_GOAL_POSITION = 116
LEN_GOAL_POSITION = 4

ADDR_TORQUE_ENABLE = 64
ADDR_PWM_LIMIT = 36

# [Rev3, Rev6, Rev12, Rev1, Rev8, Rev9, Rev4, Rev5, Rev11, Rev2, Rev7, Rev10]
# 11 12 13 - 21 22 23 - 31 32 33 - 41 42 43

joints_names = ['fr_hip_joint', 'rr_hip_joint', 'fl_hip_joint', 'rl_hip_joint', 'rl_upper_leg_joint', 'fl_upper_leg_joint', 'rr_upper_leg_joint', 'fr_upper_leg_joint', 'fr_lower_leg_joint', 'rr_lower_leg_joint', 'rl_lower_leg_joint', 'fl_lower_leg_joint']
joints_pos =   [   0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,     0.0]
joints_gpos =  [   0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,     0.0]
joints_ids =   [    21,     41,     11,     31,     32,     12,     42,     22,     23,      43,      33,      13]
joints_inv =   [  False,  True,   False,  True,  False,  False,   True,   True,   True,    True,   False,   False]
joints_off =   [   0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    1.535,     0.986,     1.800,    1.476]
joints_ratio = [   1.0,    1.0,    1.0,    1.0,    1.0,    1.0,    1.0,    1.0, 20.0/28.0, 20.0/28.0, 20.0/28.0, 20.0/28.0]
joints_conn =  12*[False]
joints_gpos_int = 12*[0]

def trajectory_callback(data):
    global joints_gpos
    # if len(data.points)>1.0:
    #     print('OJO!!!')
    # if len(data.points)==1.0:
    #     for i, name in enumerate(data.joint_names):
    #         pos = int(name[3:])-1
    #         joints_gpos[pos] = data.points[0].positions[i]

pub = rospy.Publisher('joint_states', JointState, queue_size=10)
rospy.Subscriber('/joint_group_position_controller/command', JointTrajectory, trajectory_callback)
rospy.init_node('k3lso_dynamixel', anonymous=False)
rate = rospy.Rate(200) # 10hz

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

if not portHandler.openPort():
    print("Failed to open the port")
    exit()
if not portHandler.setBaudRate(BAUDRATE):
    print("Failed to change the baudrate")
    exit()

for i, m_id in enumerate(joints_ids):
    dxl_model_number, dxl_comm_result, dxl_error = packetHandler.ping(portHandler, m_id)
    if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
        print('Couldn\'n connect with motor {}'.format(m_id))
        exit()
    groupSyncRead.addParam(m_id)

print('All motors connected')

for i, m_id in enumerate(joints_ids):
    packetHandler.write1ByteTxRx(portHandler, m_id, ADDR_TORQUE_ENABLE,  0)
    # packetHandler.write2ByteTxRx(portHandler, m_id, ADDR_PWM_LIMIT, 885) #885

while not rospy.is_shutdown():
    # READ
    dxl_comm_result = groupSyncRead.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    for i, m_id in enumerate(joints_ids):
        pos = math.pi * ( ( float(groupSyncRead.getData(m_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)) - 2048 ) / 2048 )
        if joints_inv[i]:
           pos *= -1 
        pos -= joints_off[i]
        pos *= joints_ratio[i]
        joints_pos[i] = pos
    # print(joints_pos[joints_names.index('rr_lower_leg_joint')])
    # WRITE
    for i, m_id in enumerate(joints_ids):
        x = joints_gpos[i]
        x /= joints_ratio[i]
        x += joints_off[i]
        if joints_inv[i]:
            x *= -1 
        joints_gpos_int[i] = int( ( (x / math.pi) * 2048 ) + 2048 )
    # 23 -> 8
    ###
    # m_ids = [43]
    # for m_id in m_ids:
    #     i = joints_ids.index(m_id)
        ###
        param_goal_position = [ DXL_LOBYTE(DXL_LOWORD(joints_gpos_int[i])), 
                                DXL_HIBYTE(DXL_LOWORD(joints_gpos_int[i])), 
                                DXL_LOBYTE(DXL_HIWORD(joints_gpos_int[i])), 
                                DXL_HIBYTE(DXL_HIWORD(joints_gpos_int[i]))]
        groupSyncWrite.addParam(m_id, param_goal_position)
        # print(joints_gpos_int[i] )
    # dxl_comm_result = groupSyncWrite.txPacket()     ###################
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    groupSyncWrite.clearParam()

    # Message
    jointState = JointState()
    jointState.header = std_msgs.msg.Header()
    jointState.header.stamp = rospy.Time.now()
    jointState.name = joints_names
    jointState.position = joints_pos
    pub.publish(jointState)
    rate.sleep()