#!/usr/bin/env python

from dynamixel_sdk import *
from tmotor import Spinner

import time 
import math
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
import std_msgs.msg
import numpy as np

joints_names = ['Rev1', 'Rev2', 'Rev3', 'Rev4', 'Rev5', 'Rev6', 'Rev7', 'Rev8', 'Rev9', 'Rev10', 'Rev11', 'Rev12']
joints_pos =   [   0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,     0.0]
joints_gpos =  [   0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,     0.0]
joints_ids =   [    21,     41,     11,     31,     32,     12,     42,     22,     23,      43,      33,      13]
joints_inv =   [  False,  True,   False,  True,  False,  False,   True,   True,   True,    True,   False,   False]
joints_off =   [   0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    1.914,     1.346,     2.075,    1.875]
joints_ratio = [   1.0,    1.0,    1.0,    1.0,    1.0,    1.0,    1.0,    1.0, 20.0/28.0, 20.0/28.0, 20.0/28.0, 20.0/28.0]
joints_conn =  12*[False]
joints_gpos_int = 12*[0]

def trajectory_callback(data):
    global joints_gpos
    if len(data.points)>1.0:
        print('OJO!!!')
    if len(data.points)==1.0:
        for i, name in enumerate(data.joint_names):
            pos = int(name[3:])-1
            joints_gpos[pos] = data.points[0].positions[i]

# pub = rospy.Publisher('joint_states', JointState, queue_size=10)
rospy.Subscriber('/joint_group_position_controller/command', JointTrajectory, trajectory_callback)
rospy.init_node('talker', anonymous=True)
rate = rospy.Rate(200) # 10hz

spinner = Spinner(port='/dev/fdcanusb', target=1)

while not rospy.is_shutdown():
    # Message
    # jointState = JointState()
    # jointState.header = std_msgs.msg.Header()
    # jointState.header.stamp = rospy.Time.now()
    # jointState.name = joints_names
    # jointState.position = joints_pos
    # pub.publish(jointState)
    spinner.step(np.degrees(joints_gpos[5]))
    rate.sleep()