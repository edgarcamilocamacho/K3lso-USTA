#!/usr/bin/env python

import rospy, rosparam

from champ_msgs.msg import Pose
from geometry_msgs.msg import Twist

rospy.init_node('k3lso_smoother', anonymous=False)
rate_hz = 50.0
rate = rospy.Rate(rate_hz)

class Smoother():
    def __init__(self, max_vel, rate_hz):
        self.max_vel = max_vel
        self.rate_hz = rate_hz
        self.max_delta = max_vel / rate_hz
        self.state = 0.0
        self.init = False

    def sample(self, x):
        if not self.init:
            self.state = x
            self.init = True
        diff = self.state - x
        if diff>(self.max_delta):
            self.state -= self.max_delta
        elif diff<(-self.max_delta):
            self.state += self.max_delta
        else:
            self.state -= diff
        return self.state

smt_vel_lin_x = Smoother(3.0, rate_hz)
smt_vel_lin_y = Smoother(3.0, rate_hz)
smt_vel_ang_z = Smoother(3.0, rate_hz)

smt_pos_pitch = Smoother(1.0, rate_hz)
smt_pos_roll  = Smoother(1.0, rate_hz)
smt_pos_yaw   = Smoother(1.0, rate_hz)
smt_pos_z     = Smoother(1.0, rate_hz)

msg_cmd_vel = None
pub_cmd_vel = rospy.Publisher('/cmd_vel/smooth', Twist, queue_size=10)
def cmd_vel_callback(data):
    global msg_cmd_vel
    msg_cmd_vel = data
rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)

msg_cmd_pose = None
pub_cmd_pose = rospy.Publisher('/cmd_pose/smooth', Pose, queue_size=10)
def cmd_pose_callback(data):
    global msg_cmd_pose
    msg_cmd_pose = data
rospy.Subscriber('/cmd_pose', Pose, cmd_pose_callback)

while not rospy.is_shutdown():
    if msg_cmd_vel is not None:
        msg_cmd_vel.linear.x  = smt_vel_lin_x.sample(msg_cmd_vel.linear.x)
        msg_cmd_vel.linear.y  = smt_vel_lin_y.sample(msg_cmd_vel.linear.y)
        msg_cmd_vel.angular.z = smt_vel_ang_z.sample(msg_cmd_vel.angular.z)
        pub_cmd_vel.publish(msg_cmd_vel)
    if msg_cmd_pose is not None:
        msg_cmd_pose.pitch = smt_pos_pitch.sample(msg_cmd_pose.pitch)
        msg_cmd_pose.roll  = smt_pos_roll.sample(msg_cmd_pose.roll)
        msg_cmd_pose.yaw   = smt_pos_yaw.sample(msg_cmd_pose.yaw)
        msg_cmd_pose.z     = smt_pos_z.sample(msg_cmd_pose.z)
        pub_cmd_pose.publish(msg_cmd_pose)
    rate.sleep()



