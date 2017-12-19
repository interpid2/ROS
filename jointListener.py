#!/usr/bin/env python

import rospy as rp
from sensor_msgs.msg import JointState
from math import pi
import os

def listenCb(msg):
    clear()
    print "Joint states: ", [round(joint*180/pi,2) for joint in msg.position]

clear=lambda:os.system("clear")
rp.init_node("jointListener")
listener=rp.Subscriber("/joint_states", JointState,listenCb)
rate=rp.Rate(0.5)
rate.sleep()
rp.spin()