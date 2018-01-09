#!/usr/bin/env python
from abblib.abbCtrl import abbRobot
from abblib.abbCtrl import clear
import rospy as rp
import numpy as np
from math import pi
import time as t

#0.94002,0.00003,1.45504
rp.init_node('abbMove_Main',anonymous=False)
np.random.seed(1)

robot=abbRobot()
while not rp.is_shutdown():
    choice=input("Choose: "+
    "\n0:       Init joints"+
    "\n1:       Demo mode - end effector point space"+
    "\n2:       Demo mode - end effector point space using constraints"+
    "\n3:       Demo mode - joint space"
    "\n4:       Manual end effector points"+
    "\n5:       Manual joints"+
    "\nothers:  exit"+
    "\n>> ")

    if(choice==0):
        robot.jointAction([[0,0,0,0,0,0]])
    elif(choice==1):
        jointList=[]
        for i in xrange(10):
            x=1
            y=np.random.uniform(-1,1,1)
            z=np.random.uniform(1,1.5,1)
            jointList.append([x,y[0],z[0]])
        robot.move2Point(jointList)
    elif(choice==2):
        waypoints=[]
        for i in xrange(10):
            if not rp.is_shutdown():
                x=1
                y=np.random.uniform(0,-1.5,1)[0]
                z=np.random.uniform(1,1.5,1)[0]
                waypoints.append([x,y,z])
        robot.cartesian2Point(waypoints,[0,0,0])
    elif(choice==3):
        jointList=[]
        for i in xrange(10):
            jointList.append([0,0,0,0,0,0])
            jointList[i][np.random.randint(0,6)]=np.random.uniform(-pi/4,pi/4,1)[0]
        robot.jointAction(jointList)
    elif(choice==4):
        x=input("Enter x coordinate: ")
        y=input("Enter y coordinate: ")
        z=input("Enter z coordinate: ")
        robot.move2Point([[x,y,z]])
    elif(choice==5):
        jointList=[[0,0,0,0,0,0]]
        print "Enter the joint values in degress"
        for i in xrange(6):
            temp=input("Enter the %i. joint value: "%(i+1))*pi/180
            jointList[0][i]=temp
        robot.jointAction(jointList)
    else:
        break
    # t.sleep(5)
    # clear()