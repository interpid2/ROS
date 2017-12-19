#!/usr/bin/env python

from math import pi
import rospy as rp
import numpy as np
import moveit_commander as mic
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler as qEuler
from control_msgs.msg import FollowJointTrajectoryAction as trajAction
from control_msgs.msg import FollowJointTrajectoryGoal as goal
from control_msgs.msg import FollowJointTrajectoryActionFeedback as feedback
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
import actionlib as act
import termios, sys
import time as t
import os

class abbRobot:
    errorDict={0:"Successful",
              -1:"Invalid goal",
              -2:"Invalid joints",
              -3:"Old header timestamp",
              -4:"Path tolerance violated",
              -5:"Goal tolerance violated"}

    def __init__(self):
        self.manip=mic.MoveGroupCommander("manipulator")
        self.client=act.SimpleActionClient('joint_trajectory_action',trajAction)
        rp.loginfo("Waiting for server joint_trajectory_action.")
        self.client.wait_for_server()
        rp.loginfo("Successfuly connected to server.")

    def move2Point(self, points, eAngles=[[0,0,0]], end_effector='link_6'):
        """
        Moves to a point using end effector point space.\n
        Usage:\n
            - points  - list of lists that contain x,y,z coordinates\n
            - eAngles - default [[0,0,0]]\n
                      - list of lists that contain a,b,g euler angles in sxyz convention
                      - if multiple points are passed and only one list of euler angles then those angles are going to be used for all points
            - end_effector - default link_6
                           - link whose point you want to move
        """
        print "Number of points recieved: ",len(points)
        t1=t.time()
        if (len(points)==len(eAngles) or len(eAngles)==1):
            for i in xrange(len(points)):
                if rp.is_shutdown():
                    print "ROS has been shutdown. Exiting..."
                    break
                print i+1,". point:", [round(pt,5) for pt in points[i]]
                p=Point(points[i][0],points[i][1],points[i][2])
                index=i
                if len(eAngles)==1:
                    index=0
                orient=Quaternion(*qEuler(eAngles[index][0],eAngles[index][1],eAngles[index][2]))
                self.manip.set_pose_target(Pose(p,orient),end_effector_link=end_effector)
                self.manip.go(True)
            rp.loginfo("Moving to multiple points finished.")
            self.__displayDuration(t1,t.time())
 
    def jointAction(self, jointAngles):
        """
        Moves to a point using joint space\n
        Usage:\n
            - jointAngles - list of lists that contain 6 angles in order from joint_1 to joint_6
        """
        print "Number of joint angles recieved: ",len(jointAngles)
        t1=t.time()
        jointGoal=goal()
        jointGoal.trajectory.joint_names=['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        jointGoal.trajectory.points.append(JointTrajectoryPoint())
        for joint in jointAngles:
            print "Going to joint values[degrees]: ", [round(i*180/pi,2) for i in joint]
            jointGoal.trajectory.points[0].positions=joint
            jointGoal.trajectory.points[0].velocities=[0,0,0,0,0,0]
            jointGoal.trajectory.points[0].accelerations=[0,0,0,0,0,0]
            #jointGoal.trajectory.points[0].velocities=[2.,2.,2.,6.,6.,7.]
            #jointGoal.trajectory.points[0].accelerations=[1.,1.,1.,1.,1.,1.]
            #jointGoal.trajectory.points[0].time_from_start=rp.Duration(2,0)
            self.client.send_goal(jointGoal,feedback_cb=self.__feedback)
            self.client.wait_for_result()
            print "[Result:] ",self.errorDict[self.client.get_result().error_code]
        spent=self.__displayDuration(t1,t.time())""
  
    @staticmethod
    def __feedback(msg):
        print "[Feedback:] ",msg.error

    @staticmethod
    def __displayDuration(t1,t2):
        spent=[(int(t2-t1)/60), 0, 0]
        spent[1]=int((t2-t1)-spent[0]*60)
        spent[2]=int(((t2-t1)-spent[0]*60-spent[1])*1000)
        rp.loginfo("Execution took [min:sec.ms]: %i:%02i.%03i ",spent[0],spent[1],spent[2])


def __listenCb(msg):
    clear()
    print "Joint names          :", msg.name
    print "Joint angles [degree]:", [round(joint*180/pi,2) for joint in msg.position]

def jointsInfo(printoutRate=0.5, anonym=False):
    """
    Creates new node to listen to /joint_states topic\n
    Usage:\n
        - printoutRate - print rate to terminal in Hz
                       - default: 0.5 Hz
        - anonym [True/False] - do you want to run it as anonymous node
                              - default: False
    """
    rp.init_node("abb_jointListener",anonymous=anonym)
    listener=rp.Subscriber("/joint_states", JointState,__listenCb)
    rate=rp.Rate(printoutRate)
    rate.sleep()
    rp.spin()
    
"""
Clears the terminal
"""
clear=lambda:os.system("clear")