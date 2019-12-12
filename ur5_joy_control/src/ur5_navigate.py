#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg as gm
import pickle

from math import pi
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from moveit_commander.conversions import pose_to_list
import numpy as np
from set_ur5_limits import Workspace


limits = np.array([[0.208, 0.719, 0.429],\
					[-0.220, 0.403, 0.237]])

axes = np.zeros((8,))
buttons = np.zeros((11,))

def joyCallback(data):
	
	global axes
	global buttons
	
	axes = np.array(data.axes)
	axes[np.absolute(axes)<0.3] = 0.0
	axes[4:6] = 0.5-0.5*axes[4:6]
	
	buttons = np.array(data.buttons)	

if __name__ == '__main__':	
	
	rospy.init_node('ur5_joy_node', anonymous=True)
	
	rate = rospy.Rate(100) # Hz
	
	sub = rospy.Subscriber("/joy", Joy, joyCallback, queue_size=1)
	pub = rospy.Publisher("/ur_driver/URScript", String, queue_size=1)
	
	# Setup robot UR5
	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()
	group_name = "manipulator"
	group = moveit_commander.MoveGroupCommander(group_name)
	
	with open('ws.pkl','rb') as in_put:
		ws = pickle.load(in_put)
	
	limits = np.concatenate((ws.BoxMax,ws.BoxMin),axis=0)
	print limits

	while not rospy.is_shutdown():
		
		#print group.get_current_pose()
		
		# LIMITS:
		curr_pos = group.get_current_pose().pose.position
		if (curr_pos.x<limits[1,0] and axes[0]>0) or (curr_pos.x>limits[0,0] and axes[0]<0):
			pub.publish("stopl(1.0, 5.0)")
			
		if (curr_pos.y<limits[1,1] and axes[1]<0) or (curr_pos.y>limits[0,1] and axes[1]>0):
			pub.publish("stopl(1.0, 5.0)")
			
		if (curr_pos.z<limits[1,2] and buttons[4]>0) or (curr_pos.z>limits[0,2] and buttons[5]>0):
			pub.publish("stopl(1.0, 5.0)")
		
		rate.sleep()
