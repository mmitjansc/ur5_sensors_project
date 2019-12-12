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

class Workspace:
	def __init__(self):

		pass
		
	def addBox(self):
		global finish_box
		
		finish_box = False
		
		pos_init = moveit_commander.MoveGroupCommander('manipulator').get_current_pose().pose.position
		min_box = np.array([pos_init.x, pos_init.y, pos_init.z])
		max_box = np.array([pos_init.x, pos_init.y, pos_init.z])
		
		while not finish_box and not finish_ws:
			x,y,z = (group.get_current_pose().pose.position.x, group.get_current_pose().pose.position.y, group.get_current_pose().pose.position.z)
			
			if x < min_box[0]:
				min_box[0] = x
			elif x > max_box[0]:
				max_box[0] = x
			if y < min_box[1]:
				min_box[1] = y
			elif y > max_box[1]:
				max_box[1] = y
			if z < min_box[2]:
				min_box[2] = z
			elif z > max_box[2]:
				max_box[2] = z
		
		
		if finish_ws:
			return
		
		try:
			self.BoxMin = np.concatenate((self.BoxMin,min_box),axis=0)
			self.BoxMax = np.concatenate((self.BoxMax,max_box),axis=0)
		except:
			self.BoxMin = min_box[np.newaxis,:]
			self.BoxMax = max_box[np.newaxis,:]

def joyCallback(data):
	
	global finish_box
	global finish_ws
	
	buttons = np.array(data.buttons)
	
	if buttons[3] > 0:
		finish_box = True
		
	if buttons[2] > 0:
		finish_ws = True
		
if __name__ == '__main__':	
	
	rospy.init_node('ur5_joy_node', anonymous=True)
	
	rate = rospy.Rate(100) # Hz
	
	# Setup robot UR5
	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()
	group_name = "manipulator"
	group = moveit_commander.MoveGroupCommander(group_name)
	
	joy_sub = rospy.Subscriber("/joy", Joy, joyCallback, queue_size=1)
	
	finish_box = False
	finish_ws = False
	
	print 'waiting for limits...'
	ws = Workspace()
	
	while not finish_ws:
		c = raw_input('Press enter to add new box') 
		print('Adding box...')
		ws.addBox()
		print "Box added"
	
	print "Workspace created"
	
	with open('ws.pkl','wb') as output:
		pickle.dump(ws,output,pickle.HIGHEST_PROTOCOL)
		

	print 'min limits:'
	print ws.BoxMin
	print 'max limits:'
	print ws.BoxMax
	
	print 'Limits saved!'

