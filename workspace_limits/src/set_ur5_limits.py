#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg as gm

from math import pi
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from moveit_commander.conversions import pose_to_list
import numpy as np

		
if __name__ == '__main__':	
	
	rospy.init_node('ur5_joy_node', anonymous=True)
	
	rate = rospy.Rate(100) # Hz
	
	# Setup robot UR5
	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()
	group_name = "manipulator"
	group = moveit_commander.MoveGroupCommander(group_name)

	while not rospy.is_shutdown():
		
		
		curr_pos = group.get_current_pose()
		print(curr_pos)
		
		rate.sleep()
