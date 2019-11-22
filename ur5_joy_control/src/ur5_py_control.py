#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg as gm
import numpy as np
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


if __name__ == '__main__':
	
	
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('ur5_node', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	
	pub = rospy.Publisher('chatter_test', String, queue_size=10)

	print("Entered ROS!")
