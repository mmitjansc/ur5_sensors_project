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
from sensor_msgs.msg import Joy
from moveit_commander.conversions import pose_to_list

def joyCallback(data):
	
	axes = data.axes
	buttons = data.buttons
	
	print(axes)
	
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


if __name__ == '__main__':	
	
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('ur5_node', anonymous=True)
	
	rate = rospy.Rate(100) # Hz
	
	sub = rospy.Subscriber("joy_subs", Joy, joyCallback)
	
	pub = rospy.Publisher("chatter_test", String, queue_size=10)

	print("Entered ROS!")
	
	while not rospy.is_shutdown():
		rate.sleep()
