#!/usr/bin/env python

import sys
import copy
import rospy
import geometry_msgs.msg as gm
import numpy as np
from math import pi
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import numpy as np

def joyCallback(data):
	
	global count
	#print(count)
	count += 1 
	
	axes = np.array(data.axes)
	axes[np.absolute(axes)<0.3] = 0.0
	axes[4:6] = 0.5-0.5*axes[4:6]
	
	buttons = np.array(data.buttons)	
	scale_pos = 0.15
	scale_or = 0.8
	#print(axes)

	height = 0.0
	if buttons[4] > 0:
		height = -1.0
	if buttons[5] > 0:
		height = 1.0
	
	pub.publish("speedl([%f,-%f,%f,%f,%f,%f], 0.5, 100.0, 3.0)"%(scale_pos*axes[0],scale_pos*axes[1],\
		scale_pos*height, axes[3], scale_or*axes[2], scale_or*(axes[4]-axes[5])))
		
	stop = True
	for ax in np.absolute(axes[0:6]):
		if ax > 0.0:
			stop = False
			break
	
	if stop and buttons[5]<1 and buttons[4]<1:
		pub.publish("stopl(1.0, 5.0)")
		
		
	if buttons[7] > 0:
		pub.publish("powerdown()")
		
		
if __name__ == '__main__':	
	
	count = 0
	scale = 0.01
	
	rospy.init_node('ur5_joy_node', anonymous=True)
	
	rate = rospy.Rate(100) # Hz
	
	sub = rospy.Subscriber("/joy", Joy, joyCallback, queue_size=1)
	pub = rospy.Publisher("/ur_driver/URScript", String, queue_size=1)

	while not rospy.is_shutdown():
		rate.sleep()
