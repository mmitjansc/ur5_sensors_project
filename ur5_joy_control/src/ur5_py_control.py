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
import numpy as np

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

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
		pass
		
		
if __name__ == '__main__':	
	
	count = 0
	scale = 0.01
	
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('ur5_node', anonymous=True)
	
	rate = rospy.Rate(100) # Hz
	
	sub = rospy.Subscriber("/joy", Joy, joyCallback, queue_size=1)
	pub = rospy.Publisher("/ur_driver/URScript", String, queue_size=1)
	
	# Setup robot UR5
	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()
	group_name = "manipulator"
	group = moveit_commander.MoveGroupCommander(group_name)
	
	# Setting tolerances and scaling factors:
	group.set_max_velocity_scaling_factor(0.1)
	group.set_goal_position_tolerance(1e-5)
	group.set_goal_joint_tolerance(1e-5)
	print "Goal tolerance: ", group.get_goal_joint_tolerance()	

	while not rospy.is_shutdown():
		rate.sleep()
