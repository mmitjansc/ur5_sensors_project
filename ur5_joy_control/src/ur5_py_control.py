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
	print(count)
	count += 1 
	
	axes = data.axes
	buttons = data.buttons
	
	#rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
	
	if buttons[5] > 0:
	
		'''
		pose_goal = group.get_current_pose().pose
		pose_goal.position.z += 0.0 #scale * buttons[5]
		
		group.set_pose_target(pose_goal)
		
		# Move group:
		plan = group.go(wait=True)
		# Calling `stop()` ensures that there is no residual movement
		group.stop()
		# It is always good to clear your targets after planning with poses.
		# Note: there is no equivalent function for clear_joint_value_targets()
		group.clear_pose_targets()
		'''
		waypoints = []

		wpose = group.get_current_pose().pose

		wpose.position.z += scale * buttons[5]  # First move up (z)
		waypoints.append(copy.deepcopy(wpose))

		# We want the Cartesian path to be interpolated at a resolution of 1 cm
		# which is why we will specify 0.01 as the eef_step in Cartesian
		# translation.  We will disable the jump threshold by setting it to 0.0 disabling:
		(plan, fraction) = group.compute_cartesian_path(
										   waypoints,   # waypoints to follow
										   0.01,        # eef_step
										   0.0)         # jump_threshold
		#group.execute(plan, wait=True)
		group.stop()
		
		
		pub.publish("speedl([0.0,0.0,0.05,0.0,0.0,0.0], 0.2, 100.0)") # This doesn't work
		#pub.publish(String(cmd_str))

		print("Moving...")
		
		
	if buttons[5] < 1:
	
		'''
		pose_goal = group.get_current_pose().pose
		pose_goal.position.z += 0.0 #scale * buttons[5]
		
		group.set_pose_target(pose_goal)
		
		# Move group:
		plan = group.go(wait=True)
		# Calling `stop()` ensures that there is no residual movement
		group.stop()
		# It is always good to clear your targets after planning with poses.
		# Note: there is no equivalent function for clear_joint_value_targets()
		group.clear_pose_targets()
		'''
		waypoints = []

		wpose = group.get_current_pose().pose
		wpose.position.z -= scale * buttons[4]  # First move up (z)
		waypoints.append(copy.deepcopy(wpose))

		# We want the Cartesian path to be interpolated at a resolution of 1 cm
		# which is why we will specify 0.01 as the eef_step in Cartesian
		# translation.  We will disable the jump threshold by setting it to 0.0 disabling:
		(plan, fraction) = group.compute_cartesian_path(
										   waypoints,   # waypoints to follow
										   0.01,        # eef_step
										   0.0)         # jump_threshold
		#group.execute(plan, wait=True)
		group.stop()
		
		cmd_str = "def move_arm():\n"
		cmd_str += "\tspeedl([0.0,0.0,0.05,0.0,0.0,0.0], 0.2, 1.5)\n"
		cmd_str += "end"
		pub.publish("stopl(0.3)") # This doesn't work
		#pub.publish(String(cmd_str))
		#pub.publish("")
		
		#pub.publish("powerdown()") # This works
		print("Stopping...")
		
		
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
