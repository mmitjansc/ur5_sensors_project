#!/usr/bin/env python

import rospy

from std_msgs.msg import String

from robotiq_c_model_control.msg import _CModel_robot_output  as outputMsg
import numpy as np

import actionlib
import ur5_joy_control.msg

class FibonacciAction(object):
    # create messages that are used to publish feedback/result
    _feedback = ur5_joy_control.msg.GripperFeedback()
    _result = ur5_joy_control.msg.GripperResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name,\
				ur5_joy_control.msg.GripperAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
		
		global command
		count = 0

		self._feedback.gripper_pos = 0
		
		while not self._as.is_preempt_requested():
			
			if count > 10000:
				command.rPR += goal.order
				command.rPR = min(max(command.rPR,0),255)
				gripper_pub.publish(command)
				count = 0
			count += 1
			
		self._feedback.gripper_pos = command.rPR
		self._as.publish_feedback(self._feedback)
		self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
	rospy.init_node('fibonacci')

	server = FibonacciAction('gripperAction')

	# Gripper publisher
	gripper_pub = rospy.Publisher("/CModelRobotOutput", outputMsg.CModel_robot_output, queue_size=1)
	command = outputMsg.CModel_robot_output();
	command.rACT = 0
	gripper_pub.publish(command) # Reset gripper
	command.rACT = 1
	command.rGTO = 1
	command.rSP  = 255
	command.rFR  = 150
	gripper_pub.publish(command) # Activate gripper

	rospy.spin()
