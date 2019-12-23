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

axes = np.zeros((8,))
buttons = np.zeros((11,))

force_lim = 5 # Limit of force allowed on wrist

limits = np.array([[0.208, 0.719, 0.429],\
					[-0.220, 0.403, 0.237]])

def joyCallback(data):
	
	global axes
	global buttons
	
	axes = np.array(data.axes)
	axes[np.absolute(axes)<0.3] = 0.0
	axes[4:6] = 0.5-0.5*axes[4:6]
	
	buttons = np.array(data.buttons)
    
def FTCallback(data):
    # Stop robot if force on wrist is excessive
    wrench = data.wrench
    if wrench.force.x > force_lim or wrench.force.y > force_lim or wrench.force.z > force_lim:
        pub.publish("stopl(1.0,5.0)")
    
def in_limits(curr_pos):
    # Wrong algorithm, adapt to irregular quadrilaterals
    inside = False
    for j in range(num_lims):
        if (curr_pos.x>min_lims[j,0] and curr_pos.x<max_lims[j,0] and curr_pos.y>min_lims[j,1] and curr_pos.y<max_lims[j,1]\
            and curr_pos.z>min_lims[j,2] and curr_pos.z<max_lims[j,2]):
            inside = True
            break
            
    return inside

if __name__ == '__main__':	
    
    rospy.init_node('ur5_joy_node', anonymous=True)

    rate = rospy.Rate(100) # Hz

    sub = rospy.Subscriber("/joy", Joy, joyCallback, queue_size=1)
    pub = rospy.Publisher("/ur_driver/URScript", String, queue_size=1)

    force_sub = rospy.Subscriber("/FT_sensor/robotiq_force_torque_wrench", gm.WrenchStamped, FTCallback, queue_size=1) 

    # Setup robot UR5
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)

    with open('ws.pkl','rb') as in_put:
        ws = pickle.load(in_put)

    limits = np.concatenate((ws.BoxMax,ws.BoxMin),axis=0)

    # LIMITS:
    max_lims = ws.BoxMax
    min_lims = ws.BoxMin
    
    print 'Max limits: '
    print max_lims
    print 'Min limits: '
    print min_lims, '\n'

    while not rospy.is_shutdown():

        #print group.get_current_pose()


        curr_pos = group.get_current_pose().pose.position
        stop = False
        count_x = 0
        count_y = 0
        count_z = 0

        num_lims = max_lims.shape[0]
        total_count = 0

        inside = in_limits(curr_pos)
        
        if inside:
            # Not stopping robot
            rate.sleep()
            continue
        
        else:
            # -> Find which box is the robot in
            # -> Find closest point in that box
            # -> new_vel = dot(current_pos, closest_point)
            # -> Publish new_vel
            
            
        # NOT WORKING YET:
        '''
        for j in range(num_lims):
			
			temp_count = 0
			
			if (curr_pos.x<min_lims[j,0] and axes[0]>0) or (curr_pos.x>max_lims[j,0] and axes[0]<0):
				temp_count += 1
				
			if (curr_pos.y<min_lims[j,1] and axes[1]<0) or (curr_pos.y>max_lims[j,1] and axes[1]>0):
				temp_count += 1
				
			if (curr_pos.z<min_lims[j,2] and buttons[4]>0) or (curr_pos.z>max_lims[j,2] and buttons[5]>0):
				temp_count += 1
		
			print(temp_count)
			total_count += temp_count/3
         
        if total_count == num_lims:
            pub.publish("stopl(1.0,5.0)")
		'''
        
        
        rate.sleep()
