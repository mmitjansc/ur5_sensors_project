#!/usr/bin/env python

from __future__ import print_function

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg as gm
import pickle

from math import pi
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Joy
from moveit_commander.conversions import pose_to_list
from robotiq_c_model_control.msg import _CModel_robot_output  as outputMsg
import numpy as np

import actionlib
import ur5_joy_control.msg

axes = np.zeros((8,))
buttons = np.zeros((11,))
first_time_5 = True
first_time_2 = True

def gripper_client(order):
    
    client = actionlib.SimpleActionClient('gripperAction', ur5_joy_control.msg.GripperAction)
    
    client.wait_for_server()
    
    if order == 0:
        client.cancel_all_goals()
        return None
    
    goal = ur5_joy_control.msg.GripperGoal(order=order)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Prints out the result of executing the action
    return client.get_result()  

def joyCallback(data):
    
    global axes
    global buttons
    global first_time_5
    global first_time_2

    gripper_val = 0
    
    axes = np.array(data.axes)
    axes[np.absolute(axes)<0.3] = 0.0
    
    if first_time_5:
        if int(axes[5]) != 0:
            first_time_5 = False        
        else:
            axes[5] = 1.0
    if first_time_2:
        if int(axes[2]) != 0:
            first_time_2 = False        
        else:
            axes[2] = 1.0
            
    axes[5] = 0.5-0.5*axes[5]
    axes[2] = 0.5-0.5*axes[2]
    
    buttons = np.array(data.buttons)    
    scale_pos = 0.2*2
    scale_or = 0.8*2

    height = buttons[5] - buttons[4] 
    
    if not recovering:
        print("Axes: ",axes)
        print("Buttons: ",buttons)
        print("Publishing:",scale_pos*axes[0],scale_pos*axes[1],\
            scale_pos*height, axes[4], scale_or*axes[3], scale_or*(axes[5]-axes[2]))
        pub.publish("speedl([%f,-%f,%f,%f,%f,%f], 1.5, 100.0, 3.0)"%(scale_pos*axes[0],scale_pos*axes[1],\
            scale_pos*height, axes[4], scale_or*axes[3], scale_or*(axes[5]-axes[2])))
          
            
    stop = True
    for ax in np.absolute(axes[0:6]):
        if ax > 0.0:
            stop = False
            break
    
    if stop and buttons[5]<1 and buttons[4]<1 and not recovering:
        pub.publish("stopl(1.0, 5.0)")
        
    # Gripper control:
    if buttons[0] > 0:
        gripper_position = gripper_client(1)
        #START ACTION
    
    elif buttons[1] > 0:
        gripper_position = gripper_client(-1)
        #START ACTION
    
    else:
        gripper_client(0)
        
    if buttons[7] > 0:
        pub.publish("powerdown()")

    print('---------------------------')

def insideCallback(msg): 
    global recovering 
    recovering = msg.data
        
if __name__ == '__main__':  

    recovering = False
    
    rospy.init_node('ur5_joy_node', anonymous=True)
    
    rate = rospy.Rate(100) # Hz
    
    sub = rospy.Subscriber("/joy", Joy, joyCallback, queue_size=1)
    inside_sub = rospy.Subscriber("/inside_ws", Bool, insideCallback, queue_size=1)
    pub = rospy.Publisher("/ur_driver/URScript", String, queue_size=1)
    
    ## Setup robot UR5
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)

    rospy.spin()