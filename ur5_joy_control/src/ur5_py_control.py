#!/usr/bin/env python

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

def gripper_client(order):
    
    client = actionlib.SimpleActionClient('gripperAction', ur5_joy_control.msg.GripperAction)
    
    #print("Waiting for server...")
    client.wait_for_server()
    
    if order == 0:
        client.cancel_all_goals()
        return None
    
    goal = ur5_joy_control.msg.GripperGoal(order=order)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    #client.wait_for_result() # Not needed?

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

def joyCallback(data):
    
    global axes
    global buttons

    gripper_val = 0
    
    axes = np.array(data.axes)
    axes[np.absolute(axes)<0.3] = 0.0
    axes[4:6] = 0.5-0.5*axes[4:6]
    
    buttons = np.array(data.buttons)    
    scale_pos = 0.2
    scale_or = 0.8
    #print(axes)

    height = buttons[5] - buttons[4] 
    
    if not recovering:
        #print("Recovering: %r, PUBLISHING"%(recovering))
        pub.publish("speedl([%f,-%f,%f,%f,%f,%f], 0.7, 100.0, 3.0)"%(scale_pos*axes[0],scale_pos*axes[1],\
            scale_pos*height, axes[3], scale_or*axes[2], scale_or*(axes[4]-axes[5])))
    
    else:
        pass
        #print "RECOVERING, can't publish"        
            
    stop = True
    for ax in np.absolute(axes[0:6]):
        if ax > 0.0:
            stop = False
            break
    
    if stop and buttons[5]<1 and buttons[4]<1 and not recovering:
        pub.publish("stopl(5.0, 5.0)")
        
    # Gripper control:
    if buttons[0] > 0:
        #print("Closing gripper")
        gripper_position = gripper_client(1)
        #START ACTION
    
    elif buttons[1] > 0:
        #print("Opening gripper")
        gripper_position = gripper_client(-1)
        #START ACTION
    
    else:
        gripper_client(0)
        
    if buttons[7] > 0:
        pub.publish("powerdown()")

def insideCallback(msg): 
    global recovering 
    recovering = msg.data
    #rospy.logerr(recovering)
        
if __name__ == '__main__':  

    recovering = False
    
    rospy.init_node('ur5_joy_node', anonymous=True)
    
    rate = rospy.Rate(100) # Hz
    
    sub = rospy.Subscriber("/joy", Joy, joyCallback, queue_size=1)
    inside_sub = rospy.Subscriber("/inside_ws", Bool, insideCallback, queue_size=1)
    pub = rospy.Publisher("/ur_driver/URScript", String, queue_size=1)
    
    # Setup robot UR5
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)

    while not rospy.is_shutdown():
        
        rate.sleep()
