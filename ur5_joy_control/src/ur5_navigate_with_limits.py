#!/usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg as gm
import pickle

from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

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
        invert_velocity()

def invert_velocity():
    scale_pos = 0.4 # Go back with double the speed
    scale_or = 0.8

    height = 0.0
    if buttons[4] > 0:
        height = -1.0
    if buttons[5] > 0:
        height = 1.0    

    pub.publish("speedl([-%f,%f,-%f,0,0,0], 0.7, 100.0, 3.0)"%(scale_pos*axes[0],scale_pos*axes[1],\
        scale_pos*height))
    

if __name__ == '__main__':  
    
    rospy.init_node('ur5_joy_node', anonymous=True)

    # Double check rate feasibility:
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

    # LIMITS:
    boxes = ws.Boxes
    heights = ws.heights
    polygons = [Polygon(boxes[k,:,:2]) for k in range(boxes.shape[0])]
    
    print("Boxes: ",boxes)
    print("heights: ",heights)

    while not rospy.is_shutdown():

        curr_pos = group.get_current_pose().pose.position

        point = Point(curr_pos.x, curr_pos.y)
        
        inside = False
        for i in range(boxes.shape[0]):
            z_min = boxes[i,0,-1]
            z_max = heights[i]
            if polygons[i].contains(point) and z_min < curr_pos.z and curr_pos.z < z_max:
                inside = True
                break
        
        if not inside:
            # Switch velocity input to keep EE within ws boundaries:
            invert_velocity()
    
        
        rate.sleep()
