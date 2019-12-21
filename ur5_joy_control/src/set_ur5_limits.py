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

class Workspace:
    def __init__(self):
        self.Boxes = np.zeros((4,3))
        self.heights = np.array([0])
        pass
        
    def addBox(self):
        global box_coord
        global save_coord
        global finish_box
        
        box_coord = 0
        
        pos_init = moveit_commander.MoveGroupCommander('manipulator').get_current_pose().pose.position
        
        # Boxes will be described in a 3D array, 3rd dimension corresponding to box. 1st dimension are points, 2n are point coordinates
        current_box = np.zeros((4,3))
        
        min_box = np.array([pos_init.x, pos_init.y, pos_init.z])
        max_box = np.array([pos_init.x, pos_init.y, pos_init.z])
        
        while not finish_box and not finish_ws:
            x,y,z = (group.get_current_pose().pose.position.x, group.get_current_pose().pose.position.y, group.get_current_pose().pose.position.z)
            
            if x < min_box[0]:
                min_box[0] = x
            elif x > max_box[0]:
                max_box[0] = x
            if y < min_box[1]:
                min_box[1] = y
            elif y > max_box[1]:
                max_box[1] = y
            if z < min_box[2]:
                min_box[2] = z
            elif z > max_box[2]:
                max_box[2] = z
                
                
            if save_coord:
                # Every time button Y is pressed, a new point is added to the current box
                print "Saving coordinate %d!" %(box_coord)
                if box_coord == 4:
                    height = np.array(z)
                    finish_box = True
                else:
                    current_box[box_coord,:] = np.array([x,y,z])
                    box_coord += 1
                save_coord = False           
        
        if finish_ws:
            self.Boxes = self.Boxes[:,:,1:]
            self.heights = self.heights[1:]
            return
        
        try:
            self.BoxMin = np.concatenate((self.BoxMin,min_box[np.newaxis,:]),axis=0)
            self.BoxMax = np.concatenate((self.BoxMax,max_box[np.newaxis,:]),axis=0)
            print self.BoxMin
        except:
            self.BoxMin = min_box[np.newaxis,:]
            self.BoxMax = max_box[np.newaxis,:]
        
        
        self.Boxes = np.concatenate((self.Boxes,current_box), axis=2)
        self.heights = np.concatenate((self.heights,height))
        
        print "Box added!"
        
def joyCallback(data):
    
    global save_coord
    global finish_ws
    
    buttons = np.array(data.buttons)    
    
    if buttons[3] > 0:
        save_coord = True       
        
    if buttons[2] > 0:
        finish_ws = True
        
if __name__ == '__main__':  
    
    rospy.init_node('ur5_joy_node', anonymous=True)
    
    rate = rospy.Rate(100) # Hz
    
    # Setup robot UR5
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)
    
    joy_sub = rospy.Subscriber("/joy", Joy, joyCallback, queue_size=1)
    
    finish_box = False
    finish_ws = False
    save_coord = False

    print 'waiting for limits...'
    ws = Workspace()
    
    while not finish_ws:
        c = raw_input('Press enter to add new box') 
        print 'Saving box... Press "Y" to save box point, or "X" to finish last boxes.' 
        finish_box = False
        ws.addBox()
        
    
    print "Workspace created!"
    
    with open('ws.pkl','wb') as output:
        pickle.dump(ws,output,pickle.HIGHEST_PROTOCOL)
    
    print 'Limits saved!'
    print ws.BoxMax
    print ws.BoxMin
