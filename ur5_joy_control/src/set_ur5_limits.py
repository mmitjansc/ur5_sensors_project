#!/usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg as gm
import pickle
import time

from math import pi
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from moveit_commander.conversions import pose_to_list
import numpy as np

class Workspace:
    def __init__(self):
        self.Boxes = np.zeros((1,4,3))
        self.heights = np.zeros((1,))
        
    def addBox(self):

        global save_coord
        global finish_box
        
        box_coord = 0
        
        pos_init = group.get_current_pose().pose.position
        
        # Boxes will be described in a 3D array, 1st dimension corresponding to box. 
        # 2nd dimension are points, 3rd are point coordinates
        current_box = np.zeros((1,4,3))
        
        while not finish_box and not finish_ws:
            x,y,z = (group.get_current_pose().pose.position.x, group.get_current_pose().pose.position.y, group.get_current_pose().pose.position.z)
                
            if save_coord:
                # Every time button Y is pressed, a new point is added to the current box
                
                if box_coord == 4:
                    print "Saving height!"
                    height = z
                    finish_box = True
                else:
                    print "Saving coordinate %d!" %(box_coord)
                    current_box[0,box_coord,:] = np.array([x,y,z])
                    box_coord += 1
                save_coord = False    
                time.sleep(0.5)          
        
        if finish_ws:
            self.Boxes = self.Boxes[1:,:,:]
            self.heights = self.heights[1:]
            return

        self.Boxes = np.concatenate((self.Boxes,current_box), axis=0)
        self.heights = np.concatenate((self.heights,height), axis=None)
        
        print "Box added!"
        
def joyCallback(data):
    
    global save_coord
    global finish_ws
    global restart_button
    
    buttons = np.array(data.buttons)    
    
    if buttons[3] > 0 and restart_button:
        #print 'Save coord!'    
        save_coord = True
        restart_button = False
        
    if buttons[3] < 1:
        restart_button = True
            
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

    time.sleep(0.5)
    ws = Workspace()
    
    restart_button = True
    
    while not finish_ws:
        c = raw_input('Press enter to add new box') 
        print 'Press "Y" to save box point, or "X" to finish last boxes.' 
        finish_box = False
        ws.addBox()
        
    
    print "Workspace created!"
    
    with open('ws.pkl','wb') as output:
        pickle.dump(ws,output,pickle.HIGHEST_PROTOCOL)
    
    print 'Limits saved!'

