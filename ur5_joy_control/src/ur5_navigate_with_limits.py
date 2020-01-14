#!/usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg as gm
import pickle
import time
import copy

from shapely.geometry import Point, LinearRing
from shapely.geometry.polygon import Polygon

from tf.transformations import euler_from_quaternion

from math import pi
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Joy
from moveit_commander.conversions import pose_to_list
import numpy as np
from set_ur5_limits import Workspace

axes = np.zeros((8,))
buttons = np.zeros((11,))

force_lim = 5 # Limit of force allowed on wrist

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
        pass
        #invert_velocity()

if __name__ == '__main__':  
    
    rospy.init_node('ur5_joy_node', anonymous=True)

    # Double check rate feasibility:
    rate = rospy.Rate(1000) # Hz

    sub = rospy.Subscriber("/joy", Joy, joyCallback, queue_size=1)
    pub = rospy.Publisher("/ur_driver/URScript", String, queue_size=1)

    force_sub = rospy.Subscriber("/FT_sensor/robotiq_force_torque_wrench", gm.WrenchStamped, FTCallback, queue_size=1) 
    inside_pub = rospy.Publisher("/inside_ws", Bool, queue_size=1) 

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
    eroded_polys = [Polygon(boxes[k,:,:2]).buffer(-0.02) for k in range(boxes.shape[0])]
    
    print("Boxes: ",boxes)
    print("heights: ",heights)
    
    recovering = False
    first_time = True
    z_goal,y_goal,x_goal = 0,0,0

    while not rospy.is_shutdown():
        
        if first_time:
            height = buttons[5] - buttons[4]
            recov_axes = axes
            recov_height = height

        curr_pos = group.get_current_pose().pose.position

        z = curr_pos.z
        x = curr_pos.x
        y = curr_pos.y
            
        point = Point(curr_pos.x, curr_pos.y)            
        
        inside = False
        closest_poly = 0
        dist = float('inf')
        for i in range(boxes.shape[0]):
            z_min = boxes[i,0,-1]
            z_max = heights[i]
            
            if polygons[i].contains(point) and z_min < z and z < z_max:
                inside = True
                if recovering:
                    pub.publish("stopl(5.0, 5.0)")
                    first_time = True
                    print 'RECOVERED'
                    curr_pos = group.get_current_pose().pose.position
                    recovering = False
                    inside_pub.publish(recovering)
                break
            
            d = polygons[i].distance(point)
            if (z_min > z or z > z_max) and d < 1e-8:
                d = min(abs(z-z_min),abs(z-z_max))
            if d < dist:                
                closest_poly = eroded_polys[i]
                dist = d 
                      
        
        
        if not inside:
            # Switch velocity input to keep EE within ws boundaries
            recovering = True
            inside_pub.publish(recovering)
            
            if first_time:    
                        
                #print(axes,height)
                rospy.logwarn('NOT INSIDE! Recovering')       
                
                pub.publish("stopl(5.0, 5.0)")  
                wpose = group.get_current_pose().pose

                curr_pos = np.array([wpose.position.x,wpose.position.y,wpose.position.z])
                
                if (z_min > z or z > z_max) and polygons[i].distance(point) < 1e-8:
                    if z < z_min:
                        z_goal = z_min+0.02
                    elif z > z_max:
                        z_goal = z_max-0.02
                    closest_point_coords = np.array([wpose.position.x, wpose.position.y, z_goal])
                    
                else:                
                    pol_ext = LinearRing(closest_poly.exterior.coords)
                    a = pol_ext.project(point)
                    b = pol_ext.interpolate(a)
                    closest_point_coords = list(b.coords)[0]
                    z_goal = wpose.position.z
                    closest_point_coords = np.array([closest_point_coords[0],closest_point_coords[1],z_goal])
                
                time.sleep(0.1)
                
                speed_vec = closest_point_coords - curr_pos
                speed_vec /= 10*(np.linalg.norm(speed_vec))                
                #print(np.linalg.norm(speed_vec))
                
                pub.publish("speedl([-%f,-%f,%f,0,0,0], 1., 100.0,3.0)"%(speed_vec[0],speed_vec[1],speed_vec[2]))
                
                first_time = False       
        
        rate.sleep()
