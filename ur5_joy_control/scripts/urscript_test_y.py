#!/usr/bin/env python

from __future__ import print_function

import rospy
import sys
from std_msgs.msg import String
import signal
from geometry_msgs.msg import PoseStamped
import math
import moveit_commander

class UR5TestClass(object):

    def __init__(self):
        self.stop = False

        self.data = PoseStamped()
        self.pub = rospy.Publisher("/ur_driver/URScript", String, queue_size=1)
        self.pub_pose_stamped = rospy.Publisher("/pose_tracking/goal", PoseStamped, queue_size=1)

        self.RATE = 1./2 # Frequency in Hz
        self.MAX_DURATION = 120
        self.loop_rate = rospy.Rate(self.RATE)


    def shutdown(self,sig,num):
        rospy.loginfo("Stopping...")
        self.stop = True
        
    def quaternion_to_logmap(self,q):
        # Quaternion to axis-angle represenatation. It's the orientation representation requried by URScript
        
        x = 2 * math.acos(q.w) * q.x/math.sqrt(1-q.w*q.w)
        y = 2 * math.acos(q.w) * q.y/math.sqrt(1-q.w*q.w)
        z = 2 * math.acos(q.w) * q.z/math.sqrt(1-q.w*q.w)
        return [x,y,z]

    def movelTest(self):
        start = rospy.Time.now()
        test_duration = rospy.Duration(self.MAX_DURATION); 
        rospy.loginfo("move l testing")

        while not self.stop and not rospy.is_shutdown() and (rospy.Time.now() < start+test_duration):

            # Print current position.
            # By reading the EE poses with the function below during Xbox joystick control,
            # I chose reasonable values to hardcode in the 'movel' tests.
            # print(group.get_current_pose())


            # movel(pose,accelerations,tool speed,time,blend radius)
            self.data.pose.position.x = 30/1000.
            self.data.pose.position.y = -336/1000.
            self.data.pose.position.z = 241./1000
            
            #no orientation change needed
            self.data.pose.orientation.x = -0.353747333256
            self.data.pose.orientation.y = 0.500507568716
            self.data.pose.orientation.z = 0.53862829944
            self.data.pose.orientation.w = 0.578130221416
            # The orientation needs to be in axis-angle representation
            # orientation = self.quaternion_to_logmap(self.data.pose.orientation)
            orientation = [0.055, 3.03, -1.032]

            #change the following attributes of movel function 
            accel = 1.2
            ee_speed = 0.5
            time = 0 # Needed 0 so that movel respects the speed and accel parameters
            blend_radius = 0 # Not needed

            #publish poseStamped msg for post processing
            rospy.loginfo("commanding first target pose")
            self.pub_pose_stamped.publish(self.data)
            self.pub.publish("movel(p[%f,%f,%f,%f,%f,%f], %f, %f)" % (self.data.pose.position.x,self.data.pose.position.y,self.data.pose.position.z,\
                orientation[0], orientation[1], orientation[2], accel, ee_speed))

            self.loop_rate.sleep() # Sleep

            # self.data.pose.position.x = 0
            self.data.pose.position.y = -805./1000
            # self.data.pose.position.z = 0.342343691613
            
            ## no orientation change needed
            # self.data.pose.orientation.x = -0.4
            # self.data.pose.orientation.y = 0.4
            # self.data.pose.orientation.z = 0.6
            # self.data.pose.orientation.w = 0.57
            # orientation = quaternion_to_logmap(self.data.pose.orientation)

            ## publish poseStamped msg for post processing
            rospy.loginfo("commanding second target pose")
            self.pub_pose_stamped.publish(self.data)
            self.pub.publish("movel(p[%f,%f,%f,%f,%f,%f], %f, %f, %f)" % (self.data.pose.position.x,self.data.pose.position.y,self.data.pose.position.z,\
                orientation[0], orientation[1], orientation[2], accel, ee_speed, time))

            self.loop_rate.sleep() # Sleep 

        self.pub.publish("speedl([0,0,0,0,0,0],1.5,100.0)") # Stop the UR5!
        rospy.loginfo("Test ended.")



if __name__ == '__main__':

    rospy.init_node('ur5_test', anonymous=True)

    # The following lines are needed to read the UR5 RR poses from MoveIt. Maybe the error is here...
    # robot = moveit_commander.RobotCommander()
    # scene = moveit_commander.PlanningSceneInterface()
    # group_name = "manipulator"
    # group = moveit_commander.MoveGroupCommander(group_name)

    ur5_test = UR5TestClass()
    # This signal handler allows us to safely stop the robot with CTRL+C.
    signal.signal(signal.SIGINT,ur5_test.shutdown)
    
    ur5_test.movelTest()

    rospy.loginfo("Program finished.")
    