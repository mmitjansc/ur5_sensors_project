#!/usr/bin/env python

from __future__ import print_function

import rospy
import sys
from std_msgs.msg import String
import signal
from geometry_msgs.msg import PoseStamped
import math
import moveit_commander

def shutdown(sig,num):
    print("Stopping...")
    # group.stop()
    pub.publish("speedl([0,0,0,0,0,0], 1.5, 100.0)")
    rospy.Rate(1).sleep()
    rospy.signal_shutdown("Keyboard interrupt")
    print("Closing system.")
    sys.exit(0)

def quaternion_to_logmap(q):
    x = q.x/math.sqrt(1-q.w*q.w)
    y = q.y/math.sqrt(1-q.w*q.w)
    z = q.z/math.sqrt(1-q.w*q.w)
    return [x,y,z]

def movelTest():
    start = rospy.Time.now()
    test_duration = rospy.Duration(20); 
    print("move l testing")
    while not rospy.is_shutdown() and (rospy.Time.now() < start+test_duration):

        # Print current position:
        print(group.get_current_pose())


        # movel(pose,accelerations,tool speed,time,blend radius)
        data.pose.position.x = 0
        data.pose.position.y = 0.225113553855
        data.pose.position.z = 0.342343691613
        
        #no orientation change needed
        data.pose.orientation.x = -0.477391597312
        data.pose.orientation.y = 0.297177015047
        data.pose.orientation.z = 0.675088355614
        data.pose.orientation.w = 0.477534079053
        orientation = quaternion_to_logmap(data.pose.orientation)
        #change the following attributes of movel function 
        accel = 1.2
        ee_speed = 0.05
        time = 0
        blend_radius = 0

        #publish poseStamped msg for post processing
        rospy.loginfo("commanding first target pose")
        pub_pose_stamped.publish(data)
        pub.publish("movel(p[%f,%f,%f,%f,%f,%f], %f, %f, %f)" % (data.pose.position.x,data.pose.position.y,data.pose.position.z,\
             orientation[0], orientation[1], orientation[2], accel, ee_speed, time))

        rate.sleep() # sleep at the same amount time as movel pose command 

        # movel(pose,accelerations,tool speed,time,blend radius)
        # data.pose.position.x = 0
        # data.pose.position.y = 0.225113553855
        # data.pose.position.z = 0.342343691613
        
        #no orientation change needed
        # data.pose.orientation.x = -0.4
        # data.pose.orientation.y = 0.4
        # data.pose.orientation.z = 0.6
        # data.pose.orientation.w = 0.57
        # orientation = quaternion_to_logmap(data.pose.orientation)

        #change the following attributes of movel function 
        # accel = 0
        # ee_speed = 0
        # time = 0
        # blend_radius = 0

        # publish poseStamped msg for post processing
        # rospy.loginfo("commanding second target pose")
        # pub_pose_stamped.publish(data)
        # print("movel([%f,%f,%f,%f,%f,%f], %f, %f, %f)" % (data.pose.position.x,data.pose.position.y,data.pose.position.z,\
        #      orientation[0], orientation[1], orientation[2], accel, ee_speed, time))
        # pub.publish("movel([%f,%f,%f,%f,%f,%f], %f, %f, %f)" % (data.pose.position.x,data.pose.position.y,data.pose.position.z,\
        #      orientation[0], orientation[1], orientation[2], accel, ee_speed, time))

        # rate.sleep() # sleep at the same amount time as movel pose command 



if __name__ == '__main__':
    signal.signal(signal.SIGINT,shutdown)

    rospy.init_node('ur5_test', anonymous=True)
    data = PoseStamped()
    pub = rospy.Publisher("/ur_driver/URScript", String, queue_size=1)
    pub_pose_stamped = rospy.Publisher("/pose_tracking/goal", PoseStamped, queue_size=1)
    rate = rospy.Rate(10)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)

    movelTest()
    rospy.loginfo("test completed")
    # spedlTest()
    