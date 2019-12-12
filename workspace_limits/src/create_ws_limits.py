import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg as gm

from math import pi
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from moveit_commander.conversions import pose_to_list
import numpy as np

if __name__ == '__main__':	

    rospy.init_node('ur5_ws_limits', anonymous=True)

    rate = rospy.Rate(100) # Hz

    #sub = rospy.Subscriber("/joy", Joy, joyCallback, queue_size=1)
    #pub = rospy.Publisher("/ur_driver/URScript", String, queue_size=1)

    # Setup robot UR5
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)

    while not rospy.is_shutdown():

        #print group.get_current_pose()

        # LIMITS:
        curr_pos = group.get_current_pose().pose.position

        print(curr_pos)

        rate.sleep()