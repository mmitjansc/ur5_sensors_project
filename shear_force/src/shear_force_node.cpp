#include "ros/ros.h"
#include "ur5_ros_arduino/pressures.h"
#include "std_msgs/String.h"


void pressureCallback(const ur5_ros_arduino::pressures::ConstPtr& msg)
{
	//ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void FTCallback(const std_msgs::String::ConstPtr& msg)
{
	//ROS_INFO("I heard: [%s]", msg->data.c_str());
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "shear_force");
	ros::NodeHandle n;
	ros::Rate loop_rate(1000);
	
	ros::Subscriber sub = n.subscribe("/pressure", 1000, pressureCallback);
	ros::Subscriber sub2 = n.subscribe("/FT_sensor/robotiq_force_torque_sensor", 1000, FTCallback);
	
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
