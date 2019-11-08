#include "ros/ros.h"
#include "ur5_ros_arduino/pressures.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "std_msgs/Float32MultiArray.h"
#include "robotiq_force_torque_sensor/ft_sensor.h"
#include "geometry_msgs/WrenchStamped.h"

void pressureCallback(const ur5_ros_arduino::pressures::ConstPtr& msg)
{
	std_msgs::Header header = msg->header;
	std_msgs::Float32MultiArray data = msg->vec;
}

void FTCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
	std_msgs::Header header = msg->header;
	geometry_msgs::Wrench wrench = msg->wrench;
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
