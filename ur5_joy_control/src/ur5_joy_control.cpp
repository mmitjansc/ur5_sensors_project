#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Joy.h"
#include <iostream>

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	std_msgs::Header header = msg->header;
	std::vector<float> axes = msg->axes;
	std::vector<int> buttons = msg->buttons;
	
	std::cout << axes[0] << std::endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "shear_force");
	ros::NodeHandle n;
	ros::Rate loop_rate(1000);

	ros::Subscriber sub = n.subscribe("/joy", 1000, joyCallback);
	
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
