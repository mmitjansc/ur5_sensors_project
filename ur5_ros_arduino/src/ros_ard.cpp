

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include <iostream>
#include <fstream>
#include <string>
#include "ur5_ros_arduino/pressures.h"

#include <sstream> 

namespace patch {
	// Patch to solve the non-existence of std::string::to_string()
    template <typename T> std::string to_string(const T& n) {
        std::ostringstream stm;
        stm << n;
        return stm.str();
    }
} 

std::ofstream myfile;

void chatterCallback(const ur5_ros_arduino::pressures::ConstPtr& msg) {
	//ROS_WARN("RECEIVED MESSAGE");
	for (int i=0;i < msg->vec.data.size(); i++) {
		// Every time we read a new pressure signal, we add a new row with 
		// pressures separated by commas.
		myfile << patch::to_string(msg->vec.data[i]) << ",";
	}
	// Create new row
	myfile << "\n";	
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ros_ard");
	ros::NodeHandle n;
	ros::Rate loop_rate(9600);
	
	ros::Subscriber press_sub = n.subscribe("/pressure", 1000, chatterCallback);
	
	std::string s("pressures.csv");
	
	myfile.open(s.c_str());
	
	ROS_INFO("Adding data to %s...", s.c_str());
	
	while (ros::ok())
	{

		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}
