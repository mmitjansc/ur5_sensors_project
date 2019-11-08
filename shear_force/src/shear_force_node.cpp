#include "ros/ros.h"
#include "ur5_ros_arduino/pressures.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "std_msgs/Float32MultiArray.h"
//#include "robotiq_force_torque_sensor/ft_sensor.h"
#include "geometry_msgs/WrenchStamped.h"
#include <iostream>
#include <fstream>

#include <sstream> 

namespace str {
	// Patch to solve the non-existence of std::string::to_string()
    template <typename T> std::string to_string(const T& n) {
        std::ostringstream stm;
        stm << n;
        return stm.str();
    }
} 

std::ofstream fingertip;
std::ofstream ft_sensor;

void pressureCallback(const ur5_ros_arduino::pressures::ConstPtr& msg)
{
	std_msgs::Header header = msg->header;
	std_msgs::Float32MultiArray data = msg->vec;
	
	fingertip << str::to_string(header.stamp.sec) << "," << str::to_string(header.stamp.nsec) << ",";
	
	for (int i = 0; i < data.data.size(); i++) {
		fingertip << str::to_string(data.data[i]) << ",";
	}
	fingertip << "\n";
}

void FTCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
	std_msgs::Header header = msg->header;
	geometry_msgs::Wrench wrench = msg->wrench;
	
	ft_sensor << str::to_string(header.stamp.sec) << "," << str::to_string(header.stamp.nsec) << ",";
	ft_sensor << str::to_string(wrench.force.x) << "," << str::to_string(wrench.force.y) << "," << str::to_string(wrench.force.z) << ",";
	
	ft_sensor << "\n";
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "shear_force");
	ros::NodeHandle n;
	ros::Rate loop_rate(1000);
	
	ft_sensor.open("ft_sensor.csv");
	fingertip.open("fingertip.csv");
	
	ros::Subscriber sub = n.subscribe("/pressure", 1000, pressureCallback);
	ros::Subscriber sub2 = n.subscribe("/FT_sensor/robotiq_force_torque_wrench", 1000, FTCallback);
	
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
