#include <ros.h>
#include <std_msgs/Int16.h>

#define THRESH 500

const int analogPin = A0;
int sensorVal = 0;

// ROS initialization
ros::NodeHandle nh;

std_msgs::Int16 ir_data;
ros::Publisher node("/ir_sensor", &ir_data);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

  sensorVal = analogRead(A0);
  if (sensorVal > THRESH) {
    // Publish something somewhere
  }
  
  
}
