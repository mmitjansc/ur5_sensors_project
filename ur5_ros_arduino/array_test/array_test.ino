#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

#define _baud 9600

// ROS initialization
ros::NodeHandle nh;
std_msgs::Float32MultiArray press_ros;
ros::Publisher node("pressure", &press_ros);

//Initializing LED Pin



int led_pin = 6;
int addressLength = 10;

float *arraytest = (float *) malloc(48*sizeof(float));

//uint8_t * test = (uint8_t *) malloc(10*sizeof(uint8_t));

void setup() {
  //Declaring LED pin as output
  pinMode(led_pin, OUTPUT);
  //Serial.begin(_baud);

  // ROS setup
  nh.getHardware()->setBaud(_baud);
  nh.initNode();
  press_ros.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension) * 1);
  press_ros.layout.dim[0].label = "press node";
  press_ros.layout.dim[0].size = addressLength;
  press_ros.layout.dim[0].stride = addressLength;
  press_ros.layout.data_offset = 0;
  press_ros.data = (float *)malloc(sizeof(float)*addressLength);
  press_ros.layout.dim_length = 0;
  press_ros.data_length = addressLength;
  nh.advertise(node);

  arraytest = (float *) realloc(arraytest, sizeof(float)*(addressLength));


  
}
void loop() {
  
  //Serial.println(sizeof(test));

  for(int i=0;i<addressLength;i++)
  {
    arraytest[i] = i;
    press_ros.data[i] = i;
  }
  node.publish(&press_ros); // Publish pressures to topic
  nh.spinOnce(); // spin ROS
  delay(50);
}
