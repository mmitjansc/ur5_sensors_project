/**************************************************************************/
/*!
@file takktile_arduino.ino
@author Eric Bakan & Yaroslav Tenzer
@license BSD

Driver for the TakkTile Strip sensor

This is a library for the TakkTile Strip sensor
----> http://www.takktile.com/product:takkstrip

@section HISTORY

v1.0 - First release by Eric Bakan
v1.1 - Updated for automatic sensor detection
v1.2 - Updated the code for wrapping related issues
v1.3 - Updated the code to reduce transmitted data bytes

@section NOTES

// Some logic copied from https://github.com/adafruit/Adafruit_MPL115A2/

*/
/**************************************************************************/

// Arduino addressing is using 7bit
// All the addresses were calculated in 8bit
// And were right shifted when actually used

#include <ros.h>
#include <Wire.h>


#include <std_msgs/String.h>
//#include <std_msgs/Int8MultiArray.h>
//#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <ur5_ros_arduino/pressures.h>
#include <ur5_ros_arduino/Num.h>

#define MAX_STRIPS 8
#define MAX_SENSORS 6

#define NUM_SENSORS MAX_STRIPS*MAX_SENSORS // reserve addresses for 8 strips with 6 sensors on each
#define PRECISION 0

#define FREESCALE_ADDRESS 0xC0
#define SENSOR_ALL_ON 0x0C
#define SENSOR_ALL_OFF 0x0D

#define _baud 115200


float a0[NUM_SENSORS];
float b1[NUM_SENSORS];
float b2[NUM_SENSORS];
float c12[NUM_SENSORS];

byte addressArray[NUM_SENSORS];
byte addressLength;

//float pressureHistory[NUM_SENSORS];
float *pressureHistory = (float*) malloc(sizeof(float)*NUM_SENSORS);
boolean flagHistoryExists=false;

boolean flagShowAddress=false;
boolean flagShowPressure=true;
boolean flagShowTemperature=false;

// ROS initialization
ros::NodeHandle nh;
// Modify Arduino's dedicated memory to publishers, subscribers, bytes in and bytes out:
//ros::NodeHandle_<ArduinoHardware, 10, 15, 128, 256> nh; // This shows that it was a memory issue!!
ur5_ros_arduino::Num test_;

//std_msgs::Float32MultiArray press_ros;
ur5_ros_arduino::pressures press_ros;
ros::Publisher node("/pressure", &press_ros);
ros::Publisher test("/chatter", &test_);

int count = 0;


void initialize() {

  
  
    // s 0C
  Wire.beginTransmission(SENSOR_ALL_ON>>1);
  Wire.endTransmission();
  
  // s C0 12 01
  Wire.beginTransmission(0xC0>>1);
  Wire.write(0x12);
  Wire.write(0x01);
  Wire.endTransmission();
  
  // s 0D
  Wire.requestFrom(SENSOR_ALL_ON>>1, 1);
  
  //delay(5);
}

void readCoeffs(byte addressSensor, byte num) {
  
  // Select sensor
  Wire.beginTransmission(addressSensor>>1);
  Wire.endTransmission();
  
  // Request coefficients
  Wire.beginTransmission(FREESCALE_ADDRESS>>1);
  Wire.write(0x04);
  Wire.endTransmission();
  Wire.requestFrom(FREESCALE_ADDRESS>>1, 8);
  int16_t a0coeff = (( (uint16_t) Wire.read() << 8) | Wire.read());
  int16_t b1coeff = (( (uint16_t) Wire.read() << 8) | Wire.read());
  int16_t b2coeff = (( (uint16_t) Wire.read() << 8) | Wire.read());
  int16_t c12coeff = (( (uint16_t) (Wire.read() << 8) | Wire.read())) >> 2;
  // Turn sensor off
  Wire.requestFrom(addressSensor>>1, 1);
  
  a0[num] = (float)a0coeff / 8;
  b1[num] = (float)b1coeff / 8192;
  b2[num] = (float)b2coeff / 16384;
  c12[num] = (float)c12coeff;
  c12[num] /= 4194304.0;
}

void setup () {

  //Serial.begin(_baud);
  Wire.begin();
  checkAddresses(); // check how many sensors are connected
 
  // for each found sensor, read the coefficients ..
  for(int i=0;i<addressLength;i++) {
    readCoeffs(addressArray[i],i);
  }
  

  // ROS setup
  nh.getHardware()->setBaud(_baud);
  nh.initNode();
  
  press_ros.vec.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension) * 1);
  press_ros.vec.layout.dim[0].label = "press node";
  press_ros.vec.layout.dim[0].size = addressLength;
  press_ros.vec.layout.dim[0].stride = addressLength;
  press_ros.vec.layout.data_offset = addressLength;
  press_ros.vec.data = (float *)malloc(sizeof(float)*addressLength);
  press_ros.vec.layout.dim_length = 0;
  press_ros.vec.data_length = addressLength;
  nh.advertise(node);
  pressureHistory = (float *) realloc(pressureHistory, sizeof(float)*(addressLength));

  nh.advertise(test);

}

void readData(byte addressSensor, float* oTemp, float* oPressure)
{
  
  // Select sensor
  Wire.beginTransmission(addressSensor>>1);
  Wire.endTransmission();

  // Request P/T data
  Wire.beginTransmission(FREESCALE_ADDRESS>>1);
  Wire.write((byte)0x00);
  Wire.endTransmission();

  Wire.requestFrom(FREESCALE_ADDRESS>>1, 4);
  uint16_t pressure = (( (uint16_t) Wire.read() << 8) | Wire.read()) >> 6;
  uint16_t temp = (( (uint16_t) Wire.read() << 8) | Wire.read()) >> 6;
  
  // Turn sensor off
  Wire.requestFrom(addressSensor>>1, 1);

  // ------ Ignore the calibrations for the moment
  
  float pressureComp = a0[addressSensor] + (b1[addressSensor] + c12[addressSensor] * temp) * pressure + b2[addressSensor] * temp;

  // Calculate temp & pressure
  *oPressure = ((65.0F / 1023.0F) * pressureComp) + 50.05F; // kPa
  *oTemp = ((float) temp - 498.0F) / -5.35F + 25.0F; // C
  
  // ------ 

  *oPressure = pressure;
  *oTemp = temp;
}

void checkAddresses()
{
  addressLength=0;
  int temp_add=0;
  //Serial.println(MAX_STRIPS*MAX_SENSORS);
  // check every strip
  for (int strip_n=0;strip_n<MAX_STRIPS;strip_n++){
    // check every sensor
    for (int sensor_n=0;sensor_n<MAX_SENSORS;sensor_n++){
      
      //Serial.print(strip_n);
      //Serial.print(sensor_n);
      
      temp_add=(strip_n<<4)+sensor_n*2; // calculate the address
      //Serial.print("End of line");
      // check if the Attiny responds with its address
      // this also switches ON the Chip Select line on the desired sensor
      Wire.beginTransmission(temp_add>>1); // take into account that its 7bit !
      // if response finishes with an ACK then continue
      if (Wire.endTransmission()==0) {
        // check if there is a sensor on this line
        Wire.beginTransmission(FREESCALE_ADDRESS>>1);
        // if there is an ACK then there is a sensor
        if (Wire.endTransmission()==0){
          //Serial.print("FOUND SENSOR!");
          //Serial.print(strip_n);
          //Serial.print(sensor_n);
          //Serial.print('\n');
          addressArray[addressLength]=temp_add;
          addressLength++;
          //Serial.println(addressLength);
        }
        // Turn off the chip select line
        Wire.requestFrom(temp_add>>1, 1);
      }
      //Serial.println("");
    }
    
  }

  //addressLength = 5;
  
  //addressLength++;
  //Serial.println(addressLength);
}

void loop() {

  //float t = micros();

  //Serial.println(addressLength);
  
  float oTemp=0;
  float oPressure=0;
  float p_current=0;
  float p_history=0;
  float delta_up=0;
  float delta_down=0;
  //float foo_array[3];
  
  initialize();
  
  //Serial.print('[');

  for(int i=0;i<addressLength;i++)
  {
    if (i>0){
          //Serial.print(',');
    }
    readData(addressArray[i], &oTemp, &oPressure);
  

    // the calculations of the wrapping
    // that are used to calculate the minus signs
    if (flagHistoryExists){
      p_current=oPressure;
      p_history=pressureHistory[i];
      delta_up=p_current-p_history;
      delta_down=p_history-(p_current-1024);
      
      if (delta_up<delta_down){
        oPressure=p_history+delta_up;
      } 
      else {
        oPressure=p_history-delta_down;
      }
    }
    
    
    //Serial.println(oPressure);
    pressureHistory[i] = oPressure;
    press_ros.vec.data[i] = oPressure; // Topic data
    //press_ros.data[i*2] = delta_up;
    //press_ros.data[2*i+1] = delta_down;
    //foo_array[i] = oPressure;
    //Serial.println(sizeof(pressureHistory)/sizeof(pressureHistory[0]));
    
  }
  
  flagHistoryExists=true;

  test_.first_name = "Marc";
  test_.last_name = "Mitjans";
  test_.score = count;
  test_.age = 27;

  count++;
  
  test.publish(&test_);
  press_ros.header.stamp = nh.now();
  node.publish(&press_ros); // Publish pressures to topic
  nh.spinOnce(); // spin ROS

}
