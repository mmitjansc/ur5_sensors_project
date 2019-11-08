//uino addressing is using 7bit
// All the addresses were calculated in 8bit
// And were right shifted when actually used
#include <ros.h>
#include <Wire.h>
#include <std_msgs/Float32MultiArray.h>

#define MAX_STRIPS 8
#define MAX_SENSORS 6

#define NUM_SENSORS MAX_STRIPS*MAX_SENSORS // reserve addresses for 8 strips with 6 sensors on each
#define PRECISION 0

#define FREESCALE_ADDRESS 0xC0
#define SENSOR_ALL_ON 0x0C
#define SENSOR_ALL_OFF 0x0D
#define _baud 9600


float a0[NUM_SENSORS];
float b1[NUM_SENSORS];
float b2[NUM_SENSORS];
float c12[NUM_SENSORS];

byte addressArray[NUM_SENSORS];
byte addressLength;

float pressureHistory[NUM_SENSORS];
boolean flagHistoryExists=false;

boolean flagShowAddress=false;

boolean flagShowPressure=true;
boolean flagShowTemperature=true;


// ROS initialization
ros::NodeHandle nh;
std_msgs::Float32MultiArray press_ros;
ros::Publisher node("pressure", &press_ros);

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
  
  delay(5);
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
  Wire.begin();
  Serial.begin(_baud);
  
  checkAddresses(); // check how many sensors are connected
 
  // for each found sensor, read the coefficients ..
  for(int i=0;i<addressLength;i++) {
    readCoeffs(addressArray[i],i);
  }

  
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
}

void readData(byte addressSensor, float* oTemp, float* oPressure)
{
  // Select sensor
  Wire.beginTransmission(addressSensor>>1);
     //   Serial.print('A');

  Wire.endTransmission();
   //Serial.print('B');
  // Request P/T data
  Wire.beginTransmission(FREESCALE_ADDRESS>>1);
  //Serial.print('D');
  Wire.write((byte)0x00);
  //Serial.print('E');
  Wire.endTransmission();
  //Serial.print('F');
  Wire.requestFrom(FREESCALE_ADDRESS>>1, 4);
  uint16_t pressure = (( (uint16_t) Wire.read() << 8) | Wire.read()) >> 6;
  uint16_t temp = (( (uint16_t) Wire.read() << 8) | Wire.read()) >> 6;
  //Serial.print('G');
  // Turn sensor off
  Wire.requestFrom(addressSensor>>1, 1);
  //Serial.print('H');
  // ------ Ignore the calibrations for the moment
  
  float pressureComp = a0[addressSensor] + (b1[addressSensor] + c12[addressSensor] * temp) * pressure + b2[addressSensor] * temp;

  // Calculate temp & pressure
 *oPressure = ((65.0F / 1023.0F) * pressureComp) + 50.05F; // kPa
  *oTemp = ((float) temp - 498.0F) / -5.35F + 25.0F; // C
  
  // ------ 

  *oPressure = pressure;
     // Serial.print("*oPressure: ");
     // Serial.print(pressure);
  *oTemp = temp;
    //  Serial.print("*oTemp: ");
    //  Serial.print(temp);
}

void checkAddresses()
{
  addressLength=0;
  int temp_add=0;
  // check every strip
  for (int strip_n=0;strip_n<MAX_STRIPS;strip_n++){
    // check every sensor
    for (int sensor_n=0;sensor_n<MAX_SENSORS;sensor_n++){
      temp_add=(strip_n<<4)+sensor_n*2; // calculate the address

      // check if the Attiny responds with its address
      // this also switches ON the Chip Select line on the desired sensor
      Wire.beginTransmission(temp_add>>1); // take into account that its 7bit !
      // if response finishes with an ACK then continue
      if (Wire.endTransmission()==0) {
        // check if there is a sensor on this line
        Wire.beginTransmission(FREESCALE_ADDRESS>>1);
        // if there is an ACK then there is a sensor
        if (Wire.endTransmission()==0){
          addressArray[addressLength]=temp_add;
          addressLength++;
        }
        // Turn off the chip select line
        Wire.requestFrom(temp_add>>1, 1);
      }
    }
    Serial.println(']');
  }
}

void loop() {

  float oTemp=0;
  float oPressure=0;
  float p_current=0;
  float p_history=0;
  float delta_up=0;
  float delta_down=0;
  
  initialize();
 
  //Serial.print('[');

  for(int i=0;i<addressLength;i++)
  {
    if (i>0){
         // Serial.print(',');
    }
      //Serial.print('R');

    readData(addressArray[i], &oTemp, &oPressure);
      //Serial.print('P');


    // the calculations of the wrapping
    // that are used to calculate the minus signs
    if (flagHistoryExists){
      p_current=oPressure;
      p_history=pressureHistory[i];
      delta_up=p_current-p_history;
      delta_down=p_history-(p_current-1024);
      if (delta_up<delta_down){
        oPressure=p_history+delta_up;
      }else{
        oPressure=p_history-delta_down;
      }
    }
    pressureHistory[i]=oPressure;
    
    // ------------------------------
    // Start output to the serial port
    
    //Serial.print('[');
    
    //Serial.print('[');
    // Print out sensor ID value if the flag was set
    if (flagShowAddress){
      Serial.print(addressArray[i],HEX);
    }

    // Print out Pressure values if the flag was set
    if (flagShowPressure){
      if (flagShowAddress){
        //Serial.print(',');
      }
    Serial.print(oPressure,PRECISION);
    }

    // Print out Temperature values if the flag was set
    if (flagShowTemperature){
      if (flagShowPressure){
        Serial.print(',');
      }
    //Serial.print(oTemp,PRECISION);
    }
    //Serial.print(']');
  }
  Serial.println('\t');
 
    // End output to the serial port
    // ------------------------------
  flagHistoryExists=true;
  
  // Listen to the commands from the serial port
  if (Serial.available()){
    byte inByte = (byte)
    Serial.read();
    if (inByte=='n') { flagShowAddress = !flagShowAddress; }
    if (inByte=='p') { flagShowPressure = !flagShowPressure; }
    if (inByte=='t') { flagShowTemperature = !flagShowTemperature; }
    
  }
}
