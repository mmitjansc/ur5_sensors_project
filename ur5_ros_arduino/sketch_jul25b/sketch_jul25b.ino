#include <Wire.h>

#define MAX_STRIPS 8
#define MAX_SENSORS 5

#define NUM_SENSORS MAX_STRIPS*MAX_SENSORS // reserve addresses for 8 strips with 6 sensors on each
#define PRECISION 0

#define FREESCALE_ADDRESS 0xC0
#define SENSOR_ALL_ON 0x0C
#define SENSOR_ALL_OFF 0x0D

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
boolean flagShowTemperature=false;

void setup() {
  // put your setup code here, to run once:

  Wire.begin();
  Serial.begin(115200);
 

}

void loop() {
  // put your main code here, to run repeatedly:

  addressLength=0;
  int temp_add=0;
  // check every strip
  
  for (int strip_n=0;strip_n<MAX_STRIPS;strip_n++){

    // check every sensor
    for (int sensor_n=0;sensor_n<MAX_SENSORS;sensor_n++){
      
      Serial.print("strip_n: ");
      Serial.print(strip_n);
      Serial.print("sensor_n: ");
      Serial.print(sensor_n);
      Serial.print('\n');

      temp_add=(strip_n<<4)+sensor_n*2; // calculate the address
      Serial.print("temp_add: ");
      Serial.print(temp_add);
      Serial.print('\n');
      
      // check if the Attiny responds with its address
      Wire.beginTransmission(temp_add>>1); // take into account that its 7bit !
      Serial.print(temp_add>>1);
      Wire.endTransmission();
      Serial.print("Wire.endTransmission: ");
      Serial.print(Wire.endTransmission());
      Serial.print('\n');
      Serial.print("okay");
      Serial.print('\n');


      if (Wire.endTransmission()==0)
      {
        Serial.print('A');
        Serial.print('\n');  
        // check if there is a sensor on this line
        Wire.beginTransmission(FREESCALE_ADDRESS>>1);
        Wire.endTransmission();
        Serial.print("Wire.endTransmission: ");
        Serial.print(Wire.endTransmission());
        
        Serial.print('\n');
        
        Serial.print('\n');
       
        if (Wire.endTransmission()==0){
          addressArray[addressLength]=temp_add;
          Serial.print(temp_add);
          Serial.print(':');
          Serial.println(addressLength);
          Serial.print('\n');
          addressLength++;
          Serial.print(addressLength);
          Serial.print("                              SENSOR FOUND !!!");
          Serial.print('\n');
          

        }
         
         
      }
      Serial.print('\n');
      Serial.print("NEXT");
      Serial.print('\n');
      Serial.print('\n');
    }
 }
  Serial.print("ADDRESS LENGTH: ");
      Serial.println(addressLength);

}


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
