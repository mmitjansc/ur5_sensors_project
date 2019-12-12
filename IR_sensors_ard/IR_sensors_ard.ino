#include <ros.h>

#define THRESH 500

const int inPin = A0;
const int outPin = A1;
int sensorVal = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(outPin,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  sensorVal = analogRead(A0);
  if sensorVal > THRESH {
    // Publish something somewhere
  }

  delay(100); // X milliseconds
  
  
}
