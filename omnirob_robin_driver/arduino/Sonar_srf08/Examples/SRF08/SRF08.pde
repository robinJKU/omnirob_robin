/*
* SRF08.pde - example interfacing with sonar sensor srf08
*	      library uses Wire.h 
*		 SDA - Analog Pin 4
*		 SCL - Analog Pin 5
*
* Author- Zach Foresta - Foureza87@yahoo.com
*         April 2009
*
* For component setup - http://www.arduino.cc/playground/Main/SonarSrf08
*/

#include <Wire.h>
#include <Sonar_srf08.h>

Sonar_srf08 MySonar;

#define CommandRegister 0x00
int New_Address = 248; //0xF8
#define ResultRegister  0x02

int DEBUG = 1;
char unit = 'i'; // 'i' for inches , 'c' for centimeters
float sensorReading =0;


void setup()
{
 MySonar.connect();  
 MySonar.changeAddress(CommandRegister, New_Address);
 if (DEBUG){
   Serial.begin(9600);
 }
 New_Address += 4; 
//offset address not sure why this is but it works for this address
}

void loop()
{
// set units for reading out distance
  MySonar.setUnit(CommandRegister, New_Address, unit);
//pause
  delay(70);
// set register for reading
  MySonar.setRegister(New_Address, ResultRegister);
// read data from result register
  sensorReading = MySonar.readData(New_Address, 2);
//print out distance
  Serial.print("Distance: ");
  Serial.print(sensorReading);
  Serial.print(" inches");
  Serial.println();
//pause
  delay(70);
}
