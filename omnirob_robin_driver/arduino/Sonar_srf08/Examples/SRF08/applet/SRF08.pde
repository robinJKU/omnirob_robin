#include <WProgram.h>
#include <Wire.h>
#include <Sonar_srf08.h>

Sonar_srf08 MySonar;

#define CommandRegister 0x00
int New_Address = 248; //0xF8
#define ResultRegister  0x02

int DEBUG = 1;
char unit = 'i'; // 'i' inches , 'c' -> centimeters
float sensorReading =0;


void setup()
{
 MySonar.connect();  
 MySonar.changeAddress(CommandRegister, New_Address);
 if (DEBUG){
   Serial.begin(9600);
 }
 New_Address += 4; //offset address??
}

void loop()
{
  MySonar.setUnit(CommandRegister, New_Address, unit);
  delay(70);
  MySonar.setRegister(New_Address, ResultRegister);
  sensorReading = MySonar.readData(New_Address, 2);
  Serial.print("Distance: ");
  Serial.print(sensorReading);
  Serial.print(" inches");
  Serial.println();
  delay(70);
}
