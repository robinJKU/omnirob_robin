/*

 Sonar_srf08.cpp - SRF08 sensor reader code

 Sensor connections:

 SDA - Analog pin 4

 SCL - Analog pin 5

 by Zach Foresta Apr 2009

*/

// include Wire library to read and write I2C commands:
#include "Wire.h"
#include "Sonar_srf08.h"

//#define readInches 0x50
//#define readCentimeters 0x51

// Initialize Wires

void Sonar_srf08::connect(){
  // start I2C bus
  Wire.begin();
}

// Communicates with Sonar to send commands

void Sonar_srf08::sendCommand(int commandRegister, int address, int command){
  // start I2C transmission:
  Wire.beginTransmission(address);
  // send command:
  Wire.send(commandRegister);
  Wire.send(command);
  // end I2C transmission:
  Wire.endTransmission();
}


// Sets Units for display / storage

void Sonar_srf08::setUnit(int commandRegister, int address, char units){
  switch(units) {
	  case "i": ;
  Sonar_srf08::sendCommand(commandRegister, address, 0x50);
  break;
	  case "c": ;
  Sonar_srf08::sendCommand(commandRegister, address, 0x51);
  break;
  default:
  Serial.print("Invalid Units Entered...");
  Serial.println();
  }
}

// Set to read off the register with stored result

void Sonar_srf08::setRegister(int address, int thisRegister){
  // start I2C transmission:
  Wire.beginTransmission(address);
  // send address to read from:
  Wire.send(thisRegister);
  // end I2C transmission:
  Wire.endTransmission();
}  

// Read data from register return result

int Sonar_srf08::readData(int address, int numBytes){
  int result = 0;        // the result is two bytes long
  // send I2C request for data:
  Wire.requestFrom(address, numBytes);
  // wait for two bytes to return:
  while (Wire.available() < 2 )   {
    // wait for result
  }
  // read the two bytes, and combine them into one int:
  delay(50);
  result = Wire.receive() * 256;
  result += Wire.receive();
  // return the result:
  return result;
}  
  
// Optional change Address - 
// NEW_ADDRESS can be set to any of E0, E2, E4, E6, E8, EA, EC, EE
//                                 F0, F2, F4, F6, F8, FA, FC, FE

void Sonar_srf08::changeAddress(int commandRegister, int NEW_ADDRESS){
  Sonar_srf08::sendCommand(commandRegister,commandRegister,0xA0);
  Sonar_srf08::sendCommand(commandRegister,commandRegister,0xAA);
  Sonar_srf08::sendCommand(commandRegister,commandRegister,0xA5);
  Sonar_srf08::sendCommand(commandRegister,commandRegister,NEW_ADDRESS);
}  
