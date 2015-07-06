/*
Sonar_srf08.h - library for controlling srf08
		sonar sensors
 By. Zach Foresta April 10, 2009
 */
#ifndef Sonar_srf08_h
#define Sonar_srf08_h

#include "Ardoino.h"

class Sonar_srf08
{
  public:
	void connect();
	void sendCommand(int commandRegister, int address, int command);
	void setUnit(int commandRegister, int address, char units);
	void setRegister(int address, int thisRegister);
	int readData(int address, int numBytes);
	void changeAddress(int commandRegister,int NEW_ADDRESS);
  private:
//	int _pin1;
//	int _pin2;
};

#endif

