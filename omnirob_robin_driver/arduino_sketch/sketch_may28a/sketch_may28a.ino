/*
 * rosserial SRF08 Ultrasonic Ranger Example
 *
 * This example is calibrated for the SRF08 Ultrasonic Ranger.
 */
#include <Sonar_srf08.h> //SRF08 specific library
#include <Arduino.h>
#include <Wire.h>
#include <ros.h>
#include <std_msgs/Float32.h>


//Set up the ros node and publisher
std_msgs::Float32 sonar_msg;
ros::Publisher pub_sonar("sonar", &sonar_msg);
ros::NodeHandle nh;


Sonar_srf08 MySonar; //create MySonar object

#define CommandRegister 0x00
int New_Address = 248; //0xF8
#define ResultRegister  0x02

float sensorReading =0;

char unit = 'i'; // 'i' for inches , 'c' for centimeters


void setup()
{
  MySonar.connect();
  MySonar.changeAddress(CommandRegister, New_Address);
  New_Address += 4;
  nh.initNode();
  nh.advertise(pub_sonar);

}


long publisher_timer;

void loop()
{

  if (millis() > publisher_timer) {

   // step 1: request reading from sensor
   MySonar.setUnit(CommandRegister, New_Address, unit);

   //pause
  delay(70);

  // set register for reading
  MySonar.setRegister(New_Address, ResultRegister);

  // read data from result register
  sensorReading = MySonar.readData(New_Address, 2);

  sonar_msg.data = sensorReading;
  pub_sonar.publish(&sonar_msg);

  publisher_timer = millis() + 4000; //publish once a second

  }

  nh.spinOnce();
}
