/*
 * rosserial Service Server
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#define SENSOR_COUNT 8

ros::NodeHandle  nh;

struct Sensor{
  int sensor_id;
  int trig_pin;
  int echo_pin; 
};

Sensor sensors[SENSOR_COUNT];
double distance_m;
std_msgs::String distance_info;
std_msgs::Float64MultiArray distances;
std_msgs::Float64 temperature;
//ros::Publisher pub("Arduino_Distances_str", &distance_info);
ros::Publisher distances_pub("arduino/Ultrasound_distances", &distances);
ros::Publisher temperature_pub("arduino/Temperature", &temperature);

long untraschall_echo_time_us(struct Sensor sensor)
{
  // Initialisierung
  long start = 0; 
  long duration = 0;
  int sensor_here = 1;
  
  // LOW am TRIG pin -> Sensor sendet nun 40kHz Siganl
  digitalWrite(sensor.trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(sensor.trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sensor.trig_pin, LOW);
  
  start = micros();
  while(digitalRead(sensor.echo_pin) == LOW){
    duration = micros() - start;
    if(duration >= 11644){
      sensor_here = 0;
      break; 
    }
  }
  start = micros();
  while(digitalRead(sensor.echo_pin) == HIGH){
    duration = micros() - start;
    if(duration >= 11644) break; 
  }

  if(sensor_here == 0) return -6;
  return duration;
}

void setup()
{
  nh.initNode();
  //nh.advertise(pub);
  distances.data_length = 8;
  nh.advertise(distances_pub);
  nh.advertise(temperature_pub);
  //Set PinMode of HC-SR04
  //int trig_pins[] = {22,24,26,28,30,32,34,36};
  //int echo_pins[] = {23,25,27,29,31,33,35,37};
  int trig_pins[] = {36,30,24,34,28,22,32,26};
  int echo_pins[] = {37,31,25,35,29,23,33,27};
  int sensor_ids[] ={7, 4, 1, 6, 3, 0, 5, 2}; 

  //int trig_pins[] = {10};
  //int echo_pins[] = {9};

  for(int i=0; i < SENSOR_COUNT; i++)
  {
    sensors[i].sensor_id = sensor_ids[i];
    sensors[i].trig_pin = trig_pins[i];
    sensors[i].echo_pin = echo_pins[i];
  }
  
  for(int i=0; i < SENSOR_COUNT; i++)
  {
     pinMode(sensors[i].echo_pin, INPUT); 
     pinMode(sensors[i].trig_pin, OUTPUT);
     digitalWrite(sensors[i].trig_pin, LOW);
  }
  
  delay(100);
}

int echo_time_us = 0;
long looptime_us = 0;

void loop()
{    
  float values[] = {0,0,0,0,0,0,0,0};
  for(int i=0; i < SENSOR_COUNT; i++)
  {
    looptime_us = micros();
    echo_time_us = untraschall_echo_time_us(sensors[i]);
    values[sensors[i].sensor_id] = 343.500 * echo_time_us / 2000000;
    while(looptime_us + 20000 > micros());
  }  
  
  distances.data = values;
  distances_pub.publish(&distances);
  nh.spinOnce();
  
  
  
  
}
