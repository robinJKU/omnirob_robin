/*
 * rosserial Service Server
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <OneWire.h>      //Zur Temperaturmessung
#define SENSOR_COUNT 8

ros::NodeHandle  nh;
OneWire ds(38);

struct Sensor{
  int sensor_id;
  int trig_pin;
  int echo_pin; 
};

Sensor sensors[SENSOR_COUNT];
double distance_m;
std_msgs::Float64MultiArray distances_msg;
std_msgs::Float64 temperature_msg;
std_msgs::Float64 sound_velocity_msg;
ros::Publisher distances_pub("arduino/Ultrasound_distances", &distances_msg);
ros::Publisher temperature_pub("arduino/Temperature", &temperature_msg);
ros::Publisher sound_velocity_pub("arduino/Sound_velocity",&sound_velocity_msg);

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

int count = 0;
int del_ms = 50;
int loop_times;
byte addr[8];
void setup()
{
  nh.initNode();
  distances_msg.data_length = 8;
  nh.advertise(distances_pub);
  nh.advertise(temperature_pub);
  nh.advertise(sound_velocity_pub);
  //Set PinMode of HC-SR04
  int trig_pins[] = {36,30,24,34,28,22,32,26};
  int echo_pins[] = {37,31,25,35,29,23,33,27};
  int sensor_ids[] ={7, 4, 1, 6, 3, 0, 5, 2}; 
  //int trig_pins[] = {10};
  //int echo_pins[] = {9};
  //int sensor_ids[] = {0};

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
  
  if ( !ds.search(addr)) {
    ds.reset_search();
    delay(250);
  }
  OneWire::crc8(addr, 7);
  
  delay(100);
  
  //for temp
  loop_times = 1000/(SENSOR_COUNT * del_ms)+2;
}

int echo_time_us = 0;
long looptime_us = 0;

byte i;
byte present = 0;
byte type_s = 0;
byte data[12];
float celsius;
float sound_velocity = 343.5;

void loop()
{  
  //Temp
  if(count == 0){
    //init Temp measuring 
    present = 0;
    ds.reset();
    ds.select(addr);
    ds.write(0x44,1);
  }
  count++;
  
  float values[] = {0,0,0,0,0,0,0,0};
  for(int i=0; i < SENSOR_COUNT; i++)
  {
    looptime_us = micros();
    echo_time_us = untraschall_echo_time_us(sensors[i]);
    values[sensors[i].sensor_id] = sound_velocity * echo_time_us / 2000000;
    while(looptime_us + del_ms*1000 > micros());
  }  
  
  
  if(count >= loop_times){
    //masure temperature
    present = ds.reset();
    ds.select(addr);    
    ds.write(0xBE);         // Read Scratchpad

    for ( i = 0; i < 9; i++) {           // we need 9 bytes
      data[i] = ds.read();
    }
    
    unsigned int raw = (data[1] << 8) | data[0];
    if (type_s) {
      raw = raw << 3; // 9 bit resolution default
      if (data[7] == 0x10) {
        // count remain gives full 12 bit resolution
        raw = (raw & 0xFFF0) + 12 - data[6];
      }
    } else {
      byte cfg = (data[4] & 0x60);
      if (cfg == 0x00) raw = raw << 3;  // 9 bit resolution, 93.75 ms
      else if (cfg == 0x20) raw = raw << 2; // 10 bit res, 187.5 ms
      else if (cfg == 0x40) raw = raw << 1; // 11 bit res, 375 ms
    }
    celsius = (float)raw / 16.0;
    if(celsius > 50.0 || celsius < -15.0){
      celsius = 25.0;
    }
    sound_velocity = 20.06285789*sqrt(273.15 + celsius);
    temperature_msg.data = celsius;
    temperature_pub.publish(&temperature_msg);
    sound_velocity_msg.data = sound_velocity;
    sound_velocity_pub.publish(&sound_velocity_msg);
    count = 0; 
  }

  distances_msg.data = values;
  distances_pub.publish(&distances_msg);
  nh.spinOnce();  
}
