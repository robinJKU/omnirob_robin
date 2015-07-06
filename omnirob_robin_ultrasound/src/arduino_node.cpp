#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "std_msgs/ByteMultiArray.h"
#include "sensor_msgs/Range.h"

#define SENSOR_COUNT 8
sensor_msgs::Range range[SENSOR_COUNT];
std_msgs::ByteMultiArray enableDirection;
double min_distance;
double max_distance;

void arduinoCallback(const std_msgs::Float64MultiArray values)
{
  static uint32_t seq_ = 0;
  enableDirection.data.clear();
  for(int i=0; i<SENSOR_COUNT; i++)
  {
  	if(values.data[i] > min_distance) {
  		enableDirection.data.push_back(1);
  	} else {
		enableDirection.data.push_back(0);
  	}
		range[i].radiation_type = 0; //ULTRANSOUND
		range[i].field_of_view = 0.5;
		range[i].min_range = 0.030;
		range[i].max_range = max_distance;		
		range[i].range = values.data[i];		
		range[i].header.seq = seq_;
		range[i].header.stamp = ros::Time::now();
		char buffer[40];
		sprintf(buffer,"US%d_link",i);
		range[i].header.frame_id = buffer;
  }
  seq_++;
}

int main(int argc, char **argv)
{
  //Init Node
  ros::init(argc, argv, "arduino_node");
  ros::NodeHandle n;
  //Subscriber and Publisher
  ros::Subscriber values_str = n.subscribe("/arduino/Ultrasound_distances", 100, arduinoCallback);
  ros::Publisher range_pub[SENSOR_COUNT];
  ros::Publisher enable_dir;
  //Parameter
  //n.setParam("/Ultrasound/min_distance", 0.1);
  
  enable_dir = n.advertise<std_msgs::ByteMultiArray>("/Ultrasound/enable_direction",1000);
  
  for(int i=0; i<SENSOR_COUNT; i++){
    char buffer[20];
    sprintf(buffer,"/Ultrasound/Sensor%d",i);
  	range_pub[i] = n.advertise<sensor_msgs::Range>(buffer,1000);
  }
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    if(!n.getParam("/arduino_node/min_distance", min_distance)){
     min_distance = 0.2;
    }
    if(!n.getParam("/arduino_node/max_distance", max_distance)){
     max_distance = 1.5;
    }
    for(int i=0; i<SENSOR_COUNT; i++)
    {
    	range_pub[i].publish(range[i]);   
    }
    
    enable_dir.publish(enableDirection);   
    
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
