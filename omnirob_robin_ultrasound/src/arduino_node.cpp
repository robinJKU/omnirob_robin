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
int cnt_till_last_callback;

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
  cnt_till_last_callback = 0;
  seq_++;
}

int main(int argc, char **argv)
{
  //Init Node
  ros::init(argc, argv, "arduino_node");
  ros::NodeHandle n;
  //Subscriber and Publisher
  ros::Subscriber values_sub;
  ros::Publisher range_pub[SENSOR_COUNT];
  ros::Publisher enable_dir;

  ros::Rate loop_rate(10);

  cnt_till_last_callback = 61;
  while (ros::ok())
  {
	if(cnt_till_last_callback > 60){
		n.shutdown();
		values_sub.shutdown();
		enable_dir.shutdown();
		for(int i = 0; i < SENSOR_COUNT; i++){
			range_pub[i].shutdown();
		}

		enable_dir = n.advertise<std_msgs::ByteMultiArray>("/Ultrasound/enable_direction",1000);
		//ROS_ERROR("No new ultrasonic data is received trying to reconnect");
		for(int i=0; i<SENSOR_COUNT; i++){
			char buffer[20];
			sprintf(buffer,"/Ultrasound/Sensor%d",i);
			range_pub[i] = n.advertise<sensor_msgs::Range>(buffer,1000);
		}

		values_sub.shutdown();
		ros::Duration(1.0).sleep();
		ros::spinOnce();
		values_sub = n.subscribe("/arduino/Ultrasound_distances", 100, arduinoCallback);
		ros::spinOnce();
		cnt_till_last_callback = 30;

	} else if(cnt_till_last_callback < 30) {

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

	}
	enable_dir.publish(enableDirection);
	ros::spinOnce();
	loop_rate.sleep();
	cnt_till_last_callback++;
  }

  return 0;
}
