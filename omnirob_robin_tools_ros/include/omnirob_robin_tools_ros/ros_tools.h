#ifndef __ROS_TOOLS_H
#define __ROS_TOOLS_H

#include <ros/ros.h>

bool wait_for_service( std::string topic, float for_nr_of_sec=10.0 ){
  ros::Rate rate(10.0/for_nr_of_sec);
  unsigned int waiting_cnt=0;
  while( waiting_cnt<10 && ros::ok() && !ros::service::exists(topic, false) ){
	  ROS_INFO("Waiting for service %s", topic.c_str());
	  rate.sleep();
	  waiting_cnt++;
  }
  if( waiting_cnt>for_nr_of_sec ){
	  ROS_ERROR("Service %s is not available", topic.c_str());
	  return false;
  }
  return true;
	
}// wait for service

bool wait_until_publisher_is_connected( ros::Publisher pub, float for_nr_of_sec=10.0 ){
	ros::Rate rate(10.0/for_nr_of_sec);
    unsigned int waiting_cnt=0;
	while( ros::ok() && pub.getNumSubscribers()<=0 && waiting_cnt<10 ){
		rate.sleep();
		waiting_cnt++;
	}
	
	return pub.getNumSubscribers()>0;
}

#endif
