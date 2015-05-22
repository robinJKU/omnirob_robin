#ifndef __ROS_TOOLS_H
#define __ROS_TOOLS_H

bool wait_for_service( std::string topic, unsigned int for_nr_of_sec=10 ){
  ros::Rate rate_1s(1);
  unsigned int waiting_cnt=0;
  while( waiting_cnt<for_nr_of_sec && ros::ok() && !ros::service::exists(topic, false) ){
	  ROS_INFO("Waiting for service %s", topic.c_str());
	  rate_1s.sleep();
  }
  if( waiting_cnt>for_nr_of_sec ){
	  ROS_ERROR("Service %s is not available", topic.c_str());
	  return false;
  }
  return true;
	
}// wait for service

#endif
