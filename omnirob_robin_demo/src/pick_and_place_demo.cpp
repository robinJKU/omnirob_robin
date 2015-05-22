#include <ros/ros.h>
#include <omnirob_robin_tools_ros/ros_tools.h>
#include "tf/transform_listener.h"

//services und messages
#include <omnirob_robin_msgs/localization.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

int main( int argc, char** argv) {
	 
   // initialize node
  ros::init(argc, argv, "pick_and_place_demo");
  ros::NodeHandle node_handle;
  
  // check if all required services and topics exists
  std::string hector_get_map_topic = "/dynamic_map";
  if( !wait_for_service( hector_get_map_topic, 10) ){
	  return -1;
  }
  
  std::string global_localization_topic = "/global_localization";
  if( !wait_for_service( global_localization_topic, 10) ){
	  return -1;
  }
   
  // perform a global localization
  ROS_INFO("Perform global localization");
  ros::ServiceClient global_localization_client = node_handle.serviceClient<omnirob_robin_msgs::localization>( global_localization_topic);
  omnirob_robin_msgs::localization global_localization_msg;
  
  global_localization_client.call( global_localization_msg);
  if( !global_localization_msg.response.error_message.empty() ){
	  ROS_ERROR("Can't initialize robot pose -reason: %s", global_localization_msg.response.error_message.c_str());
	  return -1;
  }
  
  // set initial values of hector mapping
  ros::Publisher hector_initialize_publisher = node_handle.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 5);
  geometry_msgs::PoseWithCovarianceStamped hector_initialize_msg;
  
  ROS_INFO("initialize hector");
  hector_initialize_msg.header.frame_id = "/map";
  hector_initialize_msg.header.stamp = ros::Time::now();
  hector_initialize_msg.pose.pose = global_localization_msg.response.base_link;
  
  //while( ros::ok() ){
	hector_initialize_publisher.publish( hector_initialize_msg);
	hector_initialize_publisher.publish( hector_initialize_msg);
	ros::spinOnce();
	
//}
  
  
  // todo1: do a global localization
  // todo2: reinitialize amcl as well as hector_slam
  
  
}

