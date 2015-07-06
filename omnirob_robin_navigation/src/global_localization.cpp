#include <ros/ros.h>
#include <omnirob_robin_tools_ros/ros_tools.h>
#include "tf/transform_listener.h"

//services und messages
#include <std_srvs/Empty.h>
#include <omnirob_robin_msgs/localization.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/String.h"

ros::ServiceServer global_localization_service;
ros::ServiceClient global_localization_client;

ros::Publisher hector_initialize_publisher;
ros::Publisher hector_reset_publisher;

int globalLocalization(){

  // perform a global localization
  ROS_INFO("Perform global localization");
  omnirob_robin_msgs::localization global_localization_msg;
  
  global_localization_client.call( global_localization_msg);
  if( !global_localization_msg.response.error_message.empty() ){
	  ROS_ERROR("Can't initialize robot pose -reason: %s", global_localization_msg.response.error_message.c_str());
	  return -1;
  }
  
  // set initial values of hector mapping
  ROS_INFO("initialize hector");
  geometry_msgs::PoseWithCovarianceStamped hector_initialize_msg;
  hector_initialize_msg.header.frame_id = "/map";
  hector_initialize_msg.header.stamp = ros::Time::now();
  hector_initialize_msg.pose.pose = global_localization_msg.response.base_link;
  
  
  if( !wait_until_publisher_is_connected( hector_initialize_publisher) ){
	  ROS_ERROR("Can't initialize hector initialization publisher");
	  return -1;
  }
  
  std_msgs::String hector_reset_msg;
  hector_reset_msg.data = "reset";  
  if( !wait_until_publisher_is_connected( hector_reset_publisher) ){
	  ROS_ERROR("Can't initialize hector reset publisher");
	  return -1;
  }
  
  hector_reset_publisher.publish( hector_reset_msg);
  hector_initialize_publisher.publish( hector_initialize_msg);
  
}

bool globalLocalizationCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){ 
	globalLocalization();
	return true;
}	

int main( int argc, char** argv) {
	 
   // initialize node
  ros::init(argc, argv, "global_localization");
  ros::NodeHandle n;
  
  global_localization_service = n.advertiseService("/global_localization", globalLocalizationCallback);
  
  // check if all required services and topics exists
  std::string hector_get_map_topic = "/hector_slam/get_map";
  if( !wait_for_service( hector_get_map_topic, 10) ){
	  return -1;
  }
  
  std::string global_localization_topic = "/marker_localization";
  if( !wait_for_service( global_localization_topic, 10) ){
	  return -1;
  }  
  
  global_localization_client = n.serviceClient<omnirob_robin_msgs::localization>( global_localization_topic);
  
  hector_initialize_publisher = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/hector_slam/set_initial_pose", 10);
  hector_reset_publisher = n.advertise<std_msgs::String>("/hector_slam/syscommand", 10);
  
  globalLocalization();
   
  ros::spin();
	  
}
