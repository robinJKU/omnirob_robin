#include <ros/ros.h>
#include <ros_common_robin_tools/common_tools.h>
#include "tf/transform_listener.h"

//services und messages
#include <std_srvs/Empty.h>
#include <ros_common_robin_msgs/localization.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

ros::ServiceServer global_localization_service;
ros::ServiceClient marker_localization_client;
ros::ServiceClient clear_costmaps_client;

ros::Publisher hector_initialize_publisher;
ros::Publisher hector_reset_publisher;
ros::Publisher is_localized_publisher;

bool is_localized;

int globalLocalization(){

  // perform a global localization
  ROS_INFO("Perform global localization");
  ros_common_robin_msgs::localization marker_localization_msg;
  
  marker_localization_client.call( marker_localization_msg);
  if( !marker_localization_msg.response.error_message.empty() ){
	  ROS_ERROR("Can't initialize robot pose -reason: %s", marker_localization_msg.response.error_message.c_str());
	  return -1;
  }
  
  // set initial values of hector mapping
  ROS_INFO("initialize hector");
  geometry_msgs::PoseWithCovarianceStamped hector_initialize_msg;
  hector_initialize_msg.header.frame_id = "/map";
  hector_initialize_msg.header.stamp = ros::Time::now();
  hector_initialize_msg.pose.pose = marker_localization_msg.response.base_link;
  
  
  if( !common_tools::wait_until_publisher_is_connected( hector_initialize_publisher) ){
	  ROS_ERROR("Can't initialize hector initialization publisher");
	  return -1;
  }
  
  std_msgs::String hector_reset_msg;
  hector_reset_msg.data = "reset";  
  if( !common_tools::wait_until_publisher_is_connected( hector_reset_publisher) ){
	  ROS_ERROR("Can't initialize hector reset publisher");
	  return -1;
  }
  
  hector_reset_publisher.publish( hector_reset_msg);
  hector_initialize_publisher.publish( hector_initialize_msg);
  ros::spinOnce();
  std_srvs::Empty srv;
  clear_costmaps_client.call(srv);
  ros::spinOnce();
  clear_costmaps_client.call(srv);
  ros::spinOnce();
  is_localized = true;  
}

bool globalLocalizationCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){ 
	globalLocalization();
	return true;
}	

int main( int argc, char** argv) {
	 
   // initialize node
  ros::init(argc, argv, "global_localization");
  ros::NodeHandle n;
  
  is_localized = false;
  
  global_localization_service = n.advertiseService("/global_localization", globalLocalizationCallback);
  
  // check if all required services and topics exists
  std::string hector_get_map_topic = "/hector_slam/get_map";
  if( !common_tools::wait_for_service( hector_get_map_topic, 10) ){
	  return -1;
  }
  
  std::string marker_localization_topic = "/marker_localization";
  if( !common_tools::wait_for_service( marker_localization_topic, 10) ){
	  return -1;
  }    
  marker_localization_client = n.serviceClient<ros_common_robin_msgs::localization>( marker_localization_topic);
  
  if( !common_tools::wait_for_service( "/move_base/clear_costmaps", 10) ) {
	  return -1;
  }
  
  clear_costmaps_client = n.serviceClient<std_srvs::Empty>( "move_base/clear_costmaps" );
  
  
  hector_initialize_publisher = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/hector_slam/set_initial_pose", 10);
  hector_reset_publisher = n.advertise<std_msgs::String>("/hector_slam/syscommand", 10);
  is_localized_publisher = n.advertise<std_msgs::Bool>("/is_localized", 10);
  
  globalLocalization();
  
  while(ros::ok()){
	  ros::Rate(1).sleep();
	  std_msgs::Bool msg;
	  msg.data = is_localized;
	  is_localized_publisher.publish(msg);
	  ros::spinOnce();
  } 
	  
}

