#include <ros/ros.h>
#include <omnirob_robin_tools_ros/ros_tools.h>
#include "tf/transform_listener.h"

//services und messages
#include <omnirob_robin_msgs/localization.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/String.h"

int main( int argc, char** argv) {
	 
   // initialize node
  ros::init(argc, argv, "pick_and_place_demo");
  ros::NodeHandle node_handle;  
   
  // perform a global localization
  /*
  std::string global_localization_topic = "/global_localization";
  ROS_INFO("Perform global localization");
  ros::ServiceClient global_localization_client = node_handle.serviceClient<omnirob_robin_msgs::localization>( global_localization_topic);
  omnirob_robin_msgs::localization global_localization_msg;
  
  global_localization_client.call( global_localization_msg);
  if( !global_localization_msg.response.error_message.empty() ){
	  ROS_ERROR("Can't initialize robot pose -reason: %s", global_localization_msg.response.error_message.c_str());
	  return -1;
  }
  */
  
  //move to table
  
  
  //detect object
  
  
  //grasp object
  
  
  
  
   
  ros::spin();
	
  
  
  
}

