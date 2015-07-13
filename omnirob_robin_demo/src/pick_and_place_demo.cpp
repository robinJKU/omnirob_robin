#include <ros/ros.h>
#include <omnirob_robin_tools_ros/ros_tools.h>
#include "tf/transform_listener.h"

//services und messages
#include <omnirob_robin_msgs/localization.h>
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Bool.h"
#include "tf/tf.h"

bool is_localized = false;

ros::Publisher goal_publisher;


void localizedCallback(const std_msgs::Bool::ConstPtr& msg){
	is_localized = msg->data;
}

int main( int argc, char** argv) {
	 
   // initialize node
  ros::init(argc, argv, "pick_and_place_demo");
  ros::NodeHandle node_handle;  
  
  goal_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);
  
  ros::Subscriber is_localized_subscriber = node_handle.subscribe("is_localized", 10, localizedCallback);
  
  //move to table     
  
  //rewrite for ACTION
  
  while(!is_localized){
	  ros::Rate(1).sleep();
	  ros::spinOnce();
  }
  
  ROS_INFO("Moving to table");
  geometry_msgs::PoseStamped goal_msg;
  goal_msg.header.frame_id = "/map";
  goal_msg.header.stamp = ros::Time::now();
  goal_msg.pose.position.x = 1.81;
  goal_msg.pose.position.y = 1.52;
  goal_msg.pose.position.z = 0;
  
  tf::Quaternion quat;
  quat.setRPY(0,0,-1.57);
  goal_msg.pose.orientation.x = quat.getX();
  goal_msg.pose.orientation.y = quat.getY();
  goal_msg.pose.orientation.z = quat.getZ();
  goal_msg.pose.orientation.w = quat.getW();
  
  
  if( !omnirob_ros_tools::wait_until_publisher_is_connected( goal_publisher ) ){
	  ROS_ERROR("Can't initialize goal publisher");
	  return -1;
  }
  
  goal_publisher.publish(goal_msg);
  
  //detect object
  
  
  //grasp object
  
  
  
  
   
  ros::spin();
	
  
  
  
}

