#include <ros/ros.h>
#include "tf/transform_listener.h"

//services und messages
#include <omnirob_robin_msgs/localization.h>

int main( int argc, char** argv) {
	 
   // initialize node
  ros::init(argc, argv, "pick_and_place_demo");
  ros::NodeHandle node_handle;
  ros::NodeHandle private_ns_node_handle("~");
  
  // check if all required services exists
  
 
  // global localization
  // todo1: do a global localization
  // todo2: reinitialize amcl as well as hector_slam
  
  
}
