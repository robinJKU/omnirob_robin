#include <ros/ros.h>
#include <sstream> 
#include <iostream>
#include <time.h> 
//services und messages

#include "std_srvs/Empty.h"

ros::ServiceClient detect_objects_client;
ros::ServiceClient detect_markers_client;

int main( int argc, char** argv) {

	ros::init(argc, argv, "detection_reels_demo");
	ros::NodeHandle node_handle;
   
    	//Service Clients
//---------------------------------------------------------------------------------------------
 	ros::service::waitForService("/omnirob_robin/detect_markers_srv");
	detect_markers_client = node_handle.serviceClient<std_srvs::Empty>("/omnirob_robin/detect_markers_srv"); 
	ros::service::waitForService("/omnirob_robin/detect_objects_srv");
	detect_objects_client = node_handle.serviceClient<std_srvs::Empty>("/omnirob_robin/detect_objects_srv");  
//---------------------------------------------------------------------------------------------
    
    	ROS_INFO("DEMO NODE READY");

	std_srvs::Empty srv1,srv2;

	detect_markers_client.call(srv1);

	detect_objects_client.call(srv2);

        ros::spinOnce();

}

