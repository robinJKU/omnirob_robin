#include <ros/ros.h>

// services
#include <std_srvs/Empty.h>

//clients
ros::ServiceClient base_canserver_start_client;
ros::ServiceClient base_canserver_stop_client;
ros::ServiceClient base_controler_start_client;
ros::ServiceClient base_controler_stop_client;

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "pan_tilt_interface");

  ros::NodeHandle n;    
  
  //Service Clients  
  ros::service::waitForService("base/canserver/start");
  ros::service::waitForService("base/canserver/stop");
  ros::service::waitForService("base/drives/control/start");
  ros::service::waitForService("base/drives/control/stop");

  base_canserver_start_client = n.serviceClient<std_srvs::Empty>("base/canserver/start"); 
  base_canserver_stop_client = n.serviceClient<std_srvs::Empty>("base/canserver/stop"); 
  base_controler_start_client = n.serviceClient<std_srvs::Empty>("base/drives/control/start"); 
  base_controler_stop_client = n.serviceClient<std_srvs::Empty>("base/drives/control/stop"); 

 
  std_srvs::Empty srv;
  base_canserver_start_client.call(srv);
  base_controler_start_client.call(srv);
 

  while( ros::ok() ){
  	ros::spinOnce();
  	ros::Rate(1).sleep();
  }
    
  return 0;
}

