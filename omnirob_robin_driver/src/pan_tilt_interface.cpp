#include <ros/ros.h>

//messages
#include <std_msgs/Float64MultiArray.h>

// services
#include <std_srvs/Empty.h>
#include <omnirob_robin_msgs/move_pan_tilt.h>


//publisher
ros::Publisher pan_tilt_goal_pub;

//server
ros::ServiceServer move_pan_tilt_server;

//clients
ros::ServiceClient pan_tilt_start_motion_srv;
ros::ServiceClient pan_tilt_init_srv;
ros::ServiceClient pan_tilt_ref_srv;

bool movePanTiltCallback(omnirob_robin_msgs::move_pan_tilt::Request& req, omnirob_robin_msgs::move_pan_tilt::Response& res){
  std_msgs::Float64MultiArray msg;
  std_srvs::Empty srv;
  
  msg.data.push_back(req.pan_goal);
  msg.data.push_back(req.tilt_goal);
  
  pan_tilt_goal_pub.publish(msg);
  
  pan_tilt_start_motion_srv.call(srv);
  
  ros::Duration(2.0).sleep();
  
  res.success = true;
  
  return true;  
}

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "pan_tilt_interface");

  ros::NodeHandle n;    
  
  //Service Clients  
  ros::service::waitForService("pan_tilt/control/start_motion");
  ros::service::waitForService("pan_tilt/control/initialize_modules");
  ros::service::waitForService("pan_tilt/control/reference_modules");
  pan_tilt_start_motion_srv = n.serviceClient<std_srvs::Empty>("pan_tilt/control/start_motion"); 
  pan_tilt_init_srv = n.serviceClient<std_srvs::Empty>("pan_tilt/control/initialize_modules"); 
  pan_tilt_ref_srv = n.serviceClient<std_srvs::Empty>("pan_tilt/control/reference_modules"); 
  
  ROS_INFO("pan_tilt_interface: all Services available");
  
  std_srvs::Empty srv;
  pan_tilt_init_srv.call(srv);
  pan_tilt_ref_srv.call(srv);
  
  //Publisher
  pan_tilt_goal_pub = n.advertise<std_msgs::Float64MultiArray> ("pan_tilt/control/commanded_joint_state", 1);  
  
  //Service Servers
  move_pan_tilt_server = n.advertiseService("pan_tilt/move_pan_tilt",movePanTiltCallback);

  ros::spin();
  
  return 0;
}

