#include <ros/ros.h>



//messages
#include <std_msgs/Float64MultiArray.h>

// services
#include <std_srvs/Empty.h>

// tf
#include <tf/transform_listener.h>

ros::ServiceServer closeGripper_server;
ros::ServiceServer openGripper_server;

ros::Publisher gripper_goal_pub;

ros::ServiceClient gripper_start_motion_srv;
ros::ServiceClient gripper_init_srv;
ros::ServiceClient gripper_ref_srv;

bool gripper_is_closed = false;

double closed_pos = 20;
double opened_pos = 50;

void moveGripper(double space){  
  std_msgs::Float64MultiArray msg;
  std_srvs::Empty srv;
  
  msg.data.push_back(space);
  
  gripper_goal_pub.publish(msg);
  
  gripper_start_motion_srv.call(srv); 
}



bool closeGripperCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
  if(!gripper_is_closed){
    gripper_is_closed = true; 
    moveGripper(closed_pos);    
  }
  return true;
}

bool openGripperCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){  
  gripper_is_closed = false; 
  moveGripper(opened_pos); 
  return true;
}


int main( int argc, char** argv) {

  ros::init(argc, argv, "gripper_interface");
  
  ros::NodeHandle n;  
  
  
  //Publisher
  gripper_goal_pub = n.advertise<std_msgs::Float64MultiArray> ("gripper/control/commanded_joint_state", 1);  
  
  //Service Servers
  closeGripper_server = n.advertiseService("gripper/close_srv", closeGripperCallback);
  openGripper_server = n.advertiseService("gripper/open_srv", openGripperCallback);   
  
  //Service Clients
  
  
  ros::service::waitForService("gripper/control/start_motion");
  ros::service::waitForService("gripper/control/initialize_modules");
  ros::service::waitForService("gripper/control/reference_module");
  gripper_start_motion_srv = n.serviceClient<std_srvs::Empty>("gripper/control/start_motion"); 
  gripper_init_srv = n.serviceClient<std_srvs::Empty>("gripper/control/initialize_modules"); 
  gripper_ref_srv = n.serviceClient<std_srvs::Empty>("gripper/control/reference_module"); 
  
  ROS_INFO("gripper_interface: all Services available");
  
  std_srvs::Empty srv;
  gripper_init_srv.call(srv);
  gripper_ref_srv.call(srv);
  
  
  ros::spin();
 
}
