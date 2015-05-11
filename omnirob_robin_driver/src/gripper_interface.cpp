#include <ros/ros.h>

//messages
#include <std_msgs/Float64MultiArray.h>

// services
#include <std_srvs/Empty.h>
#include <omnirob_robin_msgs/move_gripper.h>

ros::ServiceServer closeGripper_server;
ros::ServiceServer openGripper_server;
ros::ServiceServer moveGripper_server;

ros::Publisher gripper_goal_pub;

ros::ServiceClient gripper_start_motion_srv;
ros::ServiceClient gripper_init_srv;
ros::ServiceClient gripper_ref_srv;

bool gripper_is_closed = false;

double closed_pos = 50;
double opened_pos = 5;

bool moveGripper(double stroke){    
  if(stroke <= closed_pos && stroke >= opened_pos){
    
    std_msgs::Float64MultiArray msg;
    std_srvs::Empty srv;
    msg.data.push_back(stroke);    
    gripper_goal_pub.publish(msg);    
    gripper_start_motion_srv.call(srv);     
    ros::Duration(2.0).sleep();
    return true;
    
  }  
  return false;
}

bool closeGripperCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){ 
  gripper_is_closed = true; 
  return moveGripper(closed_pos);  
}

bool openGripperCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){   
  gripper_is_closed = false; 
  return moveGripper(opened_pos);   
}

bool moveGripperCallback(omnirob_robin_msgs::move_gripper::Request& request, omnirob_robin_msgs::move_gripper::Response& response){ 
  response.success = true;
  return moveGripper(request.stroke);   
}


int main( int argc, char** argv) {

  ros::init(argc, argv, "gripper_interface");
  
  ros::NodeHandle n;  
    
  //Service Clients  
  ros::service::waitForService("/omnirob_robin/gripper/control/start_motion");
  ros::service::waitForService("/omnirob_robin/gripper/control/initialize_modules");
  ros::service::waitForService("/omnirob_robin/gripper/control/reference_module");
  gripper_start_motion_srv = n.serviceClient<std_srvs::Empty>("/omnirob_robin/gripper/control/start_motion"); 
  gripper_init_srv = n.serviceClient<std_srvs::Empty>("/omnirob_robin/gripper/control/initialize_modules"); 
  gripper_ref_srv = n.serviceClient<std_srvs::Empty>("/omnirob_robin/gripper/control/reference_module"); 
  
  ROS_INFO("gripper_interface: all Services available");
  
  std_srvs::Empty srv;
  gripper_init_srv.call(srv);
  gripper_ref_srv.call(srv);
  
  //Publisher
  gripper_goal_pub = n.advertise<std_msgs::Float64MultiArray> ("/omnirob_robin/gripper/control/commanded_joint_state", 1);  
  
  //Service Servers
  closeGripper_server = n.advertiseService("/omnirob_robin/gripper/close_srv", closeGripperCallback);
  openGripper_server = n.advertiseService("/omnirob_robin/gripper/open_srv", openGripperCallback);  
  moveGripper_server = n.advertiseService("/omnirob_robin/gripper/stroke_srv", moveGripperCallback);  
  
  
  ros::spin();
 
}
