#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Empty.h>

ros::Publisher pan_pub;
ros::Publisher tilt_pub;

double pan_target = 0.0;
double tilt_target = 0.0;

void positionCallback( const std_msgs::Float64MultiArray& position) {  
  pan_target = position.data[0];
  tilt_target = position.data[1];
  ROS_INFO("new pan tilt target %f %f", pan_target, tilt_target);
}

bool startMotionCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
  std_msgs::Float64 pan_msg;
  std_msgs::Float64 tilt_msg;
  pan_msg.data = pan_target;
  tilt_msg.data = tilt_target;
  
  pan_pub.publish(pan_msg);
  tilt_pub.publish(tilt_msg);
  return true;
}

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "pan_tilt_interface");

  ros::NodeHandle n;   
  
  pan_pub = n.advertise<std_msgs::Float64>("pan_tilt/pan_position_controller/command", 50);
  tilt_pub = n.advertise<std_msgs::Float64>("pan_tilt/tilt_position_controller/command", 50);    
  
  ros::Subscriber sub = n.subscribe("pant_tilt/control/commanded_joint_state", 1, positionCallback);  
  ros::ServiceServer move_pan_tilt_service = n.advertiseService("pan_tilt/control/start_motion", startMotionCallback);

  ros::spin();
  
  return 0;
}

