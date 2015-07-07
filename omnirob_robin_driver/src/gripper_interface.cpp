#include <ros/ros.h>

//messages
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>

// services
#include <std_srvs/Empty.h>
#include <omnirob_robin_msgs/move_gripper.h>

ros::ServiceServer closeGripper_server;
ros::ServiceServer openGripper_server;
ros::ServiceServer moveGripper_server;

ros::Publisher gripper_goal_pub;
ros::Subscriber gripper_joint_state_sub;
ros::Subscriber gripper_is_ready_sub;
ros::Subscriber p2p_enabled_sub;

ros::ServiceClient gripper_start_motion_srv;
ros::ServiceClient gripper_init_srv;
ros::ServiceClient gripper_ref_srv;
ros::ServiceClient gripper_enable_p2p_motion_srv;

bool gripper_is_ready = false;
bool p2p_motion_enabled = false;

double closed_pos = 50.0; // gripper stroke when the gripper is closed. in mm
double opened_pos = 5.0; // gripper stroke when the gripper is opened. in mm

double stroke = 0.0;


bool gripper_is_at_position( float desired_stroke)
{
	return fabs(stroke-desired_stroke)<1.0;
}

bool gripper_is_closed()
{
	return gripper_is_at_position( closed_pos);
}

bool gripper_is_opened()
{
	return gripper_is_at_position( opened_pos);
}

void p2p_motion_enabled_callback( std_msgs::Bool is_enabled)
{
	p2p_motion_enabled = is_enabled.data;
}

void gripper_is_ready_callback( std_msgs::Float64MultiArray is_ready)
{
	if( is_ready.data.size()!=1)
	{
		ROS_ERROR("Unexpected message size of gripper is ready topic. Got %u expected 1. Discard message.", (unsigned int) is_ready.data.size());
		return;
	}
	gripper_is_ready = is_ready.data[0]>0.5;
}

void joint_state_callback( std_msgs::Float64MultiArray gripper_stroke){
	if( gripper_stroke.data.size()!=1)
	{
		ROS_ERROR("Invalid gripper data size, expected 1 got %u. Discard message.", gripper_stroke.data.size());
		return;
	}

	stroke = gripper_stroke.data[0];
}

bool moveGripper(double stroke){
  if( !gripper_is_ready)
  {
	  ROS_ERROR("Can't move gripper, gripper is not ready");
	  return false;
  }

  std_srvs::Empty empty_srv;
  if( !p2p_motion_enabled)
  {
	  gripper_enable_p2p_motion_srv.call( empty_srv);
  }
  if(  -1e-3<stroke-closed_pos && stroke-opened_pos<1e-3 ){
    std_msgs::Float64MultiArray msg;
    msg.data.push_back(stroke);    
    gripper_goal_pub.publish(msg);
    ros::spinOnce();
    gripper_start_motion_srv.call( empty_srv);
    return true;
  }
  else
  {
	  ROS_ERROR("Desired value out of range. Got %f, expected [%f, %f]", stroke, closed_pos, opened_pos);
	  return false;
  }
}

bool closeGripperCallback(omnirob_robin_msgs::move_gripper::Request& request, omnirob_robin_msgs::move_gripper::Response& response){
  moveGripper(closed_pos);
  ros::Rate rate_10Hz(10.0);

  unsigned int cnt=0;
  while( !gripper_is_closed() && cnt<10 )
  {
	  rate_10Hz.sleep();
	  ros::spinOnce();
	  cnt++;
  }
  if( !gripper_is_closed())
  {
	  response.error_message = "Can't close gripper in time";
	  ROS_ERROR("%s", response.error_message.c_str());
  }
  return true;
}

bool openGripperCallback(omnirob_robin_msgs::move_gripper::Request& request, omnirob_robin_msgs::move_gripper::Response& response){
	moveGripper(opened_pos);
	ros::Rate rate_10Hz(10.0);

	unsigned int cnt=0;
	while( !gripper_is_opened() && cnt<10 )
	{
	  rate_10Hz.sleep();
	  ros::spinOnce();
	  cnt++;
	}
	if( !gripper_is_opened())
	{
	  response.error_message = "Can't open gripper in time";
	  ROS_ERROR("%s", response.error_message.c_str());
	}
	return true;
}

bool moveGripperCallback(omnirob_robin_msgs::move_gripper::Request& request, omnirob_robin_msgs::move_gripper::Response& response){ 
	moveGripper( request.stroke);
	ros::Rate rate_10Hz(10.0);

	unsigned int cnt=0;
	while( !gripper_is_at_position( request.stroke) && cnt<10 )
	{
	  rate_10Hz.sleep();
	  ros::spinOnce();
	  cnt++;
	}
	if( !gripper_is_at_position( request.stroke) )
	{
      response.error_message = "Can't move gripper in time";
	  ROS_ERROR("%s", response.error_message.c_str());
	}
	return true;
}


int main( int argc, char** argv) {
  // initialize ros node
  ros::init(argc, argv, "gripper_interface");
  ros::NodeHandle n;

  ros::NodeHandle private_node_handle("~");

  // read parameters from server
  private_node_handle.param<double>("closed_position", opened_pos, 50.0);
  private_node_handle.param<double>("opened_position", closed_pos, 5.0);
    
  //Service Clients
  ROS_INFO("wait for gripper services");
  ros::service::waitForService("gripper/control/start_motion");
  ros::service::waitForService("gripper/control/initialize_modules");
  ros::service::waitForService("gripper/control/reference_modules");
  gripper_start_motion_srv = n.serviceClient<std_srvs::Empty>("gripper/control/start_motion"); 
  gripper_init_srv = n.serviceClient<std_srvs::Empty>("gripper/control/initialize_modules"); 
  gripper_ref_srv = n.serviceClient<std_srvs::Empty>("gripper/control/reference_modules");
  gripper_enable_p2p_motion_srv = n.serviceClient<std_srvs::Empty>("gripper/control/enable_point_to_point_motion");
  ROS_INFO("gripper_interface: all Services available");

  // Topic Subscriber
  gripper_joint_state_sub = n.subscribe("gripper/state/joint_state_array", 10, joint_state_callback);
  gripper_is_ready_sub = n.subscribe("gripper/state/info/module_is_ready", 10, gripper_is_ready_callback);
  p2p_enabled_sub = n.subscribe("gripper/state/info/point_to_point_motion_enabled", 10, p2p_motion_enabled_callback);

  ROS_INFO("Initialize gripper");
  std_srvs::Empty srv;
  gripper_init_srv.call(srv);
  gripper_ref_srv.call(srv);
  
  ros::Rate(2.0).sleep();
  ros::spinOnce();

  if( !gripper_is_ready )
  {
	  ROS_ERROR("Gripper is not ready, quit execution");
	  return -1;
  }
  ROS_INFO("Gripper is ready");

  //Publisher
  gripper_goal_pub = n.advertise<std_msgs::Float64MultiArray> ("gripper/control/commanded_joint_state", 1);  
  
  //Service Servers
  closeGripper_server = n.advertiseService("gripper/close_srv", closeGripperCallback);
  openGripper_server = n.advertiseService("gripper/open_srv", openGripperCallback);  
  moveGripper_server = n.advertiseService("gripper/stroke_srv", moveGripperCallback);
  
  ros::spin();

}
