#include <ros/ros.h>

//messages
#include <std_msgs/Float64MultiArray.h>

// services
#include <std_srvs/Empty.h>
#include <omnirob_robin_msgs/move_pan_tilt.h>

// tools
#include <omnirob_robin_tools_ros/geometry_tools.h>

// urdf
#include <urdf/model.h>

//publisher
ros::Publisher pan_tilt_goal_pub;

//server
ros::ServiceServer move_pan_tilt_server;

//clients
ros::ServiceClient pan_tilt_start_motion_srv;
ros::ServiceClient pan_tilt_init_srv;
ros::ServiceClient pan_tilt_ref_srv;

double pan_min_value; // in rad
double pan_max_value; // in rad
double tilt_min_value; // in rad
double tilt_max_value; // in rad

std::vector<double> pan_tilt_state(2,0.0);

void pan_tilt_callback( std_msgs::Float64MultiArray state_data)
{
	if( state_data.data.size()==2)
	{
		pan_tilt_state=state_data.data;
	}else{
		ROS_WARN("Unexpected message size in pan tilt callback. Got %u values expected 2", (unsigned int) state_data.data.size());
	}
}

bool movePanTiltCallback(omnirob_robin_msgs::move_pan_tilt::Request& req, omnirob_robin_msgs::move_pan_tilt::Response& res){
  std_msgs::Float64MultiArray msg;
  std_srvs::Empty srv;
  
  if( pan_min_value<req.pan_goal && req.pan_goal<pan_max_value &&
      tilt_min_value<req.tilt_goal && req.tilt_goal<tilt_max_value )
  {
	  msg.data.push_back(req.pan_goal);
	  msg.data.push_back(req.tilt_goal);

	  pan_tilt_goal_pub.publish(msg);
	  pan_tilt_start_motion_srv.call(srv);

	  ros::Rate rate10Hz(10.0);
	  while( ros::ok() && omnirob_geometry_tools::max_angle_distance(msg.data, pan_tilt_state)>3e-2)
	  {
		  rate10Hz.sleep();
		  ros::spinOnce();
	  }

  }
  
  res.success = false;
  if( omnirob_geometry_tools::max_angle_distance(msg.data, pan_tilt_state)<=3e-2)
  		  res.success = true;

  return true;  
}

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "pan_tilt_interface");
  ros::NodeHandle node_handle;

  //Subscriber
  ros::Subscriber joint_state_subscriber = node_handle.subscribe("pan_tilt/state/joint_state_array", 1, pan_tilt_callback);
  
  //Service Clients  
  ros::service::waitForService("pan_tilt/control/start_motion");
  ros::service::waitForService("pan_tilt/control/initialize_modules");
  ros::service::waitForService("pan_tilt/control/reference_modules");
  pan_tilt_start_motion_srv = node_handle.serviceClient<std_srvs::Empty>("pan_tilt/control/start_motion");
  pan_tilt_init_srv = node_handle.serviceClient<std_srvs::Empty>("pan_tilt/control/initialize_modules");
  pan_tilt_ref_srv = node_handle.serviceClient<std_srvs::Empty>("pan_tilt/control/reference_modules");
  
  ROS_INFO("pan_tilt_interface: all Services available");
  
  std_srvs::Empty srv;
  pan_tilt_init_srv.call(srv);
  pan_tilt_ref_srv.call(srv);
  
  //Publisher
  pan_tilt_goal_pub = node_handle.advertise<std_msgs::Float64MultiArray> ("pan_tilt/control/commanded_joint_state", 1);
  
  //Service Servers
  move_pan_tilt_server = node_handle.advertiseService("pan_tilt/move_pan_tilt",movePanTiltCallback);

  // parse gripper limits from urdf
  std::string robot_description;
  ros::param::get("/robot_description", robot_description);

  if( robot_description.empty())
  {
    ROS_WARN("Can't find robot description file, use default limits pan:[-2.0,2.0]rad, tilt:[-1.57,1.57]rad");
    pan_min_value = -2.0;
    pan_max_value =  2.0;
    tilt_min_value = -1.57;
    tilt_max_value =  1.57;
  }
  else
  {
    urdf::Model robot_model;
    robot_model.initString(robot_description);
    boost::shared_ptr<const urdf::Joint> pan_joint = robot_model.getJoint("pan_tilt/pan_joint");
    pan_min_value = pan_joint->limits->lower;
    pan_max_value = pan_joint->limits->upper;

    boost::shared_ptr<const urdf::Joint> tilt_joint = robot_model.getJoint("pan_tilt/tilt_joint");
    tilt_min_value = tilt_joint->limits->lower;
    tilt_max_value = tilt_joint->limits->upper;
  }

  ros::spin();
  
  return 0;
}

