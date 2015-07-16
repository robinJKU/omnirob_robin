#include <ros/ros.h>
#include <omnirob_robin_tools_ros/ros_tools.h>
#include "tf/transform_listener.h"

#include <omnirob_robin_moveit/lwa_planner_and_executer.h>
#include <actionlib_msgs/GoalStatusArray.h>

//services und messages

#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Bool.h"
#include "tf/tf.h"
#include "std_srvs/Empty.h"
#include <omnirob_robin_msgs/add_marker_srv.h>

bool is_localized = false;
bool goal_reached = false;

ros::Publisher goal_publisher;
ros::ServiceClient detect_objects_client;

ros::ServiceServer add_marker_server;


void localizedCallback(const std_msgs::Bool::ConstPtr& msg){
	is_localized = msg->data;
}

void statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg){
	if(msg->status_list.size() > 0){
		int last = msg->status_list.size()-1;
		if(msg->status_list[last].status == 3){
			goal_reached = true;
		} else {
			goal_reached = false;
		}
	}
}


bool addMarkerCallback(omnirob_robin_msgs::add_marker_srv::Request& request, omnirob_robin_msgs::add_marker_srv::Response& response){
	ROS_INFO("adding table to planning scene");

	std::string table_id = request.name;
	shape_msgs::SolidPrimitive table_primitive;
	table_primitive.type = table_primitive.BOX;
	table_primitive.dimensions.push_back(request.size.x);
	table_primitive.dimensions.push_back(request.size.y);
	table_primitive.dimensions.push_back(request.size.z);
	geometry_msgs::Pose table_pose;
	table_pose.position.x=request.pose.position.x;
	table_pose.position.y=request.pose.position.y;
	table_pose.position.z=request.pose.position.z;
	table_pose.orientation.x=request.pose.orientation.x;
	table_pose.orientation.y=request.pose.orientation.y;
	table_pose.orientation.z=request.pose.orientation.z;
	table_pose.orientation.w=request.pose.orientation.w;
	std::string table_pose_frame = "/base_link";

	lwa_continuous_path_planner planner;
	ROS_INFO("add collision object");
	planner.add_static_object( table_id, table_primitive, table_pose, table_pose_frame);

	return true;
}

int main( int argc, char** argv) {

	// initialize node
	ros::init(argc, argv, "pick_and_place_demo");
	ros::NodeHandle node_handle;

	ros::AsyncSpinner spinner(4); // required because both the server and the client run in the same node
	spinner.start();

	//publisher

	goal_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);

	//subscriber

	ros::Subscriber is_localized_subscriber = node_handle.subscribe("is_localized", 10, localizedCallback);
	ros::Subscriber status_subscriber = node_handle.subscribe("move_base/status", 10, statusCallback);

	//Service Clients
	ros::service::waitForService("/omnirob_robin/detect_objects_srv");
	detect_objects_client = node_handle.serviceClient<std_srvs::Empty>("/omnirob_robin/detect_objects_srv");

	//Service server
	add_marker_server = node_handle.advertiseService("/omnirob_robin/add_marker", addMarkerCallback);

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
	goal_msg.pose.position.y = 1.57;
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

	//wait for goal to be reached

	//detect object

	while(!goal_reached){
		ros::Rate(1).sleep();
		ros::spinOnce();
	}

	ROS_INFO("calling object detection");

	std_srvs::Empty srv;
	detect_objects_client.call(srv);

	//get table dimensions and orientation




	//add table to scene





	//grasp object





	ros::spin();




}

