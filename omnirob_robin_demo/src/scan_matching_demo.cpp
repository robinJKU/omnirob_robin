#include <ros/ros.h>
#include <ros_common_robin_tools/common_tools.h>
#include <sstream> 
#include <iostream>
using namespace std;

//services und messages

#include <actionlib_msgs/GoalStatusArray.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "tf/tf.h"
#include "std_srvs/Empty.h"
#include <ros_common_robin_msgs/localization.h>

double MAX_LIN_ACC = 0.1;
double MAX_ANG_ACC=0.1;
double MAX_LIN_VEL = 0.25;
double MAX_ANG_VEL = 0.4;
double STEP_TIME = 0.01;

bool goal_reached = false;

ros::Publisher goal_publisher;
ros::Publisher cmd_vel_publisher;
ros::ServiceServer positioning_server1, positioning_server2, positioning_server3, positioning_server4;

void move_base(double vx, double vy, double omega){
    geometry_msgs::Twist msg;
    if (sqrt(vx*vx + vy*vy) > MAX_LIN_VEL || fabs(omega) > MAX_ANG_VEL){
        ROS_INFO("speed limit reached");
    } else {
        msg.linear.x = vx;
        msg.linear.y = vy;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = omega;
        cmd_vel_publisher.publish(msg);    
    }    
}

void move_base_goal(double x, double y, double yaw){
	tf::Quaternion quat;
	quat.setRPY(0,0,yaw);
	quat.normalize();

	geometry_msgs::PoseStamped goal_msg;
	goal_msg.header.frame_id = "/map";
	goal_msg.header.stamp = ros::Time::now();
	goal_msg.pose.position.x = x;
	goal_msg.pose.position.y = y;
	goal_msg.pose.position.z = 0;

	goal_msg.pose.orientation.x = quat.getX();
	goal_msg.pose.orientation.y = quat.getY();
	goal_msg.pose.orientation.z = quat.getZ();
	goal_msg.pose.orientation.w = quat.getW();

	goal_publisher.publish(goal_msg);
	ros::Duration(1.0).sleep();

	while(!goal_reached){
		ros::Rate(1).sleep();
		ros::spinOnce();
	}
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


bool positioning1_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){

	ROS_INFO("positioning1_srv called");
    	ROS_INFO("moving to target position 1");

	move_base_goal(2.5, 2.05, -1.57);
	ros::spinOnce();

	return true;
}

bool positioning2_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){

	ROS_INFO("positioning2_srv called");
    	ROS_INFO("moving to target position 2");

	move_base_goal(2.8, 2.4, 1.57);
	ros::spinOnce();

	return true;
}

bool positioning3_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){

	ROS_INFO("positioning3_srv called");
    	ROS_INFO("moving to target position 3");

	move_base_goal(1.4, 2.15, 3.14);
	ros::spinOnce();

	return true;
}

bool positioning4_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){

	ROS_INFO("positioning4_srv called");
    	ROS_INFO("moving to target position 4");

	move_base_goal(1.9908, 1.4116, -0.7286);
	ros::spinOnce();

	return true;
}


int main( int argc, char** argv) {

	// initialize node

	ros::init(argc, argv, "scan_matching_demo");
	ros::NodeHandle node_handle;

	//publisher
	cmd_vel_publisher = node_handle.advertise<geometry_msgs::Twist>("/omnirob_robin/base/drives/control/cmd_vel", 10);

	//subscriber
    	ros::Subscriber status_subscriber = node_handle.subscribe("move_base/status", 10, statusCallback);

	//Service server
    	positioning_server1 = node_handle.advertiseService("/omnirob_robin/positioning_1_srv", positioning1_callback);  
        positioning_server2 = node_handle.advertiseService("/omnirob_robin/positioning_2_srv", positioning2_callback); 
        positioning_server3 = node_handle.advertiseService("/omnirob_robin/positioning_3_srv", positioning3_callback); 
	positioning_server4 = node_handle.advertiseService("/omnirob_robin/positioning_4_srv", positioning4_callback); 
    
    	goal_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);

    	if( !common_tools::wait_until_publisher_is_connected( goal_publisher ) ){
	  ROS_ERROR("Can't initialize goal publisher");
	  return -1;
	}
  
    ROS_INFO("DEMO NODE READY");
    
    while(ros::ok){
        ros::Rate(100).sleep();
        ros::spinOnce();
    }
	
}

