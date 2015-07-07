// general
#include <math.h>
#include <omnirob_robin_moveit/lwa_planner_and_executer.h>

// Robot state publishing

#include "ros/ros.h"


int main(int argc, char **argv)
{
	// initialize ros node
	ros::init(argc, argv, "lwa_plan_and_execute");

	// add spinner
	ros::AsyncSpinner spinner(1);
	spinner.start();

	lwa_planner_and_executer lwa;
	ROS_INFO("----------------------------------------------");
	ROS_INFO("start continuous execution");
	ROS_INFO("movet to goal pose");

	moveit::planning_interface::MoveGroup::Plan plan1, plan2;
	geometry_msgs::Pose goal_pose;

	goal_pose.orientation.x = 0.7071;
	goal_pose.orientation.y = 0.0;
	goal_pose.orientation.z = 0.7071;
	goal_pose.orientation.w = 0.0;

	goal_pose.position.x=0.9;
	goal_pose.position.y=0.0;
	goal_pose.position.z=1.0;

	for( unsigned int ii=0; ii<5; ii++){
		if( lwa.plan_continuous_path_.plan_path_to_pose(plan1, goal_pose, "/base_link") ){
			ROS_INFO("path planning succeeded, execute path");
			lwa.execute_continuous_path_.execute_path_blocking( plan1);

			ROS_INFO("path execution finished");
			break;
		}
	}
	ros::Duration(3.0).sleep();

	ROS_INFO("move to goal configuration");
	std::vector<double> goal_configuration(7,0.0);

	ros::Time start = ros::Time::now();
	for( unsigned int ii=0; ii<5; ii++){
		if( lwa.plan_continuous_path_.plan_to_configuration( plan2, std::vector<double>(7,0.0)) ){
			ROS_INFO("path planning succeeded, execute path");
			lwa.execute_continuous_path_.execute_path_blocking( plan2);

			ROS_INFO("path execution finished");
			break;
		}
	}

	ROS_INFO("----------------------------------------------");
	ROS_INFO("start p2p execution");
	goal_configuration = std::vector<double>(7,0.2);

	start = ros::Time::now();
	std::string error_message = lwa.execute_point_to_point_path_.execute( goal_configuration, true);
	ROS_INFO("finished p2p execution, time = %f, error_message = %s", (ros::Time::now()-start).toSec(), error_message.c_str());

	std::vector<double> goal_configuration2(7,0.0);
	error_message = lwa.execute_point_to_point_path_.execute( goal_configuration2, true);
	ROS_INFO("finished p2p execution, time = %f, error_message = %s", (ros::Time::now()-start).toSec(), error_message.c_str());

	ROS_INFO("----------------------------------------------");
	ROS_INFO("finished test trajectory");

	ros::spin();
	return 0;
}// main

