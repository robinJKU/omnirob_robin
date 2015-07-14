#ifndef __MOVEIT_TOOLS_H
#define __MOVEIT_TOOLS_H

#include <ros/ros.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/MoveItErrorCodes.h>

namespace moveit_tools
{
	/**
	* Reverse the trajectory specified by plan
	*/
moveit::planning_interface::MoveGroup::Plan reverse_plan( const moveit::planning_interface::MoveGroup::Plan &plan ){
		moveit::planning_interface::MoveGroup::Plan reversed_plan;
		reversed_plan.planning_time_ = plan.planning_time_;
		reversed_plan.trajectory_ = plan.trajectory_;

		std::reverse( reversed_plan.trajectory_.joint_trajectory.points.begin(), reversed_plan.trajectory_.joint_trajectory.points.end());
		reversed_plan.start_state_.joint_state.position = reversed_plan.trajectory_.joint_trajectory.points.front().positions;

		return reversed_plan;
	}
	void print_plan( const moveit::planning_interface::MoveGroup::Plan &plan )
	{
		std::stringstream stream;
		unsigned int nr_of_points = plan.trajectory_.joint_trajectory.points.size();
		stream.str("Path visualization [point index / nr of points]: nr of joints - (joint values)\n");
		for( unsigned int point_ii=0; point_ii<nr_of_points; point_ii++)
		{
			stream << "\n [" << point_ii << "/" << nr_of_points << "]: " << plan.trajectory_.joint_trajectory.points[point_ii].positions.size() << " - (";
			for( unsigned int joint_ii=0; joint_ii<plan.trajectory_.joint_trajectory.points[point_ii].positions.size(); joint_ii++)
			{
				if( joint_ii>0 )
					stream << ", ";

				stream << plan.trajectory_.joint_trajectory.points[point_ii].positions[joint_ii];
			}
			stream << ")";
		}
		ROS_INFO("%s", stream.str().c_str());
	}

	/**
	* Returns true if the joint trajectory field of the motion plan has no joint names specified
	*/
	bool plan_is_empty( const moveit::planning_interface::MoveGroup::Plan &plan ){
		return plan.trajectory_.joint_trajectory.joint_names.empty();
	} 
	
	/**
	 * Parses the moveit error code and convert it to a readable message
	 * 
	 */
	std::string parse_error_code( moveit::planning_interface::MoveItErrorCode error_code ){
		switch( error_code.val ){
			// over all behaviour
			case moveit_msgs::MoveItErrorCodes::SUCCESS:
				return std::string();
				
			case moveit_msgs::MoveItErrorCodes::FAILURE:
				return std::string("MoveIt failure");
				
			case moveit_msgs::MoveItErrorCodes::PLANNING_FAILED:
				return std::string("planning failed");
				
			case moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN:
				return std::string("invalid motion plan");
				
			case moveit_msgs::MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
				return std::string("motion plan invalidated by environment change");
			
			case moveit_msgs::MoveItErrorCodes::CONTROL_FAILED:
				return std::string("control failed");
			
			case moveit_msgs::MoveItErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA:
				return std::string("unable to aquire sensor data");
			
			case moveit_msgs::MoveItErrorCodes::TIMED_OUT:
				return std::string("time out");
				
			case moveit_msgs::MoveItErrorCodes::PREEMPTED:
				return std::string("preempted");
				
				
			// planning and kinematics request errors
			case moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION:
				return std::string("start state in collision");
				
			case moveit_msgs::MoveItErrorCodes::START_STATE_VIOLATES_PATH_CONSTRAINTS:
				return std::string("start state violates path constraints");
				
			case moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION:
				return std::string("goal in collision");
			
				
			case moveit_msgs::MoveItErrorCodes::GOAL_VIOLATES_PATH_CONSTRAINTS:
				return std::string("goal violates path constraints");
				
			case moveit_msgs::MoveItErrorCodes::GOAL_CONSTRAINTS_VIOLATED:
				return std::string("goal constraints violated");
				
			case moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME:
				return std::string("invalid group name");
				
			case moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS:
				return std::string("invalid goal constraints");
				
			case moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE:
				return std::string("invalid robot state");
				
			case moveit_msgs::MoveItErrorCodes::INVALID_LINK_NAME:
				return std::string("invalid link name");
				
			case moveit_msgs::MoveItErrorCodes::INVALID_OBJECT_NAME:
				return std::string("invalid object name");
				
				
			// system error
			case moveit_msgs::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE:
				return std::string("frame transform failure");
				
			case moveit_msgs::MoveItErrorCodes::COLLISION_CHECKING_UNAVAILABLE:
				return std::string("collision checking unavailable");
				
			case moveit_msgs::MoveItErrorCodes::ROBOT_STATE_STALE:
				return std::string("robot state stale");
				
			case moveit_msgs::MoveItErrorCodes::SENSOR_INFO_STALE:
				return std::string("sensor info stale");
				
				
			// kinematics error
			case moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION:
			return std::string("no ik solution");
				
		}// switch
	}// parse error code
	
}
#endif
