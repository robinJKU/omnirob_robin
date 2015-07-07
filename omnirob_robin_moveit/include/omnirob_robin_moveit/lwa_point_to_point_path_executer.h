#ifndef __LWA_POINT_TO_POINT_EXECUTER_H
#define __LWA_POINT_TO_POINT_EXECUTER_H

// ros
#include <ros/ros.h>

// actionlib
#include <actionlib/client/simple_action_client.h>

// actions
#include <control_msgs/FollowJointTrajectoryAction.h>

// msg
#include <std_msgs/Float64MultiArray.h>

// user specific
#include <omnirob_robin_tools_ros/common_cpp_tools.h>
#include <omnirob_robin_tools_ros/geometry_tools.h>

/**
 * This class act as an client and interface to the corresponding action controller.
 */
class lwa_point_to_point_executer{
	public:
		/**
		 * constructor
		 * @param: topic name of the action. default topic = "/omnirob_robin/lwa/controller/p2p_control"
		 */
		lwa_point_to_point_executer( const std::string &topic = "/omnirob_robin/lwa/controller/p2p_control"):
			action_client_( topic, false)
		{
			state_ = IDLE;
			lwa_joint_names_.push_back("lwa/joint_1");
			lwa_joint_names_.push_back("lwa/joint_2");
			lwa_joint_names_.push_back("lwa/joint_3");
			lwa_joint_names_.push_back("lwa/joint_4");
			lwa_joint_names_.push_back("lwa/joint_5");
			lwa_joint_names_.push_back("lwa/joint_6");
			lwa_joint_names_.push_back("lwa/joint_7");

			done_callback_handle_ = boost::bind( &lwa_point_to_point_executer::done_callback, this, _1, _2);

			ROS_INFO("Waiting 10sec for action server (topic :%s)", topic.c_str());
			int cnt=0;
			ros::Rate rate1Hz(1.0);
			while(cnt<10.0){
				if(action_client_.isServerConnected()){
					ROS_INFO("connected");
					break;
				}
				rate1Hz.sleep();
				ros::spinOnce();
				cnt++;
			}
			if( !action_client_.isServerConnected())
			{
				ROS_ERROR("Server with topic %s is not connected", topic.c_str());
			}
		}
		/**
		 * destructor
		 */
		~lwa_point_to_point_executer()
		{}

	private: // action callback
		void done_callback( const actionlib::SimpleClientGoalState& goal_state, const control_msgs::FollowJointTrajectoryResultConstPtr& result ){
			state_ = IDLE;
		}

	public: // action interface
		/**
		 * Moves the lwa to the goal configuration without considering collisions !!!
		 * @param goal_configuration: size has to bee 7
		 * @param blocking: true ... the call is blocking; otherwise not
		 * @param timeout: value of 0.0 means infinity blocking time
		 * @return error_message
		 */
		std::string execute( const std::vector<double> &goal_configuration, const bool &blocking = false, const ros::Duration &timeout = ros::Duration(0.0)){
			std::string error_message;
			if( goal_configuration.size()==7 ){
				control_msgs::FollowJointTrajectoryGoal goal;
				goal.trajectory.joint_names = lwa_joint_names_;
				trajectory_msgs::JointTrajectoryPoint point;
				point.positions = goal_configuration;
				goal.trajectory.points.push_back( point);
				if( blocking)
				{
					action_client_.sendGoalAndWait( goal, timeout);
				}else{
					action_client_.sendGoal( goal, done_callback_handle_);
				}
			}else{
				error_message = "Unexpected goal configuration size. Got " + common_cpp_tools::num_to_strin( goal_configuration.size()) + " expect 7";
				ROS_ERROR("%s", error_message.c_str());
			}
			return error_message;
		}
		void stop(){
			action_client_.cancelGoal();
		}
		/**
		 * @see execute
		 * @param map_name_configuration: A id:angle map which holds the goal state. It is not required, that all joints are included. Missing joints
		 *                                will be held constant. The id has to be a number of the set {0,1,2,3,4,5,6}
		 */
		std::string execute( const std::map<unsigned int,double> &map_id_configuration, const bool &blocking = false, const ros::Duration &timeout = ros::Duration(0.0)){
			std::string error_message;

			control_msgs::FollowJointTrajectoryGoal goal;
			goal.trajectory.points.resize(1);

			unsigned int id;
			double configuration;
			for( std::map<unsigned int,double>::iterator map_iterator; map_iterator!= map_id_configuration.end(); map_iterator++) {
				id = map_iterator->first;
				configuration = map_iterator->second;
				if( 0<=id && id<= 6){
					goal.trajectory.joint_names.push_back( lwa_joint_names_[id]);
					goal.trajectory.points[0].positions.push_back( configuration);
				}else{
					error_message = "Unexpected ID. Got " + common_cpp_tools::num_to_strin( map_iterator->first) + " expected an element out of {0,1,2,3,4,5,6}";
					break;
				}
			}
			if( error_message.empty()){
				if( blocking)
				{
					action_client_.sendGoalAndWait( goal, timeout);
				}else{
					action_client_.sendGoal( goal, done_callback_handle_);
				}
			}
			return error_message;
		}


	private: // member variables
		// action client
		actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> action_client_;
		actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>::SimpleDoneCallback done_callback_handle_;

		// state variables
		enum State {IDLE, EXECUTING, PLANNING, INITIALIZING} state_;
		std::vector<std::string> lwa_joint_names_;
};
#endif
