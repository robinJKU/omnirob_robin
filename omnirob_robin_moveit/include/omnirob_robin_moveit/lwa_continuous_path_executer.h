#ifndef __LWA_CONTINOUS_PATH_EXECUTER_H
#define __LWA_CONTINOUS_PATH_EXECUTER_H

// ros
#include <ros/ros.h>

// moveit
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <omnirob_robin_moveit/moveit_tools.h>

// msg
#include <std_msgs/Float64MultiArray.h>

// user specific
#include <ros_common_robin_tools/common_geometry_tools.h>

/**
 * This class is a client for moveits move_group node and configures the driver. The class aim is to provide methods for executing continuous trajectories.
 * It configures the node properly and simplifies the communication with the node. This includes not only the path planning but also the scene manipulation.
 */
class lwa_continuous_path_executer{
	public:
		/**
		 * constructor
		 */
		lwa_continuous_path_executer():
			lwa_is_executing_(false),
			goal_value_(7)
		{
			lwa_move_group_ = new moveit::planning_interface::MoveGroup("lwa");
			lwa_move_group_->startStateMonitor();

			ros::NodeHandle node_handle;
			lwa_state_monitor_ = node_handle.subscribe("/omnirob_robin/lwa/state/joint_state_array", 10, &lwa_continuous_path_executer::lwa_state_monitor_callback, this);
		}

		/**
		 * destructor
		 */
		~lwa_continuous_path_executer()
		{
			if( lwa_move_group_!=NULL)
				delete lwa_move_group_;
		}


	public: // execution interface
		/**
		* This function start the execution of a given path. This function is non blocking.
		* @param plan: The motion plan which should be executed.
		* @return Error message
		* @see moveit::planning_interface::MoveGroup::asyncExecute
		*/
		std::string execute_path( const moveit::planning_interface::MoveGroup::Plan &plan){
			std::string error_message;

			// update executing state
			if( lwa_is_executing_ ){
				lwa_is_executing_ = !has_finished();
			}

			// start execution
			if( !lwa_is_executing_ )
			{
				moveit::planning_interface::MoveItErrorCode error_code = lwa_move_group_->asyncExecute( plan);

				// parse error code and set new goal value
				error_message = moveit_tools::parse_error_code( error_code);
				if( error_message.empty() ){
					goal_value_ = plan.trajectory_.joint_trajectory.points.back().positions;
					lwa_is_executing_ = true;
				}
			}else{
				error_message = "lwa already executes";
			}
			if( !error_message.empty() )
				ROS_ERROR("Can't start execution: %s", error_message.c_str());
				
			return error_message;
		}
		/**
		* This function start the execution of a given path. This function is blocking.
		* @param plan: The motion plan which should be executed.
		* @return Error message
		* @see moveit::planning_interface::MoveGroup::asyncExecute
		*/
		std::string execute_path_blocking( const moveit::planning_interface::MoveGroup::Plan &plan, ros::Duration timeout=ros::Duration(100.0)){
			moveit_tools::print_plan( plan);
			
			std::string error_message = execute_path( plan);
			if( !error_message.empty())
			{
				// try a last time
				ros::Rate rate1Hz(1.0);
				rate1Hz.sleep();
				error_message = execute_path( plan);
			}
			
			if( !error_message.empty())
				return error_message;
			
			ros::Rate rate1Hz(1);
			ros::Time start_time = ros::Time::now();
			while( !has_finished() && start_time+timeout>ros::Time::now()){
				rate1Hz.sleep();
				ros::spinOnce();
			}
			
			if( !has_finished() )
			{
				error_message = "trajectory not finished";
			}
			
			return error_message;
		}
		/**
		* This function preempts a path execution.
		* @see moveit::planning_interface::MoveGroup::stop
		*/
		void stop( void){
			if( lwa_is_executing_ ){
				lwa_move_group_->stop();
				lwa_is_executing_ = false;
			}
		}
		/**
		* Returns if the execution has finished.
		*/
		bool has_finished( void){
			if( !lwa_is_executing_)
				return true;

			// get current lwa state
			std::vector<double> lwa_joint_values(7);

			// determine euclidean distance from goal state
			if( common_geometry_tools::max_angle_distance( last_state_value_, goal_value_ )<0.05 ){
				return true;
			}
			return false;
		}// path execution finished

		void lwa_state_monitor_callback(std_msgs::Float64MultiArray lwa_state){
			last_state_value_ = lwa_state.data;
		}


	private: // member variables
		// moveit interface
		moveit::planning_interface::MoveGroup *lwa_move_group_;
		ros::Subscriber lwa_state_monitor_;

		// state monitoring
		bool lwa_is_executing_; // this variable monitors the state of a local (execution commands which are given by this object) executed trajectory
		std::vector<double> goal_value_;
		std::vector<double> last_state_value_;

};

#endif
