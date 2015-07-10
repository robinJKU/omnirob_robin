#include <ros/ros.h>

// controller
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>
#include <trajectory_msgs/JointTrajectory.h>

// message types
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>

// user specific
#include <omnirob_robin_tools_ros/common_cpp_tools.h>
#include <omnirob_robin_tools_ros/geometry_tools.h>

/**
 * lwa controller acts as interface to the lwa driver. The class advertise two actions for moving the lwa along a (1) continuous path and a (2) point to point (p2p) path.
 */
class lwa_controller{
	public:
	/**
	 * constructor
	 */
	lwa_controller(std::string continuous_control_action_topic, std::string p2p_control_action_topic):
		lwa_state_(),
		continuous_control_action_server_(nodehandle_, continuous_control_action_topic, false),
		p2p_control_action_server_( nodehandle_, p2p_control_action_topic, false)
	{
		// initialize data
		lwa_joint_names_.push_back("lwa/joint_1");
		lwa_joint_names_.push_back("lwa/joint_2");
		lwa_joint_names_.push_back("lwa/joint_3");
		lwa_joint_names_.push_back("lwa/joint_4");
		lwa_joint_names_.push_back("lwa/joint_5");
		lwa_joint_names_.push_back("lwa/joint_6");
		lwa_joint_names_.push_back("lwa/joint_7");

		lwa_trajectory_time_left_ = -1.0;

		// initialize subscriber
		lwa_joint_trajectory_subscriber_ = nodehandle_.subscribe("/omnirob_robin/lwa/state/joint_state_array", 10, &lwa_controller::lwa_joint_state_callback, this);
		lwa_trajectory_time_left_subscriber_ = nodehandle_.subscribe("/omnirob_robin/lwa/trajectory/time_left", 10, &lwa_controller::lwa_trajectory_time_left_callback, this);
		lwa_module_is_ready_subscriber_ = nodehandle_.subscribe("/omnirob_robin/lwa/state/info/module_is_ready", 10, &lwa_controller::lwa_module_is_ready_callback, this);
		lwa_continuos_mode_enabled_subscriber_ = nodehandle_.subscribe("/omnirob_robin/lwa/state/info/continuos_position_tracking_mode_enabled", 10, &lwa_controller::lwa_continuos_mode_enabled_callback, this);

		// initialize lwa
		ROS_INFO("Wait for initialize service");
		if( ros::service::waitForService( "/omnirob_robin/lwa/control/initialize_modules", ros::Duration(10.0)) ){
			lwa_initialize_modules_ = nodehandle_.serviceClient<std_srvs::Empty>( "/omnirob_robin/lwa/control/initialize_modules");
			std_srvs::Empty empty_srvs;
			lwa_initialize_modules_.call( empty_srvs);
		}else{
			ROS_ERROR("Could not initialize lwa");
		}

		// initialize publisher
		lwa_commanded_joint_state_publisher_ = nodehandle_.advertise<std_msgs::Float64MultiArray>("/omnirob_robin/lwa/control/commanded_joint_state", 10);

		// initialize services
		lwa_set_point_client_ = nodehandle_.serviceClient<std_srvs::Empty>("/omnirob_robin/lwa/trajectory/set_point");
		lwa_reset_mode_ = nodehandle_.serviceClient<std_srvs::Empty>("/omnirob_robin/lwa/trajectory/reset_mode");
		lwa_start_trajectory_ = nodehandle_.serviceClient<std_srvs::Empty>("/omnirob_robin/lwa/trajectory/start");
		lwa_stop_trajectory_ = nodehandle_.serviceClient<std_srvs::Empty>("/omnirob_robin/lwa/trajectory/stop"); // todo: Kontrolle ob existent
		lwa_start_motion_ = nodehandle_.serviceClient<std_srvs::Empty>("/omnirob_robin/lwa/control/start_motion");
		lwa_stop_motion_ = nodehandle_.serviceClient<std_srvs::Empty>("/omnirob_robin/lwa/control/stop_motion");
		lwa_enable_continuos_position_tracking_ = nodehandle_.serviceClient<std_srvs::Empty>("/omnirob_robin/lwa/control/enable_continuos_position_tracking");
		lwa_enable_point_to_point_motion_ = nodehandle_.serviceClient<std_srvs::Empty>("/omnirob_robin/lwa/control/enable_point_to_point_motion");

		// register callbacks
		continuous_control_action_server_.registerGoalCallback(boost::bind(&lwa_controller::lwa_continous_control_goal_callback, this));
		continuous_control_action_server_.registerPreemptCallback(boost::bind(&lwa_controller::lwa_continous_control_preempt_callback, this));

		p2p_control_action_server_.registerGoalCallback(boost::bind(&lwa_controller::lwa_p2p_control_goal_callback, this));
		p2p_control_action_server_.registerPreemptCallback(boost::bind(&lwa_controller::lwa_p2p_control_preempt_callback, this));

		// start action server
		continuous_control_action_server_.start();
		p2p_control_action_server_.start();
	}
	/**
	 * destructor
	 */
	~lwa_controller( void)
	{}

	private: // trajectory transfer
		/**
		 * streams the whole trajectory to the driver
		 */
		void stream_trajectory( const trajectory_msgs::JointTrajectory &trajectory, const std::vector<int> &map_joint_index_to_trajectory_joint_index)
		{
			std_srvs::Empty empty_srv;
			ros::Rate rate_100Hz(100);

			// trajectory interpolator - set RESET MODE ON
			lwa_reset_mode_.call( empty_srv);

			// stream trajectory
			for( unsigned int point_ii=0; point_ii<trajectory.points.size(); point_ii++){
				set_target_point( trajectory.points[point_ii], map_joint_index_to_trajectory_joint_index );

				lwa_set_point_client_.call( empty_srv);
				rate_100Hz.sleep();
			}

			// trajectory interpolator - set RESET MODE OFF (start trajectory interpolation)
			lwa_reset_mode_.call( empty_srv);

			// check result
			// todo
		}
		/**
		 * Transmits a single target point to the driver.
		 */
		void set_target_point( const trajectory_msgs::JointTrajectoryPoint &point, const std::vector<int> &map_joint_index_to_trajectory_joint_index )
		{
			std_msgs::Float64MultiArray lwa_point;
			lwa_point.data.resize(7);

			ros::Rate rate_100Hz(100);

			for( unsigned int joint_ii=0; joint_ii<7; joint_ii++ ){
				if( map_joint_index_to_trajectory_joint_index[joint_ii]>=0 ){
					lwa_point.data[joint_ii] = point.positions[map_joint_index_to_trajectory_joint_index[joint_ii]];
				}else{
					ROS_ERROR("value not found");
					// hold joint value constant
					lwa_point.data[joint_ii] = lwa_state_.current_lwa_joint_state[joint_ii];
				}
			}
			// transmit point
			lwa_commanded_joint_state_publisher_.publish( lwa_point);
			rate_100Hz.sleep();
			ros::spinOnce();

		}

	public:// continous control
		void lwa_continous_control_goal_callback( void )
		{
			// initialize action
			control_msgs::FollowJointTrajectoryResult result;

			// get newest lwa state
			ros::spinOnce();

			// check if modules are ready
			if( !lwa_state_.modules_are_ready ){
				ROS_ERROR("Can't execute trajectory - Modules are not ready!!");
				result.error_code = result.INVALID_JOINTS;
				continuous_control_action_server_.setAborted( result);
				return;
			}

			// enable continuos position tracking mode
			std_srvs::Empty empty_srv;
			if( !lwa_state_.lwa_continous_mode_enabled ){
				lwa_enable_continuos_position_tracking_.call( empty_srv);
			}else{
				lwa_stop_motion_.call( empty_srv);
			}
			
			// parse and check inputs
			trajectory_msgs::JointTrajectory trajectory = continuous_control_action_server_.acceptNewGoal()->trajectory;
			std::vector<int> map_joint_index_to_trajectory_joint_index(7, -1);

			int jonit_index;
			for( unsigned int trajectory_joint_ii=0; trajectory_joint_ii<trajectory.joint_names.size(); trajectory_joint_ii++ ){
				jonit_index= common_cpp_tools::find_string( lwa_joint_names_, trajectory.joint_names[trajectory_joint_ii]);
				if( jonit_index<0 ){
					// found invalid joint
					ROS_ERROR("No trajectory is executed - Found invalid joint!");
					result.error_code = result.INVALID_JOINTS;
					continuous_control_action_server_.setAborted( result);
					return;
				}
				map_joint_index_to_trajectory_joint_index[(unsigned int) jonit_index] = trajectory_joint_ii;
			}

			// stream trajectory
			stream_trajectory( trajectory, map_joint_index_to_trajectory_joint_index);

			//  execute motion
			lwa_start_motion_.call( empty_srv);
			lwa_start_trajectory_.call( empty_srv);

			ros::Rate rate_10Hz(10);

			rate_10Hz.sleep();
			ros::spinOnce();

			while( ros::ok() && continuous_control_action_server_.isActive() && (lwa_trajectory_time_left_>0.0) && lwa_state_.modules_are_ready ){
				rate_10Hz.sleep();
				ros::spinOnce();
				ROS_INFO("controller time left = %f", lwa_trajectory_time_left_);
			}
			
			if( !ros::ok() ){
				return;
			}

			if( !continuous_control_action_server_.isActive() ){
				ROS_ERROR("Action canceled! Stop motion!");
				lwa_stop_trajectory_.call( empty_srv);
				lwa_stop_motion_.call( empty_srv);
				continuous_control_action_server_.setAborted( result);
				return;
			}

			if( !lwa_state_.modules_are_ready ){
				ROS_ERROR("Stop motion - modules are not ready!");
				lwa_stop_trajectory_.call( empty_srv);
				lwa_stop_motion_.call( empty_srv);
				continuous_control_action_server_.setAborted( result);
				return;
			}

			// finish action
			ROS_INFO("Finished execution");
			lwa_stop_motion_.call( empty_srv);

			result.error_code = result.SUCCESSFUL;
			continuous_control_action_server_.setSucceeded( result);
		}
		/**
		 * Is called if the action is canceled
		 */
		void lwa_continous_control_preempt_callback( void )
		{
			ROS_ERROR("continous control action is preempted!");
			continuous_control_action_server_.setPreempted();
		}// preempt callback


	public: // point to point action callback
		/**
		 * goal callback for point to point motions
		 */
		void lwa_p2p_control_goal_callback( void){
			// initialize action
			control_msgs::FollowJointTrajectoryResult result;

			// get newest lwa state
			ros::spinOnce();

			// check if modules are ready
			if( !lwa_state_.modules_are_ready ){
				ROS_ERROR("Can't execute trajectory - Modules are not ready!!");
				result.error_code = result.INVALID_JOINTS;
				p2p_control_action_server_.setAborted( result);
				return;
			}

			// enable p2p tracking mode
			std_srvs::Empty empty_srv;
			if( lwa_state_.lwa_continous_mode_enabled ){
				lwa_enable_point_to_point_motion_.call( empty_srv);
			}

			// parse and check inputs
			trajectory_msgs::JointTrajectory trajectory = p2p_control_action_server_.acceptNewGoal()->trajectory;
			if( trajectory.points.size()!=1 ){
				ROS_ERROR("Unexpected trajectory size. Got %u expected 1 - Cancel request.", (unsigned int) trajectory.points.size());
				result.error_code = result.INVALID_GOAL;
				continuous_control_action_server_.setAborted( result);
				return;
			}

			std::vector<int> map_joint_index_to_trajectory_joint_index(7, -1);
			int jonit_index;
			for( unsigned int trajectory_joint_ii=0; trajectory_joint_ii<trajectory.joint_names.size(); trajectory_joint_ii++ ){
				jonit_index= common_cpp_tools::find_string( lwa_joint_names_, trajectory.joint_names[trajectory_joint_ii]);
				if( jonit_index<0 ){
					// found invalid joint
					ROS_ERROR("No trajectory is executed - Found invalid joint!");
					result.error_code = result.INVALID_JOINTS;
					continuous_control_action_server_.setAborted( result);
					return;
				}
				map_joint_index_to_trajectory_joint_index[(unsigned int) jonit_index] = trajectory_joint_ii;
			}

			// set target point
			set_target_point( trajectory.points[0], map_joint_index_to_trajectory_joint_index);

			//  execute motion
			lwa_start_motion_.call( empty_srv);

			ros::Rate rate_10Hz(10);
			bool target_is_reached = false;
			while( ros::ok() && p2p_control_action_server_.isActive() && lwa_state_.modules_are_ready && !target_is_reached){
				rate_10Hz.sleep();
				ros::spinOnce();
				target_is_reached = omnirob_geometry_tools::max_angle_distance( lwa_state_.current_lwa_joint_state , trajectory.points[0].positions)<0.01;
			}

			if( !ros::ok() ){
				return;
			}

			if( !p2p_control_action_server_.isActive() ){
				ROS_ERROR("Action canceled! Stop motion!");
				lwa_stop_motion_.call( empty_srv);
				continuous_control_action_server_.setAborted( result);
				return;
			}

			if( !lwa_state_.modules_are_ready ){
				ROS_ERROR("Stop motion - modules are not ready!");
				lwa_stop_motion_.call( empty_srv);
				continuous_control_action_server_.setAborted( result);
				return;
			}

			// finish action
			ROS_INFO("Finished execution");
			lwa_stop_motion_.call( empty_srv);

			result.error_code = result.SUCCESSFUL;
			p2p_control_action_server_.setSucceeded( result);
		}
		/**
		 * goal callback for point to point motions
		 */
		void lwa_p2p_control_preempt_callback( void){
			ROS_ERROR("point to point control action is preempted!");
			p2p_control_action_server_.setPreempted();
		}

	public: // lwa state callbacks
		void lwa_joint_state_callback( const std_msgs::Float64MultiArray &joint_state_array ){
			for( unsigned int joint_ii=0; joint_ii<7; joint_ii++){
				lwa_state_.current_lwa_joint_state[joint_ii] = joint_state_array.data[joint_ii];
			}// for all joints

		}// lwa joint trajectory
		
		void lwa_trajectory_time_left_callback( const std_msgs::Float64 &time_left ){
			lwa_trajectory_time_left_ = time_left.data;

		}// lwa trajectory time left
		
		void lwa_module_is_ready_callback( const std_msgs::Float64MultiArray &module_is_ready ){
			lwa_state_.modules_are_ready = true;
			bool module_ii_is_ready;
			for( unsigned int joint_ii=0; joint_ii<7; joint_ii++ ){
				module_ii_is_ready = module_is_ready.data[joint_ii]>0.5 ? true : false;
				lwa_state_.modules_are_ready = lwa_state_.modules_are_ready && module_ii_is_ready;
			}
		}// lwa module is ready callback
		
		void lwa_continuos_mode_enabled_callback( const std_msgs::Bool &continuos_mode_enabled ){
			lwa_state_.lwa_continous_mode_enabled = continuos_mode_enabled.data;
		}// lwa continuos mode enabled callback
	

	private: // member variables
		// node handle
		ros::NodeHandle nodehandle_;

		// action servers
		actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> continuous_control_action_server_;
		actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> p2p_control_action_server_;

		// lwa names and state
		std::vector<std::string> lwa_joint_names_;
		struct lwa_state_struct{
			std::vector<double> current_lwa_joint_state;
			bool modules_are_ready;
			bool lwa_continous_mode_enabled;

			lwa_state_struct():
				current_lwa_joint_state(7),
				modules_are_ready(false),
				lwa_continous_mode_enabled(false)
			{}
		} lwa_state_;

		// trajectory monitoring
		double lwa_trajectory_time_left_;

		// subscriber
		ros::Subscriber lwa_joint_trajectory_subscriber_;
		ros::Subscriber lwa_trajectory_time_left_subscriber_;
		ros::Subscriber lwa_module_is_ready_subscriber_;
		ros::Subscriber lwa_continuos_mode_enabled_subscriber_;

		// publisher
		ros::Publisher lwa_commanded_joint_state_publisher_;

		// services
		ros::ServiceClient lwa_set_point_client_;
		ros::ServiceClient lwa_reset_mode_;
		ros::ServiceClient lwa_start_motion_;
		ros::ServiceClient lwa_stop_motion_;
		ros::ServiceClient lwa_start_trajectory_;
		ros::ServiceClient lwa_stop_trajectory_;
		ros::ServiceClient lwa_enable_continuos_position_tracking_;
		ros::ServiceClient lwa_enable_point_to_point_motion_;
		ros::ServiceClient lwa_initialize_modules_;
};

int main( int argc, char** argv) {

	// init ros node
	ros::init(argc, argv, "lwa_motion_controller");
	
	// initialize action server
	lwa_controller server( "/omnirob_robin/lwa/controller/continuous_control",
						   "/omnirob_robin/lwa/controller/p2p_control");
	
	// spin arround
	ros::spin();
	
}// main
