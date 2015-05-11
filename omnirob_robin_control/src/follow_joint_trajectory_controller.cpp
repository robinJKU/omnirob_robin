#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>

class FollowJointTrajectoryAction{
protected:
	// node handle
	ros::NodeHandle nodehandle_;
	
	// basic objects for action server
	actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
	std::string action_name_;
	control_msgs::FollowJointTrajectoryFeedback feedback_;
    control_msgs::FollowJointTrajectoryResult result_;
    
    // lwa names and joint state
    std::vector<std::string> lwa_joint_names_;
    std::vector<double> current_lwa_joint_state_;
    double lwa_trajectory_time_left_;
    bool modues_are_ready_;
    bool lwa_continuos_mode_enabled_;
    
    // subscriber
    ros::Subscriber lwa_joint_trajectory_subscriber_;
    ros::Subscriber lwa_trajectory_time_left_subscriber_;
    ros::Subscriber lwa_module_is_ready_subscriber_;
    ros::Subscriber lwa_continuos_mode_enabled_subscriber_;
    
    // publisher
    ros::Publisher lwa_commanded_joint_state_publisher;
    
    // services
    ros::ServiceClient lwa_set_point_client;
    ros::ServiceClient lwa_reset_mode;
    ros::ServiceClient lwa_start_motion;
    ros::ServiceClient lwa_stop_motion;
    ros::ServiceClient lwa_start_trajectory;
    ros::ServiceClient lwa_enable_continuos_position_tracking;
	
private:
	/**
	 * Returns the first appearance of the token in the array or -1 if the token isn't found.
	 * @return index of token in array or -1
	 */
	int find_string( std::vector<std::string> array, std::string token){
		for(unsigned int ii=0; ii<array.size(); ii++ ){
			if( array[ii].compare(token)== 0 ){
				return ii;
			}
		}
		return -1;
	}// find string
	
	/**
	 * Returns the minimum value of the elements of array
	 * @return min value or 0 if the array is empty
	 */
	int find_min_value( std::vector<int> array ){
		if( array.size()<1 ){
			return 0;
		}
		int min_value = array[0];
		for( unsigned int ii=1; ii<array.size(); ii++ ){
			if( array[ii]<min_value ){
				min_value = array[ii];
			}
		}
		return min_value;
	}// find min value
	
public:
	void goal_callback( void )
	{
		
		ros::spinOnce();

		// check if modules are ready
		if( !modues_are_ready_ ){
			ROS_ERROR("Can't execute trajectory - Modules are not ready!!");
			result_.error_code = result_.INVALID_JOINTS;
			as_.setAborted(result_);
			return;
		}
		
		// enable continuos position tracking mode
		std_srvs::Empty empty_srv;
		if( !lwa_continuos_mode_enabled_ ){
			lwa_enable_continuos_position_tracking.call( empty_srv);
		}else{
			lwa_stop_motion.call( empty_srv);
		}
		
		// parse input
		trajectory_msgs::JointTrajectory trajectory = as_.acceptNewGoal()->trajectory;
		std::vector<int> joint_index(7,-1);
		bool found_one_joint = false;
		for( unsigned int joint_ii=0; joint_ii<7; joint_ii++ ){
			joint_index[joint_ii] = find_string( trajectory.joint_names, lwa_joint_names_[joint_ii]  );
			if( joint_index[joint_ii]>=0 ){
				found_one_joint = true;
			}
		}
		
		if( !found_one_joint ){
			// no valid joint was found
			ROS_ERROR("No trajectory is executed - Can't finde any valid joint!");
			result_.error_code = result_.INVALID_JOINTS;
			as_.setAborted(result_);
			return;
		}
		
		
		if( find_min_value( joint_index)<0 ){
			// get newest joint values
			ros::spinOnce();
		}
		
		// initialize trajectory planner
		// set restart mode on
		lwa_reset_mode.call( empty_srv);
		
		// stream trajectory
		std_msgs::Float64MultiArray lwa_point;
		lwa_point.data.resize(7);
		ros::Rate rate_10Hz(10);
		
		for( unsigned int point_ii=0; point_ii<trajectory.points.size(); point_ii++){
			
			for( unsigned int joint_ii=0; joint_ii<7; joint_ii++ ){
				if( joint_index[joint_ii]>=0 ){
					lwa_point.data[joint_ii] = trajectory.points[point_ii].positions[joint_index[joint_ii]];
				}else{
					// hold joint constant
					lwa_point.data[joint_ii] = current_lwa_joint_state_[joint_ii];
				}
			}
			// set points
			lwa_commanded_joint_state_publisher.publish( lwa_point);
			rate_10Hz.sleep();
			ros::spinOnce();
			
			lwa_set_point_client.call( empty_srv);
			rate_10Hz.sleep();
			
		}// for all points
		
		// set restart mode off
		lwa_reset_mode.call( empty_srv);

		ros::Rate rate_1Hz(1);		
		while( lwa_trajectory_time_left_<0.0 ){
			rate_1Hz.sleep();
			ros::spinOnce();
		}

		//  move 
		lwa_start_motion.call( empty_srv);
		lwa_start_trajectory.call( empty_srv);

		printf("lwa_trajectory_time_left = %f\n", lwa_trajectory_time_left_);
		printf("modules are ready = %i\n", (int) modues_are_ready_);
		while( as_.isActive() && (lwa_trajectory_time_left_>0.0) && modues_are_ready_ ){
			rate_10Hz.sleep();
			ros::spinOnce();
		}
		
		printf("finished\n");
		if( !as_.isActive() ){
			ROS_ERROR("Action canceled!");
			as_.setAborted(result_);
			return;
		}
		
		if( !modues_are_ready_ ){
			ROS_ERROR("Stop motion - modules are not ready!");
		}
		
		// finish action
		lwa_stop_motion.call( empty_srv);
		result_.error_code = result_.SUCCESSFUL;
		as_.setSucceeded(result_);
		
	}
	
	void preempt_callback( void )
	{
		printf("server preeampted");
		as_.setPreempted();
		
	}// preempt callback
	
	void lwa_joint_trajectory_callback( std_msgs::Float64MultiArray joint_state_array ){
		for( unsigned int joint_ii=0; joint_ii<7; joint_ii++){
			current_lwa_joint_state_[joint_ii] = joint_state_array.data[joint_ii];
		}// for all joints
		
	}// lwa joint trajectory
	
	void lwa_trajectory_time_left_callback( std_msgs::Float64 time_left ){
		lwa_trajectory_time_left_ = time_left.data;
		
	}// lwa trajectory time left
	
	void lwa_module_is_ready_callback( std_msgs::Float64MultiArray module_is_ready ){
		modues_are_ready_ = true;
		bool module_ii_is_ready;
		for( unsigned int joint_ii=0; joint_ii<7; joint_ii++ ){
			module_ii_is_ready = module_is_ready.data[joint_ii]>0.5 ? true : false;
			modues_are_ready_ = modues_are_ready_ && module_ii_is_ready;
		}
	}// lwa module is ready callback
	
	void lwa_continuos_mode_enabled_callback( std_msgs::Bool continuos_mode_enabled ){
		lwa_continuos_mode_enabled_ = continuos_mode_enabled.data;
	}// lwa continuos mode enabled callback

	// constructor
	FollowJointTrajectoryAction(std::string name):
		as_(nodehandle_, name, false),
		action_name_(name),
		current_lwa_joint_state_(7),
		lwa_continuos_mode_enabled_( false)
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
		modues_are_ready_ = false;
		
		// initialize subscriber
		lwa_joint_trajectory_subscriber_ = nodehandle_.subscribe("/omnirob_robin/lwa/state/joint_state_array", 10, &FollowJointTrajectoryAction::lwa_joint_trajectory_callback, this);
		lwa_trajectory_time_left_subscriber_ = nodehandle_.subscribe("/omnirob_robin/lwa/trajectory/time_left", 10, &FollowJointTrajectoryAction::lwa_trajectory_time_left_callback, this);
		lwa_module_is_ready_subscriber_ = nodehandle_.subscribe("/omnirob_robin/lwa/state/info/module_is_ready", 10, &FollowJointTrajectoryAction::lwa_module_is_ready_callback, this);
		lwa_continuos_mode_enabled_subscriber_ = nodehandle_.subscribe("/omnirob_robin/lwa/state/info/continuos_position_tracking_mode_enabled", 10, &FollowJointTrajectoryAction::lwa_continuos_mode_enabled_callback, this);
		
		
		// initialize publisher
		lwa_commanded_joint_state_publisher = nodehandle_.advertise<std_msgs::Float64MultiArray>("/omnirob_robin/lwa/control/commanded_joint_state", 10);
		
		// initialize services
		lwa_set_point_client = nodehandle_.serviceClient<std_srvs::Empty>("/omnirob_robin/lwa/trajectory/set_point");
		lwa_reset_mode = nodehandle_.serviceClient<std_srvs::Empty>("/omnirob_robin/lwa/trajectory/reset_mode");
		lwa_start_trajectory = nodehandle_.serviceClient<std_srvs::Empty>("/omnirob_robin/lwa/trajectory/start"	);
		lwa_start_motion = nodehandle_.serviceClient<std_srvs::Empty>("/omnirob_robin/lwa/control/start_motion");
		lwa_stop_motion = nodehandle_.serviceClient<std_srvs::Empty>("/omnirob_robin/lwa/control/stop_motion");
		lwa_enable_continuos_position_tracking = nodehandle_.serviceClient<std_srvs::Empty>("/omnirob_robin/lwa/control/enable_continuos_position_tracking");
		
		// register callbacks
		as_.registerGoalCallback(boost::bind(&FollowJointTrajectoryAction::goal_callback, this));
		as_.registerPreemptCallback(boost::bind(&FollowJointTrajectoryAction::preempt_callback, this));
		
		// start action server
		as_.start();
	}
	
	// destructor
	~FollowJointTrajectoryAction( void)
	{
	}
	
};

int main( int argc, char** argv) {

	// init ros node
	ros::init(argc, argv, "follow_joint_trajectory_controller");
	
	// initialize action server
	FollowJointTrajectoryAction controller(ros::this_node::getName());
	
	// spin arround
	ros::spin();
	
}// main
