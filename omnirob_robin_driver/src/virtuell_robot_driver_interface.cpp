#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/JointTrajectory.h>

class Modules{
	protected:
		// meta data
		unsigned int nr_of_modules;
		std::string group_name;
		
		// state
		std_msgs::Float64MultiArray joint_state;
	
		// info state
		std::vector<bool> module_is_ready;
		std::vector<bool> break_activated;
		std::vector<bool> module_has_warnings;
		std::vector<bool> module_in_motion;
		std::vector<bool> module_is_enabled;
		std::vector<bool> position_reached;
		
		// error state
		std::vector<bool> initialization_error;
		std::vector<bool> module_has_errors;
		std::vector<bool> module_has_low_voltage;
		std::vector<bool> module_is_in_emergency_stop;
		std::vector<bool> module_not_referenced;
		std::vector<bool> move_blocked;
		std::vector<bool> tow_error;
		
		// mode of operation
		bool point_to_point_motion_enabled;
		bool continuos_position_tracking_mode_enabled;
		bool start_motion_trigger;
		
		// advertised services
		ros::ServiceServer enable_continuos_position_tracking_service;
		ros::ServiceServer enable_point_to_point_motion_service;
		ros::ServiceServer initialize_modules_service;
		ros::ServiceServer reboot_modules_service;
		ros::ServiceServer reference_modules_service;
		ros::ServiceServer send_acknowledge_to_modules_service;
		ros::ServiceServer start_motion_service;
		ros::ServiceServer stop_motion_service; 
		
		// subscribed topics
		ros::Subscriber commanded_joint_state_subscriber;
		ros::Subscriber e_stop_subscriber;
		
		// published topics
		ros::Publisher initialization_error_publisher;
		ros::Publisher module_has_errors_publisher;
		ros::Publisher module_has_low_voltage_publisher;
		ros::Publisher module_is_in_emergency_stop_publisher;
		ros::Publisher module_not_referenced_publisher;
		ros::Publisher move_blocked_publisher;
		ros::Publisher tow_error_publisher;
		
		ros::Publisher break_activated_publisher;
		ros::Publisher continuos_position_tracking_mode_enabled_publisher;
		ros::Publisher point_to_point_motion_enabled_publisher;
		ros::Publisher module_has_warnings_publisher;
		ros::Publisher module_in_motion_publisher;
		ros::Publisher module_is_enabled_publisher;
		ros::Publisher module_is_ready_publisher;
		ros::Publisher position_reached_publisher;
		
		ros::Publisher joint_state_array_publisher;
	
	public:
		Modules( ros::NodeHandle handle, std::string group_name, unsigned int nr_of_modules, std::string emergency_stop_topic ){ // Constructor
			// meta data
			this->nr_of_modules = nr_of_modules;
			this-> group_name = group_name;
			
			// state
			joint_state.data.resize( nr_of_modules);
			
			// info state
			module_is_ready.resize( nr_of_modules);
			break_activated.resize( nr_of_modules);
			module_has_warnings.resize( nr_of_modules);
			module_in_motion.resize( nr_of_modules);
			module_is_enabled.resize( nr_of_modules);
			position_reached.resize( nr_of_modules);
			
			// error state
			initialization_error.resize( nr_of_modules);
			module_has_errors.resize( nr_of_modules);
			for( unsigned int module=0; module<nr_of_modules; module++){
				initialization_error[module] = true;
				module_has_errors[module] = true;
			}
			
			module_has_low_voltage.resize( nr_of_modules);
			module_is_in_emergency_stop.resize( nr_of_modules);
			module_not_referenced.resize( nr_of_modules);
			move_blocked.resize( nr_of_modules);
			tow_error.resize( nr_of_modules);
			
			// mode of operation
			point_to_point_motion_enabled = true;
			continuos_position_tracking_mode_enabled = false;
			start_motion_trigger = false;
			
			// subscribe topics
			commanded_joint_state_subscriber = handle.subscribe<std_msgs::Float64MultiArray>( group_name + "/control/" + "commanded_joint_state", 10, &Modules::set_joint_state, this);
			e_stop_subscriber = handle.subscribe<std_msgs::Bool>( emergency_stop_topic, 10, &Modules::emergency_stop, this);
			
			// published topics
			initialization_error_publisher = handle.advertise<std_msgs::Float64MultiArray>( group_name + "/state/" + "error/" +  "initialization_error", 10);
			module_has_errors_publisher = handle.advertise<std_msgs::Float64MultiArray>( group_name + "/state/" + "error/" +  "module_has_errors", 10);
			module_has_low_voltage_publisher = handle.advertise<std_msgs::Float64MultiArray>( group_name + "/state/" + "error/" +  "module_has_low_voltage", 10);
			module_is_in_emergency_stop_publisher = handle.advertise<std_msgs::Float64MultiArray>( group_name + "/state/" + "error/" +  "module_is_in_emergency_stop", 10);
			module_not_referenced_publisher = handle.advertise<std_msgs::Float64MultiArray>( group_name + "/state/" + "error/" +  "module_not_referenced", 10);
			move_blocked_publisher = handle.advertise<std_msgs::Float64MultiArray>( group_name + "/state/" + "error/" +  "move_blocked", 10);
			tow_error_publisher = handle.advertise<std_msgs::Float64MultiArray>( group_name + "/state/" + "error/" +  "tow_error", 10);
			
			break_activated_publisher = handle.advertise<std_msgs::Float64MultiArray>( group_name + "/state/" + "info/" +  "break_activated", 10);
			continuos_position_tracking_mode_enabled_publisher = handle.advertise<std_msgs::Bool>( group_name + "/state/" + "info/" +  "continuos_position_tracking_mode_enabled", 10);
			point_to_point_motion_enabled_publisher = handle.advertise<std_msgs::Bool>( group_name + "/state/" + "info/" +  "point_to_point_motion_enabled", 10);
			module_has_warnings_publisher = handle.advertise<std_msgs::Float64MultiArray>( group_name + "/state/" + "info/" +  "module_has_warnings", 10);
			module_in_motion_publisher = handle.advertise<std_msgs::Float64MultiArray>( group_name + "/state/" + "info/" +  "module_in_motion", 10);
			module_is_enabled_publisher = handle.advertise<std_msgs::Float64MultiArray>( group_name + "/state/" + "info/" +  "module_is_enabled", 10);
			module_is_ready_publisher = handle.advertise<std_msgs::Float64MultiArray>( group_name + "/state/" + "info/" +  "module_is_ready", 10);
			position_reached_publisher = handle.advertise<std_msgs::Float64MultiArray>( group_name + "/state/" + "info/" +  "position_reached", 10);
			
			joint_state_array_publisher = handle.advertise<std_msgs::Float64MultiArray>( group_name + "/state/" + "joint_state_array", 10);
			
			// advertise services
			enable_continuos_position_tracking_service = handle.advertiseService( group_name + "/control/" + "enable_continuos_position_tracking",  &Modules::enable_continuos_position_tracking, this);
			enable_point_to_point_motion_service = handle.advertiseService( group_name + "/control/" + "enable_point_to_point_motion", &Modules::enable_point_to_point_motion, this);
			initialize_modules_service = handle.advertiseService( group_name + "/control/" + "initialize_modules",  &Modules::initialize_modules, this);
			reboot_modules_service = handle.advertiseService( group_name + "/control/" + "reboot_modules",  &Modules::reboot_modules, this);
			reference_modules_service = handle.advertiseService( group_name + "/control/" + "reference_module",  &Modules::reference_modules, this);
			send_acknowledge_to_modules_service = handle.advertiseService( group_name + "/control/" + "send_acknowledge_to_modules",  &Modules::send_acknowledge_to_modules, this);
			start_motion_service = handle.advertiseService( group_name + "/control/" + "start_motion",  &Modules::start_motion, this);
			stop_motion_service = handle.advertiseService( group_name + "/control/" + "stop_motion",  &Modules::stop_motion, this); 
		
			
		}// constructor
	
	
	protected:
		std_msgs::Float64MultiArray convert_bool2float_array( std::vector<bool> bool_array ){
			std_msgs::Float64MultiArray float_array;
			
			unsigned int size = bool_array.size();
			float_array.data.resize( size );
			
			for( unsigned int ii=0; ii<size; ii++ ){
				float_array.data[ii] = bool_array[ii] ? 1.0 : 0.0;
			}
			
			return float_array;
			
		}// convert bool2float_array
		
		std_msgs::Bool convert_bool2stdbool( bool value ){
			std_msgs::Bool bool_msg;
			bool_msg.data = value;
			return bool_msg;
			
		}// convert bool2float_array
	
		void clear_warning_status( void){
			for( unsigned int module=0; module<nr_of_modules; module++){
				break_activated[module] = false;
				module_has_warnings[module] = false;
				
			}
		}// clear warnings
	
		bool parse_argument( std::vector<std::string> argument, std::string token){
			if( argument.empty() ){
				return false;
			}
			
			for( unsigned int string_ii=0; string_ii<argument.size(); string_ii++){
				if( argument[string_ii].compare(token)==0){
					// found token
					return true;
				}
			}
			return false;
			
		}// parse argument
	
		void clear_error_status( std::vector<std::string> exceptions ){
			
			for( unsigned int module=0; module<nr_of_modules; module++){
				
				if( !parse_argument( exceptions, "initialization_error" ) ){
					initialization_error[module] = false;
				}
				if( !parse_argument( exceptions, "module_has_errors" ) ){
					module_has_errors[module] = false;
				}
				if( !parse_argument( exceptions, "module_has_low_voltage" ) ){
					module_has_low_voltage[module] = false;
				}
				if( !parse_argument( exceptions, "module_is_in_emergency_stop" ) ){
					module_is_in_emergency_stop[module] = false;
				}
				if( !parse_argument( exceptions, "module_not_referenced" ) ){
					module_not_referenced[module] = false;
				}
				if( !parse_argument( exceptions, "move_blocked" ) ){
					move_blocked[module] = false;
				}
				if( !parse_argument( exceptions	, "tow_error" ) ){
					tow_error[module] = false;
				}
			}
		
		}// clear error
	
		void set_modules_ready( void){
			
			for( unsigned int module=0; module<nr_of_modules; module++){
				module_is_ready[module] = true;
			}
			
		}// set modules
		
		void reset_modules_ready( void){
			
			for( unsigned int module=0; module<nr_of_modules; module++){
				module_is_ready[module] = false;
			}
			
		}// reset modules

		bool only_reference_error( unsigned int module){
			
			bool only_reference_error_true = false;
			
			only_reference_error_true =  only_reference_error_true || initialization_error[module];
			only_reference_error_true =  only_reference_error_true || module_has_low_voltage[module];
			only_reference_error_true =  only_reference_error_true || module_is_in_emergency_stop[module];
			only_reference_error_true =  only_reference_error_true || move_blocked[module];
			only_reference_error_true =  only_reference_error_true || tow_error[module];
			
			return only_reference_error_true;
		}// only reference error
		
		bool no_errors( unsigned int module ){
			
			return only_reference_error(module) || module_not_referenced[module];
			
		}// no errors
	
	
	protected:
		// services
		
		bool enable_continuos_position_tracking( std_srvs::Empty::Request &req, std_srvs::Empty::Response &res ){
			stop_motion( req, res );
			
			// mode of operation
			point_to_point_motion_enabled = false;
			continuos_position_tracking_mode_enabled = true;
			
			return true;
			
		}// enable continuos position tracking
		
		bool enable_point_to_point_motion( std_srvs::Empty::Request &req, std_srvs::Empty::Response &res ){
			stop_motion( req, res );
			
			// mode of operation
			point_to_point_motion_enabled = true;
			continuos_position_tracking_mode_enabled = false;
			
			return true;
			
		}// enable point to point motion
	
		bool reboot_modules( std_srvs::Empty::Request &req, std_srvs::Empty::Response &res ){
			clear_warning_status();
			
			std::vector<std::string> test; // todo remove
			test.push_back("");
			clear_error_status(test);
			
			reset_modules_ready();
			
			return true;
			
		}// reboot modules
		
		bool reference_modules( std_srvs::Empty::Request &req, std_srvs::Empty::Response &res ){
			for( unsigned int module=0; module<nr_of_modules; module++){
				module_not_referenced[module] = false;
				if( only_reference_error( module )){
					// reset error if there is no other error
					module_has_errors[module] = false;
					if( !module_has_warnings[module] ){
						module_is_ready[module] = true;
						
					}
					
				}
				
			}
			
			return true;
			
		}// reference modules
		
		bool send_acknowledge_to_modules( std_srvs::Empty::Request &req, std_srvs::Empty::Response &res ){
			
			// remove all errors and warnings which are easy to handle
			clear_warning_status();
			
			std::vector<std::string> non_removed_errors;
			non_removed_errors.push_back("initialization_error");
			non_removed_errors.push_back("module_not_referenced");
			non_removed_errors.push_back("move_blocked");
			
			clear_error_status( non_removed_errors);
			
			// if therer are no remaining errors / warnings -> set module ready
			for( unsigned int module=0; module<nr_of_modules; module++){
				
				if( no_errors( module) ){
					module_is_ready[module] = true;
				}
				
			}
			
			return true;
		
		}// send acknowledge
		
		bool initialize_modules( std_srvs::Empty::Request &req, std_srvs::Empty::Response &res ){
			
			send_acknowledge_to_modules( req, res);
			
			set_modules_ready();
			
			// todo: start continous sending
			
			return true;
			
		}
		
		bool start_motion( std_srvs::Empty::Request &req, std_srvs::Empty::Response &res ){
			for( unsigned int module=0; module<nr_of_modules; module++){
				module_is_enabled[module] = true;
				break_activated[module] = false;
			}
			
			start_motion_trigger = true;
			
			// todo: enable position control with respect to the enabled modi
			// @ point to point motion: start_motion callback ->  command joints
			// @ continuos position control: 
			
			return true;
			
		}// start motion 
		
		bool stop_motion( std_srvs::Empty::Request &req, std_srvs::Empty::Response &res ){
			for( unsigned int module=0; module<nr_of_modules; module++){
				module_is_enabled[module] = false;
				break_activated[module] = true;
			}
			
			// todo: disable position control with respect to the enabled modi
			
			return true;
			
		}// stop motion
		
		// topics
		void set_joint_state( std_msgs::Float64MultiArray desired_joint_state ){
			joint_state = desired_joint_state;
			
		}
		
		void emergency_stop( std_msgs::Bool e_stop ){
			if( e_stop.data ){
				for( unsigned int module=0; module<nr_of_modules; module++ ){
					module_is_ready[module] = false;
					break_activated[module] = true;
					module_in_motion[module] = false;
					
					// error state
					module_has_errors[module] = true;
					module_is_in_emergency_stop[module] = true;
					
				}
				
			}
			
		}// do emergency stop
	
	public:
		void publish_module_state( void ){
			
			initialization_error_publisher.publish( convert_bool2float_array( initialization_error ));
			module_has_errors_publisher.publish( convert_bool2float_array( module_has_errors ));
			module_has_low_voltage_publisher.publish( convert_bool2float_array( module_has_low_voltage ));
			module_is_in_emergency_stop_publisher.publish( convert_bool2float_array( module_is_in_emergency_stop ));
			module_not_referenced_publisher.publish( convert_bool2float_array( module_not_referenced ));
			move_blocked_publisher.publish( convert_bool2float_array( move_blocked ));
			tow_error_publisher.publish( convert_bool2float_array( tow_error ));
			
			break_activated_publisher.publish( convert_bool2float_array( break_activated ));
			continuos_position_tracking_mode_enabled_publisher.publish( convert_bool2stdbool( continuos_position_tracking_mode_enabled) );
			point_to_point_motion_enabled_publisher.publish( convert_bool2stdbool( point_to_point_motion_enabled) );
			module_has_warnings_publisher.publish( convert_bool2float_array( module_has_warnings ));
			module_in_motion_publisher.publish( convert_bool2float_array( module_in_motion ));
			module_is_enabled_publisher.publish( convert_bool2float_array( module_is_enabled ));
			module_is_ready_publisher.publish( convert_bool2float_array( module_is_ready ));
			position_reached_publisher.publish( convert_bool2float_array( position_reached ));
			
			joint_state_array_publisher.publish( joint_state);
			
		}// publish module state
			
	
};// class Modules


class LWA : public Modules{
	private:
		ros::Publisher commanded_joint_state_publisher;
		trajectory_msgs::JointTrajectory joint_trajectory;
		
	public:
		LWA( ros::NodeHandle handle, std::string group_name, std::string controller_name, std::string emergency_stop_topic )
			: Modules( handle, group_name, 7, emergency_stop_topic )
		{
			
			joint_trajectory.joint_names.push_back("lwa/joint_1");
			joint_trajectory.joint_names.push_back("lwa/joint_2");
			joint_trajectory.joint_names.push_back("lwa/joint_3");
			joint_trajectory.joint_names.push_back("lwa/joint_4");
			joint_trajectory.joint_names.push_back("lwa/joint_5");
			joint_trajectory.joint_names.push_back("lwa/joint_6");
			joint_trajectory.joint_names.push_back("lwa/joint_7");
			
			joint_trajectory.points.resize(1);
			joint_trajectory.points[0].positions.resize( 7);
			
			joint_trajectory.points[0].time_from_start = ros::Duration(0.021);
			
			commanded_joint_state_publisher = handle.advertise<trajectory_msgs::JointTrajectory>( controller_name + "/command" ,  1); 
			
		}
		
		void publish_commanded_state( void ){
			
			joint_trajectory.header.stamp = ros::Time::now();

			for( unsigned int module = 0; module<nr_of_modules; module++ ){
				if( (continuos_position_tracking_mode_enabled &&  module_is_enabled[module] && module_is_ready[module]) ||
					(point_to_point_motion_enabled && module_is_ready[module] && start_motion_trigger) ){
						
					joint_trajectory.points[0].positions[module] =  joint_state.data[module];
				}
			}
			
			commanded_joint_state_publisher.publish( joint_trajectory);
			
			if( start_motion_trigger ){
				start_motion_trigger = false;
			}
			
		}
		
	
}; // class LWA

class PAN_TILT : public Modules{
	private:
		ros::Publisher pan_commanded_joint_state_publisher;
		ros::Publisher tilt_commanded_joint_state_publisher;
		
	public:
		PAN_TILT( ros::NodeHandle handle, std::string group_name, std::string controller_name_pan, std::string controller_name_tilt, std::string emergency_stop_topic )
			: Modules( handle, group_name, 2, emergency_stop_topic )
		{
			
			pan_commanded_joint_state_publisher = handle.advertise<std_msgs::Float64>( controller_name_pan + "/command" ,  1); 
			tilt_commanded_joint_state_publisher = handle.advertise<std_msgs::Float64>( controller_name_tilt + "/command" ,  1); 
			
		}
		
		void publish_commanded_state( void ){
			// pan
			if( (continuos_position_tracking_mode_enabled &&  module_is_enabled[0] && module_is_ready[0]) ||
			    (point_to_point_motion_enabled && module_is_ready[0] && start_motion_trigger) ){
					
				std_msgs::Float64 pan;
				pan.data = joint_state.data[0];
				pan_commanded_joint_state_publisher.publish( pan );
			}
			
			// tilt
			if( (continuos_position_tracking_mode_enabled &&  module_is_enabled[1] && module_is_ready[1]) ||
			    (point_to_point_motion_enabled && module_is_ready[1] && start_motion_trigger) ){
					
				std_msgs::Float64 tilt;
				tilt.data = joint_state.data[1];
				tilt_commanded_joint_state_publisher.publish( tilt );
			}
			
			if( start_motion_trigger ){
				start_motion_trigger = false;
			}
							
		}
		
	
}; // class PAN TILT


class GRIPPER : public Modules{
	private:
		ros::Publisher left_commanded_joint_state_publisher;
		ros::Publisher right_commanded_joint_state_publisher;
		
	public:
		GRIPPER( ros::NodeHandle handle, std::string group_name, std::string controller_name_left,  std::string controller_name_right, std::string emergency_stop_topic )
			: Modules( handle, group_name, 1, emergency_stop_topic )
		{
			left_commanded_joint_state_publisher = handle.advertise<std_msgs::Float64>( controller_name_left + "/command" ,  1); 
			right_commanded_joint_state_publisher = handle.advertise<std_msgs::Float64>( controller_name_right + "/command" ,  1); 
		}
		
		void publish_commanded_state( void ){
			if( (continuos_position_tracking_mode_enabled &&  module_is_enabled[0] && module_is_ready[0]) ||
			    (point_to_point_motion_enabled && module_is_ready[0] && start_motion_trigger) ){
					
				start_motion_trigger = false;
				
				std_msgs::Float64 left, right;
				left.data = joint_state.data[0]/2.0 * 0.001; // gripper command in mm!!
				right.data = joint_state.data[0]/2.0 * 0.001; // gripper command in mm!!
				
				left_commanded_joint_state_publisher.publish( left );
				right_commanded_joint_state_publisher.publish( right );
			}
			
		}
		
}; // class GRIPPER


class BASE{
	private:
		// state
		bool can_server_enabled;
		bool drives_enabled;
		bool drives_have_error;
		geometry_msgs::Twist cmd_vel;
		
		// advertised services
		ros::ServiceServer start_can_server_service, stop_can_server_service;
		ros::ServiceServer start_drives_service, stop_drives_service;
		
		// subscribed topics
		ros::Subscriber cmd_vel_subscriber;
		ros::Subscriber e_stop_subscriber;
		
		// published topics
		ros::Publisher cmd_vel_publisher;
		ros::Publisher canserver_state_publisher;
		ros::Publisher base_state_ready_publisher;
		ros::Publisher base_state_fault_publisher;

	public:
		BASE( ros::NodeHandle handle, std::string group_name, std::string emergency_stop_topic ){
			// state
			can_server_enabled = false;
			drives_enabled = false;
			drives_have_error = false;
			
			// advertised services
			start_can_server_service = handle.advertiseService( group_name + "/canserver/" + "start",  &BASE::start_can_server, this);
			stop_can_server_service = handle.advertiseService( group_name + "/canserver/" + "stop",  &BASE::stop_can_server, this);
			start_drives_service = handle.advertiseService( group_name + "/drives/control/" + "start",  &BASE::start_drives, this);
			stop_drives_service = handle.advertiseService( group_name + "/drives/control/" + "stop",  &BASE::stop_drives, this);
			
			// subscribed topics
			cmd_vel_subscriber = handle.subscribe( group_name + "/drives/control/" + "cmd_vel", 10, &BASE::read_cmd_vel, this);
			e_stop_subscriber = handle.subscribe( emergency_stop_topic , 10, &BASE::e_stop, this);
			
			// published topics
			cmd_vel_publisher = handle.advertise<geometry_msgs::Twist>( group_name + "/cmd_vel", 10);
			canserver_state_publisher = handle.advertise<std_msgs::Bool>( group_name + "/canserver/" + "state", 10);
			base_state_ready_publisher = handle.advertise<std_msgs::Bool>( group_name + "/drives/state/info/motors_ready_end_enabled", 1000);
			base_state_fault_publisher = handle.advertise<std_msgs::Bool>( group_name + "/drives/state/error/motors_have_error", 1000);
		
		}
		
	private:
		void reset_cmd_vel( void ){
			cmd_vel.linear.x = 0.0;
			cmd_vel.linear.y = 0.0;
			cmd_vel.angular.z = 0.0;
		}
	
		bool start_can_server( std_srvs::Empty::Request &req, std_srvs::Empty::Response &res ){
			can_server_enabled = true;
		}
		
		bool stop_can_server( std_srvs::Empty::Request &req, std_srvs::Empty::Response &res ){
			can_server_enabled = false;
		}
		
		bool start_drives( std_srvs::Empty::Request &req, std_srvs::Empty::Response &res ){
			drives_enabled = true;
		}
		
		bool stop_drives( std_srvs::Empty::Request &req, std_srvs::Empty::Response &res ){
			// stop motion
			reset_cmd_vel();
			cmd_vel_publisher.publish( cmd_vel);
			
			// disable drive
			drives_enabled = false;
		}
		
		void read_cmd_vel( const geometry_msgs::Twist & cmd_vel ){
			this->cmd_vel = cmd_vel;
		}
		
		void e_stop( const std_msgs::Bool & e_stop ){
			if( e_stop.data ){
				// stop motion
				reset_cmd_vel();
				cmd_vel_publisher.publish( cmd_vel);
				
				// disable drive
				drives_enabled = false;
			}
		}
		
	public:
		void publish_cmd_vel( void ){
			if( can_server_enabled && drives_enabled ){
				cmd_vel_publisher.publish( cmd_vel);
			}
		}
		
		void publish_state( void ){
			std_msgs::Bool state;
			state.data = can_server_enabled;
			canserver_state_publisher.publish( state );
			
			state.data = drives_enabled;
			base_state_ready_publisher.publish( state);
			
			state.data = drives_have_error;
			base_state_fault_publisher.publish( state);
			
		}
		
	
}; // class base


int main( int argc, char** argv) {

	// init ros node
	ros::init(argc, argv, "virtuell_robot_driver_interface");
	ros::NodeHandle handle;
	
	LWA lwa(handle, "/omnirob_robin/lwa", "/omnirob_robin/lwa/joint_controllers", "/omnirob_robin/emergency_stop" );
	PAN_TILT pan_tilt(handle, "/omnirob_robin/pan_tilt", "/omnirob_robin/pan_tilt/pan_position_controller", "/omnirob_robin/pan_tilt/tilt_position_controller", "/omnirob_robin/emergency_stop" );
	GRIPPER gripper(handle, "/omnirob_robin/gripper", "/omnirob_robin/gripper/left_position_controller", "/omnirob_robin/gripper/right_position_controller" , "/omnirob_robin/emergency_stop" );
	BASE base( handle, "/omnirob_robin/base", "/omnirob_robin/emergency_stop" );
	
	ros::Duration rate(0.01);
	
	while( ros::ok() ){
		
		lwa.publish_commanded_state();
		pan_tilt.publish_commanded_state();
		gripper.publish_commanded_state();
		base.publish_cmd_vel();
		ros::spinOnce();
		
		lwa.publish_module_state();
		pan_tilt.publish_module_state();
		gripper.publish_module_state();
		base.publish_state();
		ros::spinOnce();
		
		rate.sleep();
		
	}
	
}// main
