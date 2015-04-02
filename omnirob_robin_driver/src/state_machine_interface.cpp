#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Empty.h>

class Modules{
	private:
		// meta data
		unsigned int nr_of_modules;
	
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
	
	public:
		Modules( unsigned int nr_of_modules ){ // Constructor
			this->nr_of_modules = nr_of_modules;
			
			// info state
			module_is_ready.resize( nr_of_modules);
			break_activated.resize( nr_of_modules);
			module_has_warnings.resize( nr_of_modules);
			module_in_motion.resize( nr_of_modules);
			module_is_enabled.resize( nr_of_modules);
			position_reached.resize( nr_of_modules);
			
			// error state
			initialization_error.resize( nr_of_modules);
			for( unsigned int module=0; module<nr_of_modules; module++){
				initialization_error[module] = true;
			}
			
			module_has_errors.resize( nr_of_modules);
			module_has_low_voltage.resize( nr_of_modules);
			module_is_in_emergency_stop.resize( nr_of_modules);
			module_not_referenced.resize( nr_of_modules);
			move_blocked.resize( nr_of_modules);
			tow_error.resize( nr_of_modules);
			
			// mode of operation
			point_to_point_motion_enabled = true;
			continuos_position_tracking_mode_enabled = false;
			start_motion_trigger = false;
		
		}// constructor
	
	
	private:
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
	
	
	public:
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
		
	bool emergency_stop( std_srvs::Empty::Request &req, std_srvs::Empty::Response &res ){
		for( unsigned int module=0; module<nr_of_modules; module++ ){
			module_is_ready[module] = false;
			break_activated[module] = true;
			module_in_motion[module] = false;
			
			// error state
			module_has_errors[module] = true;
			module_is_in_emergency_stop[module] = true;
			
		}
		
	}// do emergency stop
	
	// topics
		
	
};// class Modules


int main( int argc, char** argv) {

	// init ros node
	ros::init(argc, argv, "state_machine_interface");
	ros::NodeHandle handle;

	Modules lwa(7);
	std::string lwa_namespace = "/omnirob_robin/lwa/";
		// subscribe to topics
		
		// advertise topics
		ros::Publisher lwa_commanded_joint_state_pub = handle.advertise<std_msgs::Float64MultiArray>( lwa_namespace + "control/" + "commanded_joint_state", 1000);
		/*
		joint_state_array

		initialization_error
		module_has_errors
		module_has_low_voltage		
		module_is_in_emergency_stop
		module_not_referenced
		move_blocked
		tow_error
		
		break_activated
		continuos_position_tracking_mode_enabled
		point_to_point_motion_enabled
		module_has_warnings
		module_in_motion
		module_is_enabled
		module_is_ready
		position_reached
		
/omnirob_robin/gripper/state/error/module_not_referenced    		[std_msgs/Float64MultiArray]
/omnirob_robin/gripper/state/error/move_blocked    					[std_msgs/Float64MultiArray]
/omnirob_robin/gripper/state/error/tow_error    					[std_msgs/Float64MultiArray]
/omnirob_robin/gripper/state/info/break_activated			    	[std_msgs/Float64MultiArray]
/omnirob_robin/gripper/state/info/continuos_position_tracking_mode_enabled     [std_msgs/Bool]
/omnirob_robin/gripper/state/info/point_to_point_motion_enabled     		   [std_msgs/Bool]
/omnirob_robin/gripper/state/info/module_has_warnings	    		[std_msgs/Float64MultiArray]
/omnirob_robin/gripper/state/info/module_in_motion    				[std_msgs/Float64MultiArray]
/omnirob_robin/gripper/state/info/module_is_enabled    				[std_msgs/Float64MultiArray]
/omnirob_robin/gripper/state/info/module_is_ready	    			[std_msgs/Float64MultiArray]
/omnirob_robin/gripper/state/info/position_reached   				[std_msgs/Float64MultiArray]
/omnirob_robin/gripper/state/joint_state_array	   					[std_msgs/Float64MultiArray]
		
		
		// advertise services
		ros::ServiceServer lwa_enable_continuos_position_tracking_service = handle.advertiseService( lwa_namespace + "control/" + "enable_continuos_position_tracking",  1000, &Modules::enable_continuos_position_tracking	, &lwa);
		ros::ServiceServer lwa_enable_point_to_point_motion_service = handle.advertiseService( lwa_namespace + "control/" + "enable_point_to_point_motion", 1000, &Modules::enable_point_to_point_motion	, &lwa);
		ros::ServiceServer lwa_initialize_modules_service = handle.advertiseService( lwa_namespace + "control/" + "initialize_modules",  1000, &Modules::initialize_modules	, &lwa);
		ros::ServiceServer lwa_reboot_modules_service = handle.advertiseService( lwa_namespace + "control/" + "reboot_modules",  1000, &Modules::reboot_modules	, &lwa);
		ros::ServiceServer lwa_reference_modules_service = handle.advertiseService( lwa_namespace + "control/" + "reference_module",  1000, &Modules::reference_module	, &lwa);
		ros::ServiceServer lwa_send_acknowledge_to_modules_service = handle.advertiseService( lwa_namespace + "control/" + "send_acknowledge_to_modules",  1000, &Modules::send_acknowledge_to_modules	, &lwa);
		ros::ServiceServer lwa_start_motion_service = handle.advertiseService( lwa_namespace + "control/" + "start_motion",  1000, &Modules::start_motion	, &lwa);
		ros::ServiceServer lwa_stop_motion_service = handle.advertiseService( lwa_namespace + "control/" + "stop_motion",  1000, &Modules::stop_motion	, &lwa);
		
		*/
		
	
}
