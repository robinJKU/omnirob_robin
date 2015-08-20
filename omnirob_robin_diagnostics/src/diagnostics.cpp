#include <iostream>

#include "ros/ros.h"

#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"

#include "diagnostic_msgs/DiagnosticArray.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_msgs/KeyValue.h"

enum DiagType {INFO, WARNING, ERROR};
enum MsgType {FLOAT, BOOL};

ros::Time last_ultrasonic_time;

struct diagnostic_message{
	diagnostic_message()
	{}

	diagnostic_message(std::string topic, std::string ns, std::string msg_ns, DiagType diag_type, MsgType  msg_type, int decimals, double correct_val){
		topic_ = topic;
		ns_ = ns;
		msg_ns_ = msg_ns;
		diag_type_ = diag_type;
		msg_type_ = msg_type;
		decimals_ = decimals;
		correct_val_ = correct_val;

	}
	std::string topic_;
	std::string ns_;
	std::string msg_ns_;
	DiagType diag_type_;
	MsgType  msg_type_;
	int decimals_;
	double correct_val_;
};


class module_diagnostics_processor{
public:
	module_diagnostics_processor()
	{}

	module_diagnostics_processor(diagnostic_message diag_msg)
	{
		diag_msg_ = diag_msg;

		topic = "/omnirob_robin/" + diag_msg_.ns_ + "/state/" + diag_msg_.msg_ns_ + "/" + diag_msg_.topic_;

		//ROS_INFO(topic.c_str());
		if(diag_msg_.msg_type_ == FLOAT){
			subscriber_ = node_handle_.subscribe<std_msgs::Float64MultiArray>(topic, 1, &module_diagnostics_processor::cb_float, this);
		} else {
			subscriber_ = node_handle_.subscribe<std_msgs::Bool>(topic, 1, &module_diagnostics_processor::cb_bool, this);
		}
		diagnostics_pub_ = node_handle_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);

	}

	~module_diagnostics_processor()
	{
		//spinner_.stop();
		//subscriber_.shutdown();
		//node_handle_.shutdown();
	}

	void cb_float(std_msgs::Float64MultiArray msg)
	{
		switch(diag_msg_.diag_type_){
			case INFO:
				publish_info(msg, diag_msg_.topic_, diag_msg_.ns_, diag_msg_.decimals_);
				break;
			case WARNING:
				publish_warning(msg, diag_msg_.topic_, diag_msg_.ns_);
				break;
			case ERROR:
				publish_error(msg, diag_msg_.topic_, diag_msg_.ns_, diag_msg_.correct_val_);
				break;
		}
	}

	void cb_bool(std_msgs::Bool msg)
	{
		std_msgs::Float64MultiArray msg2;
		if(msg.data){
			msg2.data.push_back(1);
		} else {
			msg2.data.push_back(0);
		}
		cb_float(msg2);
	}

private:

	void publish_info(std_msgs::Float64MultiArray msg, std::string name, std::string id, int decimals){
		publish_msg(msg, name, id, diagnostic_msgs::DiagnosticStatus::OK, 1, decimals);
	}

	void publish_warning(std_msgs::Float64MultiArray msg, std::string name, std::string id){
		publish_msg(msg, name, id, diagnostic_msgs::DiagnosticStatus::WARN, 0, 0);
	}

	void publish_error(std_msgs::Float64MultiArray msg, std::string name, std::string id, int correct_val){
		publish_msg(msg, name, id, diagnostic_msgs::DiagnosticStatus::ERROR, correct_val, 0);
	}

	void publish_msg(std_msgs::Float64MultiArray msg, std::string name, std::string id, int status, int correct_value, int decimals){
		diagnostic_msgs::DiagnosticArray diag_arr;
		diag_arr.header.stamp = ros::Time::now();
		diagnostic_msgs::DiagnosticStatus diag_status;
		diag_status.level = diagnostic_msgs::DiagnosticStatus::OK;
		diag_status.name = id + "_" + name;
		diag_status.hardware_id = id;
		bool has_errors = false;
		std::stringstream message;
		message << "[";
		for(int i = 0; i < msg.data.size(); i++) {
			if(msg.data[i] < 0.5 ^ correct_value != 1){
				has_errors = true;
			}
			diagnostic_msgs::KeyValue key_val;
			std::stringstream ss;
			ss << diag_status.name << "_" << i;
			key_val.key = ss.str();
			ss.str(std::string());
			ss << std::fixed << std::setprecision( decimals ) << msg.data[i];
			message << std::fixed << std::setprecision( decimals ) <<  msg.data[i];
			if(i < msg.data.size() -1){
				message << ", ";
			}
			key_val.value = ss.str();
			diag_status.values.push_back(key_val);
		}
		message << "]";
		diag_status.message = message.str();
		if(has_errors){
			diag_status.level = status;
		}

		diag_arr.status.push_back(diag_status);
		diagnostics_pub_.publish(diag_arr);
	}

	ros::NodeHandle node_handle_;

	ros::Subscriber subscriber_;
	ros::Publisher diagnostics_pub_;

	diagnostic_message diag_msg_;
	std::string topic;
}; //end of processor class

// global variables


ros::Publisher diagnostics_pub;

// Base callback functions
void cb_base_canserver_server_state(std_msgs::Float64 msg) {
	diagnostic_msgs::DiagnosticArray diag_arr;
	diag_arr.header.stamp = ros::Time::now();
	diagnostic_msgs::DiagnosticStatus diag_status;
	diag_status.level = diagnostic_msgs::DiagnosticStatus::OK;
	diag_status.name = "base_canserver_state";
	diag_status.message = "";
	diag_status.hardware_id = "base";
	diagnostic_msgs::KeyValue key_val;
	key_val.key = diag_status.name;
	std::stringstream ss;
	ss << (int) msg.data;
	key_val.value = ss.str();
	diag_status.values.push_back(key_val);
	diag_arr.status.push_back(diag_status);
	diagnostics_pub.publish(diag_arr);
}

void cb_base_drives_state_vel(geometry_msgs::Twist msg) {
	diagnostic_msgs::DiagnosticArray diag_arr;
	diag_arr.header.stamp = ros::Time::now();
	diagnostic_msgs::DiagnosticStatus diag_status;
	diag_status.level = diagnostic_msgs::DiagnosticStatus::OK;
	diag_status.name = "base_drives_state_vel";
	diag_status.message = "";
	diag_status.hardware_id = "base";
	diagnostic_msgs::KeyValue key_val;
	
	std::stringstream ss;
	ss << diag_status.name << "_linear_x";
	key_val.key = ss.str();
	ss.str(std::string());
	ss << msg.linear.x;
	key_val.value = ss.str();
	diag_status.values.push_back(key_val);
	
	ss << diag_status.name << "_linear_y";
	key_val.key = ss.str();
	ss.str(std::string());
	ss << msg.linear.y;
	key_val.value = ss.str();
	diag_status.values.push_back(key_val);
	
	ss << diag_status.name << "_linear_z";
	key_val.key = ss.str();
	ss.str(std::string());
	ss << msg.linear.z;
	key_val.value = ss.str();
	diag_status.values.push_back(key_val);
	
	ss << diag_status.name << "_angular_x";
	key_val.key = ss.str();
	ss.str(std::string());
	ss << msg.angular.x;
	key_val.value = ss.str();
	diag_status.values.push_back(key_val);
	
	ss << diag_status.name << "_angular_y";
	key_val.key = ss.str();
	ss.str(std::string());
	ss << msg.angular.y;
	key_val.value = ss.str();
	diag_status.values.push_back(key_val);
	
	ss << diag_status.name << "_angular_z";
	key_val.key = ss.str();
	ss.str(std::string());
	ss << msg.angular.z;
	key_val.value = ss.str();
	diag_status.values.push_back(key_val);
	diag_arr.status.push_back(diag_status);
	diagnostics_pub.publish(diag_arr);
}

void base_state_callback( std_msgs::Float64MultiArray state_word ){

	// check if all motors are enabled and ready
	unsigned int switched_on, opeartion_mode_enabled, voltage_enabled;
	bool motors_enabled_and_ready = true;
	for( unsigned int motor=0; motor<4; motor++){
		switched_on = ( ((unsigned int) state_word.data[motor]) & 0x2);
		opeartion_mode_enabled = ( ((unsigned int) state_word.data[motor]) & 0x4);
		voltage_enabled = ( ((unsigned int) state_word.data[motor]) & 0x16);

		if( switched_on==0 || opeartion_mode_enabled==0 || voltage_enabled==0 ){
			motors_enabled_and_ready = false;
			break;
		}
	}

	// check if the motor has errors
	bool fault;
	for( unsigned int motor=0; motor<4; motor++){
		fault = fault || (( ((unsigned int) state_word.data[motor]) & 0x8)>0.0);
	}

	// publish results
	std_msgs::Bool motors_enabled_and_ready_msgs, fault_msgs;
	motors_enabled_and_ready_msgs.data = motors_enabled_and_ready;
	fault_msgs.data = fault;

	//base_state_ready_publisher.publish( motors_enabled_and_ready_msgs);
	//base_state_fault_publisher.publish( fault_msgs);

}// base state callback

void cb_ultrasonic(sensor_msgs::Range msg){
	last_ultrasonic_time = ros::Time::now();
}

void publish_ultrasonic(){
	diagnostic_msgs::DiagnosticArray diag_arr;
	diag_arr.header.stamp = ros::Time::now();
	diagnostic_msgs::DiagnosticStatus diag_status;
	diag_status.name = "ultrasonic status";
	diag_status.hardware_id = "ultrasonic";
	if(ros::Time::now().toSec()-last_ultrasonic_time.toSec() < 5){
		diag_status.level = diagnostic_msgs::DiagnosticStatus::OK;
		diag_status.message = "OK";
	} else {
		diag_status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
		diag_status.message = "No messages received";
	}

	diag_arr.status.push_back(diag_status);
	diagnostics_pub.publish(diag_arr);
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "omnirob_robin_diagnostics");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	// diagnostics publisher
	diagnostics_pub = n.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);

	// Base subscribers
	ros::Subscriber sub_base_canserver_state = n.subscribe<std_msgs::Float64>("/omnirob_robin/base/canserver/server_state", 1000, cb_base_canserver_server_state);
	//ros::Subscriber sub_base_canserver_is_ready = n.subscribe<std_msgs::Bool>("/omnirob_robin/base/canserver/server_is_ready", 1000, cb_base_canserver_server_is_ready);
	ros::Subscriber sub_base_drives_state_vel = n.subscribe<geometry_msgs::Twist>("/omnirob_robin/base/drives/state/vel", 1000, cb_base_drives_state_vel);

	ros::Subscriber sub_ultrasonic = n.subscribe<sensor_msgs::Range>("/Ultrasound/Sensor0", 1000, cb_ultrasonic);

	std::vector<diagnostic_message> diag_messages;
	std::vector<module_diagnostics_processor*> module_processors;

	//base diagnostic messages
	//diag_messages.push_back(diagnostic_message("motor_ready_to_switch_on", "base", "info", ERROR, FLOAT, 0, 1));
	diag_messages.push_back(diagnostic_message("motor_switched_on", "base", "info", ERROR, FLOAT, 0, 1));
	diag_messages.push_back(diagnostic_message("motor_operation_enabled", "base", "info", INFO, FLOAT, 0, 1));
	diag_messages.push_back(diagnostic_message("motor_has_errors", "base", "error", ERROR, FLOAT, 0, 0));
	diag_messages.push_back(diagnostic_message("motor_has_warnings", "base", "info", WARNING, FLOAT, 0, 1));


	//different messages for gripper lwa and pan_tilt
	diag_messages.push_back(diagnostic_message("move_blocked", "gripper", "error", INFO, FLOAT, 0, 0));
	diag_messages.push_back(diagnostic_message("move_blocked", "lwa", "error", ERROR, FLOAT, 0, 0));
	//diag_messages.push_back(diagnostic_message("move_blocked", "pan_tilt", "error", ERROR, FLOAT, 0, 0));

	for(int i = 0; i < diag_messages.size(); i++){
		module_processors.push_back(new module_diagnostics_processor(diag_messages[i]));
	}


	diag_messages.clear();
	//this diag messages are the same for gripper, lwa and pan_tilt

	diag_messages.push_back(diagnostic_message("initialization_error", "gripper", "error", ERROR, FLOAT, 0, 0));
	diag_messages.push_back(diagnostic_message("module_has_errors", "gripper", "error", ERROR, FLOAT, 0, 0));
	diag_messages.push_back(diagnostic_message("module_has_low_voltage", "gripper", "error", ERROR, FLOAT, 0, 0));
	diag_messages.push_back(diagnostic_message("module_is_in_emergency_stop", "gripper", "error", ERROR, FLOAT, 0, 0));
	diag_messages.push_back(diagnostic_message("module_not_referenced", "gripper", "error", ERROR, FLOAT, 0, 0));

	diag_messages.push_back(diagnostic_message("tow_error", "gripper", "error", ERROR, FLOAT, 0, 0));
	diag_messages.push_back(diagnostic_message("break_activated", "gripper", "info", INFO, FLOAT, 0, 0));
	diag_messages.push_back(diagnostic_message("continuos_position_tracking_mode_enabled", "gripper", "info", INFO, BOOL, 0, 0));
	diag_messages.push_back(diagnostic_message("point_to_point_motion_enabled", "gripper","info", INFO, BOOL, 0, 0));
	diag_messages.push_back(diagnostic_message("module_has_warnings", "gripper", "info", WARNING, FLOAT, 0, 0));
	diag_messages.push_back(diagnostic_message("module_in_motion", "gripper", "info", INFO, FLOAT, 0, 0));
	diag_messages.push_back(diagnostic_message("module_is_enabled", "gripper", "info", INFO, FLOAT, 0, 0));
	diag_messages.push_back(diagnostic_message("module_is_ready", "gripper", "info", ERROR, FLOAT, 1, 1));
	diag_messages.push_back(diagnostic_message("position_reached", "gripper", "info", INFO, FLOAT, 0, 0));
	diag_messages.push_back(diagnostic_message("joint_state_array", "gripper", "", INFO, FLOAT, 0, 2));
	diag_messages.push_back(diagnostic_message("module_not_responding", "gripper", "error", ERROR, FLOAT, 0, 0));

	for(int i = 0; i < diag_messages.size(); i++){
		module_processors.push_back(new module_diagnostics_processor(diag_messages[i]));
		diag_messages[i].ns_ = "lwa";
		module_processors.push_back(new module_diagnostics_processor(diag_messages[i]));
		//diag_messages[i].ns_ = "pan_tilt";
	    //module_processors.push_back(new module_diagnostics_processor(diag_messages[i]));
	}

	while( ros::ok())
	{
		loop_rate.sleep();
		publish_ultrasonic();
		ros::spinOnce();
	}
	// ros::spin();

	return 0;
}
