#include <iostream>

#include "ros/ros.h"

#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Twist.h"

#include "diagnostic_msgs/DiagnosticArray.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_msgs/KeyValue.h"

// global variables
ros::Publisher diagnostics_pub; // diagnostics publisher

void publish_msg(std_msgs::Float64MultiArray msg, std::string name, std::string id, int status, int correct_value){
	diagnostic_msgs::DiagnosticArray diag_arr;
	diag_arr.header.stamp = ros::Time::now();
	diagnostic_msgs::DiagnosticStatus diag_status;
	diag_status.level = diagnostic_msgs::DiagnosticStatus::OK;
	diag_status.name = name;
	diag_status.hardware_id = id;
	bool has_errors = false;
	std::stringstream message;
	message << "[";
	for(int i = 0; i < msg.data.size(); i++) {
		if(msg.data[i] > 0.5 && correct_value != 1){
			has_errors = true;
		}
		diagnostic_msgs::KeyValue key_val;
		std::stringstream ss;
		ss << diag_status.name << "_" << i;
		key_val.key = ss.str();
		ss.str(std::string());
		ss << (int) msg.data[i];
		message << (int) msg.data[i];
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
	diagnostics_pub.publish(diag_arr);
}

void publish_error(std_msgs::Float64MultiArray msg, std::string name, std::string id, int correct_val){
	publish_msg(msg, name, id, diagnostic_msgs::DiagnosticStatus::ERROR, correct_val);
}

void publish_warning(std_msgs::Float64MultiArray msg, std::string name, std::string id){
	publish_msg(msg, name, id, diagnostic_msgs::DiagnosticStatus::WARN, 0);
}

void publish_info(std_msgs::Float64MultiArray msg, std::string name, std::string id){
	publish_msg(msg, name, id, diagnostic_msgs::DiagnosticStatus::OK, 1);
}

void publish_info(std_msgs::Bool msg, std::string name, std::string id){
	std_msgs::Float64MultiArray msg2;
	if(msg.data){
		msg2.data.push_back(1);
	} else {
		msg2.data.push_back(0);
	}

	publish_msg(msg2, name, id, diagnostic_msgs::DiagnosticStatus::ERROR, 1);
}


// Base callback functions
void cb_base_canserver_state(std_msgs::Float64 msg) {
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

// Gripper callback functions
void cb_gripper_error_initialization_error(std_msgs::Float64MultiArray msg) {
	publish_error(msg, "gripper_initialization_error", "gripper", 0);
}

void cb_gripper_error_module_has_errors(std_msgs::Float64MultiArray msg) {
	publish_error(msg, "gripper_module_has_error", "gripper", 0);
}

void cb_gripper_error_module_has_low_voltage(std_msgs::Float64MultiArray msg) {
	publish_error(msg, "gripper_module_has_low_voltage", "gripper", 0);
}

void cb_gripper_error_module_is_in_emergency_stop(std_msgs::Float64MultiArray msg) {
	publish_error(msg, "gripper_module_is_in_emergency_stop", "gripper", 0);
}

void cb_gripper_error_module_not_referenced(std_msgs::Float64MultiArray msg) {
	publish_error(msg, "gripper_module_not_referenced", "gripper", 0);
}

void cb_gripper_error_move_blocked(std_msgs::Float64MultiArray msg) {
	publish_error(msg, "gripper_move_blocked", "gripper", 0);
}

void cb_gripper_error_tow_error(std_msgs::Float64MultiArray msg) {
	publish_error(msg, "gripper_tow_error", "gripper", 0);
}

void cb_gripper_info_break_activated(std_msgs::Float64MultiArray msg) {
	publish_info(msg, "gripper_break_activated", "gripper");
}

void cb_gripper_info_continuos_position_tracking_mode_enabled(std_msgs::Bool msg) {
	publish_info(msg, "gripper_continuos_position_tracking_mode_enabled", "gripper");
}

void cb_gripper_info_point_to_point_motion_enabled(std_msgs::Bool msg) {
	publish_info(msg, "gripper_point_to_point_motion_enabled", "gripper");
}

void cb_gripper_info_module_has_warnings(std_msgs::Float64MultiArray msg) {
	publish_warning(msg, "gripper_module_has_warnings", "gripper");
}

void cb_gripper_info_module_in_motion(std_msgs::Float64MultiArray msg) {
	publish_info(msg, "gripper_module_in_motion", "gripper");
}

void cb_gripper_info_module_is_enabled(std_msgs::Float64MultiArray msg) {
	publish_error(msg, "gripper_module_is_enabled", "gripper", 1);
}

void cb_gripper_info_module_is_ready(std_msgs::Float64MultiArray msg) {
	publish_error(msg, "gripper_module_is_ready", "gripper", 1);
}

void cb_gripper_info_position_reached(std_msgs::Float64MultiArray msg) {
	publish_info(msg, "gripper_position_reached", "gripper");
}

void cb_gripper_info_joint_state_array(std_msgs::Float64MultiArray msg) {
	publish_info(msg, "gripper_joint_state_array", "gripper");
}






// LWA callback functions
void cb_lwa_error_initialization_error(std_msgs::Float64MultiArray msg) {
	publish_error(msg, "lwa_initialization_error", "lwa", 0);
}

void cb_lwa_error_module_has_errors(std_msgs::Float64MultiArray msg) {
	publish_error(msg, "lwa_module_has_errors", "lwa", 0);
}

void cb_lwa_error_module_has_low_voltage(std_msgs::Float64MultiArray msg) {
	publish_error(msg, "lwa_module_has_low_voltage", "lwa", 0);
}

void cb_lwa_error_module_is_in_emergency_stop(std_msgs::Float64MultiArray msg) {
	publish_error(msg, "lwa_module_is_in_emergency_stop", "lwa", 0);
}

void cb_lwa_error_module_not_referenced(std_msgs::Float64MultiArray msg) {
	publish_error(msg, "lwa_module_not_referenced", "lwa", 0);
}

void cb_lwa_error_move_blocked(std_msgs::Float64MultiArray msg) {
	publish_error(msg, "lwa_move_blocked", "lwa", 0);
}

void cb_lwa_error_tow_error(std_msgs::Float64MultiArray msg) {
	publish_error(msg, "lwa_tow_error", "lwa", 0);
}

void cb_lwa_info_break_activated(std_msgs::Float64MultiArray msg) {
	publish_info(msg, "lwa_break_activated", "lwa");
}

void cb_lwa_info_continuos_position_tracking_mode_enabled(std_msgs::Bool msg) {
	publish_info(msg, "lwa_continuos_position_tracking_mode_enabled", "lwa");
}

void cb_lwa_info_point_to_point_motion_enabled(std_msgs::Bool msg) {
	publish_info(msg, "lwa_point_to_point_motion_enabled", "lwa");
}

void cb_lwa_info_module_has_warnings(std_msgs::Float64MultiArray msg) {
	publish_warning(msg, "lwa_module_has_warning", "lwa");
}

void cb_lwa_info_module_in_motion(std_msgs::Float64MultiArray msg) {
	publish_info(msg, "lwa_module_in_motion", "lwa");
}

void cb_lwa_info_module_is_enabled(std_msgs::Float64MultiArray msg) {
	publish_error(msg, "lwa_module_is_enabled", "lwa", 1);
}

void cb_lwa_info_module_is_ready(std_msgs::Float64MultiArray msg) {
	publish_error(msg, "lwa_module_is_ready", "lwa", 1);
}

void cb_lwa_info_position_reached(std_msgs::Float64MultiArray msg) {
	publish_info(msg, "lwa_position_reached", "lwa");
}

void cb_lwa_info_joint_state_array(std_msgs::Float64MultiArray msg) {
	publish_info(msg, "lwa_joint_state_array", "lwa");
}







// Pan-Tilt callback functions
void cb_pan_tilt_error_initialization_error(std_msgs::Float64MultiArray msg) {
	publish_error(msg, "pan_tilt_initialization_error", "pan_tilt", 0);
}

void cb_pan_tilt_error_module_has_errors(std_msgs::Float64MultiArray msg) {
	publish_error(msg, "pan_tilt_module_has_errors", "pan_tilt", 0);
}

void cb_pan_tilt_error_module_has_low_voltage(std_msgs::Float64MultiArray msg) {
	publish_error(msg, "pan_tilt_module_hase_low_voltage", "pan_tilt", 0);
}

void cb_pan_tilt_error_module_is_in_emergency_stop(std_msgs::Float64MultiArray msg) {
	publish_error(msg, "pan_tilt_module_is_in_mergency_stop", "pan_tilt", 0);
}

void cb_pan_tilt_error_module_not_referenced(std_msgs::Float64MultiArray msg) {
	publish_error(msg, "pan_tilt_module_not_referenced", "pan_tilt", 0);
}

void cb_pan_tilt_error_move_blocked(std_msgs::Float64MultiArray msg) {
	publish_error(msg, "pan_tilt_move_blocked", "pan_tilt", 0);
}

void cb_pan_tilt_error_tow_error(std_msgs::Float64MultiArray msg) {
	publish_error(msg, "pan_tilt_tow_error", "pan_tilt", 0);
}

void cb_pan_tilt_info_break_activated(std_msgs::Float64MultiArray msg) {
	publish_info(msg, "pan_tilt_break_activated", "pan_tilt");
}

void cb_pan_tilt_info_continuos_position_tracking_mode_enabled(std_msgs::Bool msg) {
	publish_info(msg, "pan_tilt_continuos_position_tracking_mode_enabled", "pan_tilt");
}

void cb_pan_tilt_info_point_to_point_motion_enabled(std_msgs::Bool msg) {
	publish_info(msg, "pan_tilt_point_to_point_motion_enabled", "pan_tilt");
}

void cb_pan_tilt_info_module_has_warnings(std_msgs::Float64MultiArray msg) {
	publish_warning(msg, "pan_tilt_module_has_warnings", "pan_tilt");
}

void cb_pan_tilt_info_module_in_motion(std_msgs::Float64MultiArray msg) {
	publish_info(msg, "pan_tilt_module_in_motion", "pan_tilt");
}

void cb_pan_tilt_info_module_is_enabled(std_msgs::Float64MultiArray msg) {
	publish_info(msg, "pan_tilt_module_is_enabled", "pan_tilt");
}

void cb_pan_tilt_info_module_is_ready(std_msgs::Float64MultiArray msg) {
	publish_error(msg, "pan_tilt_module_is_ready", "pan_tilt", 1);
}

void cb_pan_tilt_info_position_reached(std_msgs::Float64MultiArray msg) {
	publish_info(msg, "pan_tilt_position_reached", "pan_tilt");
}

void cb_pan_tilt_info_joint_state_array(std_msgs::Float64MultiArray msg) {
	publish_info(msg, "pan_tilt_joint_state_array", "pan_tilt");
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "omnirob_robin_diagnostics");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
	
	// diagnostics publisher
	diagnostics_pub = n.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1000);
	
	// Base subscribers
	ros::Subscriber sub_base_canserver_state = n.subscribe<std_msgs::Float64>("/omnirob_robin/base/canserver/state", 1000, cb_base_canserver_state);
	ros::Subscriber sub_base_drives_state_vel = n.subscribe<geometry_msgs::Twist>("/omnirob_robin/base/drives/state/vel", 1000, cb_base_drives_state_vel);
	
    // Gripper subscribers
	ros::Subscriber sub_gripper_error_initialization_error = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/gripper/state/error/initialization_error", 1000, cb_gripper_error_initialization_error);
	ros::Subscriber sub_gripper_error_module_has_errors = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/gripper/state/error/module_has_errors", 1000, cb_gripper_error_module_has_errors);
	ros::Subscriber sub_gripper_error_module_has_low_voltage = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/gripper/state/error/module_has_low_voltage", 1000, cb_gripper_error_module_has_low_voltage);
	ros::Subscriber sub_gripper_error_module_is_in_emergency_stop = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/gripper/state/error/module_is_in_emergency_stop", 1000, cb_gripper_error_module_is_in_emergency_stop);
	ros::Subscriber sub_gripper_error_module_not_referenced = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/gripper/state/error/module_not_referenced", 1000, cb_gripper_error_module_not_referenced );
	ros::Subscriber sub_gripper_error_move_blocked = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/gripper/state/error/move_blocked", 1000, cb_gripper_error_move_blocked );
	ros::Subscriber sub_gripper_error_tow_error = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/gripper/state/error/tow_error", 1000, cb_gripper_error_tow_error );
	ros::Subscriber sub_gripper_info_break_activated = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/gripper/state/info/break_activated", 1000, cb_gripper_info_break_activated);
	ros::Subscriber sub_gripper_info_continuos_position_tracking_mode_enabled = n.subscribe<std_msgs::Bool>("/omnirob_robin/gripper/state/info/continuos_position_tracking_mode_enabled", 1000, cb_gripper_info_continuos_position_tracking_mode_enabled);
	ros::Subscriber sub_gripper_info_point_to_point_motion_enabled = n.subscribe<std_msgs::Bool>("/omnirob_robin/gripper/state/info/point_to_point_motion_enabled", 1000, cb_gripper_info_point_to_point_motion_enabled);
	ros::Subscriber sub_gripper_info_module_has_warnings = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/gripper/state/info/module_has_warnings", 1000, cb_gripper_info_module_has_warnings);
	ros::Subscriber sub_gripper_info_module_in_motion = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/gripper/state/info/module_in_motion", 1000, cb_gripper_info_module_in_motion);
	ros::Subscriber sub_gripper_info_module_is_enabled = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/gripper/state/info/module_is_enabled", 1000, cb_gripper_info_module_is_enabled);
	ros::Subscriber sub_gripper_info_module_is_ready = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/gripper/state/info/module_is_ready", 1000, cb_gripper_info_module_is_ready);
	ros::Subscriber sub_gripper_info_position_reached = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/gripper/state/info/position_reached", 1000, cb_gripper_info_position_reached);
	ros::Subscriber sub_gripper_info_joint_state_array = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/gripper/state/info/joint_state_array", 1000, cb_gripper_info_joint_state_array);
	
    // LWA subscribers
	ros::Subscriber sub_lwa_error_initialization_error = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/lwa/state/error/initialization_error", 1000, cb_lwa_error_initialization_error);
	ros::Subscriber sub_lwa_error_module_has_errors = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/lwa/state/error/module_has_errors", 1000, cb_lwa_error_module_has_errors);
	ros::Subscriber sub_lwa_error_module_has_low_voltage = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/lwa/state/error/module_has_low_voltage", 1000, cb_lwa_error_module_has_low_voltage);
	ros::Subscriber sub_lwa_error_module_is_in_emergency_stop = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/lwa/state/error/module_is_in_emergency_stop", 1000, cb_lwa_error_module_is_in_emergency_stop);
	ros::Subscriber sub_lwa_error_module_not_referenced = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/lwa/state/error/module_not_referenced", 1000, cb_lwa_error_module_not_referenced );
	ros::Subscriber sub_lwa_error_move_blocked = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/lwa/state/error/move_blocked", 1000, cb_lwa_error_move_blocked );
	ros::Subscriber sub_lwa_error_tow_error = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/lwa/state/error/tow_error", 1000, cb_lwa_error_tow_error );
	ros::Subscriber sub_lwa_info_break_activated = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/lwa/state/info/break_activated", 1000, cb_lwa_info_break_activated);
	ros::Subscriber sub_lwa_info_continuos_position_tracking_mode_enabled = n.subscribe<std_msgs::Bool>("/omnirob_robin/lwa/state/info/continuos_position_tracking_mode_enabled", 1000, cb_lwa_info_continuos_position_tracking_mode_enabled);
	ros::Subscriber sub_lwa_info_point_to_point_motion_enabled = n.subscribe<std_msgs::Bool>("/omnirob_robin/lwa/state/info/point_to_point_motion_enabled", 1000, cb_lwa_info_point_to_point_motion_enabled);
	ros::Subscriber sub_lwa_info_module_has_warnings = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/lwa/state/info/module_has_warnings", 1000, cb_lwa_info_module_has_warnings);
	ros::Subscriber sub_lwa_info_module_in_motion = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/lwa/state/info/module_in_motion", 1000, cb_lwa_info_module_in_motion);
	ros::Subscriber sub_lwa_info_module_is_enabled = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/lwa/state/info/module_is_enabled", 1000, cb_lwa_info_module_is_enabled);
	ros::Subscriber sub_lwa_info_module_is_ready = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/lwa/state/info/module_is_ready", 1000, cb_lwa_info_module_is_ready);
	ros::Subscriber sub_lwa_info_position_reached = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/lwa/state/info/position_reached", 1000, cb_lwa_info_position_reached);
	ros::Subscriber sub_lwa_info_joint_state_array = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/lwa/state/info/joint_state_array", 1000, cb_lwa_info_joint_state_array);
	
    // Pan-tilt subscribers
	ros::Subscriber sub_pan_tilt_error_initialization_error = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/pan_tilt/state/error/initialization_error", 1000, cb_pan_tilt_error_initialization_error);
	ros::Subscriber sub_pan_tilt_error_module_has_errors = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/pan_tilt/state/error/module_has_errors", 1000, cb_pan_tilt_error_module_has_errors);
	ros::Subscriber sub_pan_tilt_error_module_has_low_voltage = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/pan_tilt/state/error/module_has_low_voltage", 1000, cb_pan_tilt_error_module_has_low_voltage);
	ros::Subscriber sub_pan_tilt_error_module_is_in_emergency_stop = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/pan_tilt/state/error/module_is_in_emergency_stop", 1000, cb_pan_tilt_error_module_is_in_emergency_stop);
	ros::Subscriber sub_pan_tilt_error_module_not_referenced = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/pan_tilt/state/error/module_not_referenced", 1000, cb_pan_tilt_error_module_not_referenced );
	ros::Subscriber sub_pan_tilt_error_move_blocked = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/pan_tilt/state/error/move_blocked", 1000, cb_pan_tilt_error_move_blocked );
	ros::Subscriber sub_pan_tilt_error_tow_error = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/pan_tilt/state/error/tow_error", 1000, cb_pan_tilt_error_tow_error );
	ros::Subscriber sub_pan_tilt_info_break_activated = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/pan_tilt/state/info/break_activated", 1000, cb_pan_tilt_info_break_activated);
	ros::Subscriber sub_pan_tilt_info_continuos_position_tracking_mode_enabled = n.subscribe<std_msgs::Bool>("/omnirob_robin/pan_tilt/state/info/continuos_position_tracking_mode_enabled", 1000, cb_pan_tilt_info_continuos_position_tracking_mode_enabled);
	ros::Subscriber sub_pan_tilt_info_point_to_point_motion_enabled = n.subscribe<std_msgs::Bool>("/omnirob_robin/pan_tilt/state/info/point_to_point_motion_enabled", 1000, cb_pan_tilt_info_point_to_point_motion_enabled);
	ros::Subscriber sub_pan_tilt_info_module_has_warnings = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/pan_tilt/state/info/module_has_warnings", 1000, cb_pan_tilt_info_module_has_warnings);
	ros::Subscriber sub_pan_tilt_info_module_in_motion = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/pan_tilt/state/info/module_in_motion", 1000, cb_pan_tilt_info_module_in_motion);
	ros::Subscriber sub_pan_tilt_info_module_is_enabled = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/pan_tilt/state/info/module_is_enabled", 1000, cb_pan_tilt_info_module_is_enabled);
	ros::Subscriber sub_pan_tilt_info_module_is_ready = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/pan_tilt/state/info/module_is_ready", 1000, cb_pan_tilt_info_module_is_ready);
	ros::Subscriber sub_pan_tilt_info_position_reached = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/pan_tilt/state/info/position_reached", 1000, cb_pan_tilt_info_position_reached);
	ros::Subscriber sub_pan_tilt_info_joint_state_array = n.subscribe<std_msgs::Float64MultiArray>("/omnirob_robin/pan_tilt/state/info/joint_state_array", 1000, cb_pan_tilt_info_joint_state_array);
	
	while(ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
