/**
 * @file   omnirob_joint_state_publisher.cpp
 * @Author Christoph Stöger
 * @date   März, 2015
 * @brief  Publishs the joint state of the robot.
 *
 * This node read in sensor data from the driver and publish it to
 * the /joint_state topic.
 */
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"

double x = 0.0;
double y = 0.0;
double yaw = 0.0, yaw0;
double yaw_imu = 0.0, offset_imu=0.0;
double vx = 0.0;
double vy = 0.0;
double omegaz = 0.0;
double omegaz_imu = 0.0;

bool no_imu_data_received = true;

sensor_msgs::JointState lwa_joint_state;
sensor_msgs::JointState pan_tilt_joint_state;
sensor_msgs::JointState gripper_joint_state;
sensor_msgs::JointState base_joint_state;
nav_msgs::Odometry odom;

ros::Publisher joint_state_publisher;
ros::Publisher odometry_publisher;
ros::Publisher base_state_ready_publisher;
ros::Publisher base_state_fault_publisher;

ros::Time current_time, last_time;

void lwa_callback( std_msgs::Float64MultiArray joint_state_array ){
	lwa_joint_state.header.stamp = ros::Time::now();
	
	// fill joint state
	unsigned int nr_of_modules = 7;
	for( unsigned int module=0; module<nr_of_modules; module++){
		lwa_joint_state.position[module] = joint_state_array.data[module];
	}
	
	// publish joint state
	joint_state_publisher.publish( lwa_joint_state);
	
	return;
}

void pan_tilt_callback( std_msgs::Float64MultiArray joint_state_array ){
	pan_tilt_joint_state.header.stamp = ros::Time::now();
	
	unsigned int nr_of_modules=2;
	for( unsigned int module=0; module<nr_of_modules; module++){
		pan_tilt_joint_state.position[module] = joint_state_array.data[module];
	}
	
	joint_state_publisher.publish( pan_tilt_joint_state);
	
	return;
}

void gripper_callback( std_msgs::Float64MultiArray joint_state_array ){
	gripper_joint_state.header.stamp	 = ros::Time::now();
	
	// fill joint state
	unsigned int nr_of_modules=1;
	for( unsigned int module=0; module<nr_of_modules; module++){
		gripper_joint_state.position[module] = joint_state_array.data[module]/2.0;
		gripper_joint_state.position[module+1] = joint_state_array.data[module]/2.0;
	}
	
	//publish joint state
	joint_state_publisher.publish( gripper_joint_state);
	
	return;
}

void base_callback( const geometry_msgs::Twist base_vel ){
	base_joint_state.header.stamp 	= ros::Time::now();
	last_time = current_time;
	current_time = base_joint_state.header.stamp;
	
	// fill joint state
	vx = base_vel.linear.x;
	vy = base_vel.linear.y;
	omegaz = base_vel.angular.z;  
	
	double dt = (current_time - last_time).toSec();
    double I_vx = (vx * cos(yaw) - vy * sin(yaw));
    double I_vy = (vx * sin(yaw) + vy * cos(yaw));
    
    double delta_x = I_vx * dt;
    double delta_y = I_vy * dt;
    double delta_yaw = omegaz * dt;

    x += delta_x ;
    y += delta_y ;
    yaw += delta_yaw;    
    
    base_joint_state.position[0] = x;
	base_joint_state.position[1] = y;
	base_joint_state.position[2] = yaw_imu;
	
	base_joint_state.velocity[0] = I_vx;
	base_joint_state.velocity[1] = I_vy;
	base_joint_state.velocity[2] = omegaz_imu;
 	
 	// publish odometry for navigation stack
 	odom.header.stamp = current_time;
 	
 	odom.header.frame_id = "odom";
 	odom.pose.pose.position.x = x;
 	odom.pose.pose.position.y = y;
 	odom.pose.pose.position.z = 0.0;
 	odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw (0.0, 0.0, yaw_imu);
 	
 	odom.child_frame_id = "base_link";
 	odom.twist.twist.linear.x = vx;
 	odom.twist.twist.linear.y = vy;
 	odom.twist.twist.linear.z = 0.0;
 	odom.twist.twist.angular.x = 0.0;
 	odom.twist.twist.angular.y = 0.0;
 	odom.twist.twist.angular.z = omegaz_imu;
 	
 	odometry_publisher.publish( odom );
 	
 	return;
 	
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
	
	base_state_ready_publisher.publish(  motors_enabled_and_ready_msgs);
	base_state_fault_publisher.publish( fault_msgs);
		
}// base state callback

void imu_callback( sensor_msgs::Imu data ){
	tf::Quaternion qt;
	tf::quaternionMsgToTF ( data.orientation, qt);
	
	yaw_imu = qt.getAngle() + offset_imu;
	omegaz_imu = data.angular_velocity.z;
	
	if( no_imu_data_received ){
		// offset callibration
		offset_imu =  yaw0 - yaw_imu;
		yaw_imu = yaw0;
		no_imu_data_received = true;
	}
	
}// imu callback


int main(int argc, char **argv)
{
  //  init node
  ros::init(argc, argv, "omnirob_joint_state_publisher");
  ros::NodeHandle n;
  
  // init subscriber and msg - LWA
  std::string lwa_prefix = "lwa/";
  lwa_joint_state.name.resize(7);
  lwa_joint_state.name[0] = lwa_prefix + "joint_1";
  lwa_joint_state.name[1] = lwa_prefix + "joint_2";
  lwa_joint_state.name[2] = lwa_prefix + "joint_3";
  lwa_joint_state.name[3] = lwa_prefix + "joint_4";
  lwa_joint_state.name[4] = lwa_prefix + "joint_5";
  lwa_joint_state.name[5] = lwa_prefix + "joint_6";
  lwa_joint_state.name[6] = lwa_prefix + "joint_7";
  
  lwa_joint_state.position.resize(7);
  lwa_joint_state.velocity.resize(7);
  lwa_joint_state.effort.resize(7);
  
  ros::Subscriber lwa_subscriber = n.subscribe("/omnirob_robin/lwa/state/joint_state_array", 1000, lwa_callback);
  
  // init subscriber and msg - PAN TILT
  std::string pan_tilt_prefix = "pan_tilt/";
  pan_tilt_joint_state.name.resize(2);
  pan_tilt_joint_state.name[0] = pan_tilt_prefix + "pan_joint";
  pan_tilt_joint_state.name[1] = pan_tilt_prefix + "tilt_joint";
  
  pan_tilt_joint_state.position.resize(2);
  pan_tilt_joint_state.velocity.resize(2);
  pan_tilt_joint_state.effort.resize(2);
  
  ros::Subscriber pan_tilt_subscriber = n.subscribe("/omnirob_robin/pan_tilt/state/joint_state_array", 1000, pan_tilt_callback);
  
  // init subscriber and msg - GRIPPER
  std::string gripper_prefix = "gripper/";
  gripper_joint_state.name.resize(2);
  gripper_joint_state.name[0] = gripper_prefix + "finger_right_joint";
  gripper_joint_state.name[1] = gripper_prefix + "finger_left_joint";
  
  gripper_joint_state.position.resize(2);
  gripper_joint_state.velocity.resize(2);
  gripper_joint_state.effort.resize(2);
  
  ros::Subscriber gripper_subscriber = n.subscribe("/omnirob_robin/gripper/state/joint_state_array", 1000, gripper_callback);
  
  // init subscriber and msg - BASE
  std::string base_prefix = "base/";
  base_joint_state.name.resize(3);
  base_joint_state.name[0] = base_prefix + "x_joint";
  base_joint_state.name[1] = base_prefix + "y_joint";
  base_joint_state.name[2] = base_prefix + "yaw_joint";
  
  base_joint_state.position.resize(3);
  base_joint_state.velocity.resize(3);
  base_joint_state.effort.resize(3);
  
  // read parameters
  if(  n.hasParam( "/omnirob_robin/base/odometry/x0" ) ){
	n.getParam( "/omnirob_robin/base/odometry/x0", x);
  }
  if(  n.hasParam( "/omnirob_robin/base/odometry/y0" ) ){
	n.getParam( "/omnirob_robin/base/odometry/y0", y);
  }
  if(  n.hasParam( "/omnirob_robin/base/odometry/yaw0" ) ){
	n.getParam( "/omnirob_robin/base/odometry/yaw0", yaw0);
	yaw = yaw0;
  }
  
  std::vector<double> covariance;
  covariance.resize(36);
  if( n.hasParam( "/omnirob_robin/base/odometry/pose_covariance" ) ){
	n.getParam( "/omnirob_robin/base/odometry/pose_covariance", covariance);
	for(unsigned int i=0; i<36; i++){
		odom.pose.covariance[i] = (double) covariance[i];
	}
  }else{
	ROS_WARN("pose covariance not specified, use default values");
	odom.pose.covariance[0] = 1e3;
	odom.pose.covariance[1+6] = 1e3;
	odom.pose.covariance[2+2*6] = 1e6;
	odom.pose.covariance[3+3*6] = 1e6;
	odom.pose.covariance[4+4*6] = 1e6;
	odom.pose.covariance[5+5*6] = 1e-3;
  }
  if( n.hasParam( "/omnirob_robin/base/odometry/twist_covariance") ){
	n.getParam( "/omnirob_robin/base/odometry/twist_covariance", covariance);
	for( unsigned int i=0; i<36; i++){
		odom.twist.covariance[i] = (double) covariance[i];
	}
  }else{
	ROS_WARN("velocity covariance not specified, use default values");
	odom.twist.covariance[0] = 1e3;
	odom.twist.covariance[1+6] = 1e3;
	odom.twist.covariance[2+2*6] = 1e6;
	odom.twist.covariance[3+3*6] = 1e6;
	odom.twist.covariance[4+4*6] = 1e6;
	odom.twist.covariance[5+5*6] = 1e-3;
  }
 	
  
  ros::Subscriber base_subscriber = n.subscribe( "/omnirob_robin/base/drives/state/state_word", 1000, base_state_callback );
  ros::Subscriber base_state_subscriber = n.subscribe( "/omnirob_robin/base/drives/state/vel", 1000, base_callback );
  ros::Subscriber imu_subscriber = n.subscribe( "/imu/data", 1000, imu_callback );

  // init publisher and start loop
  joint_state_publisher = n.advertise<sensor_msgs::JointState>("/joint_states", 1000);
  odometry_publisher = n.advertise<nav_msgs::Odometry>("/odom", 1000);
  base_state_ready_publisher = n.advertise<std_msgs::Bool>("/base/drives/state/info/motors_ready_end_enabled", 1000);
  base_state_fault_publisher = n.advertise<std_msgs::Bool>("/base/drives/state/error/motors_have_error", 1000);
  
  current_time = ros::Time::now();
  
  ros::spin();

  return 0;

}
