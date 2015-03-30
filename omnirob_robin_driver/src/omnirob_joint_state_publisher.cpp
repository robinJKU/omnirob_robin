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
#include "std_msgs/Float64MultiArray.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/JointState.h"


sensor_msgs::JointState lwa_joint_state;
sensor_msgs::JointState pan_tilt_joint_state;
sensor_msgs::JointState gripper_joint_state;

ros::Publisher joint_state_publisher;

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

int main(int argc, char **argv)
{
  //  init node
  ros::init(argc, argv, "omnirob_joint_state_publisher");
  ros::NodeHandle n;
  
  // init subscriber and msg
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
  
  std::string pan_tilt_prefix = "pan_tilt/";
  pan_tilt_joint_state.name.resize(2);
  pan_tilt_joint_state.name[0] = pan_tilt_prefix + "pan_joint";
  pan_tilt_joint_state.name[1] = pan_tilt_prefix + "tilt_joint";
  
  pan_tilt_joint_state.position.resize(2);
  pan_tilt_joint_state.velocity.resize(2);
  pan_tilt_joint_state.effort.resize(2);
  
  ros::Subscriber pan_tilt_subscriber = n.subscribe("/omnirob_robin/pan_tilt/state/joint_state_array", 1000, pan_tilt_callback);
  
  std::string gripper_prefix = "gripper/";
  gripper_joint_state.name.resize(2);
  gripper_joint_state.name[0] = gripper_prefix + "finger_right_joint";
  gripper_joint_state.name[1] = gripper_prefix + "finger_left_joint";
  
  gripper_joint_state.position.resize(2);
  gripper_joint_state.velocity.resize(2);
  gripper_joint_state.effort.resize(2);
  
  ros::Subscriber gripper_subscriber = n.subscribe("/omnirob_robin/gripper/state/joint_state_array", 1000, gripper_callback);
  
  // init publisher and start loop
  joint_state_publisher = n.advertise<sensor_msgs::JointState>("/joint_states", 1000);
  
  ros::spin();

  return 0;

}
