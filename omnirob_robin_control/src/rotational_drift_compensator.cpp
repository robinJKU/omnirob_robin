#include <ros/ros.h>

#include<geometry_msgs/Twist.h>
#include<sensor_msgs/Imu.h>

double measured_angular_velocity=0.0;
ros::Publisher cmd_vel_publisher;
double p_value=10.0;

void cmd_vel_callback( geometry_msgs::Twist desired_twist ){
	geometry_msgs::Twist controlled_twist;
	controlled_twist = desired_twist;
	controlled_twist.angular.z = desired_twist.angular.z + p_value*(desired_twist.angular.z-measured_angular_velocity);
	
	cmd_vel_publisher.publish( controlled_twist);
}// cmd vel callback

void imu_callback( sensor_msgs::Imu data ){
	measured_angular_velocity = -data.angular_velocity.z;
}// imu callback

int main( int argc, char** argv) {

	// init ros node
	ros::init(argc, argv, "rotational_drift_compensator");
	ros::NodeHandle handle;
	
	// get control parameter
	if( handle.hasparam("/omnirob_robin/base/drives/ang_vel_control_p_value") ){
		handle.getparam("/omnirob_robin/base/drives/ang_vel_control_p_value", p_value)
	}
	
	// init subscriber
	ros::Subscriber cmd_vel_controlled_subscriber = handle.subscribe("/omnirob_robin/base/drives/control/cmd_vel_controlled", 10, cmd_vel_callback );
	ros::Subscriber imu_subscriber = handle.subscribe("/imu/data", 10, imu_callback );
	
	// init publisher
	cmd_vel_publisher = handle.advertise<geometry_msgs::Twist>("/omnirob_robin/base/drives/control/cmd_vel", 10 );
	
	ros::spin();
	
}// main
