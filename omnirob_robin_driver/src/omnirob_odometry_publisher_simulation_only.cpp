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
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "sensor_msgs/JointState.h"

nav_msgs::Odometry odom;
ros::Publisher odometry_publisher;

void base_callback( const sensor_msgs::JointState joint_state ){
	
	// get index of base variables
	int index_x=-1, index_y=-1, index_yaw=-1;
	for( unsigned int ii=0; ii<joint_state.name.size(); ii++ ){
		if( joint_state.name[ii].compare("base_x_joint")==0 ){
			index_x=ii;
		}else if( joint_state.name[ii].compare("base_y_joint")==0 ){
			index_y=ii;
		}else if( joint_state.name[ii].compare("base_yaw_joint")==0 ){
			index_yaw=ii;
		}
		
		if( (index_x>=0) && (index_y>=0) && (index_yaw>=0) ){
			// everything found
			break;
		}
	}
	
	if( (index_x<0) || (index_y<0) || (index_yaw<0) ){
		ROS_ERROR("can't find all base joints in joint state");
		return;
	}
	
	// publish odometry for navigation stack
 	odom.header.stamp = ros::Time::now();
 	
 	odom.header.frame_id = "odom";
 	odom.pose.pose.position.x = joint_state.position[index_x];
 	odom.pose.pose.position.y = joint_state.position[index_y];
 	odom.pose.pose.position.z = 0.0;
 	odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw (0.0, 0.0, joint_state.position[index_yaw]);
 	
 	odom.child_frame_id = "odom";
 	odom.twist.twist.linear.x = joint_state.velocity[index_x];
 	odom.twist.twist.linear.y = joint_state.velocity[index_y];
 	odom.twist.twist.linear.z = 0.0;
 	odom.twist.twist.angular.x = 0.0;
 	odom.twist.twist.angular.y = 0.0;
 	odom.twist.twist.angular.z = joint_state.velocity[index_yaw];
 	
 	odometry_publisher.publish( odom );
 	
 	return;
 	
}

int main(int argc, char **argv)
{
  //  init node
  ros::init(argc, argv, "omnirob_odometry_publisher");
  ros::NodeHandle n;
  
  // read parameters
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
 	
  
  ros::Subscriber base_subscriber = n.subscribe( "/omnirob_robin/joint_states", 1000, base_callback );

  // init publisher and start loop
  odometry_publisher = n.advertise<nav_msgs::Odometry>("/odom", 1000);
  
  ros::spin();

  return 0;

}
