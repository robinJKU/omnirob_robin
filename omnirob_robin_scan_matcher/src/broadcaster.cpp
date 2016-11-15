#include <ros/ros.h>
#include <sstream> 
#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "BroadCaster");

  ros::NodeHandle n;

  std::vector <double> translation,rotation;

  ros::param::get("/position_frame/translation",translation);
  ros::param::get("/position_frame/rotation",rotation);

  std::cout<<translation[0]<<std::endl;
  std::cout<<translation[1]<<std::endl;
  std::cout<<translation[2]<<std::endl;

  std::cout<<rotation[0]<<std::endl;
  std::cout<<rotation[1]<<std::endl;
  std::cout<<rotation[2]<<std::endl;
  std::cout<<rotation[3]<<std::endl;

  tf::Vector3 origin (translation[0],translation[1],translation[2]);
  tf::Quaternion q (rotation[0],rotation[1],rotation[2],rotation[3]);
  tf::TransformBroadcaster broadcaster;
  tf::Transform base_to_target_pose;
  base_to_target_pose.setOrigin(origin);
  base_to_target_pose.setRotation(q);

	ros::Rate r(50);
	while(ros::ok){

		broadcaster.sendTransform(tf::StampedTransform(base_to_target_pose, ros::Time::now(), "map", "perfect_goal"));
		r.sleep();
		ros::spinOnce();
	}

  return 0;
}
