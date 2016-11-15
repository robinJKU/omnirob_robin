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

using namespace std;

bool ver=true;
tf::Transform map_to_base_;
void LaserLineFilterNode_callback (const sensor_msgs::LaserScan::ConstPtr& _msg);
//bool getMapToBase ();

void LaserLineFilterNode_callback (const sensor_msgs::LaserScan::ConstPtr& _msg) {


  	ros::Time t = ros::Time::now();

	tf::TransformListener tf_listener_;
  	tf::StampedTransform map_to_base;
	bool success = false;

	while (!success) {
  		try{
    		tf_listener_.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));
    		tf_listener_.lookupTransform ("map", "base_link", ros::Time(0), map_to_base);
		success = true;
  		}
  		catch (tf::TransformException ex){
    		ROS_WARN("Could not get initial transform from odom to base frame, %s", ex.what());
  		}
	}

  	map_to_base_=map_to_base;

        ofstream myfile("/home/aless/catkin_ws/src/omnirob_robin/omnirob_robin_scan_matcher/data/scan_ref.yaml", ios::out);
	myfile.precision(17);
	myfile << "header:\n";
	myfile << "  seq: "<<_msg->header.seq<<endl;
	myfile << "  stamp: "<<endl;
	myfile << "    secs: "<<_msg->header.stamp.sec<<endl;
	myfile << "    nsecs: "<<_msg->header.stamp.nsec<<endl;
	myfile << "  frame_id: "<<_msg->header.frame_id<<endl;
        myfile << "angle_min: "<<_msg->angle_min<<endl;
        myfile << "angle_max: "<<_msg->angle_max<<endl;
        myfile << "angle_increment: "<<_msg->angle_increment<<endl;
        myfile << "time_increment: "<<_msg->time_increment<<endl;
        myfile << "scan_time: "<<_msg->scan_time<<endl;
        myfile << "range_min: "<<_msg->range_min<<endl;
        myfile << "range_max: "<<_msg->range_max<<endl;
	myfile << "ranges: [";

	for (int i=0;i<(_msg->ranges.size()-1);i++){
	
		myfile <<_msg->ranges[i]<<", ";

	}

	myfile <<_msg->ranges[_msg->ranges.size()-1]<<"]"<<endl;


	myfile << "intensities: [";

	for (int i=0;i<(_msg->intensities.size()-1);i++){
	
		myfile <<_msg->intensities[i]<<", ";

	}

	myfile <<_msg->intensities[_msg->intensities.size()-1]<<"]"<<endl;

	myfile <<"position_frame: "<<endl;
	myfile <<"  translation: ["<<map_to_base_.getOrigin().getX()<<", "<<map_to_base_.getOrigin().getY()<<", "<<map_to_base_.getOrigin().getZ()<<"]"<<endl;

	myfile <<"  rotation: ["<<map_to_base_.getRotation().getX()<<", "<<map_to_base_.getRotation().getY()<<", "<<map_to_base_.getRotation().getZ()<<", "<<map_to_base_.getRotation().getW()<<"]"<<endl;


        myfile.close();
	ver=false;
    }



int main( int argc, char** argv) {

	ros::init(argc, argv, "write_scan_node");
	ros::AsyncSpinner spinner(4); // required because both the server and the client run in the same node
	spinner.start();
	ros::NodeHandle n;
        ros::Subscriber sub = n.subscribe("laser_front/scan", 1, LaserLineFilterNode_callback);

	while (ver){
	ros::spinOnce();
	}
}

