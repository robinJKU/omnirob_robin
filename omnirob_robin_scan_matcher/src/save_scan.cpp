#include <ros/ros.h>
#include <sstream> 
#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

using namespace std;
int cnt=0;

tf::Transform map_to_base_;
ofstream myfile("/home/aless/catkin_ws/src/omnirob_robin/omnirob_robin_scan_matcher/data/save_scan.txt", ios::out);

void LaserLineFilterNode_callback (const sensor_msgs::LaserScan::ConstPtr& _msg) {

        cout<<"numbero ranges"<<_msg->ranges.size()<<endl;
        int n=_msg->ranges.size();
        int n_half;
        if (n%2==0){
		n_half=n/2;
	}else{
		n_half=(n-1)/2;
	}

        cout<<"numbero half"<<n_half<<endl;
  	ros::Time t = ros::Time::now();
	myfile.precision(17);
	myfile <<_msg->ranges[n_half]<<"\n";
        cnt++;
        
    }



int main( int argc, char** argv) {

	ros::init(argc, argv, "save_scan_node");
	ros::NodeHandle n;
        ros::Subscriber sub = n.subscribe("laser_front/scan", 1, LaserLineFilterNode_callback);

	while (ros::ok()){
	ros::spinOnce();
	}
        myfile.close();
}
