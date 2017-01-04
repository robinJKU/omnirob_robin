#include <ros/ros.h>
#include <sstream> 
#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>

using namespace std;
int cnt=0;

ofstream myfile("/home/omnirob/catkin_ws/src/omnirob_robin/omnirob_robin_scan_matcher/data/save_global_plan.txt", ios::out);

void LineFilterNode_callback (const nav_msgs::Path::ConstPtr& _msg) {

        cout<<"sequence"<<_msg->header.seq<<endl;
	myfile.precision(17);
        myfile <<_msg->header.seq<<"\n";
        myfile <<_msg->header.stamp.nsec<<"\n";
        int n=_msg->poses.size();
        myfile <<n<<"\n";
        for (int i=0;i<n;i++){
	myfile <<_msg->poses[i].pose.position.x<<"\n";
        myfile <<_msg->poses[i].pose.position.y<<"\n";
        double yaw=tf::getYaw(_msg->poses[i].pose.orientation);
        myfile <<yaw<<"\n";
        }
        
    }



int main( int argc, char** argv) {

	ros::init(argc, argv, "save_global_plan_node");
	ros::NodeHandle n;
        ros::Subscriber sub = n.subscribe("move_base/GlobalPlanner/plan", 1, LineFilterNode_callback);

	while (ros::ok()){
	ros::spinOnce();
	}
        myfile.close();
}
