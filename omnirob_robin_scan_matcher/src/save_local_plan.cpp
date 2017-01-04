#include <ros/ros.h>
#include <sstream> 
#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include "tf/transform_listener.h"

using namespace std;
int cnt=0;
tf::TransformListener* pListener;
tf::StampedTransform from_map_to_odom, from_odom_to_base;

ofstream myfile("/home/omnirob/catkin_ws/src/omnirob_robin/omnirob_robin_scan_matcher/data/save_local_plan.txt", ios::out);

void LineFilterNode_callback (const nav_msgs::Path::ConstPtr& _msg) {

        cout<<"sequence"<<_msg->header.seq<<endl;
	myfile.precision(17);
        myfile <<_msg->header.seq<<"\n";
	myfile <<_msg->header.stamp.nsec<<"\n";

	try {
		pListener->waitForTransform("/map", _msg->header.frame_id, _msg->header.stamp, ros::Duration(3.0) );
		pListener->lookupTransform("/map", _msg->header.frame_id, _msg->header.stamp, from_map_to_odom);
		}
		catch(tf::TransformException e){
		ROS_INFO("Transform object frame error Error: %s", e.what());
		}


        int n=_msg->poses.size();
        myfile <<n<<"\n";
        for (int i=0;i<n;i++){

        from_odom_to_base.setOrigin(tf::Vector3(_msg->poses[i].pose.position.x, _msg->poses[i].pose.position.y, _msg->poses[i].pose.position.z));
	tf::Quaternion q1,q2;
	tf::quaternionMsgToTF(_msg->poses[i].pose.orientation,q1);
	from_odom_to_base.setRotation(q1);
	tf::Transform from_map_to_base;
        from_map_to_base=from_map_to_odom*from_odom_to_base;
  
	tf::Vector3 new_position=from_map_to_base.getOrigin();
        q2=from_map_to_base.getRotation();
	myfile <<new_position[0]<<"\n";
        myfile <<new_position[1]<<"\n";
        double yaw=tf::getYaw(q2);
        myfile <<yaw<<"\n";
        }
        
    }



int main( int argc, char** argv) {

	ros::init(argc, argv, "save_local_plan_node");
	ros::NodeHandle n;
        ros::Subscriber sub = n.subscribe("move_base/DWAPlannerROS/local_plan", 1, LineFilterNode_callback);
        pListener = new(tf::TransformListener);

	while (ros::ok()){
	ros::spinOnce();
	}
        myfile.close();
}
