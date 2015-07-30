#include <ros/ros.h>
#include <sstream> 

//pcl includes
#include <pcl/conversions.h>
#include <pcl/common/common.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>


//ros includes
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include <actionlib/server/simple_action_server.h>

//services und messages
#include <std_srvs/Empty.h>
#include <omnirob_robin_msgs/get_object_pose.h>
#include <omnirob_robin_msgs/HackObjRecAction.h>
#include <omnirob_robin_msgs/add_marker_srv.h>

//robin object detection library
#include <robin_odlib.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::Normal NormalType;
typedef actionlib::SimpleActionServer<omnirob_robin_msgs::HackObjRecAction> Server;

ros::Subscriber pointcloud_sub;

ros::ServiceServer detectObjectsService;
ros::ServiceServer getObjectPoseService;

ros::ServiceClient add_marker_client;

tf::TransformListener* pListener;

bool detecting = false;
bool newCloud = false;
robin_object_detector object_detector;

pcl::PointCloud<PointType>::Ptr cloud;
std::vector <tf::Transform> transforms;
std::vector <std::string> transform_names;

//function definitions
void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input_cloud);
bool detectObjectsCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
bool getObjectPoseCallback(omnirob_robin_msgs::get_object_pose::Request& request, omnirob_robin_msgs::get_object_pose::Response& response);
void detect_objects();
void addMarker(std::vector <double> size, tf::Transform transform);

void addMarker(std::vector <double> size, tf::Transform transform){
	ROS_INFO("add marker service called");
	omnirob_robin_msgs::add_marker_srv srv;
	srv.request.name = "table";
	srv.request.size.x = size[0];
	srv.request.size.y = size[1];
	srv.request.size.z = size[2];
	srv.request.pose.position.x = transform.getOrigin().getX();
	srv.request.pose.position.y = transform.getOrigin().getY();
	srv.request.pose.position.z = transform.getOrigin().getZ();
	srv.request.pose.orientation.x = transform.getRotation().getX();
	srv.request.pose.orientation.y = transform.getRotation().getY();
	srv.request.pose.orientation.z = transform.getRotation().getZ();
	srv.request.pose.orientation.w = transform.getRotation().getW();

	add_marker_client.call(srv);
}


void execute(const omnirob_robin_msgs::HackObjRecGoalConstPtr& goal, Server* as)
{
	// Do lots of awesome groundbreaking robot stuff here
	omnirob_robin_msgs::HackObjRecResult res;
	ROS_INFO("executing object recognition");

	detecting = true;
	detect_objects();

	if(transform_names.size() == 0){
		res.nObjects = 0;
		res.res = res.RES_ERROR1;		
		as->setAborted(res);
	}  else {
		res.nObjects = transform_names.size();
		for(int i = 0; i < transform_names.size(); i++){
			res.objNames.push_back(transform_names[i]);
		}
		res.res = res.RES_SUCCESS;
		as->setSucceeded(res);    
	}
}


void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input_cloud) {

	ROS_INFO("object detection called, image received");
	detecting = false;

	sensor_msgs::PointCloud2::Ptr cloud_transformed (new sensor_msgs::PointCloud2);
	//Transform cloud to target frame

	if(input_cloud->data.size() > 0){

		try {
			pListener->waitForTransform("/table_1", (*input_cloud).header.frame_id, ros::Time::now(), ros::Duration(60.0) );
			pcl_ros::transformPointCloud ("/table_1", *input_cloud, *cloud_transformed, *pListener);
		}
		catch(tf::TransformException e){
			ROS_INFO("Transform_PointCloud Error: %s", e.what());
		}

		pcl::fromROSMsg (*cloud_transformed, *cloud);

		if(cloud->size() > 0){
			newCloud = true;
			object_detector.set_cloud(cloud);
		}

	}
}

void detect_objects(){


	object_detector.detect_objects();
	/*
	tf::Transform table_transform;
	pose_to_tf(table_pose, table_transform);

	addMarker(table_size, table_transform);
	ROS_INFO("add marker finished");
	*/
	std::vector<Object> detected_objects = object_detector.get_detected_objects();
	for(int i = 0; i < detected_objects.size(); i++){
		tf::Transform to_object_from_table;
		to_object_from_table = detected_objects[i].get_transform();

		tf::StampedTransform to_table_from_map;
		try {
				pListener->waitForTransform("/table_1", "/map", ros::Time::now(), ros::Duration(60.0) );
				pListener->lookupTransform("/table_1", "/map", ros::Time::now(), to_table_from_map);
			}
			catch(tf::TransformException e){
				ROS_INFO("Transform object frame error Error: %s", e.what());
			}
		to_object_from_table.mult(to_table_from_map.inverse(), to_object_from_table);
		transforms.push_back(to_object_from_table);
		transform_names.push_back(detected_objects[i].getName());
	}

}

bool detectObjectsCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){   



	ros::Duration(5.0).sleep();

	ros::NodeHandle n;
	pointcloud_sub = n.subscribe("/input_cloud", 1, pointcloudCallback);

	while(!newCloud){
		ros::Rate(1).sleep();
		ros::spinOnce();
	}

	pointcloud_sub.shutdown();
	n.shutdown();

	newCloud = false;

	detect_objects();

	return true;
}

bool getObjectPoseCallback(omnirob_robin_msgs::get_object_pose::Request& request, omnirob_robin_msgs::get_object_pose::Response& response){
	int index = -1;
	for(int i = 0; i < transform_names.size(); i++){
		if(transform_names[i].compare(request.name) == 0){
			index = i;
		}
	}

	if(index < 0){
		return false;
	}

	tf::poseTFToMsg(transforms[index], response.pose);

	return true;
}

int main( int argc, char** argv) {

	ros::init(argc, argv, "object_detection");

	ros::AsyncSpinner spinner(4); // required because both the server and the client run in the same node
	spinner.start();

	ros::NodeHandle n;

	//Transform Listener / Broadcaster
	pListener = new(tf::TransformListener);
	tf::TransformBroadcaster broadcaster;

	object_detector = robin_object_detector();

	//Action Server
	Server server(n, "hackObjRecActionServer", boost::bind(&execute, _1, &server), false);
	server.start();

	//variables
	cloud = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());

	//subscriber
	//pointcloud_sub = n.subscribe("/input_cloud", 1, pointcloudCallback);


	//Services Server
	detectObjectsService = n.advertiseService("detect_objects_srv", detectObjectsCallback);
	getObjectPoseService = n.advertiseService("get_object_pose_srv", getObjectPoseCallback);

	//Service Clients


	//ros::service::waitForService("add_marker");
	//add_marker_client = n.serviceClient<omnirob_robin_msgs::add_marker_srv>("add_marker");

	while(!ros::param::has("/detectable_objects")){
		ros::spinOnce();
	}

	ros::Rate r(50);
	while(ros::ok){
		for(int i = 0; i < transforms.size(); i++){
			int size = transforms.size();
			broadcaster.sendTransform(tf::StampedTransform(transforms[i], ros::Time::now(), "/map", transform_names[i]));
		}
		r.sleep();
		ros::spinOnce();
	}


}
