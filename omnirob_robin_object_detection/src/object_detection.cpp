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

ros::Publisher table_pub;
ros::Publisher objects_pub;
ros::Publisher test_pub;
ros::Publisher pan_tilt_goal_pub;

ros::ServiceServer detectObjectsService;
ros::ServiceServer getObjectPoseService;


ros::ServiceClient add_marker_client;

tf::TransformListener* pListener;

bool detecting = false;
bool newCloud = false;
std::vector <Object> objects;

pcl::PointCloud<PointType>::Ptr cloud;
pcl::PointCloud<PointType>::Ptr objectCloud;
pcl::PointCloud<PointType>::Ptr tableCloud;
std::vector <tf::Transform> transforms;
std::vector <std::string> transform_names;

//function definitions
void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input_cloud);
bool detectObjectsCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
bool getObjectPoseCallback(omnirob_robin_msgs::get_object_pose::Request& request, omnirob_robin_msgs::get_object_pose::Response& response);
void detectObjects();
void addMarker(std::vector <double> size, tf::Transform transform);
void pose_to_tf(std::vector <double> pose, tf::Transform& transform);

void pose_to_tf(std::vector <double> pose, tf::Transform& transform){
	transform.setOrigin(tf::Vector3(pose[0], pose[1], pose[2]));
	tf::Quaternion quat;
	quat.setRPY(pose[3], pose[4], pose[5]);
	transform.setRotation(quat);
}


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
	detectObjects();

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
		}

	}
}


void detectObjects(){

	tableCloud = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
	ROS_INFO("detecting table");
	//seperate the table from the cloud

	double table_height;
	robin_odlib::seperateTable(cloud, tableCloud, table_height);

	//publish table

	sensor_msgs::PointCloud2::Ptr output_cloud (new sensor_msgs::PointCloud2);
	pcl::toROSMsg (*tableCloud, *output_cloud);
	output_cloud->header.stamp = ros::Time::now();
	output_cloud->header.frame_id = "/table_1";
	table_pub.publish(output_cloud);



	//calculate table bounding box
	std::vector <double> table_size;
	std::vector <double> table_pose;

	//reduce table_cloud
	pcl::VoxelGrid<PointType> sor;
	sor.setInputCloud (tableCloud);
	sor.setLeafSize (0.02f, 0.02f, 0.02f);
	sor.filter (*tableCloud);


	robin_odlib::fitBoundingBox(tableCloud, table_size, table_pose, 0.0);

	ROS_INFO("table_size = %f, %f, %f", table_size[0], table_size[1], table_size[2]);
	ROS_INFO("table_pose = %f, %f, %f", table_pose[0], table_pose[1], table_pose[2]);

	//reset table height und table pose to table height /2

	table_size[2] = table_height;
	table_pose[2] = table_height/2.0;

	tf::Transform table_transform;
	pose_to_tf(table_pose, table_transform);

	addMarker(table_size, table_transform);
	ROS_INFO("add marker finished");

	//publish objects



	std::vector <pcl::PointIndices> clusters;
	//segment the remaining cloud into objects and retrieve the indices
	robin_odlib::Segmentation(cloud, clusters);


	//search for shape in the seperated clouds by shape
	std::vector <double> pose;
	pose.resize(6);

	transforms.clear();
	transform_names.clear();

	for(int i = 0; i < objects.size(); i++){
		int count = 0;
		for(int k = 0; k < clusters.size(); k++){
			bool found = robin_odlib::searchObject(cloud, clusters[k], objects[i], pose, table_height);
			if(found){

				pcl::PointCloud<PointType>::Ptr objectCloud (new pcl::PointCloud<PointType>);
				pcl::ExtractIndices<PointType> extract_indices;
				pcl::IndicesPtr object_indices (new std::vector <int>);
				*object_indices = clusters[k].indices;

				extract_indices.setInputCloud (cloud);
				extract_indices.setIndices (object_indices);
				extract_indices.setNegative (false);
				extract_indices.filter(*objectCloud);


				pcl::toROSMsg (*objectCloud, *output_cloud);
				output_cloud->header.stamp = ros::Time::now();
				output_cloud->header.frame_id = "/table_1";
				objects_pub.publish(output_cloud);


				ROS_INFO("object %s found", objects[0].getName().c_str());
				ROS_INFO("objects pose = %f %f %f", pose[0], pose[1], pose[2]);
				tf::Transform to_object_from_table;
				pose_to_tf(pose, to_object_from_table);


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
				std::stringstream s;
				s << count;
				transform_names.push_back(objects[0].getName() + s.str());
				count++;
			}
		}
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

	detectObjects();

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

	response.pose.position.x = transforms[index].getOrigin().getX();
	response.pose.position.y = transforms[index].getOrigin().getY();
	response.pose.position.z = transforms[index].getOrigin().getZ();

	response.pose.orientation.x = transforms[index].getRotation().getX();
	response.pose.orientation.y = transforms[index].getRotation().getY();
	response.pose.orientation.z = transforms[index].getRotation().getZ();
	response.pose.orientation.w = transforms[index].getRotation().getW();

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

	//Action Server
	Server server(n, "hackObjRecActionServer", boost::bind(&execute, _1, &server), false);
	server.start();

	//variables
	cloud = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());

	//subscriber
	//pointcloud_sub = n.subscribe("/input_cloud", 1, pointcloudCallback);

	//publisher
	table_pub = n.advertise<sensor_msgs::PointCloud2> ("table_cloud", 1);
	objects_pub = n.advertise<sensor_msgs::PointCloud2> ("objects_cloud", 1);
	test_pub = n.advertise<sensor_msgs::PointCloud2> ("test_cloud", 1);

	//Services Server
	detectObjectsService = n.advertiseService("detect_objects_srv", detectObjectsCallback);
	getObjectPoseService = n.advertiseService("get_object_pose_srv", getObjectPoseCallback);

	//Service Clients


	ros::service::waitForService("add_marker");
	add_marker_client = n.serviceClient<omnirob_robin_msgs::add_marker_srv>("add_marker");

	while(!ros::param::has("/detectable_objects")){
		ros::spinOnce();
	}

	robin_odlib::loadObjects(objects);

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
