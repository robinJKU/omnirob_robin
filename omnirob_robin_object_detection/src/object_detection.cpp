#include <ros/ros.h>


//pcl includes
#include <pcl/conversions.h>
#include <pcl/common/common.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/normal_3d.h>

//ros includes
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include <actionlib/server/simple_action_server.h>

//services und messages
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/Empty.h>
#include <omnirob_robin_msgs/get_object_pose.h>
#include <omnirob_robin_msgs/HackObjRecAction.h>

//robin object detection library
#include <robin_odlib.h>
#include <robin_odlib_ros.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::Normal NormalType;
typedef actionlib::SimpleActionServer<omnirob_robin_msgs::HackObjRecAction> Server;

ros::Subscriber pointcloud_sub;

ros::Publisher table_pub;
ros::Publisher objects_pub;
ros::Publisher pan_tilt_goal_pub;
ros::Publisher box_pub;

ros::ServiceServer detectObjectsService;
ros::ServiceServer getObjectPoseService;

ros::ServiceClient pan_tilt_start_motion_srv;

tf::TransformListener* pListener;

bool detecting = false;
bool newCloud = false;
std::vector <Object> objects;

pcl::PointCloud<PointType>::Ptr cloud;
std::vector <tf::Transform> transforms;
std::vector <std::string> transform_names;

//function definitions
void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input_cloud);
bool detectObjectsCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
bool getObjectPoseCallback(omnirob_robin_msgs::get_object_pose::Request& request, omnirob_robin_msgs::get_object_pose::Response& response);
void detectObjects();

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
  if(detecting){
    ROS_INFO("object detection called, image received");
    detecting = false;   
    
    sensor_msgs::PointCloud2::Ptr cloud_transformed (new sensor_msgs::PointCloud2);				
    //Transform cloud to origin frame
    try {            
      pListener->waitForTransform("/base_link", (*input_cloud).header.frame_id, ros::Time::now(), ros::Duration(60.0) );
      pcl_ros::transformPointCloud ("/base_link", *input_cloud, *cloud_transformed, *pListener);
    }
    catch(tf::TransformException e){
      ROS_INFO("Transform_PointCloud Error: %s", e.what());
    }
    
    pcl::fromROSMsg (*cloud_transformed, *cloud);	
    newCloud = true;    
  }  
}

void detectObjects(){
  
  while(!newCloud){
    ros::spinOnce();
  }    
  
  newCloud = false;
    
  pcl::PointCloud<PointType>::Ptr table_cloud = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
  
  
  //seperate the table from the cloud
  double table_height;
  robin_odlib::seperateTable(cloud, table_cloud, table_height);
  
  //publish table
  sensor_msgs::PointCloud2::Ptr output_cloud (new sensor_msgs::PointCloud2);
  pcl::toROSMsg (*table_cloud, *output_cloud);
  output_cloud->header.stamp = ros::Time::now();
  output_cloud->header.frame_id = "/base_link";
  table_pub.publish(output_cloud);
  
  //publish objects    
  pcl::toROSMsg (*cloud, *output_cloud);
  output_cloud->header.stamp = ros::Time::now();
  output_cloud->header.frame_id = "/base_link";
  objects_pub.publish(output_cloud);
    
  
  std::vector <pcl::PointIndices> clusters;
  //segment the remaining cloud into objects and retrieve the indices    
  robin_odlib::Segmentation(cloud, clusters);
  
      
  //search for shape in the seperated clouds by shape       
  robin_odlib::searchObject(cloud, clusters[0], objects[0], table_height);
  
  
  //set transforms
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(objects[0].getPosition()[0], objects[0].getPosition()[1], objects[0].getPosition()[2]));
  tf::Quaternion quat;
  quat.setRPY(objects[0].getOrientation()[0], objects[0].getOrientation()[1], objects[0].getOrientation()[2]);
  transform.setRotation(quat);
  transforms.clear();
  transforms.push_back(transform);
  transform_names.clear();
  transform_names.push_back(objects[0].getName());
  
  
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "/base_link";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = 1;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = visualization_msgs::Marker::CUBE;

  // Set the marker action.  Options are ADD and DELETE
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = objects[0].getPosition()[0];
  marker.pose.position.y = objects[0].getPosition()[1];
  marker.pose.position.z = objects[0].getPosition()[2];
  
  quat.setRPY(0,0, objects[0].getOrientation()[2]);
  
  marker.pose.orientation.x = quat.getX();
  marker.pose.orientation.y = quat.getY();
  marker.pose.orientation.z = quat.getZ();
  marker.pose.orientation.w = quat.getW();

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = objects[0].getSize()[0];
  marker.scale.y = objects[0].getSize()[1];
  marker.scale.z = objects[0].getSize()[2];
    

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 255;
  marker.color.g = 0;
  marker.color.b = 0;
  marker.color.a = 0.4;

  marker.lifetime = ros::Duration(1000.0);
  
  box_pub.publish(marker);
    
  
}

bool detectObjectsCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
  detecting = true;    
  
  std_msgs::Float64MultiArray msg;
  std_srvs::Empty srv;
  
  msg.data.push_back(0.2);
  msg.data.push_back(-0.6);
  
  pan_tilt_goal_pub.publish(msg);
  pan_tilt_start_motion_srv.call(srv); 
  
  ros::Duration(5.0).sleep();
  
  detectObjects();
  
  return true;
}

bool getObjectPoseCallback(omnirob_robin_msgs::get_object_pose::Request& request, omnirob_robin_msgs::get_object_pose::Response& response){
  int index = -1;
  for(int i = 0; i < transform_names.size(); i++){
    if(transform_names[i].compare(request.name)){
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
  pointcloud_sub = n.subscribe("/input_cloud", 1, pointcloudCallback);
  
  //publisher
  table_pub = n.advertise<sensor_msgs::PointCloud2> ("table_cloud", 1);
  objects_pub = n.advertise<sensor_msgs::PointCloud2> ("objects_cloud", 1);
  pan_tilt_goal_pub = n.advertise<std_msgs::Float64MultiArray> ("pan_tilt/control/commanded_joint_state", 1);
  box_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  
  //Services Server
  detectObjectsService = n.advertiseService("detect_objects_srv", detectObjectsCallback);
  getObjectPoseService = n.advertiseService("get_object_pose_srv", getObjectPoseCallback);
  
  //Service Client
  ros::service::waitForService("pan_tilt/control/start_motion");
  pan_tilt_start_motion_srv = n.serviceClient<std_srvs::Empty>("pan_tilt/control/start_motion"); 
  
  while(!ros::param::has("/detectable_objects")){
    ros::spinOnce();    
  }
  
  robin_odlib_ros::loadObjects(objects);  
  
  while(ros::ok){
    for(int i = 0; i < transforms.size(); i++){      
      broadcaster.sendTransform(tf::StampedTransform(transforms[i], ros::Time::now(), "base_link", transform_names[i]));
    }
    ros::spinOnce();
  }
  
  
}
