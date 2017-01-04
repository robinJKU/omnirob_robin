#include <ros/ros.h>
#include <sstream> 
#include <fstream>
#include <iostream>

#include <ros_common_robin_tools/common_tools.h>
#include <dynamic_reconfigure/Reconfigure.h>

//pcl includes
#include <pcl/conversions.h>
#include <pcl/common/common.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

//ros include
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
#include <actionlib/server/simple_action_server.h>

//services und messages
#include <std_srvs/Empty.h>
#include <ros_common_robin_msgs/get_object_pose.h>
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "sensor_msgs/CameraInfo.h"

//robin object detection library
#include <reels_robin_odlib.h>

//robin RGB image library
#include <image_detection.h>

//moveKinect
#include <omnirob_robin_msgs/move_pan_tilt.h>

//include necessary for the analysis of the RGB image
#include "boost/multi_array.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

/**
*This code analyses the point cloud and the RGB image coming from the kinect sensor.
*The code is based on the detection of different AR-Markers. In particular the markers
*with ID number 2-3-4 are used to detect the vertical base while the marker 5 is used to
*control the position of the cylindrical objects.
 */

using namespace std;

typedef pcl::PointXYZRGB PointType;
typedef pcl::Normal NormalType;

ros::Subscriber pointcloud_sub;
ros::Subscriber markers_sub;
ros::Subscriber camera_sub;
image_transport::Subscriber image_sub;

ros::ServiceServer detectMarkersService;
ros::ServiceServer detectObjectsService;
ros::ServiceServer getObjectPoseService;
tf::TransformListener* pListener;
tf::Transform base_to_target_pose;
tf::StampedTransform calibrated_to_base;

bool detecting = false;
bool newCloud = false;
bool detection_markers=false;
bool detection_camera=false;
bool detection_image=false;
bool target=false;
bool reel_presence;

robin_object_detector *object_detector;
ImageProcessor *image_detector;

pcl::PointCloud<PointType>::Ptr cloud;
std::vector <tf::Transform> transforms;
std::vector <std::string> transform_names;
sensor_msgs::Image image_ellipse;


//function definitions
void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input_cloud);
void CameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
void MarkersCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
bool detectObjectsCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
bool getObjectPoseCallback(ros_common_robin_msgs::get_object_pose::Request& request, ros_common_robin_msgs::get_object_pose::Response& response);
bool detectMarkersCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
bool ReceiveInfoCamera();
bool ReceiveImage();
void imageCb(const sensor_msgs::ImageConstPtr& msg);
void detect_objects();


void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input_cloud) {

	ROS_INFO("Object detection called, image received");
	detecting = false;
	sensor_msgs::PointCloud2::Ptr cloud_transformed (new sensor_msgs::PointCloud2);
	//Transform cloud to target frame

	if(input_cloud->data.size() > 0){
        bool success = false;

        while (!success) {
    		try {
            		pListener->waitForTransform("/base_link", (*input_cloud).header.frame_id, ros::Time::now(), ros::Duration(1.0) );
			pcl_ros::transformPointCloud ("/base_link", *input_cloud, *cloud_transformed, *pListener);
            		success = true;
            		//The point cloud is now transformed in the base_link frame
          	}catch (tf::TransformException e) {
              		ROS_INFO("Transform_PointCloud Error: %s", e.what());
          	}
        }
        
	ROS_INFO("Trasformation of the point cloud in the target frame that is orinted as the plane of the 3 markers");
	pcl::fromROSMsg (*cloud_transformed, *cloud);
	pcl_ros::transformPointCloud (*cloud, *cloud, base_to_target_pose.inverse());

	if(cloud->size() > 0){

		newCloud = true;
	        //ROS_INFO("Number of points of the received cloud %d",(int)cloud->size());
		object_detector->set_cloud(cloud);

		}
	}
}


void CameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg){
  	
	boost::array<double, 9> K;
	std::vector<double> D;
	K=msg->K;
	D=msg->D;
	image_detector->set_intrinsic_matrix(K,D);
	detection_camera=true;
}


void MarkersCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg){

    	unsigned int nr_of_markers = msg->markers.size();
    	std::vector<AR_Marker> mark(nr_of_markers);
	ROS_INFO("Number of markers %d",nr_of_markers);

	if (nr_of_markers>0){
    	
		for(unsigned int marker_ii=0; marker_ii<nr_of_markers; marker_ii++ ){

         		mark[marker_ii].marker_detected = true;
            		mark[marker_ii].marker_id = msg->markers[marker_ii].id;
            		mark[marker_ii].reference_frame_id = msg->markers[marker_ii].header.frame_id;
            		mark[marker_ii].observed_pose= msg->markers[marker_ii].pose.pose;

			ROS_INFO("ID Marker detected %d", mark[marker_ii].marker_id);
		
			double x, y, z, rx, ry, rz, rw;
			x=mark[marker_ii].observed_pose.position.x;
			y=mark[marker_ii].observed_pose.position.y;
			z=mark[marker_ii].observed_pose.position.z;
			rx=mark[marker_ii].observed_pose.orientation.x;
			ry=mark[marker_ii].observed_pose.orientation.y;
			rz=mark[marker_ii].observed_pose.orientation.z;
			tf::Quaternion q(rx, ry, rz, rw);
			q.normalize();

			tf::Transform to_marker_from_camera, to_marker_from_base;
	  		to_marker_from_camera.setOrigin( tf::Vector3(x, y, z) );
			to_marker_from_camera.setRotation(q);
			tf::StampedTransform to_camera_from_base;
			ROS_INFO("Trasformation of the markers in the base_link frame");

			try {
				pListener->waitForTransform("/base_link", "/kinect2_rgb_optical_frame", ros::Time(0), ros::Duration(3.0) );
				pListener->lookupTransform("/base_link", "/kinect2_rgb_optical_frame", ros::Time(0), to_camera_from_base);
				}
				catch(tf::TransformException e){
				ROS_INFO("Transform object frame error Error: %s", e.what());
				}

			to_marker_from_base.mult(to_camera_from_base, to_marker_from_camera);

			double roll, pitch, yaw;
			tf::Vector3 Position;
			tf::Quaternion q2;
			Position=to_marker_from_base.getOrigin();
			mark[marker_ii].position = Position;
			q2=to_marker_from_base.getRotation();
			tf::Matrix3x3 rotation(q2);
			rotation.getRPY(roll, pitch, yaw);
			ROS_INFO("Position marker detected x= %f y= %f z= %f", Position[0], Position[1], Position[2]);
            		//ROS_INFO("Orientation marker detected r =%f p= %f y= %f", roll,pitch,yaw);
			object_detector->set_markers(mark[marker_ii]);
			image_detector->set_markers(mark[marker_ii]);

   		}// for all markeres
	}else{
		ROS_INFO("No marker present");
	}
	
	detection_markers=true;
}


void detect_objects(){
	
	ROS_INFO("Start of the objects detection");
	clock_t time_a = clock();
	object_detector->detect_objects();
	std::vector<Cylinder> detected_cylinder = object_detector->get_detected_cylinder();
	clock_t time_b = clock();
	double time_detection=(double)(time_b-time_a) / CLOCKS_PER_SEC;
	ROS_INFO("Time necessary for the detection = %f",time_detection);

	ROS_INFO("Number of detected objects= %d", (int)detected_cylinder.size());

	for(int i = 0; i < detected_cylinder.size(); i++){

		tf::Transform to_object_from_base, to_object_from_target,to_object_from_calibrated;
		tf::poseMsgToTF(detected_cylinder[i].get_pose(), to_object_from_target);
		to_object_from_base.mult(base_to_target_pose,to_object_from_target);	
                to_object_from_calibrated.mult(calibrated_to_base,to_object_from_base);
		transforms.push_back(to_object_from_calibrated);
		transform_names.push_back(detected_cylinder[i].getName());

		double roll, pitch, yaw;
		tf::Vector3 Position;
		tf::Quaternion q2;
		Position=to_object_from_calibrated.getOrigin();
		q2=to_object_from_calibrated.getRotation();
		tf::Matrix3x3 rotation(q2);
		rotation.getRPY(roll, pitch, yaw);
		ROS_INFO("\nOBJECT DETECTED");

		ROS_INFO("Pose of object detected %s",detected_cylinder[i].getName().c_str());
		ROS_INFO("Position object detected with respect to the calibration frame x= %f y= %f z= %f", Position[0], Position[1], Position[2]);
            	ROS_INFO("Orientation object detected with respect to the calibration frame r= %f p= %f y= %f", roll,pitch,yaw);
	}

}


bool detectMarkersCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){   

	clock_t time_e = clock();

	ros::NodeHandle n;
    	std::string ar_track_set_param_srv = "/ar_track_alvar/set_parameters";

    	if( !common_tools::wait_for_service( ar_track_set_param_srv) ){
	    ROS_ERROR("Error in waiting service");
	    return true;
    	}
		
	ros::ServiceClient ar_set_parameters_client = n.serviceClient<dynamic_reconfigure::Reconfigure>( ar_track_set_param_srv);
	dynamic_reconfigure::Reconfigure ar_parameters;
	ar_parameters.request.config.bools.resize(1);
	ar_parameters.request.config.bools[0].name="enabled";
	ar_parameters.request.config.bools[0].value=true;
	ar_set_parameters_client.call( ar_parameters);
	markers_sub = n.subscribe("/ar_pose_marker", 1, &MarkersCallback);

	while(!detection_markers){
		ros::Rate(10).sleep();
		ros::spinOnce();
        }
	markers_sub.shutdown();
        ROS_INFO("Detect markers finished");

	// disable ar track alvar node
	ar_parameters.request.config.bools[0].value=false;
	ar_set_parameters_client.call( ar_parameters);

	// elaborate the rgb image
	image_detector->elaboration();
        image_detector->get_image_msg(image_ellipse);
	image_detector->get_reel_presence(reel_presence);
	object_detector->set_reel_presence(reel_presence);
	
	ROS_INFO("Analysis of the RGB image finished");
	clock_t time_f = clock();
	double timeTOT2_detection=(double)(time_f-time_e) / CLOCKS_PER_SEC; 
	ROS_INFO("Time necessary for the WHOLE RGB image analysis = %f",timeTOT2_detection);

	return true;
}


bool ReceiveInfoCamera(){   

	image_detector= new ImageProcessor();
	//ros::Duration(0.01).sleep();
	ros::NodeHandle n;
	camera_sub = n.subscribe("/kinect2/hd/camera_info", 1, &CameraInfoCallback);

	while(!detection_camera){

		ros::spinOnce();
        }
	camera_sub.shutdown();

	return true;
}


void imageCb(const sensor_msgs::ImageConstPtr& msg){

	cv_bridge::CvImagePtr cv_ptr;
	try{

      		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    	}
    	catch (cv_bridge::Exception& e){

      		ROS_ERROR("cv_bridge exception: %s", e.what());
      		return;
    	}
	
  	image_detector->set_image(cv_ptr->image);
  	detection_image=true;
}


bool ReceiveImage(){   

	ros::NodeHandle nh_;
  	image_transport::ImageTransport it_(nh_);
	image_sub = it_.subscribe("/kinect2/hd/image_color", 1, &imageCb);

	while(!detection_image){

		ros::spinOnce();
	
        }
	image_sub.shutdown();

	return true;
}


bool detectObjectsCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){  

	ROS_INFO("\n\nANALYSIS OF THE POINTCLOUD");
	clock_t time_c = clock(); 

	//ros::Duration(0.01).sleep(); //duration in the time domain
	tf::Vector3 normal, center, width, half_height;
	double distance, h ,b;
	object_detector->getPlaneMachine(normal,center,distance,width,half_height);
  	tf::Vector3 axis_vector(-normal);
	tf::Vector3 up_vector(0.0, 0.0, 1.0);
	tf::Vector3 down_vector(0.0, 0.0, -1.0);
	tf::Vector3 x_vector(1.0, 0.0, 0.0);

	tf::Quaternion q,qi;
	tf::Vector3 right_vector = axis_vector.cross(x_vector);
  	right_vector.normalized();
	q.setRotation(right_vector, -acos(axis_vector.dot(x_vector)));
	q.normalize();
	qi=q.inverse();

	double dist=0.5;
	tf::Vector3 origin= center+half_height+dist*normal;
    	base_to_target_pose.setOrigin(origin);
    	base_to_target_pose.setRotation(q);
	tf::Matrix3x3 rotation_direct(q);
	object_detector->set_change(origin, rotation_direct);
	tf::Matrix3x3 rotation(qi);
	target=true;

	ROS_INFO("Change the reference frame of the markers");
	std::vector<AR_Marker> markers_2;
	object_detector->get_markers(markers_2);

	for (int u=0; u<markers_2.size(); u++){

		markers_2[u].position=markers_2[u].position-origin;
		markers_2[u].position=rotation*markers_2[u].position;

	}

	object_detector->set_markers_frame(markers_2);
	ros::NodeHandle n;
	pointcloud_sub = n.subscribe("/input_cloud", 1, pointcloudCallback);	

	while(!newCloud){

		ros::Rate(1).sleep(); //frequency domain (1 Hz)
		ros::spinOnce();
	
        }

	pointcloud_sub.shutdown();
	n.shutdown();
	newCloud = false;
	detect_objects();//call to the function detect_objects()
    ROS_INFO("Detect objects finished");
	clock_t time_d = clock();
	double timeTOT_detection=(double)(time_d-time_c) / CLOCKS_PER_SEC; 
	ROS_INFO("Time necessary for the WHOLE point cloud analysis = %f",timeTOT_detection);

	return true;
}


bool getObjectPoseCallback(ros_common_robin_msgs::get_object_pose::Request& request, ros_common_robin_msgs::get_object_pose::Response& response){

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
        ros::Publisher image_pub=n.advertise<sensor_msgs::Image> ("/image_ellipse", 10, true);

	try {
		pListener->waitForTransform("/calibration_frame", "/base_link", ros::Time(0), ros::Duration(3.0) );
		pListener->lookupTransform("/calibration_frame", "/base_link", ros::Time(0), calibrated_to_base);
		}
	catch(tf::TransformException e){
		ROS_INFO("Transform object frame error Error: %s", e.what());
		}

	//Receive the camera information and the 2D RGB image
	ReceiveInfoCamera();
	ReceiveImage();

	object_detector = new robin_object_detector();
	object_detector->set_frame("/target");

	//variables
	cloud = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());

	//Services Server
	detectMarkersService = n.advertiseService("detect_markers_srv", detectMarkersCallback);
	detectObjectsService = n.advertiseService("detect_objects_srv", detectObjectsCallback);
	getObjectPoseService = n.advertiseService("get_object_pose_srv", getObjectPoseCallback);

	while(!ros::param::has("/detectable_objects")){

		ros::spinOnce();
	}

	ros::Rate r(50);
	while(ros::ok){
		image_pub.publish(image_ellipse);
		for(int i = 0; i < transforms.size(); i++){
			int size = transforms.size();
			broadcaster.sendTransform(tf::StampedTransform(transforms[i], ros::Time::now(), "/calibration_frame", transform_names[i]));
		}
		if(target){
		broadcaster.sendTransform(tf::StampedTransform(base_to_target_pose, ros::Time::now(), "base_link", "target"));	
		}
		r.sleep();
		ros::spinOnce();
	}


}
