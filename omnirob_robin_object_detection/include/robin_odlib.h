#ifndef ROBIN_ODLIB_H
#define ROBIN_ODLIB_H

//The Robin Object Detection Library is a collection of useful function to segment a pointlcloud into parts which than can be searched for Shapes of type Shape

#include <ros/ros.h>
#include <sstream>
#include <robin_object.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <Eigen/Geometry> 
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <visualization_msgs/Marker.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::Normal NormalType;

class robin_object_detector{
public: //functions

	// ******* constructor
	robin_object_detector():
		node_handle_(),
		spinner_(1)
	{
		load_objects();
		cloud = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());

		marker_pub = node_handle_.advertise<visualization_msgs::Marker> ("/od_marker", 10, true);
		table_pub = node_handle_.advertise<sensor_msgs::PointCloud2> ("/table_cloud", 10, true);
		objects_pub = node_handle_.advertise<sensor_msgs::PointCloud2> ("/objects_cloud", 10, true);
	}

	// ******* destructor
	~robin_object_detector()
	{
		//delete objects if necessary
	}

	void set_cloud(pcl::PointCloud<PointType>::Ptr &input_cloud){
		//pcl::copyPointCloud(*input_cloud, *cloud);
		pcl::VoxelGrid<PointType> sor;
		sor.setInputCloud (input_cloud);
		sor.setLeafSize (0.005f, 0.005f, 0.005f);
		sor.filter (*cloud);
	}

	void set_frame(std::string frame){
		this->frame = frame;
	}

	std::vector<Object> get_objects(){
		return objects;
	}

	std::vector<Object> get_detected_objects(){
		return objects;
	}

	void detect_objects()
	{
		ROS_INFO("Starting Objects Detection");
		ROS_INFO("Detecting table");
		//seperate the table from the cloud


		pcl::PointCloud<PointType>::Ptr table_cloud = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
		pcl::PointCloud<PointType>::Ptr objects_cloud = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
		double table_height;
		seperateTable(table_cloud,objects_cloud, table_height);

		ROS_INFO("Calculating table bounding box");
		pcl::VoxelGrid<PointType> sor;
		sor.setInputCloud (table_cloud);
		sor.setLeafSize (0.02f, 0.02f, 0.02f);
		sor.filter (*table_cloud);
		Box box;
		fit_bounding_box(table_cloud, box, 0.0);
		publish_marker(box, 3);

		std::vector<pcl::PointCloud<PointType>::Ptr> objects_clouds;
		ROS_INFO("Segmentation of objects and filtering of stray points");
		segmentation(objects_cloud, objects_clouds);

		ROS_INFO("Detecting objects for all segmented clouds");
		std::vector<int> cnt;
		cnt.resize(objects.size());
		detected_objects.clear();
		for(int i = 0; i < objects_clouds.size(); i++)
		{
			ROS_INFO("Detecting objects in cloud nr %d", i);
			for(int k = 0; k < objects.size(); k++){
			if(search_object(objects_clouds[i], objects[k], table_height))
				{
					detected_objects.push_back(objects[k]);
					detected_objects.back().set_number(cnt[i]);
					cnt[i] += 1;
				}
			}
		}

	}


private: //functions
	void load_objects(){

	  std::vector <std::string> object_names;
	  ros::param::get("/detectable_objects", object_names);
	  for(int i = 0; i < object_names.size(); i++){
	    //create the variables for the constructor + param namespace
	    std::string name = object_names[i];
	    std::string name_space = "/objects/" + name;
	    std::string color;
	    std::vector <int> RGBcolor;

	    //print name + get the color and RGBcolor value
	    ros::param::get(name_space + "/color", color);
	    ros::param::get(name_space + "/RGBcolor", RGBcolor);
	    int size = RGBcolor.size();

	    //construct a new object and push it in our vector
	    objects.push_back(Object(name, color, RGBcolor));

	    //now load all the primitives
	    int k = 0;
	    std::ostringstream s;
	    s << k;
	    std::string  primitive_space = name_space + "/primitive" + s.str();
	    while(ros::param::has(primitive_space + "_type")) {
	      //set the name space for our primitive;


	      //primitive variables
	      std::string type;
	      std::vector <double> size;
	      std::vector <double> position;
	      std::vector <double> orientation;
	      ros::param::get(primitive_space + + "_type", type);
	      ros::param::get(primitive_space + + "_size", size);
	      ros::param::get(primitive_space + + "_position", position);
	      ros::param::get(primitive_space + + "_orientation", orientation);

	      objects[i].addPrimitive(type, size, position, orientation);

	      k++;
	      s.str(std::string());
	      s << k;
	      primitive_space = name_space + "/primitive" + s.str();
	    }

	    objects[i].printObject();

	  }

	  int size = objects.size();
	  ROS_INFO("%d objects loaded \n", size);

	}// load_objects

	void publish_marker(Box box, int id = 0){
		std::vector<double> scale;
		geometry_msgs::Pose pose;

		scale = box.size;
		pose = box.get_pose();

		publish_marker(pose, scale, id);
	}

	void publish_marker(pcl::CropBox<PointType> filter, int id = 0){
		std::vector<double> scale;
		geometry_msgs::Pose pose;

		scale.push_back(filter.getMax()[0] - filter.getMin()[0]);
		scale.push_back(filter.getMax()[1] - filter.getMin()[1]);
		scale.push_back(filter.getMax()[2] - filter.getMin()[2]);

		pose.position.x = filter.getTranslation()[0]+filter.getMin()[0]+scale[0]/2.0;
		pose.position.y = filter.getTranslation()[1]+filter.getMin()[1]+scale[1]/2.0;
		pose.position.z = filter.getTranslation()[2]+filter.getMin()[2]+scale[2]/2.0;

		publish_marker(pose, scale, id);
		ros::Duration(0.01).sleep();
	}

	void publish_marker(geometry_msgs::Pose pose, std::vector<double> scale, int id = 0){
		visualization_msgs::Marker msg;
		msg.header.frame_id = frame;
		msg.header.stamp = ros::Time().now();
		msg.id = id;
		msg.type = 1;
		msg.action = 0;
		msg.pose = pose;
		msg.scale.x = scale[0];
		msg.scale.y = scale[1];
		msg.scale.z = scale[2];
		msg.color.r = 1;
		msg.color.g = 0;
		msg.color.b = 0;
		msg.color.a = 1;
		msg.lifetime = ros::Duration(10.0);

		marker_pub.publish(msg);
	} //publish_marker

	void publish_table(pcl::PointCloud<PointType>::Ptr  cloud){
		sensor_msgs::PointCloud2::Ptr output_cloud (new sensor_msgs::PointCloud2);
		pcl::toROSMsg (*cloud, *output_cloud);
		output_cloud->header.stamp = ros::Time::now();
		output_cloud->header.frame_id = frame;
		table_pub.publish(output_cloud);
	}

	void publish_objects(pcl::PointCloud<PointType>::Ptr  cloud){
		sensor_msgs::PointCloud2::Ptr output_cloud (new sensor_msgs::PointCloud2);
		pcl::toROSMsg (*cloud, *output_cloud);
		output_cloud->header.stamp = ros::Time::now();
		output_cloud->header.frame_id = frame;
		objects_pub.publish(output_cloud);
	}

	void seperateTable(pcl::PointCloud<PointType>::Ptr &table_cloud, pcl::PointCloud<PointType>::Ptr &objects_cloud, double& table_height){
	  double depth = 1.0;
	  double width = 1.0;
	  double max_table_height = 1.2;
	  double min_table_height = 0.6;
	  double step_size = 0.05;
	  double z_step_size = 0.01;

	  pcl::PointCloud<PointType>::Ptr cloudCroped (new pcl::PointCloud<PointType>);
	  pcl::PointCloud<PointType>::Ptr temp (new pcl::PointCloud<PointType>);

	  //initialize cropBox
	  pcl::CropBox<PointType> cropBoxFilter;
	  cropBoxFilter.setInputCloud(cloud);

	  //filter out our scan box
	  cropBoxFilter.setMin(Eigen::Vector4f(0.5, -0.5, 0.1, 0.0));
	  cropBoxFilter.setMax(Eigen::Vector4f(1.3, 0.5, 2.0, 0.0));

	  cropBoxFilter.setNegative(false);
	  cropBoxFilter.filter(*temp);
	  *cloud = *temp;
	  publish_table(temp);
	  ros::spinOnce();
	  ros::Duration(2.0).sleep();
	  cropBoxFilter.setInputCloud(cloud);

	  //find the table edge in front of the robot trough 2 x values
	  // =========================== edge
	  //    x1                x2
	  //            robot

	  cropBoxFilter.setMin(Eigen::Vector4f(-0.02, -0.01, 0.0, 0.0));
	  cropBoxFilter.setMax(Eigen::Vector4f(0.0, 0.01, max_table_height, 0.0));


	  double x = 0;
	  double y = 0;
	  double z = 0;
	  double step = 0.01;
	  double x0, x1;
	  bool collision = false;
	  y = -0.1;
	  x = 0.5; //base + laser
	  while(!collision && x < 2.0){
	    cropBoxFilter.setTranslation(Eigen::Vector3f (x, y, z));
	    cropBoxFilter.setRotation(Eigen::Vector3f (0, 0, 0));

	    cropBoxFilter.setNegative(false);
	    cropBoxFilter.filter(*cloudCroped);
	    publish_marker(cropBoxFilter, 1);
	    if(cloudCroped->size() > 0){
	      if(step == 0.01){
	        collision = true;
	      } else {
	        x -= step;
	        step = step/10;
	      }
	    }
	    x += step;
	  }

	  x0 = x;
	  y = 0.1;
	  x = 0.45; //base + laser
	  collision = false;
	  while(!collision && x < 2.0){
	    cropBoxFilter.setTranslation(Eigen::Vector3f (x, y, z));
	    cropBoxFilter.setRotation(Eigen::Vector3f (0, 0, 0));

	    cropBoxFilter.setNegative(false);
	    cropBoxFilter.filter(*cloudCroped);
	    publish_marker(cropBoxFilter, 2);
	    if(cloudCroped->size() > 0){
	      if(step == 0.01){
	        collision = true;
	      } else {
	        x -= step;
	        step = step/10;
	      }
	    }
	    x += step;
	  }
	  x1 = x;


	  double yaw = atan2((x0-x1), 0.2);
	  ROS_INFO("Table angle = %f", yaw*180.0/M_PI);


	  //detect the table height from below
	  cropBoxFilter.setMin(Eigen::Vector4f(-0.02, -0.02, -0.02, 0.0));
	  cropBoxFilter.setMax(Eigen::Vector4f(0.02, 0.02, 0.0, 0.0));

	  if(x0 < x1){
	      x = x1 + 0.05;
	  } else {
	      x = x0 + 0.05;
	  }
	  y = 0.0;
	  z = min_table_height;
	  step = 0.01;
	  collision = false;
	  while(!collision && z < max_table_height){
	    cropBoxFilter.setTranslation(Eigen::Vector3f (x, y, z));
	    cropBoxFilter.setRotation(Eigen::Vector3f (0, 0, 0));

	    cropBoxFilter.setNegative(false);
	    cropBoxFilter.filter(*cloudCroped);
	    publish_marker(cropBoxFilter, 3);
	    if(cloudCroped->size() > 0){
	      if(step == 0.001){
	        collision = true;
	      } else {
	        z -= step;
	        step = step/10;
	      }
	    }
	    z += step;
	  }

	  table_height = z;
	  ROS_INFO("Table height = %f", table_height);

	  //remove table and set table cloud
	  //remove everything below the table plane
	  cropBoxFilter.setMin(Eigen::Vector4f(0.35, -0.5, z-0.005, 0.0));
	  cropBoxFilter.setMax(Eigen::Vector4f(1.5, 0.5, z+1, 0.0));

	  cropBoxFilter.setTranslation(Eigen::Vector3f (0, 0, 0));
	  cropBoxFilter.setRotation(Eigen::Vector3f (0, 0, 0));

	  cropBoxFilter.setNegative(false);
	  cropBoxFilter.filter(*temp);
	  *cloud = *temp;
	  cropBoxFilter.setInputCloud(cloud);


	  //seperate the table for visualisation
	  cropBoxFilter.setMin(Eigen::Vector4f(0.0, -5.0, z-0.005, 0.0));
	  cropBoxFilter.setMax(Eigen::Vector4f(1.5, 5.0, z+0.02, 0.0));

	  cropBoxFilter.setNegative(false);
	  cropBoxFilter.filter(*table_cloud);
	  publish_table(table_cloud);

	  //remove the table from the cloud and return only objects
	  cropBoxFilter.setNegative(true);
	  cropBoxFilter.filter(*objects_cloud);
	  publish_objects(objects_cloud);
	}

	void fit_bounding_box(pcl::PointCloud<PointType>::Ptr cloud, Box &bounding_box, double z_level){

	  Eigen::Vector4f min, max;
	  Eigen::Vector4f centroid;

	  double minVol = 10000;
	  double minYaw = 0.0;

	  pcl::PointCloud<PointType>::Ptr cloud_transformed (new pcl::PointCloud<PointType>);

	  //now fit the bounding box by rotating around z and computing the new size
	  for(double yaw = 0; yaw < 90; yaw+=0.5){
	    //set the rotation
	    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	    transform.rotate (Eigen::AngleAxisf (M_PI/180.0*yaw, Eigen::Vector3f::UnitZ()));

	    //transform the cloud
	    pcl::transformPointCloud (*cloud, *cloud_transformed, transform);

	    pcl::getMinMax3D(*cloud_transformed, min, max);
	    bounding_box.size[0] = max[0]-min[0];
	    bounding_box.size[1] = max[1]-min[1];
	    bounding_box.size[2] = max[2]-z_level;

	    centroid[0] = min[0] + bounding_box.size[0]/2.0;
	    centroid[1] = min[1] + bounding_box.size[1]/2.0;
	    centroid[2] = z_level + bounding_box.size[2]/2.0;



	    double vol = bounding_box.size[0]*bounding_box.size[1]*bounding_box.size[2];
	    if(vol < 0){

	    } else if(vol < minVol){
	      minVol = vol;
	      minYaw = yaw;
	    }
	  }

	  //use best value to calculate final result

	  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	  transform.rotate (Eigen::AngleAxisf (M_PI/180.0*minYaw, Eigen::Vector3f::UnitZ()));

	  //transform the cloud
	  pcl::transformPointCloud (*cloud, *cloud_transformed, transform);

	  pcl::getMinMax3D(*cloud_transformed, min, max);
	  bounding_box.size[0] = max[0]-min[0];
	  bounding_box.size[1] = max[1]-min[1];
	  bounding_box.size[2] = max[2]-z_level;

	  centroid[0] = min[0] + bounding_box.size[0]/2.0;
	  centroid[1] = min[1] + bounding_box.size[1]/2.0;
	  centroid[2] = z_level + bounding_box.size[2]/2.0;

	  //rotate center back to inertial frame
	  centroid = transform.inverse()*centroid;

	  bounding_box.position[0] = centroid[0];
	  bounding_box.position[1] = centroid[1];
	  bounding_box.position[2] = centroid[2];
	  bounding_box.orientation[0] = 0;
	  bounding_box.orientation[1] = 0;
	  bounding_box.orientation[2] = -M_PI/180.0*minYaw;
	} //fit_bounding_box

	void segmentation(pcl::PointCloud<PointType>::Ptr  cloud,  std::vector<pcl::PointCloud<PointType>::Ptr> &objects_clouds){
		std::vector <pcl::PointIndices> clusters;
	    //set up the euclidean Cluster objects
	    pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
	    tree->setInputCloud (cloud);

		//remove all clusters smaller than 100 points
	    pcl::EuclideanClusterExtraction<PointType> ec;
	    ec.setClusterTolerance (0.005); // 10mm distance between points
	    ec.setMinClusterSize (1);
	    ec.setMaxClusterSize (50000);
	    ec.setSearchMethod (tree);
	    ec.setInputCloud (cloud);
	    ec.extract (clusters);

	    ROS_INFO("number of clusters with 1 or more points %d", (int)clusters.size());

	    if(clusters.size() > 0){
			pcl::PointIndices clean_cluster;
			clean_cluster.header = clusters[0].header;
			//extract all cluster into new cloud
			for(int i = 0; i < clusters.size(); i++){
				//ROS_INFO("clusters size = %d \n", (int)clusters[i].indices.size());
				if(clusters[i].indices.size() > 100){
				clean_cluster.indices.reserve( clean_cluster.indices.size() + clusters[i].indices.size());
				clean_cluster.indices.insert(clean_cluster.indices.end(), clusters[i].indices.begin(), clusters[i].indices.end());
				}
			}

		pcl::PointCloud<PointType>::Ptr objectsCloud (new pcl::PointCloud<PointType>);
		pcl::ExtractIndices<PointType> extract_indices;
		pcl::IndicesPtr object_indices (new std::vector <int>);
		*object_indices = clean_cluster.indices;

		ROS_INFO("number of points before filtering %d", (int)cloud->size());

		extract_indices.setInputCloud (cloud);
		extract_indices.setIndices (object_indices);
		extract_indices.setNegative (false);
		extract_indices.filter(*cloud);

		ROS_INFO("number of points afterfiltering %d", (int)objectsCloud->size());

		clusters.clear();

		tree = pcl::search::KdTree<PointType>::Ptr(new pcl::search::KdTree<PointType>);
		ec = pcl::EuclideanClusterExtraction<PointType>();
		tree->setInputCloud (cloud);

		//remove all clusters smaller than 100 points
	    ec.setClusterTolerance (0.02); // 20mm distance between points
	    ec.setMinClusterSize (200);
	    ec.setMaxClusterSize (5000);
	    ec.setSearchMethod (tree);
	    ec.setInputCloud (cloud);
	    ec.extract (clusters);

	    }
	    //extract the clustern into their own pointclouds
	    ROS_INFO("%d clusters found \n", (int)clusters.size());
	    objects_clouds.clear();
	    for(int i = 0; i < clusters.size(); i++){
	    	objects_clouds.push_back(pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>()));
	    	pcl::ExtractIndices<PointType> extract_indices;
			pcl::IndicesPtr object_indices (new std::vector <int>);
			*object_indices = clusters[i].indices;
			extract_indices.setInputCloud (cloud);
			extract_indices.setIndices (object_indices);
			extract_indices.setNegative (false);
			extract_indices.filter(*objects_clouds[i]);
	    }
	}

	bool search_object(pcl::PointCloud<PointType>::Ptr  cloud, Object object, double table_height){
	  //extract the indices in a new pointcloud

	  ROS_INFO("Fitting Bounding box to object pointcloud");
	  Box box;
	  fit_bounding_box(cloud, box, table_height);
	  publish_objects(cloud);
	  publish_marker(box);

	  ROS_INFO("comparing bounding box with object %s", object.getName().c_str());
	  if(compare_size(box, object)){
		  publish_marker(object.getBox());
		  return true;
	  }
	  return false;

	}

	bool compare_size(Box bounding_box, Object &object){
	  double vol = bounding_box.size[0] * bounding_box.size[1] * bounding_box.size[2];
	  Box object_box = object.getBox();
	  double object_vol = object_box.size[0] * object_box.size[1] * object_box.size[2];

	  ROS_INFO("vol %f object_vol %f", vol, object_vol);

	  std::vector<double> rpy;
	  rpy.resize(3);
	  rpy[0] = 0;
	  rpy[1] = 0;
	  rpy[2] = 0;


	  if(fabs(vol - object_vol) < object_vol * 0.4){
	   ROS_INFO("Checking Bounding Box orientation \n");

	   int i;
	   double error = 10;
	   int index;
	   std::vector <double> final_rpy;
	    for(i = 0; i < 8; i++){
	      Eigen::Vector3f transformed_size(bounding_box.size[0], bounding_box.size[1], bounding_box.size[2]);
	      Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	      switch(i) {
	        case 0:
	        break;
	        case 1:
	          rpy[0] = M_PI/2.0;
	        break;
	        case 2:
	          rpy[1] = M_PI/2.0;
	        break;
	        case 3:
	          rpy[2] = M_PI/2.0;
	        break;
	        case 4:
	          rpy[0] = M_PI/2.0;
	          rpy[2] = M_PI/2.0;
	        break;
	        case 5:
	          rpy[1] = M_PI/2.0;
	          rpy[2] = M_PI/2.0;
	        break;
	        case 6:
	          rpy[0] = M_PI/2.0;
	          rpy[1] = M_PI/2.0;
	        break;
	        case 7:
	          rpy[0] = M_PI/2.0;
	          rpy[1] = M_PI/2.0;
	          rpy[2] = M_PI/2.0;
	        break;
	      }
	      transform.rotate (Eigen::AngleAxisf (rpy[0], Eigen::Vector3f::UnitX()));
	      transform.rotate (Eigen::AngleAxisf (rpy[1], Eigen::Vector3f::UnitY()));
	      transform.rotate (Eigen::AngleAxisf (rpy[2], Eigen::Vector3f::UnitZ()));
	      transformed_size = transform*transformed_size;
	      //ROS_INFO("transformed size = %f, %f, %f \n", transformed_size[0], transformed_size[1], transformed_size[2]);
	      double compare = fabs(fabs(transformed_size[0]) - object_box.size[0]);
	      compare = compare * fabs(fabs(transformed_size[1]) - object_box.size[1]);
	      compare = compare * fabs(fabs(transformed_size[2]) - object_box.size[2]);

	      if(compare < error){
	        index = i;
	        final_rpy = rpy;
	        error = compare;
	      }
	    }

	    ROS_INFO("Found Orientation %d \n", index);

	    rpy = final_rpy;
	    ROS_INFO("rpy = %f %f %f \n", rpy[0], rpy[1], rpy[2]);
	    object_box.position = bounding_box.position;
	    object_box.orientation[0] = rpy[0];
	    object_box.orientation[1] = rpy[1];
	    object_box.orientation[2] += rpy[2];
	    object.setBox(object_box);
	  }
	  return true;
	}


public: //member variables

private: //member variables
	ros::NodeHandle node_handle_;
	ros::AsyncSpinner spinner_;

	ros::Publisher test_cloud_pub;
	ros::Publisher table_pub;
	ros::Publisher objects_pub;
	ros::Publisher marker_pub;

	std::string frame;

	std::vector <Object> objects;
	std::vector <Object> detected_objects;

	pcl::PointCloud<PointType>::Ptr cloud;

};

namespace robin_odlib{

  void transform_cloud(pcl::PointCloud<PointType>::Ptr &cloud,const sensor_msgs::PointCloud2::ConstPtr input_cloud, std::string target_frame);
  void loadObjects(std::vector <Object>& objects);
  void seperateTable(pcl::PointCloud<PointType>::Ptr  cloud, pcl::PointCloud<PointType>::Ptr  table_cloud, double& table_height);
  void Segmentation(pcl::PointCloud<PointType>::Ptr  cloud, std::vector <pcl::PointIndices>& clusters);
  bool searchObject(pcl::PointCloud<PointType>::Ptr  cloud, pcl::PointIndices cluster, Object& object, std::vector <double>& pose, double table_height);
  void fitBoundingBox(pcl::PointCloud<PointType>::Ptr cloud, std::vector <double>& size, std::vector <double>& pose, double table_height);
  bool compareColor(pcl::PointCloud<PointType>::Ptr  cloud, pcl::PointIndices cluster, std::vector <int> HSVcolor);
  bool compareSize(std::vector <double> size, std::vector <double> object_size, std::vector <double>& rpy);
  void publish_cloud(pcl::PointCloud<PointType>::Ptr cloud, std::string frame, std::string name);
}

#endif
