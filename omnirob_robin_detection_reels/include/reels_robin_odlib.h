#ifndef ROBIN_ODLIB_H
#define ROBIN_ODLIB_H

#include <iostream>

//The Robin Object Detection Library is a collection of useful function to segment a pointlcloud into parts which than can be searched for Shapes of type Shape

#include <ros/ros.h>
#include <sstream>
#include <reels_robin_object.h>
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
#include <pcl/point_types.h>

//Library for RANSAC algorith (pcl)
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/random_sample.h>

#include <pcl/features/normal_3d.h>
#include <geometry_msgs/PoseArray.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/centroid.h>
#include "tf/transform_datatypes.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "tf_conversions/tf_eigen.h"
#include <pcl/sample_consensus/sac_model_circle.h>

/**
*The code implements the analysis of the point cloud for the estimation of the position
*and the dimensions of the cylindrical objects.
*It uses also the results coming from the RGB image to verify the effective presence of 
*the object with the specific color.
 */

using namespace std;

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
		objects_pub = node_handle_.advertise<sensor_msgs::PointCloud2> ("/objects_cloud", 10, true);
		base_pub = node_handle_.advertise<sensor_msgs::PointCloud2> ("/base_cloud", 10, true);

	}

	// ******* destructor
	~robin_object_detector()
	{
		//delete objects if necessary
	}

	void set_cloud(pcl::PointCloud<PointType>::Ptr &input_cloud){
		
	        cloud=input_cloud;
	}

	void set_change(tf::Vector3 &origin_in, tf::Matrix3x3 &rotation_direct_in){

	        origin_target=origin_in;
	        rotation_direct_target=rotation_direct_in;
	}

	void set_markers(AR_Marker &marker){

	        markers.push_back(marker);
	}

	void get_markers(std::vector<AR_Marker> &markers_2){
		markers_2.resize(markers.size());
	        markers_2=markers;
	}

	void set_markers_frame(std::vector<AR_Marker> &markers_3){

	        markers=markers_3;
	}

	void set_frame(std::string frame){
		this->frame = frame;
	}

	std::vector<Object> get_objects(){
		return objects;
	}

	std::vector<Cylinder> get_detected_cylinder(){
		return detected_cylinder;
	}

	void set_reel_presence(bool &reel_presence_in){

	        reel_presence=reel_presence_in;
	}


//FUNCTION FOR THE OBJECT DETECTION

	void detect_objects(){

		pcl::PointCloud<PointType>::Ptr base_cloud = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
		pcl::PointCloud<PointType>::Ptr objects_cloud = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
		Eigen::VectorXf coefficients_base(4);
		ros::param::get("/debugging", debugging);

		if (debugging==1){
			ROS_INFO("Number of markers found= %d",(int)markers.size());
		}
 
		seperateBase(base_cloud,objects_cloud,coefficients_base);

		tf::Vector3 normal_objects(-coefficients_base[0],-coefficients_base[1],-coefficients_base[2]);

	       	publish_objects(objects_cloud);
	  	publish_base(base_cloud);

		if (debugging==1){
	  		ros::Duration(5).sleep();
		}

	        ROS_INFO("Segmentation of objects and detection of the cylinders");

		std::vector<Cylinder> cylinder_found;
		findCylinders(objects_cloud,cylinder_found,normal_objects,coefficients_base);

		std::vector<int> cnt;
		cnt.resize(objects.size());

		double distance_from_base=0.0;//Fixed distance between the vertical plane of the machine and the reel
		int n_max=2;//Maximum number of reels in the warehouse

		ROS_INFO("\nCORRECTION OF THE POSITION USING THE REFERENCE OBJECTS AND MARKERS");
		for (int i=0;i<cylinder_found.size();i++){
			for (int j=0;j<objects.size();j++){
	
				comparison_cylinder(objects[j], cylinder_found[i],detected_cylinder,cnt[j],coefficients_base,distance_from_base,n_max);		
				
			}
		}

		Object reel_object, pin_object;
		for (int j=0;j<objects.size();j++){
			if (objects[j].getName().compare("empty_reel")==0){
				reel_object=objects[j];
			}
			if (objects[j].getName().compare("pin")==0){
				pin_object=objects[j];
			}
			
		}

		ROS_INFO("\nCOMPARISON WITH THE RESULT OF THE RGB IMAGE ANALYSIS");
		comparison_imageRGB(detected_cylinder,coefficients_base,reel_object,pin_object,distance_from_base);

		for (int i=0;i<detected_cylinder.size();i++){

		publish_cylinder(detected_cylinder[i], i);

		}
	}


	void getPlaneMachine(tf::Vector3 &normal,tf::Vector3 &center,double &distance, tf::Vector3 &width, tf::Vector3 &half_height){

		tf::Vector3 position2,position3,position4;

		for (int k=0;k<markers.size();k++){

			if (markers[k].marker_id==2){
			position2=markers[k].position;
			}
			else if(markers[k].marker_id==3){
			position3=markers[k].position;
			}
			else if(markers[k].marker_id==4){
			position4=markers[k].position;
			}
		}
		
		width=position3-position2;
		half_height=position4-position2;
		normal=((position3-position2).cross(position4-position2))/((position3-position2).cross(position4-position2)).length();
		distance=-normal.dot(position2);
		ROS_INFO("Distance of the plane from the origin of the frame = %f",distance);
		center=position4+width/2;


	}
private: //functions

	void load_objects(){
		
		std::vector <std::string> object_names;
	  	ros::param::get("/detectable_objects", object_names);

	  	for(int i = 0; i < object_names.size(); i++){
	    		//create the variables for the constructor + param namespace
	    		std::string name = object_names[i];
	    		std::string name_space = "/objects/" + name;

	    		//construct a new object and push it in our vector
	    		objects.push_back(Object(name));

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

	}

	void publish_marker(Box box, int id = 0){
		std::vector<double> scale;
		geometry_msgs::Pose pose;
		scale = box.size;
		pose = box.get_pose();

		publish_marker(pose, scale, id);
	}

	void publish_cylinder(Cylinder cylinder, int id = 0){
		std::vector<double> scale;
		geometry_msgs::Pose pose;
		scale = cylinder.size;
		pose = cylinder.get_pose();

		publish_cylinder(pose, scale, id);
	}

	void publish_cylinder(geometry_msgs::Pose pose, std::vector<double> scale, int id = 0){
		visualization_msgs::Marker msg;
		msg.header.frame_id = frame;
		msg.header.stamp = ros::Time().now();
		msg.id = id;
		msg.type = 3;
		msg.action = 0;
		msg.pose = pose;
		msg.scale.x = scale[0];
		msg.scale.y = scale[1];
		msg.scale.z = scale[2];
		msg.color.r = 0;
		msg.color.g = 1;
		msg.color.b = 0;
		msg.color.a = 1;
		msg.lifetime = ros::Duration(100.0);
		marker_pub.publish(msg);
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
		if (debugging==1){
			ros::Duration(5).sleep();
		}
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
		if (debugging==1){
		msg.lifetime = ros::Duration(10);
		}
		else{
		msg.lifetime = ros::Duration(0.01);
		}
		marker_pub.publish(msg);
	} 

	void publish_base(pcl::PointCloud<PointType>::Ptr  cloud){
		sensor_msgs::PointCloud2::Ptr output_cloud (new sensor_msgs::PointCloud2);
		pcl::toROSMsg (*cloud, *output_cloud);
		output_cloud->header.stamp = ros::Time::now();
		output_cloud->header.frame_id = frame;
		base_pub.publish(output_cloud);
	}

	void publish_objects(pcl::PointCloud<PointType>::Ptr  cloud){
		sensor_msgs::PointCloud2::Ptr output_cloud (new sensor_msgs::PointCloud2);
		pcl::toROSMsg (*cloud, *output_cloud);
		output_cloud->header.stamp = ros::Time::now();
		output_cloud->header.frame_id = frame;
		objects_pub.publish(output_cloud);
	}


//FUNCTION FOR THE SEPARATION OF THE BASE


	void seperateBase(pcl::PointCloud<PointType>::Ptr &base_cloud, pcl::PointCloud<PointType>::Ptr &objects_cloud,Eigen::VectorXf &coefficients_base){

	  	ROS_INFO("Separation of the interesting scene with a CropBox using the three markers");

	  	pcl::PointCloud<PointType>::Ptr cloudCroped (new pcl::PointCloud<PointType>);
	  	pcl::PointCloud<PointType>::Ptr temp (new pcl::PointCloud<PointType>);
		tf::Vector3 normal, center,width,half_height;
		double distance, h ,b;
		getPlaneMachine(normal,center,distance,width,half_height);
		b=width.length();
		h=half_height.length()*2;

		double profondplus=0.05;
		double profondminus=0.20;


	  	//initialize cropBox to focus only on the scene
	  	pcl::CropBox<PointType> cropBoxFilter;
	  	cropBoxFilter.setInputCloud(cloud);
		
		//--------------SET OF THE CROPBOX FOR THE SCENE----------

	  	cropBoxFilter.setMin(Eigen::Vector4f(center.x()-profondminus, center.y()-b/2, center.z()+0.10,0.0));
	  	cropBoxFilter.setMax(Eigen::Vector4f(center.x()+profondplus, center.y()+b/2, center.z()+h/2,0.0));

		//--------------------------------------------------------

	  	cropBoxFilter.setNegative(false);
	  	cropBoxFilter.filter(*temp);
		publish_marker(cropBoxFilter, 10);

		if (debugging==1){
			ros::Duration(10).sleep();
		}

	  	*cloud = *temp;
		
		if (debugging==1){
  			ROS_INFO("Number of points of the cloud after the CropBox application = %d",(int)cloud->size());
		}

		publish_objects(cloud);
		
		if (debugging==1){
			ros::Duration(5).sleep();
		}

	  	ros::spinOnce();


		if (debugging==1){
	  		ROS_INFO("Detection of the vertical base with the RANSAC algorithm");
	  		pcl::ExtractIndices<PointType> extract_objects;
	  		pcl::ExtractIndices<PointType> extract_base;
	  		boost::shared_ptr<vector<int> > inliers (new vector<int>);
	  		pcl::SampleConsensusModelPlane<PointType>::Ptr model_p (new pcl::SampleConsensusModelPlane<PointType> (cloud));
	  		pcl::RandomSampleConsensus<PointType> ransac (model_p);
    	  		ransac.setDistanceThreshold (0.01);
    	  		ransac.computeModel();
          		ransac.getInliers (*inliers);
			ransac.getModelCoefficients (coefficients_base); //Hessian coefficients of the plane
	  		ROS_INFO("Found a plane with %d inliers",(int)inliers->size());
			ROS_INFO("Distance of the plane from the origin with ransac= %f",coefficients_base[3]);
		}

	  	ROS_INFO("Elimination of the plane with a cropbox filter");

	  	pcl::CropBox<PointType> cropBoxFilterBase;
	  	cropBoxFilterBase.setInputCloud(cloud);
	  	cropBoxFilterBase.setMin(Eigen::Vector4f(center.x()-0.004, center.y()-b/2, center.z(),0.0));
	  	cropBoxFilterBase.setMax(Eigen::Vector4f(center.x()+0.10, center.y()+b/2, center.z()+h/2,0.0));

	  	cropBoxFilterBase.setNegative(true);
	  	cropBoxFilterBase.filter(*objects_cloud);
		publish_marker(cropBoxFilterBase, 10);
	
		if (debugging==1){
			ros::Duration(10).sleep();
		}

	  	cropBoxFilterBase.setNegative(false);
	  	cropBoxFilterBase.filter(*base_cloud);
		
		coefficients_base[0]=normal.x();
		coefficients_base[1]=normal.y();
		coefficients_base[2]=normal.z();
		coefficients_base[3]=distance;
	}

//FUNCTION FOR THE RESEARCH OF CYLINDERS

	void findCylinders(pcl::PointCloud<PointType>::Ptr &objects_cloud,std::vector<Cylinder> &cylinder_found,tf::Vector3 &normal_objects,Eigen::VectorXf &coefficients_base){

		// Application of the calusterisation to separate the cloud in clusters
	  	ROS_INFO("Segmentation of objects and filtering of stray points");
	      	std::vector<pcl::PointCloud<PointType>::Ptr> objects_clouds;
		segmentation(objects_cloud, objects_clouds);
		ROS_INFO("Number of cluster found in the point cloud = %d", (int)objects_clouds.size());
	        ROS_INFO("Analysis of the found clusters");
		int n_cylinder=0;
          	
		for(int j=0; j<objects_clouds.size();j++){

          		*objects_cloud=*objects_clouds[j];
	  		publish_objects(objects_cloud);

			if (debugging==1){
	  			ros::Duration(5).sleep();
			}

			std::vector<pcl::PointCloud<PointType>::Ptr> plane_clouds_tot;//Vector containing the total planes
			std::vector<pcl::PointCloud<PointType>::Ptr> plane_clouds;//Vector containing only the interesting plane
	  		pcl::PointCloud<PointType>::Ptr plane (new pcl::PointCloud<PointType>);
			pcl::PointCloud<PointType>::Ptr cloud_temp (new pcl::PointCloud<PointType>);

	  		pcl::ExtractIndices<PointType> extract_plane;
			pcl::ExtractIndices<PointType> extract_objects;
			boost::shared_ptr<vector<int> > inliers (new vector<int>);

	  		std::vector<Eigen::VectorXf> coeff;//Vector containing the coefficients of the planes
			
	  		int n_plane=0;
			bool ver=true;
	  		int i=0;
			int n_min=300;

		        ROS_INFO("Detection of planes in the cluster with RANSAC");

	  		while(ver){

				//Application of the RANSAC angorithm to detect all the possible planes
				if (objects_cloud->size()<n_min){
					break;
				}

	  			pcl::SampleConsensusModelPlane<PointType>::Ptr model_p (new pcl::SampleConsensusModelPlane<PointType> (objects_cloud));
	  			pcl::RandomSampleConsensus<PointType> ransac (model_p);
    	  			ransac.setDistanceThreshold (0.008);
    	  			ransac.computeModel();
				ransac.getInliers (*inliers);
          			ROS_INFO("Found a plane with %d inliers",(int)inliers->size());
	  
				if (inliers->size()<n_min){
	  				break;
          			};

	  			extract_objects.setInputCloud (objects_cloud);
	  			extract_objects.setIndices (inliers);
	  			extract_objects.setNegative (true);
	  			extract_objects.filter (*cloud_temp);

	  			extract_plane.setInputCloud (objects_cloud);
	  			extract_plane.setIndices (inliers);
	  			extract_plane.setNegative (false);
	  			extract_plane.filter (*plane);
	  			plane_clouds_tot.push_back(pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>()));
          			*plane_clouds_tot[i]=*plane;
				n_plane=plane_clouds_tot.size();

				publish_objects(plane);
				
				if (debugging==1){
					ros::Duration(2).sleep();
				}

          			Eigen::VectorXf coefficiente;
	  			ransac.getModelCoefficients (coefficiente); 
	  			coeff.push_back(coefficiente);
	  			*objects_cloud=*cloud_temp;

	  			i++;
          		};


		        ROS_INFO("Number of planes found in the cluster %d = %d",j+1,(int)plane_clouds_tot.size());

			int h=0;
			std::vector<tf::Vector3> normals;
			std::vector<tf::Vector3> origins;
			std::vector<pcl::PointCloud<PointType>::Ptr>clouds_projected;
			
			for(int k=0;k<plane_clouds_tot.size();k++){


	  			pcl::PointCloud<PointType>::Ptr cloud_proj (new pcl::PointCloud<PointType>);	
  	  			pcl::ProjectInliers<PointType> proj;
  	  			proj.setModelType (pcl::SACMODEL_PLANE);
          			proj.setInputCloud (plane_clouds_tot[k]);
	  			pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
          			coefficients->values.resize (4);

          			coefficients->values[0] = coeff[k][0];
	  			coefficients->values[1] = coeff[k][1];
	  			coefficients->values[2] = coeff[k][2];
	  			coefficients->values[3] = coeff[k][3];

          			proj.setModelCoefficients (coefficients);
          			proj.filter (*cloud_proj);
	  
	  			std::vector<pcl::PointCloud<PointType>::Ptr> cluster_plane_clouds;
		
				if (debugging==1){
	  				ROS_INFO("Segmentation of the plane to eliminate stray points");
				}
	  			segmentation1(cloud_proj, cluster_plane_clouds);

				if (cluster_plane_clouds.size()>0){
					*cloud_proj=*cluster_plane_clouds[0];

					if (debugging==1){
						ROS_INFO("Search of the centroid of the point cloud");
					}

	  				Eigen::Vector4f c;
	  				pcl::compute3DCentroid<PointType>(*cloud_proj,c);
	  
	  				tf::Vector3 origine(c[0], c[1], c[2]);
					tf::Vector3 normale;
				
					if (debugging==1){
						ROS_INFO("Check on the normal direction");
					}

					directionNormal(origine,coeff,normale,k,normal_objects);

					if (debugging==1){
						ROS_INFO("Normal of the plane x= %f y= %f z= %f",normale[0],normale[1],normale[2]);
					}

					//Test with the normal of the real reels
					double product=normale.dot(normal_objects);
					if (product>0.8){
						plane_clouds.resize(h+1);
						normals.resize(h+1);
						origins.resize(h+1);
						clouds_projected.resize(h+1);
						plane_clouds[h]=plane_clouds_tot[k];
						normals[h]=normale;
						origins[h]=origine;
						clouds_projected[h]=cloud_proj;
						h++;
					}
				}
			}

			ROS_INFO("Number of planes found in the cluster %d with correct orientation = %d",j+1,(int)plane_clouds.size());
	                pcl::PointCloud<PointType>::Ptr object_cloud = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());

			if (debugging==1){			
				ROS_INFO("Research of a cylinder for every plane found with the correct normal");
			}

			for (int s=0; s<plane_clouds.size(); s++){
 
	  			*object_cloud=*plane_clouds[s];

	  			publish_objects(object_cloud);
				
				if (debugging==1){
	  				ros::Duration(0.01).sleep();
				}

				tf::Vector3 normal=normals[s];
				tf::Vector3 origin=origins[s];

				pcl::PointCloud<PointType>::Ptr cloud_projected (new pcl::PointCloud<PointType>);
				*cloud_projected=*clouds_projected[s];

	   			//Transformation of the normal of the plane in quaternions
				tf::Quaternion q;
				fromNormalToQuaternion_z(normal, q);
	  			
				pcl::PointCloud<PointType>::Ptr cloud_transformed (new pcl::PointCloud<PointType>);
	  			tf::Quaternion qi=q.inverse();

				//Move of the point cloud to the target_pose frame for the diameter analysis
	  			pcl::transformPointCloud (*cloud_projected, *cloud_transformed, Eigen::Vector3f (-origin[0], -origin[1], -origin[2]),Eigen::Quaternionf (1,0,0,0));
	  			publish_objects(cloud_transformed);
		
				if (debugging==1){
					ros::Duration(0.01).sleep();
				}

          			*cloud_projected=*cloud_transformed;
	  			pcl::transformPointCloud (*cloud_projected, *cloud_transformed, Eigen::Vector3f (0, 0, 0),Eigen::Quaternionf (qi[3],qi[0],qi[1],qi[2]));
	  			publish_objects(cloud_transformed);
				
				if (debugging==1){
					ros::Duration(0.01).sleep();
				}

				//Searcing for dyameter
				double diameter, standard_dev, c_x, c_y, height;

				findDiameter(cloud_transformed,diameter, standard_dev, c_x, c_y);

				if (debugging==1){

	    				ROS_INFO("Diameter =%f",diameter);
	    				ROS_INFO("Standard deviation =%f",standard_dev);
					ROS_INFO("Coordinate x of the center =%f",c_x);
					ROS_INFO("Coordinate y of the center =%f",c_y);
				}

				if (standard_dev*100<1){

					findHeight2 (clouds_projected[s], diameter, c_x, c_y, normal, origin, q, height,coefficients_base);
					ROS_INFO("\nATTENTION!!!!!!     FOUND A CIRCLE");
					ROS_INFO("Diameter =%f",diameter);
					ROS_INFO("Distance of the plane containing the circle from the vertical base =%f",height);

					cylinder_found.resize(n_cylinder+1);
				
					cylinder_found[n_cylinder].size[0]=diameter;
					cylinder_found[n_cylinder].size[1]=diameter;
					cylinder_found[n_cylinder].size[2]=height;

					tf::Vector3 localization;
					tf::Vector3 set_origin(c_x,c_y,0);
					tf::Matrix3x3 rotation(q);
					tf::Vector3 set_origin2=rotation*set_origin;

					localization=origin+set_origin2;

					cylinder_found[n_cylinder].normal[0]=normal[0];
					cylinder_found[n_cylinder].normal[1]=normal[1];
					cylinder_found[n_cylinder].normal[2]=normal[2];

					cylinder_found[n_cylinder].position[0]=localization[0];
					cylinder_found[n_cylinder].position[1]=localization[1];
					cylinder_found[n_cylinder].position[2]=localization[2];

					double roll, pitch, yaw;
					rotation.getRPY(roll, pitch, yaw);
					cylinder_found[n_cylinder].orientation[0]=roll;
					cylinder_found[n_cylinder].orientation[1]=pitch;
					cylinder_found[n_cylinder].orientation[2]=yaw;

					n_cylinder++;
				}else{
					ROS_INFO("the plane doesn't contain any circle");
				}
			}
		}
	}


//FUNCTION TO DIRECT THE NORMAL IN THE RIGHT DIRECTION (VIEW OF THE CAMERA)

   	void directionNormal(tf::Vector3 &origin,std::vector<Eigen::VectorXf> &coeff,tf::Vector3 &normal, int &k,tf::Vector3 &normal_objects){
	    	normal[0]=coeff[k][0];
	    	normal[1]=coeff[k][1];
	    	normal[2]=coeff[k][2];
		
		double scalar= normal.dot(normal_objects);
	
		if(scalar<0){

			//The normal is oriented opposite to the view point, CHANGE!
			normal=-normal;
		}
	}


//FUNCTION TO FIND THE DIAMETER OF THE CYLINDER

	void findDiameter(pcl::PointCloud<PointType>::Ptr &cloud_transformed, double &diameter, double &standard_dev, double &c_x, double &c_y){

		Box bounding_box;
	  	Eigen::Vector4f min, max;
	  	Eigen::Vector4f centroid;
		pcl::PointCloud<PointType>::Ptr cloud_transf (new pcl::PointCloud<PointType>);

	  	int cont=0;
	  	std::vector<double> diameters;
	  	std::vector<double> centroidx;
	  	std::vector<double> centroidy;

	  	for(double yaw = 0; yaw < 90; yaw+=5.0){
	    		//set the rotation
	    		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	    		transform.rotate (Eigen::AngleAxisf (M_PI/180.0*yaw, Eigen::Vector3f::UnitZ()));

	    		//transform the cloud
	    		pcl::transformPointCloud (*cloud_transformed, *cloud_transf, transform);

	    		pcl::getMinMax3D(*cloud_transf, min, max);
	    		bounding_box.size[0] = max[0]-min[0];
	    		bounding_box.size[1] = max[1]-min[1];
	    		bounding_box.size[2] = max[2]-min[2];

	    		centroid[0] = min[0] + bounding_box.size[0]/2.0;
	    		centroid[1] = min[1] + bounding_box.size[1]/2.0;
	    		centroid[2] = min[2] + bounding_box.size[2]/2.0;

	    		diameters.push_back(bounding_box.size[0]);
            		diameters.push_back(bounding_box.size[1]);

	    		centroidx.push_back(centroid[0]);
            		centroidy.push_back(centroid[1]);
	    	}
	    
	    	int n=diameters.size();
	    	int m=centroidx.size();

	   	double sum1=0,sum2=0,sum3=0,tempv=0;
	    
		for(int i=0;i<n;i++){
			sum1+=diameters[i];
	    	}
	    	for(int i=0;i<m;i++){
			sum2+=centroidx[i];
	        	sum3+=centroidy[i];
	    	}
	    	diameter=sum1/n;

	    	for(int i=0;i<n;i++){
			tempv+=(diameter-diameters[i])*(diameter-diameters[i]);
	    	}
	    	double variance=tempv/n;
	    	standard_dev=sqrt(variance);

  	  	pcl::ModelCoefficients::Ptr rad_center (new pcl::ModelCoefficients);
  	  	pcl::PointIndices::Ptr inliers_2 (new pcl::PointIndices);

	  	pcl::SACSegmentation<PointType> seg;
          	seg.setOptimizeCoefficients (true);
	  	seg.setModelType (pcl::SACMODEL_CIRCLE2D);
          	seg.setMethodType (pcl::SAC_RANSAC);
          	seg.setRadiusLimits(0.0,0.5);
  	  	seg.setDistanceThreshold (0.005);
	  	seg.setInputCloud (cloud_transformed);
	  	seg.segment (*inliers_2, *rad_center);
		c_x=rad_center->values[0];
		c_y=rad_center->values[1];

		
	}


//FUNCTION TO FIND THE DISTANCE BETWEEN THE FRONT FACE OF THE CYLINDER AND THE PLANE OF THE VERTICAL BASE

	void findHeight2 (pcl::PointCloud<PointType>::Ptr &objects_clouds, double &diameter, double &c_x, double &c_y,tf::Vector3 &normal,tf::Vector3 &origin,tf::Quaternion q,double &height,Eigen::VectorXf &coefficients_base){

		
	  	pcl::SampleConsensusModelPlane<PointType>::Ptr model_p (new pcl::SampleConsensusModelPlane<PointType> (objects_clouds));
		std::vector< double > distances;
		model_p->getDistancesToModel (coefficients_base,distances);	

		int n=distances.size();

	   	double sum=0;
	    
		for(int i=0;i<n;i++){
			sum+=distances[i];
	    	}

	    	height=sum/n;	
		 
}


//FUNCTION FOR THE COMPARISON BETWEEN THE REAL REELS AND THE HEIGHT

	void comparison_cylinder(Object &object, Cylinder &cylinder, std::vector<Cylinder> &detected_cylinder,int &cnt,Eigen::VectorXf &coefficients_base,double &distance_from_base,int &n_max){

		Box object_box = object.getBox();
		tf::Vector3 normal(-coefficients_base[0],-coefficients_base[1],-coefficients_base[2]);
		tf::Quaternion q;
		fromNormalToQuaternion_z(normal, q);
		tf::Matrix3x3 rotation(q);
		double roll, pitch, yaw;
		rotation.getRPY(roll, pitch, yaw);


		if (abs(cylinder.size[0]-object_box.size[0])<0.02){

			double delta_h;
			int n_object=1;
			cylinder.size[0]=object_box.size[0];
			cylinder.size[1]=object_box.size[1];
			delta_h=abs(cylinder.size[2]-object_box.size[2]);
			ROS_INFO("Found an object of type %s",object.getName().c_str());
			
			tf::Vector3 pose(cylinder.position[0],cylinder.position[1],cylinder.position[2]);
			tf::Vector3 position_correct;
			bool flag=1;
			bool correspondence=0;
			tf::Vector3 distance_reel_stop=coefficients_base[3]*normal-distance_from_base*normal;
			tf::Vector3 distance_plane=coefficients_base[3]*normal;

			tf::Vector3 position_stamped;
			tf::Vector3 position_found(cylinder.position[0],cylinder.position[1],cylinder.position[2]);
			position_stamped=position_found+origin_target;
			position_stamped=rotation_direct_target*position_stamped;

			ROS_INFO("Position face of object detected %s with respect to the base_link",object.getName().c_str());
			ROS_INFO("x= %f y= %f z= %f", position_stamped[0], position_stamped[1], position_stamped[2]);
		
			if (debugging==1){
    				ROS_INFO("Orientation of the face of the object detected with respect to the target r= %f p= %f y= %f",  cylinder.orientation[0], cylinder.orientation[1], cylinder.orientation[2]);
			}
			
			for (int k=0;k<markers.size();k++){


				if (abs(markers[k].position.x()-pose[0])<0.02 && abs(markers[k].position.y()-pose[1])<0.02 && abs(markers[k].position.z()-pose[2])<0.02){

					pose[0]=coefficients_base[3]-object_box.size[2];
					pose[1]=markers[k].position[1];
					pose[2]=markers[k].position[2];
					ROS_INFO("Found the marker %d as correspondent to the pin",markers[k].marker_id);	
					cylinder.size[2]=coefficients_base[3]-markers[k].position.dot(normal);
					flag=0;
					correspondence=1;

				}

				tf::Vector3 cross=markers[k].position.dot(normal)*normal;
				tf::Vector3 height_marker=markers[k].position-cross;
				tf::Vector3 distance_reel_stop=coefficients_base[3]*normal+height_marker-distance_from_base*normal;
					
				if(flag){

					for (int j=0;j<n_max;j++){

						tf::Vector3 reel_prima=distance_reel_stop-object_box.size[2]*normal*j;					
						tf::Vector3 reel=distance_reel_stop-object_box.size[2]*normal*(j+1);

						if(reel_prima[0]>pose[0] && reel[0]<pose[0] && abs(pose[1]-reel[1])<0.02 && abs(pose[2]-reel[2])<0.02){

							pose=reel;
							cylinder.size[2]=(j+1)*object_box.size[2];
							ROS_INFO("Found the marker %d as correspondent to the reel",markers[k].marker_id);
							correspondence=1;
						}
					}
				}
			}

 			if (correspondence){
				
				if (cylinder.size[2]>object_box.size[2]){
					double rapporto=cylinder.size[2]/object_box.size[2];
					double approx=round(rapporto);
					n_object=approx;
				}
	
				ROS_INFO("Number of objects of type %s = %d",object.getName().c_str(),n_object);			

				for (int t=0;t<n_object;t++){

					position_correct=pose+object_box.size[2]/2*normal+t*object_box.size[2]*normal;
					cylinder.position[0]=position_correct[0];
					cylinder.position[1]=position_correct[1];
					cylinder.position[2]=position_correct[2];
					cylinder.orientation[0]=roll;
					cylinder.orientation[1]=pitch;
					cylinder.orientation[2]=yaw;
					cylinder.size[2]=object_box.size[2];
					cylinder.name=object.getName().c_str();
					detected_cylinder.push_back(cylinder);
					detected_cylinder.back().set_number(cnt);
					cnt += 1;
				}
			}else{
	
			ROS_INFO("No correspondance with any marker");

			}

		}

	}


	void fromNormalToQuaternion_z(tf::Vector3 &normal, tf::Quaternion &q){

	  	tf::Vector3 axis_vector(normal);
	  	tf::Vector3 up_vector(0.0, 0.0, 1.0);
		tf::Vector3 down_vector(0.0, 0.0, -1.0);
		tf::Vector3 x_vector(1.0, 0.0, 0.0);

		if (axis_vector.dot(down_vector)>0.99){

			q.setRotation(x_vector, -1.0*acos(axis_vector.dot(up_vector)));
					
		}else{

			tf::Vector3 right_vector = axis_vector.cross(up_vector);
  			right_vector.normalized();
			q.setRotation(right_vector, -acos(axis_vector.dot(up_vector)));
				
		}

		q.normalize();

	}




	void comparison_imageRGB(std::vector<Cylinder> &detected_cylinder,Eigen::VectorXf &coefficients_base, Object &reel_object, Object &pin_object,double &distance_from_base){

		int cnt=0;
		Cylinder cylinder;
		tf::Vector3 position_correct;
		tf::Vector3 normal(-coefficients_base[0],-coefficients_base[1],-coefficients_base[2]);
		tf::Quaternion q;
		fromNormalToQuaternion_z(normal, q);
		tf::Matrix3x3 rotation(q);
		double roll, pitch, yaw;
		rotation.getRPY(roll, pitch, yaw);
		for (int i=0;i<detected_cylinder.size();i++){

			if (detected_cylinder[i].name.compare("empty_reel_0")==0){
				cnt++;
			}
		}
		
		if (cnt==0){
			ROS_INFO("No reel found with the analysis of the point cloud");
			if (reel_presence==false){
				ROS_INFO("No reel found with the analysis of the RGB image");
			}else{
				ROS_INFO("From the analysis of the RGB image we found a reel");
				Box object_box = reel_object.getBox();
				cylinder.size[0]=object_box.size[0];
				cylinder.size[1]=object_box.size[1];
				tf::Vector3 pose;

				for (int k=0;k<markers.size();k++){

					if (markers[k].marker_id==5){
				
						
						tf::Vector3 cross=markers[k].position.dot(normal)*normal;
						tf::Vector3 height_marker=markers[k].position-cross;
						tf::Vector3 distance_reel_stop=coefficients_base[3]*normal+height_marker-distance_from_base*normal;
						tf::Vector3 reel=distance_reel_stop-object_box.size[2]*normal;
						pose=reel;
						position_correct=pose+object_box.size[2]/2*normal;
						cylinder.position[0]=position_correct[0];
						cylinder.position[1]=position_correct[1];
						cylinder.position[2]=position_correct[2];
						cylinder.orientation[0]=roll;
						cylinder.orientation[1]=pitch;
						cylinder.orientation[2]=yaw;
						cylinder.size[2]=object_box.size[2];
						cylinder.name=reel_object.getName().c_str();
						detected_cylinder.push_back(cylinder);
						detected_cylinder.back().set_number(0);
					}
				}
			}
		}else{
			ROS_INFO("Reel already found with the analysis of the point cloud => analysis of the RGB image not necessary");
		}
		
		cnt=0;

		for (int i=0;i<detected_cylinder.size();i++){

			if (detected_cylinder[i].name.compare("pin_0")==0){
				cnt++;
			}
		}
		
		if (cnt==0){

			ROS_INFO("No pin found with the analysis of the point cloud");

			Box object_box = pin_object.getBox();
			cylinder.size[0]=object_box.size[0];
			cylinder.size[1]=object_box.size[1];
			tf::Vector3 pose;

			for (int k=0;k<markers.size();k++){

				if (markers[k].marker_id==5){
				
						
					tf::Vector3 cross=markers[k].position.dot(normal)*normal;
					tf::Vector3 height_marker=markers[k].position-cross;
					tf::Vector3 distance_base=coefficients_base[3]*normal+height_marker;
					tf::Vector3 pin=distance_base-object_box.size[2]*normal;
					pose=pin;
					position_correct=pose+object_box.size[2]/2*normal;
					cylinder.position[0]=position_correct[0];
					cylinder.position[1]=position_correct[1];
					cylinder.position[2]=position_correct[2];
					cylinder.orientation[0]=roll;
					cylinder.orientation[1]=pitch;
					cylinder.orientation[2]=yaw;
					cylinder.size[2]=object_box.size[2];
					cylinder.name=pin_object.getName().c_str();
					detected_cylinder.push_back(cylinder);
					detected_cylinder.back().set_number(0);
				}
			}

		}else{

			ROS_INFO("Pin already found with the analysis of the point cloud");

		}


	}

//FUNCTION THE SEGMENTATION/CLUSTERIZATION

	void segmentation(pcl::PointCloud<PointType>::Ptr  cloud,  std::vector<pcl::PointCloud<PointType>::Ptr> &objects_clouds){

	    std::vector <pcl::PointIndices> clusters;
	    //set up the euclidean Cluster objects
	    pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
	    tree->setInputCloud (cloud);

	    //remove all clusters smaller than 200 points
	    pcl::EuclideanClusterExtraction<PointType> ec;
	    ec.setClusterTolerance (0.004); // 4 mm distance between points (si setta infatti il raggio della sfera)
	    ec.setMinClusterSize (200);
	    ec.setMaxClusterSize (50000);
	    ec.setSearchMethod (tree);
	    ec.setInputCloud (cloud);
	    ec.extract (clusters);

		if (debugging==1){
	    		//extract the clustern into their own pointclouds
	    		ROS_INFO("%d clusters found ", (int)clusters.size());
		}

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

	void segmentation1(pcl::PointCloud<PointType>::Ptr  cloud,  std::vector<pcl::PointCloud<PointType>::Ptr> &objects_clouds){

	    std::vector <pcl::PointIndices> clusters;
	    //set up the euclidean Cluster objects
	    pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
	    tree->setInputCloud (cloud);

	    //remove all clusters smaller than 150 points
	    pcl::EuclideanClusterExtraction<PointType> ec;
	    ec.setClusterTolerance (0.008); // 8 mm distance between points (si setta infatti il raggio della sfera)
	    ec.setMinClusterSize (150);
	    ec.setMaxClusterSize (50000);
	    ec.setSearchMethod (tree);
	    ec.setInputCloud (cloud);
	    ec.extract (clusters);

		if (debugging==1){
	    		//extract the clustern into their own pointclouds
	    		ROS_INFO("%d clusters found ", (int)clusters.size());
		}

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

public: //member variables

private: //member variables
	ros::NodeHandle node_handle_;
	ros::AsyncSpinner spinner_;

	ros::Publisher test_cloud_pub;
	ros::Publisher objects_pub;
	ros::Publisher marker_pub;
	std::vector<AR_Marker> markers;

	std::string frame;

	std::vector <Object> objects;
	std::vector <Cylinder> detected_cylinder;

	pcl::PointCloud<PointType>::Ptr cloud;

	ros::Publisher base_pub;

	tf::Vector3 origin_target; 
	tf::Matrix3x3 rotation_direct_target;

	bool reel_presence;
	bool debugging;
};

namespace robin_odlib{

  void transform_cloud(pcl::PointCloud<PointType>::Ptr &cloud,const sensor_msgs::PointCloud2::ConstPtr input_cloud, std::string target_frame);
  void loadObjects(std::vector <Object>& objects);
  void segmentation(pcl::PointCloud<PointType>::Ptr  cloud, std::vector <pcl::PointIndices>& clusters);
  void publish_cloud(pcl::PointCloud<PointType>::Ptr cloud, std::string frame, std::string name);
}

#endif
