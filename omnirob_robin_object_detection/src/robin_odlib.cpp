#include <robin_odlib.h>
#include <ros/ros.h>
#include <sstream>


void robin_odlib::transform_cloud(pcl::PointCloud<PointType>::Ptr &cloud,const sensor_msgs::PointCloud2::ConstPtr input_cloud, std::string target_frame){
	tf::TransformListener *transform_listener;
	transform_listener = new tf::TransformListener();

	if( target_frame.empty() ){
		ROS_ERROR("Invalid frame (target fame %s), can't transform cloud", target_frame.c_str());
		return;
	}

	sensor_msgs::PointCloud2::Ptr cloud_transformed (new sensor_msgs::PointCloud2);
	//Transform cloud to target frame
	try {
		transform_listener->waitForTransform(target_frame, (*input_cloud).header.frame_id, (*input_cloud).header.stamp, ros::Duration(6.0) );
		pcl_ros::transformPointCloud (target_frame, *input_cloud, *cloud_transformed, *transform_listener);
	}
	catch(tf::TransformException e){
		ROS_INFO("Transform_PointCloud Error: %s", e.what());
	}

	pcl::fromROSMsg (*cloud_transformed, *cloud);

	if( transform_listener!=NULL ){
		delete transform_listener;
	}

}

void robin_odlib::publish_cloud(pcl::PointCloud<PointType>::Ptr cloud, std::string frame, std::string name){
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2> (name, 2);

	sensor_msgs::PointCloud2::Ptr output_cloud (new sensor_msgs::PointCloud2);
	pcl::toROSMsg (*cloud, *output_cloud);
	output_cloud->header.stamp = ros::Time::now();
	output_cloud->header.frame_id = frame;
	pub.publish(output_cloud);
	pub.publish(output_cloud);
	ROS_INFO("cloud published");


	//n.shutdown();
}

void robin_odlib::loadObjects(std::vector <Object>& objects){

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

}

  
void robin_odlib::seperateTable(pcl::PointCloud<PointType>::Ptr  cloud, pcl::PointCloud<PointType>::Ptr  table_cloud, double& table_height){
  double depth = 1.0;
  double width = 1.0;
  double max_table_height = 1.2;
  double min_table_height = 0.6;
  double step_size = 0.05;
  double z_step_size = 0.01;
  
  pcl::PointCloud<PointType>::Ptr cloudCroped (new pcl::PointCloud<PointType>);	
  
  //initialize cropBox
  pcl::CropBox<PointType> cropBoxFilter;
  cropBoxFilter.setInputCloud(cloud);
  
  //filter out our scan box
  cropBoxFilter.setMin(Eigen::Vector4f(0.5, -0.5, 0.1, 0.0));
  cropBoxFilter.setMax(Eigen::Vector4f(1.3, 0.5, 2.0, 0.0));
  
  cropBoxFilter.setNegative(false);
  
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
  ROS_INFO("x0 x1 %f %f", x0, x1);
  ROS_INFO("Table angle = %f", yaw*180.0/M_PI);
   
 
  //detect the table height from below
  cropBoxFilter.setMin(Eigen::Vector4f(-0.05, -0.05, -0.02, 0.0));
  cropBoxFilter.setMax(Eigen::Vector4f(0.05, 0.05, 0.0, 0.0)); 
  
  if(x0 < x1){
      x = x1 + 0.3;
  } else {
      x = x0 + 0.3;
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
  cropBoxFilter.filter(*cloud);
  
  cropBoxFilter.setInputCloud(cloud);
   
  
  //seperate the table for visualisation
  cropBoxFilter.setMin(Eigen::Vector4f(0.0, -5.0, z-0.005, 0.0));
  cropBoxFilter.setMax(Eigen::Vector4f(1.5, 5.0, z+0.02, 0.0));
    
  cropBoxFilter.setNegative(false);
  cropBoxFilter.filter(*table_cloud); 
  
  //remove the table from the cloud and return only objects
  cropBoxFilter.setNegative(true);
  cropBoxFilter.filter(*cloud);
}
  
  
//======================================================================================================================================  
//======================================================================================================================================  
//======================================================================================================================================  
//this function returns the indices of each objects cluster on the table  
  
void robin_odlib::Segmentation(pcl::PointCloud<PointType>::Ptr  cloud,  std::vector <pcl::PointIndices>& clusters){
        
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
    
    ROS_INFO("number of clusters with 1 or more points %d \n", (int)clusters.size());
    


    if(clusters.size() > 0){
		pcl::PointIndices clean_cluster;
		clean_cluster.header = clusters[0].header;
		//extract all cluster into new cloud
		for(int i = 0; i < clusters.size(); i++){
			ROS_INFO("clusters size = %d \n", (int)clusters[i].indices.size());
			if(clusters[i].indices.size() > 400){
			clean_cluster.indices.reserve( clean_cluster.indices.size() + clusters[i].indices.size());
			clean_cluster.indices.insert(clean_cluster.indices.end(), clusters[i].indices.begin(), clusters[i].indices.end());		
			}
		}
		
	pcl::PointCloud<PointType>::Ptr objectsCloud (new pcl::PointCloud<PointType>);	
	pcl::ExtractIndices<PointType> extract_indices; 
	pcl::IndicesPtr object_indices (new std::vector <int>);
	*object_indices = clean_cluster.indices; 

	ROS_INFO("number of points before filtering %d \n", (int)cloud->size());
  
	extract_indices.setInputCloud (cloud);
	extract_indices.setIndices (object_indices);
	extract_indices.setNegative (false);
	extract_indices.filter(*cloud);
	
	ROS_INFO("number of points afterfiltering %d \n", (int)objectsCloud->size());

	clusters.clear();
	
	tree = pcl::search::KdTree<PointType>::Ptr(new pcl::search::KdTree<PointType>);
	ec = pcl::EuclideanClusterExtraction<PointType>();
	tree->setInputCloud (cloud);
	
	//remove all clusters smaller than 100 points
    ec.setClusterTolerance (0.02); // 20mm distance between points
    ec.setMinClusterSize (1000);
    ec.setMaxClusterSize (50000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (clusters);


    //clusters.push_back(clean_cluster);
		     
    }    
    
    int size = clusters.size();
    ROS_INFO("%d clusters found \n", size);
}

//======================================================================================================================================  
//======================================================================================================================================  
//======================================================================================================================================  


bool robin_odlib::searchObject(pcl::PointCloud<PointType>::Ptr  cloud, pcl::PointIndices cluster, Object& object, std::vector <double>& pose, double table_height){
  //extract the indices in a new pointcloud
  ROS_INFO("searching for object %s \n", object.getName().c_str());
  pcl::PointCloud<PointType>::Ptr objectCloud (new pcl::PointCloud<PointType>);	
  pcl::ExtractIndices<PointType> extract_indices; 
  pcl::IndicesPtr object_indices (new std::vector <int>);
  *object_indices = cluster.indices; 
  
  extract_indices.setInputCloud (cloud);
  extract_indices.setIndices (object_indices);
  extract_indices.setNegative (false);
  extract_indices.filter(*objectCloud);
  
  std::vector <double> size;
  
  ROS_INFO("Fitting Bounding box to object pointcloud \n");
  fitBoundingBox(objectCloud, size, pose, table_height);
  
  std::vector <double> rpy;

  ROS_INFO("comparing size for correct orientation \n");
  compareSize(size, object.getSize(), rpy);
  pose[3] = rpy[0];
  pose[4] = rpy[1];
  pose[5] += rpy[2];
  
    
  return true;
}

//======================================================================================================================================  
//======================================================================================================================================  
//======================================================================================================================================

void robin_odlib::fitBoundingBox(pcl::PointCloud<PointType>::Ptr cloud, std::vector <double>& size, std::vector <double>& pose, double table_height){
  
  Eigen::Vector4f min, max; 
  Eigen::Vector4f centroid;  
  
  size.resize(3); 
  pose.resize(6); 
  
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
    size[0] = max[0]-min[0];
    size[1] = max[1]-min[1];
    size[2] = max[2]-table_height;  
  
    centroid[0] = min[0] + size[0]/2.0; 
    centroid[1] = min[1] + size[1]/2.0; 
    centroid[2] = table_height + size[2]/2.0; 
    
    
    
    double vol = size[0]*size[1]*size[2];
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
  size[0] = max[0]-min[0];
  size[1] = max[1]-min[1];
  size[2] = max[2]-table_height; 
  
  ROS_INFO("object size %f %f %f \n", size[0], size[1], size[2]);
  
  centroid[0] = min[0] + size[0]/2.0; 
  centroid[1] = min[1] + size[1]/2.0; 
  centroid[2] = table_height + size[2]/2.0;
  
  //rotate center back to inertial frame
  centroid = transform.inverse()*centroid;
  
  pose[0] = centroid[0];
  pose[1] = centroid[1];
  pose[2] = centroid[2];
  pose[3] = 0;
  pose[4] = 0;
  pose[5] = -M_PI/180.0*minYaw;
}

//======================================================================================================================================  
//======================================================================================================================================  
//======================================================================================================================================

bool robin_odlib::compareSize(std::vector <double> size, std::vector <double> object_size, std::vector <double>& rpy){
  double vol = size[0] * size[1] * size[2];
  double object_vol = object_size[0] * object_size[1] * object_size[2];
  
  ROS_INFO("vol %f object_vol %f", vol, object_vol);
  
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
      Eigen::Vector3f transformed_size(size[0], size[1], size[2]);
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
      double compare = fabs(fabs(transformed_size[0]) - object_size[0]);
      compare = compare * fabs(fabs(transformed_size[1]) - object_size[1]);
      compare = compare * fabs(fabs(transformed_size[2]) - object_size[2]);
      
      if(compare < error){
        index = i;
        final_rpy = rpy;       
        error = compare;
      }
    }
      
    ROS_INFO("Found Orientation %d \n", index);
      
    rpy = final_rpy;
    ROS_INFO("rpy = %f %f %f \n", rpy[0], rpy[1], rpy[2]);
  }  
  return true;
}

//======================================================================================================================================  
//======================================================================================================================================  
//======================================================================================================================================

bool robin_odlib::compareColor(pcl::PointCloud<PointType>::Ptr  cloud, pcl::PointIndices cluster, std::vector <int> HSVcolor){
  return true;
}
