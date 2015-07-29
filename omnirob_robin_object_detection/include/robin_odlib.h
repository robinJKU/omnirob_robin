#ifndef ROBIN_ODLIB_H
#define ROBIN_ODLIB_H

//The Robin Object Detection Library is a collection of useful function to segment a pointlcloud into parts which than can be searched for Shapes of type Shape

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

typedef pcl::PointXYZRGB PointType;
typedef pcl::Normal NormalType;

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
