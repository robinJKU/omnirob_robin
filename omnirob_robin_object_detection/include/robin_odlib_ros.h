#ifndef ROBIN_ODLIB_ROS_H
#define ROBIN_ODLIB_ROS_H

//This library provides functions to integrate the Robin Object Detection Library with ROS

#include <robin_object.h>

namespace robin_odlib_ros{
  
  void loadObjects(std::vector <Object>& objects);


}

#endif
