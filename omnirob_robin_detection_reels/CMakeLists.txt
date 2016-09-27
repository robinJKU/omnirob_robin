cmake_minimum_required(VERSION 2.8.3)
project(omnirob_robin_detection_reels)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  eigen_conversions
  tf
  roscpp
  std_msgs
  message_generation
  diagnostic_msgs
  pcl_ros
  omnirob_robin_msgs
  ar_track_alvar_msgs
  ros_common_robin_tools
  dynamic_reconfigure
  cv_bridge
  image_transport

)

find_package(OpenCV REQUIRED)

# Generate used messages and services
# generate_messages(DEPENDENCIES std_msgs diagnostic_msgs omnirob_robin_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES omnirob_robin_driver
#  CATKIN_DEPENDS geometry_msgs/Twist roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)


add_executable(omnirob_robin_detection_reels src/detection_reels.cpp)
add_executable(omnirob_robin_detection_demo src/detection_demo.cpp)
add_dependencies(omnirob_robin_detection_reels omnirob_robin_msgs_gencpp)
add_dependencies(omnirob_robin_detection_demo omnirob_robin_msgs_gencpp)
target_link_libraries(omnirob_robin_detection_reels ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(omnirob_robin_detection_demo ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
