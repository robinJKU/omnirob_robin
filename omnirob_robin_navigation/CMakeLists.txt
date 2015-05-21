cmake_minimum_required(VERSION 2.8.3)
project(omnirob_robin_navigation)

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
  ar_track_alvar_msgs
  dynamic_reconfigure
)

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
  include
)

add_executable(marker_localization_global src/marker_localization_global.cpp)
target_link_libraries(marker_localization_global ${catkin_LIBRARIES})