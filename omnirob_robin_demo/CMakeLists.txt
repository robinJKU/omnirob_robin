cmake_minimum_required(VERSION 2.8.3)
project(omnirob_robin_demo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	omnirob_robin_msgs
	omnirob_robin_tools_ros
	geometry_msgs
	omnirob_robin_moveit)

## Generate added messages and services with any dependencies listed here
#generate_messages(DEPENDENCIES omnirob_robin_msgs geometry_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES omnirob_robin_driver
#  CATKIN_DEPENDS geometry_msgs/Twist roscpp std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pick_and_place_demo src/pick_and_place_demo.cpp)
target_link_libraries(pick_and_place_demo ${catkin_LIBRARIES})
add_dependencies(pick_and_place_demo omnirob_robin_msgs_gencpp)
