cmake_minimum_required(VERSION 2.8.3)
project(omnirob_robin_demo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
	omnirob_robin_msgs)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES omnirob_robin_msgs)

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

add_executable(pick_and_place_demo src/pick_and_place_demo.cpp)
target_link_libraries(pick_and_place_demo ${catkin_LIBRARIES})
