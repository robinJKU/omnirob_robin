cmake_minimum_required(VERSION 2.8.3)
project(omnirob_robin_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  genmsg
  sensor_msgs
  diagnostic_msgs
  tf
  controller_manager
  hardware_interface
)

## Generate used messages and services
generate_messages(DEPENDENCIES std_msgs sensor_msgs diagnostic_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES omnirob_robin_driver
#  CATKIN_DEPENDS geometry_msgs/Twist roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(omnirob_joint_state_publisher src/omnirob_joint_state_publisher.cpp)
target_link_libraries(omnirob_joint_state_publisher ${catkin_LIBRARIES})

add_executable(base_interface src/base_interface.cpp)
target_link_libraries(base_interface ${catkin_LIBRARIES})

add_executable(gripper_interface src/gripper_interface.cpp)
target_link_libraries(gripper_interface ${catkin_LIBRARIES})

add_executable(omnirob_robin_hardware_interface_node src/omnirob_robin_hardware_interface_node.cpp)
target_link_libraries(omnirob_robin_hardware_interface_node ${catkin_LIBRARIES})
