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
  nav_msgs
  trajectory_msgs
  omnirob_robin_msgs
  omnirob_robin_tools_ros
  sensor_msgs
  urdf
)

## Generate used messages and services
# generate_messages(DEPENDENCIES std_msgs sensor_msgs diagnostic_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES omnirob_robin_driver
#  CATKIN_DEPENDS geometry_msgs/Twist roscpp std_msgs
#  DEPENDS system_lib
   INCLUDE_DIRS include
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(omnirob_joint_state_publisher src/omnirob_joint_state_publisher.cpp)
target_link_libraries(omnirob_joint_state_publisher ${catkin_LIBRARIES})

add_executable(base_interface src/base_interface.cpp)
target_link_libraries(base_interface ${catkin_LIBRARIES})

add_executable(gripper_hack src/gripper_hack.cpp)
target_link_libraries(gripper_hack ${catkin_LIBRARIES})

add_executable(gripper_interface src/gripper_interface.cpp)
add_dependencies(gripper_interface omnirob_robin_msgs_gencpp)
target_link_libraries(gripper_interface ${catkin_LIBRARIES})

add_executable(pan_tilt_interface src/pan_tilt_interface.cpp)
add_dependencies(pan_tilt_interface omnirob_robin_msgs_gencpp)
target_link_libraries(pan_tilt_interface ${catkin_LIBRARIES})

add_executable(omnirob_robin_lwa_interface src/omnirob_robin_lwa_interface.cpp)
target_link_libraries(omnirob_robin_lwa_interface ${catkin_LIBRARIES})

add_executable(virtuell_robot_driver_interface src/virtuell_robot_driver_interface.cpp)
target_link_libraries(virtuell_robot_driver_interface ${catkin_LIBRARIES})

add_executable(omnirob_odometry_publisher_simulation_only src/omnirob_odometry_publisher_simulation_only.cpp)
target_link_libraries(omnirob_odometry_publisher_simulation_only ${catkin_LIBRARIES})

add_executable(teleop_joy src/teleop_joy.cpp)
target_link_libraries(teleop_joy ${catkin_LIBRARIES})

add_executable(base_driver src/base_driver.cpp)
target_link_libraries(base_driver ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
