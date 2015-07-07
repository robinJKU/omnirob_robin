cmake_minimum_required(VERSION 2.8.3)
project(omnirob_robin_moveit)

find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  actionlib
  moveit_core
  moveit_msgs
  moveit_ros_perception
  moveit_ros_planning_interface
  pluginlib
  cmake_modules
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
  tf
  sensor_msgs
  shape_msgs
  omnirob_robin_tools_ros
  omnirob_robin_msgs
)

## System dependencies are found with CMake's conventions
# see CMakeLists pr2_moveit_tutorials
find_package(Boost REQUIRED system filesystem date_time thread)


## Generate services in the 'srv' folder
 add_service_files(
   FILES
   PlanPath.srv
   PlanHome.srv
   ExecutePath.srv
   AddCollisionObj.srv
 )
 
 add_action_files(DIRECTORY action FILES 
   move_home.action
   move_to_configuration.action
   pick_and_place.action
 )


# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   sensor_msgs
   shape_msgs
   actionlib_msgs
   moveit_msgs
   omnirob_robin_msgs
 )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

# see CMakeLists pr2_moveit_tutorials
catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES omnirob_robin_moveit
  CATKIN_DEPENDS 
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    roscpp
    rospy
    std_msgs
    message_runtime
    geometry_msgs
    tf
    sensor_msgs
    shape_msgs
  DEPENDS system_lib
)


# see CMakeLists pr2_moveit_tutorials
find_package(Eigen REQUIRED)


###########
## Build ##
###########

include_directories( include SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})


##################
## Executeables ##
##################

# This will be the node which is used to create the needed services
add_executable(omnirob_moveit src/omnirob_moveit.cpp)
target_link_libraries(omnirob_moveit ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS omnirob_moveit DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# This node is for testing rviz and gazebo with the moveit interface
add_executable(omnirob_moveit_simulation src/omnirob_moveit_simulation.cpp)
target_link_libraries(omnirob_moveit_simulation ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS omnirob_moveit_simulation DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# This node is our joint_states_publisher
add_executable(joint_states_publisher src/joint_states_publisher.cpp)
target_link_libraries(joint_states_publisher ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS joint_states_publisher DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(test_lwa_interface src/test_lwa_interface.cpp)
target_link_libraries(test_lwa_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS test_lwa_interface DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


##############################################################
## aus CMakeList vom setup_assitenten
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})



