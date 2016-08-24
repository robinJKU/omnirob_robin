cmake_minimum_required(VERSION 2.8.3)
project(omnirob_robin_control)


find_package(catkin REQUIRED roscpp COMPONENTS sensor_msgs geometry_msgs actionlib actionlib_msgs control_msgs message_generation trajectory_msgs ros_common_robin_tools)

#generate_messages( DEPENDENCIES actionlib_msgs control_msgs trajectory_msgs)

catkin_package( CATKIN_DEPENDS actionlib_msgs control_msgs trajectory_msgs)

add_executable(follow_joint_trajectory_controller src/follow_joint_trajectory_controller.cpp)
target_link_libraries(follow_joint_trajectory_controller ${catkin_LIBRARIES})

add_executable(follow_joint_trajectory_controller_test_client src/follow_joint_trajectory_controller_test_client.cpp)
target_link_libraries(follow_joint_trajectory_controller_test_client ${catkin_LIBRARIES})



include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
