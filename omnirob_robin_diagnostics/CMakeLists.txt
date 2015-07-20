cmake_minimum_required(VERSION 2.8.3)
project(omnirob_robin_diagnostics)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  roscpp
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS diagnostic_msgs roscpp std_msgs
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(omnirob_robin_diagnostics src/diagnostics.cpp)

target_link_libraries(omnirob_robin_diagnostics
   ${catkin_LIBRARIES}
)

#############
## Install ##
#############


#############
## Testing ##
#############

