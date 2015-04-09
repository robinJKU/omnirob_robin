cmake_minimum_required(VERSION 2.8.3)
project(omnirob_robin_control)


find_package(catkin REQUIRED roscpp COMPONENTS sensor_msgs geometry_msgs)


catkin_package( )

# add_executable(rotational_drift_compensator src/rotational_drift_compensator.cpp)
# target_link_libraries(rotational_drift_compensator ${catkin_LIBRARIES})

include_directories(
  ${catkin_INCLUDE_DIRS}
)


install(
    PROGRAMS
      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
