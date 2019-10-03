cmake_minimum_required(VERSION 2.8.3)
project(gpsd_client CXX)

############
## Catkin ##
############

find_package(catkin REQUIRED COMPONENTS
  gps_msgs
  roscpp
  sensor_msgs
)

find_package(PkgConfig)
pkg_check_modules (libgps REQUIRED libgps)

###################################################
## Declare things to be passed to other projects ##
###################################################

catkin_package()

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS} ${libgps_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} src/client.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${libgps_LIBRARIES})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
