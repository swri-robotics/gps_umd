cmake_minimum_required(VERSION 2.8.3)
project(gpsd_client)

############
## Catkin ##
############

find_package(catkin REQUIRED COMPONENTS
  gps_common
  roscpp
  sensor_msgs
)

find_package(PkgConfig)
pkg_check_modules (libgps REQUIRED libgps)

###################################################
## Declare things to be passed to other projects ##
###################################################

catkin_package(
  CATKIN_DEPENDS
    gps_common
    roscpp
    sensor_msgs
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS} ${libgps_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} src/client.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${libgps_LIBRARIES})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
