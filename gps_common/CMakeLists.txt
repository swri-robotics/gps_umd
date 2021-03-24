cmake_minimum_required(VERSION 2.8.3)
project(gps_common)

############
## Catkin ##
############

set(DEPS
  message_filters
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  rospy
)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  ${DEPS}
)

catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
  FILES
    GPSStatus.msg
    GPSFix.msg
)

generate_messages(DEPENDENCIES 
  nav_msgs
  sensor_msgs
  std_msgs
)

###################################################
## Declare things to be passed to other projects ##
###################################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    message_runtime
    ${DEPS}
)

###########
## Build ##
###########
include_directories(${catkin_INCLUDE_DIRS})

add_executable(utm_odometry_node src/utm_odometry_node.cpp)
add_dependencies(utm_odometry_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS} 
)
target_link_libraries(utm_odometry_node ${catkin_LIBRARIES})

add_executable(utm_odometry_to_navsatfix_node src/utm_odometry_to_navsatfix_node.cpp)
add_dependencies(utm_odometry_to_navsatfix_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS} 
)
target_link_libraries(utm_odometry_to_navsatfix_node ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS
  utm_odometry_node
  utm_odometry_to_navsatfix_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/ 
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

catkin_install_python(PROGRAMS nodes/fix_translator scripts/bag_converter
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
