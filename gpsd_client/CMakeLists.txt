cmake_minimum_required(VERSION 3.10)
project(gpsd_client)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)

find_package(gps_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED rclcpp_components)
find_package(sensor_msgs REQUIRED)

# Try to find libgps, first with CMake's usual library search method, then by
# querying pkg-config.
find_library(libgps_LIBRARIES NAMES gps)
find_path(libgps_INCLUDE_DIRS NAMES libgpsmm.h gps.h)

if(NOT libgps_LIBRARIES)
  message(STATUS "Checking pkg-config for libgps")
  find_package(PkgConfig)
  if(PkgConfig_FOUND)
    pkg_check_modules(libgps libgps)
  endif()
endif()

if(NOT libgps_LIBRARIES)
  message(FATAL_ERROR "Could not find libgps "
                      "(hint for Debian/Ubuntu: apt install libgps-dev)")
else()
  message(STATUS "Found libgps: ${libgps_LIBRARIES}")
endif()

add_library(${PROJECT_NAME} SHARED
  src/client.cpp
  src/gpsd_parser_factory.cpp
  src/parsers/gpsd_parser_base.cpp)

# Jazzy was the first distro to define an unknown fix, so this flag is used to correctly
# initialize variables 
if ("$ENV{ROS_DISTRO}" STRLESS "jazzy")
  target_compile_definitions(${PROJECT_NAME} PRIVATE "-DNO_UNKNOWN_FIX")
endif()

target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
    ${rclcpp_components_INCLUDE_DIRS}
    ${libgps_INCLUDE_DIRS}
    ${gps_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS})

target_compile_definitions(${PROJECT_NAME}
  PRIVATE "COMPOSITION_BUILDING_DLL")

rclcpp_components_register_nodes(${PROJECT_NAME} "gpsd_client::GPSDClientComponent")
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${libgps_LIBRARIES}
  ${gps_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_components::component_manager
  sensor_msgs::sensor_msgs_library
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gpsd_parser test/test_gpsd_parser.cpp)
  target_link_libraries(test_gpsd_parser ${PROJECT_NAME})
endif()

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}/)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_libraries(${PROJECT_NAME})

ament_package()
