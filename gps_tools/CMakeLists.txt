cmake_minimum_required(VERSION 3.10)
project(gps_tools)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

# Install the python module for this package
ament_python_install_package(${PROJECT_NAME})

add_library(utm_odometry_component SHARED
  src/utm_odometry_component.cpp)
target_include_directories(utm_odometry_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${rclcpp_components_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
)
target_compile_definitions(utm_odometry_component
  PRIVATE "COMPOSITION_BUILDING_DLL")

rclcpp_components_register_nodes(utm_odometry_component "gps_tools::UtmOdometryComponent")
target_link_libraries(utm_odometry_component PUBLIC
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_components::component_manager
  sensor_msgs::sensor_msgs_library
)

add_library(utm_odometry_to_navsatfix_component SHARED
  src/utm_odometry_to_navsatfix_component.cpp)

target_include_directories(utm_odometry_to_navsatfix_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${nav_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
)
target_compile_definitions(utm_odometry_to_navsatfix_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
rclcpp_components_register_nodes(utm_odometry_to_navsatfix_component "gps_tools::UtmOdometryToNavSatFixComponent")
target_link_libraries(utm_odometry_to_navsatfix_component PUBLIC
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_components::component_manager
  sensor_msgs::sensor_msgs_library
)

install(TARGETS
    utm_odometry_component
    utm_odometry_to_navsatfix_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
install(PROGRAMS
    scripts/bag_converter
    nodes/fix_translator
  DESTINATION bin
)

# Export package dependencies
ament_export_dependencies(ament_cmake
  ament_cmake_python
  rclcpp
  rclpy
)
ament_export_include_directories(include)
ament_export_libraries(utm_odometry_component utm_odometry_to_navsatfix_component)

ament_package()
