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
)
target_compile_definitions(utm_odometry_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
rclcpp_components_register_nodes(utm_odometry_component "gps_tools::UtmOdometryComponent")
ament_target_dependencies(utm_odometry_component
  "nav_msgs"
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
)

add_library(utm_odometry_to_navsatfix_component SHARED
  src/utm_odometry_to_navsatfix_component.cpp)
target_include_directories(utm_odometry_to_navsatfix_component
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(utm_odometry_to_navsatfix_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
rclcpp_components_register_nodes(utm_odometry_to_navsatfix_component "gps_tools::UtmOdometryToNavSatFixComponent")
ament_target_dependencies(utm_odometry_to_navsatfix_component
  "nav_msgs"
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
)

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

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
ament_export_libraries(${PROJECT_NAME})

ament_package()
